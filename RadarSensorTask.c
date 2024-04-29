/*
 * RadarSensorTask.c
 *
 *  Created on: 2024-03-26
 *      Author: Gintaras
 */

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "RadarSensorTask.h"
#include "SmartDisplayTask.h"
#include "resource_map.h"
#include "radar_settings.h"
#include "xensiv_bgt60trxx_mtb.h"
#include "arm_math.h"
#include "ifx_sensor_dsp.h"

#define RAD_IRQ_PRIORITY					(1)
#define XENSIV_BGT60TRXX_SPI_FREQUENCY      (5000000UL)
#define NUM_SAMPLES_PER_FRAME               (XENSIV_BGT60TRXX_CONF_NUM_SAMPLES_PER_CHIRP *\
                                             XENSIV_BGT60TRXX_CONF_NUM_CHIRPS_PER_FRAME *\
                                             XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS)
#define NUM_CHIRPS_PER_FRAME                XENSIV_BGT60TRXX_CONF_NUM_CHIRPS_PER_FRAME
#define NUM_SAMPLES_PER_CHIRP               XENSIV_BGT60TRXX_CONF_NUM_SAMPLES_PER_CHIRP

static int32_t init_sensor(void);
static void xensiv_bgt60trxx_interrupt_handler(void* args, cyhal_gpio_event_t event);
static void DistanceProcess(void *param);

TaskHandle_t RadarSensorTaskHandle = NULL;
TaskHandle_t DistanceProcessHandle = NULL;
_Bool process_idle = true;

cyhal_spi_t spi_obj;
xensiv_bgt60trxx_mtb_t bgt60_obj;
uint16_t bgt60_buffer[NUM_SAMPLES_PER_FRAME] __attribute__((aligned(2)));
float32_t frame[NUM_SAMPLES_PER_FRAME];
float32_t avg_chirp[NUM_SAMPLES_PER_CHIRP];
/*Range measurement Global Variables*/
float32_t window[NUM_SAMPLES_PER_CHIRP];
//float32_t rframe[NUM_SAMPLES_PER_FRAME];
cfloat32_t range[NUM_SAMPLES_PER_FRAME];
float32_t range_mag[NUM_SAMPLES_PER_FRAME];
float32_t range_chirps[NUM_SAMPLES_PER_CHIRP];
#define MAX_PEAKS (3) /*Maximum quantity of objects we are looking for*/
int32_t peaks = 0;
int32_t peaks_arr[MAX_PEAKS];

/*Exported Global Variables*/
float32_t peak_distance = 0;
float32_t max_peak = 0;
int32_t max_peak_arg = -1;

void RadarSensorTask(void *param)
{
	(void) param;

    if (init_sensor() != 0)
    {
        CY_ASSERT(0);
    }

    /* Distance Processing Task */
    xTaskCreate(DistanceProcess, "dist task", configMINIMAL_STACK_SIZE*2, NULL, configMAX_PRIORITIES - 6, &DistanceProcessHandle);
    if(DistanceProcessHandle == NULL)
    {
    	printf("Error: could not create distance processing task.\r\n");
    	CY_ASSERT(0);
    }

    if (xensiv_bgt60trxx_start_frame(&bgt60_obj.dev, true) != XENSIV_BGT60TRXX_STATUS_OK)
    {
        CY_ASSERT(0);
    }

	for(;;)
	{
        /* Wait for the GPIO interrupt to indicate that another slice is available */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (xensiv_bgt60trxx_get_fifo_data(&bgt60_obj.dev, bgt60_buffer, NUM_SAMPLES_PER_FRAME) == XENSIV_BGT60TRXX_STATUS_OK)
        {
        	/*Skip the frame if busy */
        	if(process_idle)
        	{
        		process_idle = false;
                /* Data preprocessing */
                uint16_t *bgt60_buffer_ptr = &bgt60_buffer[0];
                float32_t *frame_ptr = &frame[0];
                for (int32_t sample = 0; sample < NUM_SAMPLES_PER_FRAME; ++sample)
                {
                    *frame_ptr++ = ((float32_t)(*bgt60_buffer_ptr++) / 4096.0F);
                }

                /* Calculate the average of the chirps first */
                arm_fill_f32(0, avg_chirp, NUM_SAMPLES_PER_CHIRP);

                for (int chirp = 0; chirp < NUM_CHIRPS_PER_FRAME; chirp++)
                {
                    arm_add_f32(avg_chirp, &frame[NUM_SAMPLES_PER_CHIRP * chirp], avg_chirp, NUM_SAMPLES_PER_CHIRP);
                }

                arm_scale_f32(avg_chirp, 1.0f / NUM_CHIRPS_PER_FRAME, avg_chirp, NUM_SAMPLES_PER_CHIRP);

                /*Process the Data*/
                xTaskNotifyGive(DistanceProcessHandle);
        	}
        }
	}
}

/*Static Distance Processing Task*/
static void DistanceProcess(void *param)
{
	uint32_t ulPreviousValue;
    int32_t dsp_stat = IFX_SENSOR_DSP_STATUS_OK;
    ifx_window_hann_f32(window, NUM_SAMPLES_PER_CHIRP);
    ifx_peak_search_opts_f32_t peak_opts =
    {
    		.distance = 5,
			.height = 0.35,
			.threshold = 0.25,
			.width = 2
    };

	for(;;)
	{
	    /* Wait for data ready notification */
	    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		/*Measure the range FFT, using Hann window and removal of the mean*/
		dsp_stat = ifx_range_fft_f32(frame, range, true, window,NUM_SAMPLES_PER_CHIRP, NUM_CHIRPS_PER_FRAME);
		/*Do the processing if FFT measurement succeeds*/
		if (dsp_stat == IFX_SENSOR_DSP_STATUS_OK)
		{
			/*Calculate the magnitudes of all the peaks*/
			arm_cmplx_mag_f32((float32_t*) range, range_mag,
					NUM_SAMPLES_PER_FRAME);
			/*Clear the data arrays for processing*/
			arm_fill_f32(0, range_chirps, NUM_SAMPLES_PER_CHIRP);
			arm_fill_q31(0, peaks_arr, MAX_PEAKS);
			/*Coherent integration off all chirps*/
			for (int chirp = 0; chirp < NUM_CHIRPS_PER_FRAME; chirp++)
			{
				arm_add_f32(range_chirps, &range_mag[NUM_SAMPLES_PER_CHIRP * chirp], range_chirps, NUM_SAMPLES_PER_CHIRP);
			}
			/*Removal of mirror image in data*/
			arm_fill_f32(0, &range_chirps[NUM_SAMPLES_PER_CHIRP / 2], NUM_SAMPLES_PER_CHIRP / 2);

			/*Remove the DC component*/
			range_chirps[0] = 0;
			range_chirps[1] = 0;
			range_chirps[2] = 0;

			/*Peaks detection, the number of MAX_PEAKS will be stored if found*/
			peaks = 0;
			peaks = ifx_peak_search_f32(range_chirps, NUM_SAMPLES_PER_CHIRP, peaks_arr, MAX_PEAKS, &peak_opts);
			if (peaks)
			{
				/*Look for the maximum peak*/
				max_peak = 0;
				max_peak_arg = -1;
				for (int max_ch = 0; max_ch < peaks; max_ch++)
				{
					if (range_chirps[peaks_arr[max_ch]] > max_peak)
					{
						max_peak = range_chirps[peaks_arr[max_ch]];
						max_peak_arg = peaks_arr[max_ch];
					}
				}

				/*Calculate the distance*/
				if (max_peak_arg > -1)
				{
					peak_distance = IFX_LIGHT_SPEED_M_S / (2 * (XENSIV_BGT60TRXX_CONF_END_FREQ_HZ- XENSIV_BGT60TRXX_CONF_START_FREQ_HZ));
					peak_distance = peak_distance * max_peak_arg;
				}
			}
			else
			{
				max_peak = 0;
				max_peak_arg = -1;
				peak_distance = 1.0 / 0.0;
			}
		}

		while( xTaskNotifyAndQuery( SmartDisplayTaskHandle,sigDISTANCE,eSetValueWithoutOverwrite, &ulPreviousValue ) != pdPASS )
		{
			vTaskDelay(pdMS_TO_TICKS(10));
		}

	    /*Processing is over*/
	    process_idle = true;
	}
}

/*******************************************************************************
* Function Name: init_sensor
********************************************************************************
* Summary:
* This function configures the SPI interface, initializes radar and interrupt
* service routine to indicate the availability of radar data.
*
* Parameters:
*  void
*
* Return:
*  Success or error
*
*******************************************************************************/
static int32_t init_sensor(void)
{
    if (cyhal_spi_init(&spi_obj,
                       PIN_XENSIV_BGT60TRXX_SPI_MOSI,
                       PIN_XENSIV_BGT60TRXX_SPI_MISO,
                       PIN_XENSIV_BGT60TRXX_SPI_SCLK,
                       NC,
                       NULL,
                       8,
                       CYHAL_SPI_MODE_00_MSB,
                       false) != CY_RSLT_SUCCESS)
    {
        printf("ERROR: cyhal_spi_init failed\n");
        return -1;
    }

    /* Set the data rate to XENSIV_BGT60TRXX_SPI_FREQUENCY Mbps */
    if (cyhal_spi_set_frequency(&spi_obj, XENSIV_BGT60TRXX_SPI_FREQUENCY) != CY_RSLT_SUCCESS)
    {
        printf("ERROR: cyhal_spi_set_frequency failed\n");
        return -1;
    }

    /* Enable Sensor */
    if (cyhal_gpio_init(PIN_XENSIV_BGT60TRXX_LDO_EN,
                        CYHAL_GPIO_DIR_OUTPUT,
                        CYHAL_GPIO_DRIVE_STRONG,
                        true) != CY_RSLT_SUCCESS)
    {
        printf("ERROR: LDO_EN cyhal_gpio_init failed\n");
        return -1;
    }

    /* Wait LDO stable */
    (void)cyhal_system_delay_ms(5);

    if (xensiv_bgt60trxx_mtb_init(&bgt60_obj,
                                  &spi_obj,
                                  PIN_XENSIV_BGT60TRXX_SPI_CSN,
                                  PIN_XENSIV_BGT60TRXX_RSTN,
                                  register_list,
                                  XENSIV_BGT60TRXX_CONF_NUM_REGS) != CY_RSLT_SUCCESS)
    {
        printf("ERROR: xensiv_bgt60trxx_mtb_init failed\n");
        return -1;
    }

    if (xensiv_bgt60trxx_mtb_interrupt_init(&bgt60_obj,
                                            NUM_SAMPLES_PER_FRAME,
                                            PIN_XENSIV_BGT60TRXX_IRQ,
											RAD_IRQ_PRIORITY,
                                            xensiv_bgt60trxx_interrupt_handler,
                                            NULL) != CY_RSLT_SUCCESS)
    {
        printf("ERROR: xensiv_bgt60trxx_mtb_interrupt_init failed\n");
        return -1;
    }

    return 0;
}

/*******************************************************************************
* Function Name: xensiv_bgt60trxx_interrupt_handler
********************************************************************************
* Summary:
* This is the interrupt handler to react on sensor indicating the availability
* of new data
*    1. Notifies main task on interrupt from sensor
*
* Parameters:
*  void
*
* Return:
*  none
*
*******************************************************************************/
#if defined(CYHAL_API_VERSION) && (CYHAL_API_VERSION >= 2)
static void xensiv_bgt60trxx_interrupt_handler(void *args, cyhal_gpio_event_t event)
#else
static void xensiv_bgt60trxx_interrupt_handler(void *args, cyhal_gpio_irq_event_t event)
#endif
{
    CY_UNUSED_PARAMETER(args);
    CY_UNUSED_PARAMETER(event);

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    vTaskNotifyGiveFromISR(RadarSensorTaskHandle, &xHigherPriorityTaskWoken);

    /* Context switch needed? */
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

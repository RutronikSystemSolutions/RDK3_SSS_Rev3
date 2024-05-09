/*
 * ThermalSensorTask.c
 *
 *  Created on: 2024-03-25
 *      Author: Gintaras
 */

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "FreeRTOS.h"
#include "task.h"

#include "ThermalSensorTask.h"
#include "SmartDisplayTask.h"
#include "d6t32l01a.h"

#define D6T_I2C_FREQ	1000000UL

TaskHandle_t ThermalSensorTaskHandle = NULL;

/*D6T32L01A Variables*/
uint8_t rbuf[N_READ];
float ptat;
float pix_data[N_PIXEL];

/*I2C Device Global Variables*/
cyhal_i2c_t I2C_scb1;
cyhal_i2c_cfg_t i2c_scb1_cfg =
{
		.is_slave = false,
	    .address = 0,
	    .frequencyhal_hz = D6T_I2C_FREQ,
};

void ThermalSensorTask(void *param)
{
	(void) param;
    cy_rslt_t result;
    uint32_t d6t_rslt;
    uint32_t ulPreviousValue;

    /*Initialize I2C Master*/
    result = cyhal_i2c_init(&I2C_scb1, D6T_SDA, D6T_SCL, NULL);
    if (result != CY_RSLT_SUCCESS)
    {
    	CY_ASSERT(0);
    }
    result = cyhal_i2c_configure(&I2C_scb1, &i2c_scb1_cfg);
    if (result != CY_RSLT_SUCCESS)
    {
    	CY_ASSERT(0);
    }

    /*Initialize D6T32L01A Power Control Pin*/
    result = cyhal_gpio_init(ARDU_ADC3, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, true);
    if (result != CY_RSLT_SUCCESS)
    {
    	CY_ASSERT(0);
    }

    /*Initialize the D6T32L01A */
    vTaskDelay(pdMS_TO_TICKS(400));
    d6t_rslt = d6t32_init();
    if(d6t_rslt != D6T_OK)
    {
    	printf("D6T32L01 Initialization Failure.\r\n");
    	CY_ASSERT(0);
    }

    memset(rbuf, 0x00, sizeof(rbuf));

	for(;;)
	{
    	if( xTaskNotifyAndQuery( SmartDisplayTaskHandle,sigTHERMO,eSetValueWithoutOverwrite, &ulPreviousValue ) == pdPASS )
    	{
    	    /* The task's notification value was updated. */
        	/*Read the values from the sensor*/
        	d6t_rslt =  D6T_getvalue(rbuf, &ptat, pix_data);

        	/*Check if the temperature values were read without any errors*/
            if(d6t_rslt != D6T_OK)
            {
            	/*Try to recover*/
            	memset(rbuf, 0x00, sizeof(rbuf));
            	cyhal_gpio_write((cyhal_gpio_t)ARDU_ADC3, false);
            	vTaskDelay(pdMS_TO_TICKS(100));
            	cyhal_gpio_write((cyhal_gpio_t)ARDU_ADC3, true);
            	vTaskDelay(pdMS_TO_TICKS(400));
            	d6t32_init();
            }
    	}
    	else
    	{
    		vTaskDelay(pdMS_TO_TICKS(50));
    		cyhal_gpio_toggle(LED3);
    	}
	}
}

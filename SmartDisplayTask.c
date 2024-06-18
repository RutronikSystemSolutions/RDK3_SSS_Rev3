/*
 * SmartDisplayTask.c
 *
 *  Created on: 2024-04-03
 *      Author: Gintaras
 *
 */

#include "SmartDisplayTask.h"
#include "ThermalSensorTask.h"
#include "GestureSensorTask.h"
#include "RadarSensorTask.h"
#include "d6t32l01a.h"
#include "image_lut.h"
#include "arm_math.h"
#include "watchdog.h"

TaskHandle_t SmartDisplayTaskHandle = NULL;

/*Arduino UART object and configuration*/
cyhal_uart_t ardu_uart;

/*Thermal image data*/
uint8_t thermal_image[N_PIXEL] = {0};
uint8_t thermal_cache[N_PIXEL] = {0};
float max_temp;
uint32_t max_temp_index;
float min_temp;
uint32_t min_temp_index;

/*Function prototypes*/
static void ResetDisplay(void);
cy_rslt_t ardu_uart_init(void);
static void DrawStaticDisplay(void);
static void DrawThermalImage(void);
static void DrawTemperatures(float max_t, float min_t);
static void DrawChevrons(void);
static void DrawDistance(void);
static void DrawPresence(_Bool presence);

void SmartDisplayTask(void *param)
{
	(void) param;
	cy_rslt_t result;
	float scale_unit = 0;
	float thermal_diff = 0;
	int32_t iron_map_index = 0;
	uint32_t ulNotifiedValue;
	static _Bool wdt_deinit = false;

	/*Reset the Display*/
	ResetDisplay();

    /*Initialize The Arduino UART*/
    result = ardu_uart_init();
    if (result != CY_RSLT_SUCCESS)
    {
    	printf("Could not initialize Arduino UART.\r\n");
    	CY_ASSERT(0);
    }

	/*POR Delay*/
	vTaskDelay(pdMS_TO_TICKS(STARTUP_DELAY_MS));

	/*Setup the environment*/
	DrawStaticDisplay();

	for(;;)
	{
		 xTaskNotifyWait( 0x00, 0xFF, &ulNotifiedValue, portMAX_DELAY);

		 if(!wdt_deinit)
		 {
			 watchdog_stop();
			 watchdog_deinit();
			 wdt_deinit = true;
		 }

		 /*Drawing thermal image*/
	     if( ( ulNotifiedValue & sigTHERMO ) != 0 )
	     {
	    	 /*Calculate & Convert Color Scale*/
	    	arm_max_f32(pix_data, N_PIXEL, &max_temp, &max_temp_index);
	    	arm_min_f32(pix_data, N_PIXEL, &min_temp, &min_temp_index);
	    	scale_unit = (max_temp - min_temp)/BITS_UINT8;
		    for(uint32_t x = 0; x < N_PIXEL; x++)
		    {
		    	thermal_diff = pix_data[x] - min_temp;
		    	iron_map_index = thermal_diff/scale_unit - 1;
		    	if(iron_map_index < 0)
		    	{
		    		iron_map_index = 0;
		    	}
		    	else if(iron_map_index > 254)
		    	{
		    		iron_map_index = 254;
		    	}
		    		thermal_image[x] = iron_map[iron_map_index];
		    }

	    	/*Draw a Thermal Image*/
	    	DrawThermalImage();

		    /*Draw Min/Max Temperatures*/
		    DrawTemperatures(max_temp, min_temp);
	     }

	     /*Draw Static Distance*/
	     else if( (ulNotifiedValue & sigDISTANCE ) != 0)
	     {
	    	 DrawDistance();
	     }

	     /*Draw the recognized gestures*/
	     else if( (ulNotifiedValue & sigGESTURE ) != 0)
		 {
	    	 DrawChevrons();
		 }

	     /*Draw the ABSENCE*/
	     else if( (ulNotifiedValue & sigABSENCE ) != 0)
		 {
	    	 DrawPresence(false);
		 }

	     /*Draw the PRESENCE*/
	     else if( (ulNotifiedValue & sigPRESENCE ) != 0)
		 {
	    	 DrawPresence(true);
		 }
	}
}

cy_rslt_t ardu_uart_init(void)
{
	cy_rslt_t result;
	uint32_t actualbaud;

    /* Initialize the UART configuration structure */
    const cyhal_uart_cfg_t uart_config =
    {
        .data_bits = 8,
        .stop_bits = 1,
        .parity = CYHAL_UART_PARITY_NONE,
        .rx_buffer = NULL,
        .rx_buffer_size = 0
    };

    /* Initialize the UART Block */
    result = cyhal_uart_init(&ardu_uart, ARDU_TX, ARDU_RX, NC, NC, NULL, &uart_config);
	if (result != CY_RSLT_SUCCESS)
	{return result;}

	result = cyhal_uart_set_baud(&ardu_uart, ARDU_BAUD_RATE, &actualbaud);
	if (result != CY_RSLT_SUCCESS)
	{return result;}

	/*Connect internal pull-up resistor*/
	cyhal_gpio_configure(ARDU_RX, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP);

	return result;
}

/*Reset Display Function*/
static void ResetDisplay(void)
{
	cyhal_gpio_write(ARDU_IO8, false);
	vTaskDelay(pdMS_TO_TICKS(500));
	cyhal_gpio_write(ARDU_IO8, true);
	vTaskDelay(pdMS_TO_TICKS(3000));
}

/*Draw Static Display Function*/
static void DrawStaticDisplay(void)
{
	int x=0, y=0;
	uint32_t position = 0;

	/*RULER*/
	cyhal_uart_putc(&ardu_uart, POSLEFT_CMD);
	cyhal_uart_putc(&ardu_uart, (RULER_POSLEFT) & 0xFF);
	cyhal_uart_putc(&ardu_uart, ((RULER_POSLEFT) >> 8) & 0xFF);
	cyhal_uart_putc(&ardu_uart, DUMMY_CMD);
	cyhal_uart_putc(&ardu_uart, POSTOP_CMD);
	cyhal_uart_putc(&ardu_uart, (RULER_POSTOP) & 0xFF);
	cyhal_uart_putc(&ardu_uart, ((RULER_POSTOP) >> 8) & 0xFF);
	cyhal_uart_putc(&ardu_uart, DUMMY_CMD);

	/*Draw the ruler image*/
	position = 0;
	for(y = 0; y < 20; y++)
	{
		for(x = 0; x < 4; x++)
		{
	    	cyhal_uart_putc(&ardu_uart, x);
	    	cyhal_uart_putc(&ardu_uart, y);
	    	cyhal_uart_putc(&ardu_uart, 0x20);
	    	cyhal_uart_putc(&ardu_uart, ruler_map[position]);
	    	position++;
		}
	}
}

static void DrawThermalImage(void)
{
	cy_rslt_t result;
	int x=0, y=0;
	uint32_t position = 0;
	uint8_t byte;

	/*POSLEFT*/
	cyhal_uart_putc(&ardu_uart, POSLEFT_CMD);
	cyhal_uart_putc(&ardu_uart, TH_IMG_POSLEFT & 0xFF);
	cyhal_uart_putc(&ardu_uart, (TH_IMG_POSLEFT >> 8) & 0xFF);
	cyhal_uart_putc(&ardu_uart, DUMMY_CMD);

	/*POSTOP*/
	cyhal_uart_putc(&ardu_uart, POSTOP_CMD);
	cyhal_uart_putc(&ardu_uart, TH_IMG_POSTOP & 0xFF);
	cyhal_uart_putc(&ardu_uart, (TH_IMG_POSTOP >> 8) & 0xFF);
	cyhal_uart_putc(&ardu_uart, DUMMY_CMD);

	/*Draw the thermal image*/
	position = 0;
	for(y = 0; y < 32; y++)
	{
		for(x = 0; x < 32; x++)
		{
	    	/*Send if data changes*/
			if(thermal_image[position] != thermal_cache[position])
			{
		    	cyhal_uart_putc(&ardu_uart, x);
		    	cyhal_uart_putc(&ardu_uart, y);
		    	cyhal_uart_putc(&ardu_uart, 0x20);
		    	cyhal_uart_putc(&ardu_uart, thermal_image[position]);
		    	thermal_cache[position] = thermal_image[position];
			}
	    	position++;

	    	/*Check if a display data buffer is now overflowing*/
	    	result = cyhal_uart_readable(&ardu_uart);
	        if (result > 0)
	        {
	        	cyhal_uart_getc(&ardu_uart, &byte,0xFFFFFFFF);
	        	if(byte == 0xFF)
	        	{
	        		/*Wait for ready signal with a timeout*/
	        		for(uint8_t j = 0; j < BUFF_OVF_TOUT_MS; j++)
	        		{
	        			CyDelay(1);
	        			result = cyhal_uart_readable(&ardu_uart);
	        			if (result > 0)
	        			{
	        				cyhal_uart_getc(&ardu_uart, &byte,0xFFFFFFFF);
	        			}
	        			if(byte == 0xFE)
	        			{
	        				break;
	        			}
	        		}
	        	}
	        }
		}
	}
}

static void DrawTemperatures(float max_t, float min_t)
{
	char temp[6] = {0};
	uint8_t pos;

	/*Set the position above the ruler*/
	cyhal_uart_putc(&ardu_uart, POSLEFT_CMD);
	cyhal_uart_putc(&ardu_uart, (RULER_POSLEFT) & 0xFF);
	cyhal_uart_putc(&ardu_uart, ((RULER_POSLEFT) >> 8) & 0xFF);
	cyhal_uart_putc(&ardu_uart, DUMMY_CMD);
	cyhal_uart_putc(&ardu_uart, POSTOP_CMD);
	cyhal_uart_putc(&ardu_uart, (RULER_POSTOP-10) & 0xFF);
	cyhal_uart_putc(&ardu_uart, ((RULER_POSTOP-10) >> 8) & 0xFF);
	cyhal_uart_putc(&ardu_uart, DUMMY_CMD);

	/*Convert maximum temperature to string*/
	sprintf(temp, "%d", (int)max_t);

	/*Clean old data*/
	for(pos = 0; pos < sizeof(temp); pos++)
	{
		cyhal_uart_putc(&ardu_uart, pos);
		cyhal_uart_putc(&ardu_uart, 0);
		cyhal_uart_putc(&ardu_uart, 0x20);
		cyhal_uart_putc(&ardu_uart, 0x00);
	}

	/*Draw the maximum temperature*/
	for(pos = 0; pos < strlen(temp); pos++)
	{
		cyhal_uart_putc(&ardu_uart, pos);
		cyhal_uart_putc(&ardu_uart, 0);
		cyhal_uart_putc(&ardu_uart, temp[pos]);
		cyhal_uart_putc(&ardu_uart, 0x00);
	}

	/*Draw the degrees symbol*/
	cyhal_uart_putc(&ardu_uart, pos);
	cyhal_uart_putc(&ardu_uart, 0);
	cyhal_uart_putc(&ardu_uart, 0xA1);
	cyhal_uart_putc(&ardu_uart, 0x00);
	pos++;

	/*Draw the Celsius symbol*/
	cyhal_uart_putc(&ardu_uart, pos);
	cyhal_uart_putc(&ardu_uart, 0);
	cyhal_uart_putc(&ardu_uart, 'C');
	cyhal_uart_putc(&ardu_uart, 0x00);

	/*Set the position below the ruler*/
	cyhal_uart_putc(&ardu_uart, POSLEFT_CMD);
	cyhal_uart_putc(&ardu_uart, (RULER_POSLEFT) & 0xFF);
	cyhal_uart_putc(&ardu_uart, ((RULER_POSLEFT) >> 8) & 0xFF);
	cyhal_uart_putc(&ardu_uart, DUMMY_CMD);
	cyhal_uart_putc(&ardu_uart, POSTOP_CMD);
	cyhal_uart_putc(&ardu_uart, (RULER_POSTOP+160) & 0xFF);
	cyhal_uart_putc(&ardu_uart, ((RULER_POSTOP+160) >> 8) & 0xFF);
	cyhal_uart_putc(&ardu_uart, DUMMY_CMD);

	/*Convert maximum temperature to string*/
	memset(temp, 0x00, sizeof(temp));
	sprintf(temp, "%d", (int)min_t);

	/*Clean old data*/
	for(pos = 0; pos < sizeof(temp); pos++)
	{
		cyhal_uart_putc(&ardu_uart, pos);
		cyhal_uart_putc(&ardu_uart, 0);
		cyhal_uart_putc(&ardu_uart, 0x20);
		cyhal_uart_putc(&ardu_uart, 0x00);
	}

	/*Draw the mainimum temperature*/
	for(pos = 0; pos < strlen(temp); pos++)
	{
		cyhal_uart_putc(&ardu_uart, pos);
		cyhal_uart_putc(&ardu_uart, 0);
		cyhal_uart_putc(&ardu_uart, temp[pos]);
		cyhal_uart_putc(&ardu_uart, 0x00);
	}

	/*Draw the degrees symbol*/
	cyhal_uart_putc(&ardu_uart, pos);
	cyhal_uart_putc(&ardu_uart, 0);
	cyhal_uart_putc(&ardu_uart, 0xA1);
	cyhal_uart_putc(&ardu_uart, 0x00);
	pos++;

	/*Draw the Celsius symbol*/
	cyhal_uart_putc(&ardu_uart, pos);
	cyhal_uart_putc(&ardu_uart, 0);
	cyhal_uart_putc(&ardu_uart, 'C');
	cyhal_uart_putc(&ardu_uart, 0x00);
}

static void DrawChevrons(void)
{
	int x=0, y=0;
	uint32_t position = 0;
	static uint8_t current_gesture = GESTURE_INIT;
	char temp_arr[16] = {0};

	if(current_gesture != gesture_data.gesture)
	{
		current_gesture = (uint8_t)gesture_data.gesture;

		switch (current_gesture)
		{
			case GESTURE_INIT:
			{
				break;
			}
			case GESTURE_RIGHT:
			{
				/*Set the chevron position*/
				cyhal_uart_putc(&ardu_uart, POSLEFT_CMD);
				cyhal_uart_putc(&ardu_uart, CHEVRIGHT_POSLEFT & 0xFF);
				cyhal_uart_putc(&ardu_uart, (CHEVRIGHT_POSLEFT >> 8) & 0xFF);
				cyhal_uart_putc(&ardu_uart, DUMMY_CMD);
				cyhal_uart_putc(&ardu_uart, POSTOP_CMD);
				cyhal_uart_putc(&ardu_uart, CHEVRIGHT_POSTOP & 0xFF);
				cyhal_uart_putc(&ardu_uart, (CHEVRIGHT_POSTOP >> 8) & 0xFF);
				cyhal_uart_putc(&ardu_uart, DUMMY_CMD);

				/**/
				position = 0;
				for(y = 0; y < 7; y++)
				{
					for(x = 0; x < 12; x++)
					{
				    	cyhal_uart_putc(&ardu_uart, x);
				    	cyhal_uart_putc(&ardu_uart, y);
				    	cyhal_uart_putc(&ardu_uart, 0x20);
				    	cyhal_uart_putc(&ardu_uart, chev_right[position]);
				    	position++;
					}
				}

				/*Set the string position*/
				cyhal_uart_putc(&ardu_uart, POSLEFT_CMD);
				cyhal_uart_putc(&ardu_uart, CHEVRIGHT_POSLEFT & 0xFF);
				cyhal_uart_putc(&ardu_uart, (CHEVRIGHT_POSLEFT >> 8) & 0xFF);
				cyhal_uart_putc(&ardu_uart, DUMMY_CMD);
				cyhal_uart_putc(&ardu_uart, POSTOP_CMD);
				cyhal_uart_putc(&ardu_uart, (CHEVRIGHT_POSTOP+70) & 0xFF);
				cyhal_uart_putc(&ardu_uart, ((CHEVRIGHT_POSTOP+70) >> 8) & 0xFF);
				cyhal_uart_putc(&ardu_uart, DUMMY_CMD);

				/*Clean the memory*/
				memset(temp_arr, 0x00, sizeof(temp_arr));
				sprintf(temp_arr, "GESTURE: RIGHT");

				/*Clean old data*/
				for(position = 0; position < sizeof(temp_arr); position++)
				{
					cyhal_uart_putc(&ardu_uart, position);
					cyhal_uart_putc(&ardu_uart, 0);
					cyhal_uart_putc(&ardu_uart, 0x20);
					cyhal_uart_putc(&ardu_uart, 0x00);
				}

				/*Draw the gesture string*/
				for(position = 0; position < strlen(temp_arr); position++)
				{
					cyhal_uart_putc(&ardu_uart, position);
					cyhal_uart_putc(&ardu_uart, 0);
					cyhal_uart_putc(&ardu_uart, temp_arr[position]);
					cyhal_uart_putc(&ardu_uart, 0x00);
				}
				break;
			}

			case GESTURE_LEFT:
			{
				/*Set the chevron position*/
				cyhal_uart_putc(&ardu_uart, POSLEFT_CMD);
				cyhal_uart_putc(&ardu_uart, CHEVLEFT_POSLEFT & 0xFF);
				cyhal_uart_putc(&ardu_uart, (CHEVLEFT_POSLEFT >> 8) & 0xFF);
				cyhal_uart_putc(&ardu_uart, DUMMY_CMD);
				cyhal_uart_putc(&ardu_uart, POSTOP_CMD);
				cyhal_uart_putc(&ardu_uart, CHEVLEFT_POSTOP & 0xFF);
				cyhal_uart_putc(&ardu_uart, (CHEVLEFT_POSTOP >> 8) & 0xFF);
				cyhal_uart_putc(&ardu_uart, DUMMY_CMD);

				/**/
				position = 0;
				for(y = 0; y < 7; y++)
				{
					for(x = 0; x < 12; x++)
					{
				    	cyhal_uart_putc(&ardu_uart, x);
				    	cyhal_uart_putc(&ardu_uart, y);
				    	cyhal_uart_putc(&ardu_uart, 0x20);
				    	cyhal_uart_putc(&ardu_uart, chev_left[position]);
				    	position++;
					}
				}

				/*Set the string position*/
				cyhal_uart_putc(&ardu_uart, POSLEFT_CMD);
				cyhal_uart_putc(&ardu_uart, CHEVRIGHT_POSLEFT & 0xFF);
				cyhal_uart_putc(&ardu_uart, (CHEVRIGHT_POSLEFT >> 8) & 0xFF);
				cyhal_uart_putc(&ardu_uart, DUMMY_CMD);
				cyhal_uart_putc(&ardu_uart, POSTOP_CMD);
				cyhal_uart_putc(&ardu_uart, (CHEVRIGHT_POSTOP+70) & 0xFF);
				cyhal_uart_putc(&ardu_uart, ((CHEVRIGHT_POSTOP+70) >> 8) & 0xFF);
				cyhal_uart_putc(&ardu_uart, DUMMY_CMD);

				/*Clean the memory*/
				memset(temp_arr, 0x00, sizeof(temp_arr));
				sprintf(temp_arr, "GESTURE: LEFT");

				/*Clean old data*/
				for(position = 0; position < sizeof(temp_arr); position++)
				{
					cyhal_uart_putc(&ardu_uart, position);
					cyhal_uart_putc(&ardu_uart, 0);
					cyhal_uart_putc(&ardu_uart, 0x20);
					cyhal_uart_putc(&ardu_uart, 0x00);
				}

				/*Draw the gesture string*/
				for(position = 0; position < strlen(temp_arr); position++)
				{
					cyhal_uart_putc(&ardu_uart, position);
					cyhal_uart_putc(&ardu_uart, 0);
					cyhal_uart_putc(&ardu_uart, temp_arr[position]);
					cyhal_uart_putc(&ardu_uart, 0x00);
				}
				break;
			}

			case GESTURE_UP:
			{
				/*Set the chevron position*/
				cyhal_uart_putc(&ardu_uart, POSLEFT_CMD);
				cyhal_uart_putc(&ardu_uart, CHEVUP_POSLEFT & 0xFF);
				cyhal_uart_putc(&ardu_uart, (CHEVUP_POSLEFT >> 8) & 0xFF);
				cyhal_uart_putc(&ardu_uart, DUMMY_CMD);
				cyhal_uart_putc(&ardu_uart, POSTOP_CMD);
				cyhal_uart_putc(&ardu_uart, (CHEVUP_POSTOP) & 0xFF);
				cyhal_uart_putc(&ardu_uart, ((CHEVUP_POSTOP) >> 8) & 0xFF);
				cyhal_uart_putc(&ardu_uart, DUMMY_CMD);

				/**/
				position = 0;
				for(y = 0; y < 7; y++)
				{
					for(x = 0; x < 12; x++)
					{
				    	cyhal_uart_putc(&ardu_uart, x);
				    	cyhal_uart_putc(&ardu_uart, y);
				    	cyhal_uart_putc(&ardu_uart, 0x20);
				    	cyhal_uart_putc(&ardu_uart, chev_up[position]);
				    	position++;
					}
				}
				/*Set the string position*/
				cyhal_uart_putc(&ardu_uart, POSLEFT_CMD);
				cyhal_uart_putc(&ardu_uart, CHEVRIGHT_POSLEFT & 0xFF);
				cyhal_uart_putc(&ardu_uart, (CHEVRIGHT_POSLEFT >> 8) & 0xFF);
				cyhal_uart_putc(&ardu_uart, DUMMY_CMD);
				cyhal_uart_putc(&ardu_uart, POSTOP_CMD);
				cyhal_uart_putc(&ardu_uart, (CHEVRIGHT_POSTOP+70) & 0xFF);
				cyhal_uart_putc(&ardu_uart, ((CHEVRIGHT_POSTOP+70) >> 8) & 0xFF);
				cyhal_uart_putc(&ardu_uart, DUMMY_CMD);

				/*Clean the memory*/
				memset(temp_arr, 0x00, sizeof(temp_arr));
				sprintf(temp_arr, "GESTURE: UP");

				/*Clean old data*/
				for(position = 0; position < sizeof(temp_arr); position++)
				{
					cyhal_uart_putc(&ardu_uart, position);
					cyhal_uart_putc(&ardu_uart, 0);
					cyhal_uart_putc(&ardu_uart, 0x20);
					cyhal_uart_putc(&ardu_uart, 0x00);
				}

				/*Draw the gesture string*/
				for(position = 0; position < strlen(temp_arr); position++)
				{
					cyhal_uart_putc(&ardu_uart, position);
					cyhal_uart_putc(&ardu_uart, 0);
					cyhal_uart_putc(&ardu_uart, temp_arr[position]);
					cyhal_uart_putc(&ardu_uart, 0x00);
				}
				break;
			}

			case GESTURE_DOWN:
			{
				/*Set the chevron position*/
				cyhal_uart_putc(&ardu_uart, POSLEFT_CMD);
				cyhal_uart_putc(&ardu_uart, CHEVDOWN_POSLEFT & 0xFF);
				cyhal_uart_putc(&ardu_uart, (CHEVDOWN_POSLEFT >> 8) & 0xFF);
				cyhal_uart_putc(&ardu_uart, DUMMY_CMD);
				cyhal_uart_putc(&ardu_uart, POSTOP_CMD);
				cyhal_uart_putc(&ardu_uart, CHEVDOWN_POSTOP & 0xFF);
				cyhal_uart_putc(&ardu_uart, (CHEVDOWN_POSTOP >> 8) & 0xFF);
				cyhal_uart_putc(&ardu_uart, DUMMY_CMD);

				/**/
				position = 0;
				for(y = 0; y < 7; y++)
				{
					for(x = 0; x < 12; x++)
					{
				    	cyhal_uart_putc(&ardu_uart, x);
				    	cyhal_uart_putc(&ardu_uart, y);
				    	cyhal_uart_putc(&ardu_uart, 0x20);
				    	cyhal_uart_putc(&ardu_uart, chev_down[position]);
				    	position++;
					}
				}
				/*Set the string position*/
				cyhal_uart_putc(&ardu_uart, POSLEFT_CMD);
				cyhal_uart_putc(&ardu_uart, CHEVRIGHT_POSLEFT & 0xFF);
				cyhal_uart_putc(&ardu_uart, (CHEVRIGHT_POSLEFT >> 8) & 0xFF);
				cyhal_uart_putc(&ardu_uart, DUMMY_CMD);
				cyhal_uart_putc(&ardu_uart, POSTOP_CMD);
				cyhal_uart_putc(&ardu_uart, (CHEVRIGHT_POSTOP+70) & 0xFF);
				cyhal_uart_putc(&ardu_uart, ((CHEVRIGHT_POSTOP+70) >> 8) & 0xFF);
				cyhal_uart_putc(&ardu_uart, DUMMY_CMD);

				/*Clean the memory*/
				memset(temp_arr, 0x00, sizeof(temp_arr));
				sprintf(temp_arr, "GESTURE: DOWN");

				/*Clean old data*/
				for(position = 0; position < sizeof(temp_arr); position++)
				{
					cyhal_uart_putc(&ardu_uart, position);
					cyhal_uart_putc(&ardu_uart, 0);
					cyhal_uart_putc(&ardu_uart, 0x20);
					cyhal_uart_putc(&ardu_uart, 0x00);
				}

				/*Draw the gesture string*/
				for(position = 0; position < strlen(temp_arr); position++)
				{
					cyhal_uart_putc(&ardu_uart, position);
					cyhal_uart_putc(&ardu_uart, 0);
					cyhal_uart_putc(&ardu_uart, temp_arr[position]);
					cyhal_uart_putc(&ardu_uart, 0x00);
				}
				break;
			}

			case GESTURE_NONE:
			{
				/*Set the chevron position*/
				cyhal_uart_putc(&ardu_uart, POSLEFT_CMD);
				cyhal_uart_putc(&ardu_uart, CHEVUP_POSLEFT & 0xFF);
				cyhal_uart_putc(&ardu_uart, (CHEVUP_POSLEFT >> 8) & 0xFF);
				cyhal_uart_putc(&ardu_uart, DUMMY_CMD);
				cyhal_uart_putc(&ardu_uart, POSTOP_CMD);
				cyhal_uart_putc(&ardu_uart, (CHEVUP_POSTOP) & 0xFF);
				cyhal_uart_putc(&ardu_uart, ((CHEVUP_POSTOP) >> 8) & 0xFF);
				cyhal_uart_putc(&ardu_uart, DUMMY_CMD);

				/**/
				position = 0;
				for(y = 0; y < 7; y++)
				{
					for(x = 0; x < 12; x++)
					{
				    	cyhal_uart_putc(&ardu_uart, x);
				    	cyhal_uart_putc(&ardu_uart, y);
				    	cyhal_uart_putc(&ardu_uart, 0x20);
				    	cyhal_uart_putc(&ardu_uart, chev_none[position]);
				    	position++;
					}
				}

				/*Set the string position*/
				cyhal_uart_putc(&ardu_uart, POSLEFT_CMD);
				cyhal_uart_putc(&ardu_uart, CHEVRIGHT_POSLEFT & 0xFF);
				cyhal_uart_putc(&ardu_uart, (CHEVRIGHT_POSLEFT >> 8) & 0xFF);
				cyhal_uart_putc(&ardu_uart, DUMMY_CMD);
				cyhal_uart_putc(&ardu_uart, POSTOP_CMD);
				cyhal_uart_putc(&ardu_uart, (CHEVRIGHT_POSTOP+70) & 0xFF);
				cyhal_uart_putc(&ardu_uart, ((CHEVRIGHT_POSTOP+70) >> 8) & 0xFF);
				cyhal_uart_putc(&ardu_uart, DUMMY_CMD);

				/*Clean the memory*/
				memset(temp_arr, 0x00, sizeof(temp_arr));
				sprintf(temp_arr, "GESTURE: NONE");

				/*Clean old data*/
				for(position = 0; position < sizeof(temp_arr); position++)
				{
					cyhal_uart_putc(&ardu_uart, position);
					cyhal_uart_putc(&ardu_uart, 0);
					cyhal_uart_putc(&ardu_uart, 0x20);
					cyhal_uart_putc(&ardu_uart, 0x00);
				}

				/*Draw the gesture string*/
				for(position = 0; position < strlen(temp_arr); position++)
				{
					cyhal_uart_putc(&ardu_uart, position);
					cyhal_uart_putc(&ardu_uart, 0);
					cyhal_uart_putc(&ardu_uart, temp_arr[position]);
					cyhal_uart_putc(&ardu_uart, 0x00);
				}
				break;
			}
		}
	}
}

static void DrawSlider(float distance)
{
	uint32_t x=0;
	uint32_t position = 0;
	uint32_t last_position = 0;
	int32_t current_slider = distance*100/SLIDER_MAX_DIST;
	current_slider = 100 - current_slider;
	if(current_slider < 0){current_slider = 0;}
	if(current_slider > 100){current_slider = 100;}

	/*Set the slider position*/
	cyhal_uart_putc(&ardu_uart, POSLEFT_CMD);
	cyhal_uart_putc(&ardu_uart, SLIDER_POSLEFT & 0xFF);
	cyhal_uart_putc(&ardu_uart, (SLIDER_POSLEFT >> 8) & 0xFF);
	cyhal_uart_putc(&ardu_uart, DUMMY_CMD);
	cyhal_uart_putc(&ardu_uart, POSTOP_CMD);
	cyhal_uart_putc(&ardu_uart, (SLIDER_POSTOP) & 0xFF);
	cyhal_uart_putc(&ardu_uart, ((SLIDER_POSTOP) >> 8) & 0xFF);
	cyhal_uart_putc(&ardu_uart, DUMMY_CMD);

	position = 0;
	last_position = (uint32_t)(current_slider*SLIDER_LAST_POS/100);
	for(x = 0; x < SLIDER_LAST_POS; x++)
	{
    	cyhal_uart_putc(&ardu_uart, x);
    	cyhal_uart_putc(&ardu_uart, 0);
    	cyhal_uart_putc(&ardu_uart, 0x20);

    	if(position < last_position)
    	{
    		cyhal_uart_putc(&ardu_uart, 0x5C);
    	}
    	else
    	{
    		cyhal_uart_putc(&ardu_uart, 0x00);
    	}

    	cyhal_uart_putc(&ardu_uart, x);
    	cyhal_uart_putc(&ardu_uart, 1);
    	cyhal_uart_putc(&ardu_uart, 0x20);

    	if(position < last_position)
    	{
    		cyhal_uart_putc(&ardu_uart, 0x5C);
    	}
    	else
    	{
    		cyhal_uart_putc(&ardu_uart, 0x00);
    	}
    	position++;
	}
}

static void DrawDistance(void)
{
	static float distance = 1;
	char temp_arr[32] = {0};
	uint8_t pos;

	if(distance != peak_distance)
	{
		distance = peak_distance;
		DrawSlider(distance);

		/*Set the distance string position*/
		cyhal_uart_putc(&ardu_uart, POSLEFT_CMD);
		cyhal_uart_putc(&ardu_uart, DISTANCE_POSLEFT & 0xFF);
		cyhal_uart_putc(&ardu_uart, (DISTANCE_POSLEFT >> 8) & 0xFF);
		cyhal_uart_putc(&ardu_uart, DUMMY_CMD);
		cyhal_uart_putc(&ardu_uart, POSTOP_CMD);
		cyhal_uart_putc(&ardu_uart, (DISTANCE_POSTOP) & 0xFF);
		cyhal_uart_putc(&ardu_uart, ((DISTANCE_POSTOP) >> 8) & 0xFF);
		cyhal_uart_putc(&ardu_uart, DUMMY_CMD);

		/*Convert measured distance to string*/
		sprintf(temp_arr, "DISTANCE: %.2f", peak_distance);

		/*Clean old data*/
		for(pos = 0; pos < sizeof(temp_arr); pos++)
		{
			cyhal_uart_putc(&ardu_uart, pos);
			cyhal_uart_putc(&ardu_uart, 0);
			cyhal_uart_putc(&ardu_uart, 0x20);
			cyhal_uart_putc(&ardu_uart, 0x00);
		}

		/*Draw the measured distance*/
		for(pos = 0; pos < strlen(temp_arr); pos++)
		{
			cyhal_uart_putc(&ardu_uart, pos);
			cyhal_uart_putc(&ardu_uart, 0);
			cyhal_uart_putc(&ardu_uart, temp_arr[pos]);
			cyhal_uart_putc(&ardu_uart, 0x00);
		}

		/*Draw the meters symbol*/
		cyhal_uart_putc(&ardu_uart, pos);
		cyhal_uart_putc(&ardu_uart, 0);
		cyhal_uart_putc(&ardu_uart, 0x6D);
		cyhal_uart_putc(&ardu_uart, 0x00);

		//////////////////////////////////////////////

		/*Set the fft string position*/
		cyhal_uart_putc(&ardu_uart, POSLEFT_CMD);
		cyhal_uart_putc(&ardu_uart, DISTANCE_POSLEFT & 0xFF);
		cyhal_uart_putc(&ardu_uart, (DISTANCE_POSLEFT >> 8) & 0xFF);
		cyhal_uart_putc(&ardu_uart, DUMMY_CMD);
		cyhal_uart_putc(&ardu_uart, POSTOP_CMD);
		cyhal_uart_putc(&ardu_uart, (DISTANCE_POSTOP+10) & 0xFF);
		cyhal_uart_putc(&ardu_uart, ((DISTANCE_POSTOP+10) >> 8) & 0xFF);
		cyhal_uart_putc(&ardu_uart, DUMMY_CMD);

		/*Clean old data*/
		for(pos = 0; pos < sizeof(temp_arr); pos++)
		{
			cyhal_uart_putc(&ardu_uart, pos);
			cyhal_uart_putc(&ardu_uart, 0);
			cyhal_uart_putc(&ardu_uart, 0x20);
			cyhal_uart_putc(&ardu_uart, 0x00);
		}

		memset(temp_arr, 0x00, sizeof(temp_arr));

		/*Convert measured fft maximum peak to string*/
		sprintf(temp_arr, "FFT PEAK: %.2f", max_peak);

		/*Draw the measured fft peak*/
		for(pos = 0; pos < strlen(temp_arr); pos++)
		{
			cyhal_uart_putc(&ardu_uart, pos);
			cyhal_uart_putc(&ardu_uart, 0);
			cyhal_uart_putc(&ardu_uart, temp_arr[pos]);
			cyhal_uart_putc(&ardu_uart, 0x00);
		}

		//////////////////////////////////////////////

		/*Set the bin string position*/
		cyhal_uart_putc(&ardu_uart, POSLEFT_CMD);
		cyhal_uart_putc(&ardu_uart, DISTANCE_POSLEFT & 0xFF);
		cyhal_uart_putc(&ardu_uart, (DISTANCE_POSLEFT >> 8) & 0xFF);
		cyhal_uart_putc(&ardu_uart, DUMMY_CMD);
		cyhal_uart_putc(&ardu_uart, POSTOP_CMD);
		cyhal_uart_putc(&ardu_uart, (DISTANCE_POSTOP+20) & 0xFF);
		cyhal_uart_putc(&ardu_uart, ((DISTANCE_POSTOP+20) >> 8) & 0xFF);
		cyhal_uart_putc(&ardu_uart, DUMMY_CMD);

		/*Clean old data*/
		for(pos = 0; pos < sizeof(temp_arr); pos++)
		{
			cyhal_uart_putc(&ardu_uart, pos);
			cyhal_uart_putc(&ardu_uart, 0);
			cyhal_uart_putc(&ardu_uart, 0x20);
			cyhal_uart_putc(&ardu_uart, 0x00);
		}

		memset(temp_arr, 0x00, sizeof(temp_arr));

		/*Convert measured bin range to string*/
		sprintf(temp_arr, "RANGE BIN: %d", (int)max_peak_arg);

		/*Draw the measured range bin*/
		for(pos = 0; pos < strlen(temp_arr); pos++)
		{
			cyhal_uart_putc(&ardu_uart, pos);
			cyhal_uart_putc(&ardu_uart, 0);
			cyhal_uart_putc(&ardu_uart, temp_arr[pos]);
			cyhal_uart_putc(&ardu_uart, 0x00);
		}
	}
}

static void DrawPresence(_Bool presence)
{
	int x=0, y=0;
	uint32_t position = 0;
	char temp_arr[30] = {0};

	/*Set the indicator position*/
	cyhal_uart_putc(&ardu_uart, POSLEFT_CMD);
	cyhal_uart_putc(&ardu_uart, NJR_IND_POSLEFT & 0xFF);
	cyhal_uart_putc(&ardu_uart, (NJR_IND_POSLEFT >> 8) & 0xFF);
	cyhal_uart_putc(&ardu_uart, DUMMY_CMD);
	cyhal_uart_putc(&ardu_uart, POSTOP_CMD);
	cyhal_uart_putc(&ardu_uart, NJR_IND_POSTOP & 0xFF);
	cyhal_uart_putc(&ardu_uart, (NJR_IND_POSTOP >> 8) & 0xFF);
	cyhal_uart_putc(&ardu_uart, DUMMY_CMD);
	if(presence)
	{
		position = 0;
		for(y = 0; y < 7; y++)
		{
			for(x = 0; x < 12; x++)
			{
		    	cyhal_uart_putc(&ardu_uart, x);
		    	cyhal_uart_putc(&ardu_uart, y);
		    	cyhal_uart_putc(&ardu_uart, 0x20);
		    	cyhal_uart_putc(&ardu_uart, presence_ind[position]);
		    	position++;
			}
		}

		/*Set the presence string position*/
		cyhal_uart_putc(&ardu_uart, POSLEFT_CMD);
		cyhal_uart_putc(&ardu_uart, NJR_IND_POSLEFT & 0xFF);
		cyhal_uart_putc(&ardu_uart, (NJR_IND_POSLEFT >> 8) & 0xFF);
		cyhal_uart_putc(&ardu_uart, DUMMY_CMD);
		cyhal_uart_putc(&ardu_uart, POSTOP_CMD);
		cyhal_uart_putc(&ardu_uart, (NJR_IND_POSTOP+70) & 0xFF);
		cyhal_uart_putc(&ardu_uart, ((NJR_IND_POSTOP+70) >> 8) & 0xFF);
		cyhal_uart_putc(&ardu_uart, DUMMY_CMD);

		/*Convert detected state to string*/
		memset(temp_arr, 0x00, sizeof(temp_arr));
		sprintf(temp_arr, "State: PRESENCE");

		/*Clean old data*/
		for(position = 0; position < sizeof(temp_arr); position++)
		{
			cyhal_uart_putc(&ardu_uart, position);
			cyhal_uart_putc(&ardu_uart, 0);
			cyhal_uart_putc(&ardu_uart, 0x20);
			cyhal_uart_putc(&ardu_uart, 0x00);
		}

		/*Draw the detected state*/
		for(position = 0; position < strlen(temp_arr); position++)
		{
			cyhal_uart_putc(&ardu_uart, position);
			cyhal_uart_putc(&ardu_uart, 0);
			cyhal_uart_putc(&ardu_uart, temp_arr[position]);
			cyhal_uart_putc(&ardu_uart, 0x00);
		}
	}
	else
	{
		position = 0;
		for(y = 0; y < 7; y++)
		{
			for(x = 0; x < 12; x++)
			{
		    	cyhal_uart_putc(&ardu_uart, x);
		    	cyhal_uart_putc(&ardu_uart, y);
		    	cyhal_uart_putc(&ardu_uart, 0x20);
		    	cyhal_uart_putc(&ardu_uart, absence_ind[position]);
		    	position++;
			}
		}

		/*Set the presence string position*/
		cyhal_uart_putc(&ardu_uart, POSLEFT_CMD);
		cyhal_uart_putc(&ardu_uart, NJR_IND_POSLEFT & 0xFF);
		cyhal_uart_putc(&ardu_uart, (NJR_IND_POSLEFT >> 8) & 0xFF);
		cyhal_uart_putc(&ardu_uart, DUMMY_CMD);
		cyhal_uart_putc(&ardu_uart, POSTOP_CMD);
		cyhal_uart_putc(&ardu_uart, (NJR_IND_POSTOP+70) & 0xFF);
		cyhal_uart_putc(&ardu_uart, ((NJR_IND_POSTOP+70) >> 8) & 0xFF);
		cyhal_uart_putc(&ardu_uart, DUMMY_CMD);

		/*Convert detected state to string*/
		memset(temp_arr, 0x00, sizeof(temp_arr));
		sprintf(temp_arr, "State: ABSENCE");

		/*Clean old data*/
		for(position = 0; position < sizeof(temp_arr); position++)
		{
			cyhal_uart_putc(&ardu_uart, position);
			cyhal_uart_putc(&ardu_uart, 0);
			cyhal_uart_putc(&ardu_uart, 0x20);
			cyhal_uart_putc(&ardu_uart, 0x00);
		}

		/*Draw the detected state*/
		for(position = 0; position < strlen(temp_arr); position++)
		{
			cyhal_uart_putc(&ardu_uart, position);
			cyhal_uart_putc(&ardu_uart, 0);
			cyhal_uart_putc(&ardu_uart, temp_arr[position]);
			cyhal_uart_putc(&ardu_uart, 0x00);
		}
	}
}

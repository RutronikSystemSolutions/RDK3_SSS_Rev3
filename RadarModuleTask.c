/*
 * RadarModuleTask.c
 *
 *  Created on: 2024-04-09
 *      Author: Gintaras
 */


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

/*Priority for NJR4652F2S1 interrupts*/
#define NJR_IRQ_PRIORITY		2
#define NJR_STATE_POR_MS		4000

void absence_irq_handler(void *handler_arg, cyhal_gpio_event_t event);
void presence_irq_handler(void *handler_arg, cyhal_gpio_event_t event);

TaskHandle_t RadarModuleTaskHandle = NULL;

cyhal_gpio_callback_data_t gpio0_data =
{
		.callback = absence_irq_handler,
		.callback_arg = NULL,

};

cyhal_gpio_callback_data_t gpio1_data =
{
		.callback = presence_irq_handler,
		.callback_arg = NULL,

};

_Bool absence;
_Bool presence;

void RadarModuleTask(void *param)
{
	(void) param;
	cy_rslt_t result;
	uint32_t ulPreviousValue;

    /*Initialize Module Power Control Pin*/
    result = cyhal_gpio_init( ARDU_IO7, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, true);
    if (result != CY_RSLT_SUCCESS)
    {
    	CY_ASSERT(0);
    }

    /*Initialize NJR4652F2S2 RESET pin*/
    result = cyhal_gpio_init(ARDU_IO5, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_OPENDRAINDRIVESLOW, true);
    if (result != CY_RSLT_SUCCESS)
    {
    	CY_ASSERT(0);
    }
    CyDelay(100);
    cyhal_gpio_write(ARDU_IO5, false);

	/* Wait for the module to startup */
	vTaskDelay(pdMS_TO_TICKS(NJR_STATE_POR_MS));

    /*Initialize Radar Module Interrupts*/
    result = cyhal_gpio_init(ARDU_IO1, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, false);
    if (result != CY_RSLT_SUCCESS)
    {
    	CY_ASSERT(0);
    }
    result = cyhal_gpio_init(ARDU_IO2, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, false);
    if (result != CY_RSLT_SUCCESS)
    {
    	CY_ASSERT(0);
    }

    /*Register callback functions */
    cyhal_gpio_register_callback(ARDU_IO1, &gpio0_data);
    cyhal_gpio_register_callback(ARDU_IO2, &gpio1_data);

	absence = cyhal_gpio_read(ARDU_IO1);
	presence = cyhal_gpio_read(ARDU_IO2);
    if(absence)
    {
		while( xTaskNotifyAndQuery(SmartDisplayTaskHandle,sigABSENCE,eSetValueWithoutOverwrite, &ulPreviousValue ) != pdPASS)
		{
			vTaskDelay(pdMS_TO_TICKS(5));
		}
    }
    else if(presence)
    {
		while( xTaskNotifyAndQuery(SmartDisplayTaskHandle,sigPRESENCE,eSetValueWithoutOverwrite, &ulPreviousValue ) != pdPASS)
		{
			vTaskDelay(pdMS_TO_TICKS(5));
		}
    }

    /* Enable rising edge interrupt events */
    cyhal_gpio_enable_event(ARDU_IO1, CYHAL_GPIO_IRQ_RISE, NJR_IRQ_PRIORITY, true);
    cyhal_gpio_enable_event(ARDU_IO2, CYHAL_GPIO_IRQ_RISE, NJR_IRQ_PRIORITY, true);

	for(;;)
	{
	    /* Wait for the GPIO interrupts to indicate that state has changed */
	    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

	    /* We can check the pins now */
	    absence = cyhal_gpio_read(ARDU_IO1);
	    presence = cyhal_gpio_read(ARDU_IO2);

	    if(absence)
	    {
			while( xTaskNotifyAndQuery(SmartDisplayTaskHandle,sigABSENCE,eSetValueWithoutOverwrite, &ulPreviousValue ) != pdPASS)
			{
				vTaskDelay(pdMS_TO_TICKS(5));
			}
	    }
	    else if(presence)
	    {
			while( xTaskNotifyAndQuery(SmartDisplayTaskHandle,sigPRESENCE,eSetValueWithoutOverwrite, &ulPreviousValue ) != pdPASS)
			{
				vTaskDelay(pdMS_TO_TICKS(5));
			}
	    }
	}
}

/* Interrupt handler callback function */
void absence_irq_handler(void *handler_arg, cyhal_gpio_event_t event)
{
	CY_UNUSED_PARAMETER(handler_arg);
    CY_UNUSED_PARAMETER(event);

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    vTaskNotifyGiveFromISR(RadarModuleTaskHandle,&xHigherPriorityTaskWoken);

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

/* Interrupt handler callback function */
void presence_irq_handler(void *handler_arg, cyhal_gpio_event_t event)
{
	CY_UNUSED_PARAMETER(handler_arg);
    CY_UNUSED_PARAMETER(event);

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    vTaskNotifyGiveFromISR(RadarModuleTaskHandle,&xHigherPriorityTaskWoken);

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

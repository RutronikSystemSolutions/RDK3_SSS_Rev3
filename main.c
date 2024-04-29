/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the RDK3 Smart Sensor Station Rev3
*              Application for ModusToolbox.
*
* Related Document: See README.md
*
*
*  Created on: 2024-03-25
*  Company: Rutronik Elektronische Bauelemente GmbH
*  Address: Jonavos g. 30, Kaunas 44262, Lithuania
*  Author: GDR
*
*******************************************************************************
* (c) 2019-2021, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*
* Rutronik Elektronische Bauelemente GmbH Disclaimer: The evaluation board
* including the software is for testing purposes only and,
* because it has limited functions and limited resilience, is not suitable
* for permanent use under real conditions. If the evaluation board is
* nevertheless used under real conditions, this is done at one’s responsibility;
* any liability of Rutronik is insofar excluded
*******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "psram.h"
#include "dio59020.h"
#include "cy_retarget_io.h"
#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOSHeap.h"
#include "BLETask.h"
#include "ThermalSensorTask.h"
#include "RadarSensorTask.h"
#include "GestureSensorTask.h"
#include "SmartDisplayTask.h"
#include "RadarModuleTask.h"

/*I2C SCB3 Device Global Variables*/
SemaphoreHandle_t i2c_mutex = NULL;
cyhal_i2c_t I2C_scb3;
cyhal_i2c_cfg_t i2c_scb3_cfg =
{
		.is_slave = false,
	    .address = 0,
	    .frequencyhal_hz = 1000000UL,
};

cy_rslt_t hardware_init (void);

int main(void)
{
    cy_rslt_t result;

    result = hardware_init();
    if (result != CY_RSLT_SUCCESS)
    {
    	CY_ASSERT(0);
    }

    ble_cmdQ = xQueueCreate(BLE_CMD_Q_LEN, sizeof(uint8_t));
    if(ble_cmdQ == NULL)
    {
    	printf("Error: could not create BLE command queue.\r\n");
    	CY_ASSERT(0);
    }

    /* Create a mutex for the I2C. */
    i2c_mutex = xSemaphoreCreateMutex();
    if( i2c_mutex == NULL )
    {
    	printf("Error: could not create I2C SCB3 mutex.\r\n");
    	CY_ASSERT(0);
    }

    /* Start BLE Task */
    xTaskCreate(BLETask, "ble task", configMINIMAL_STACK_SIZE*4, NULL, configMAX_PRIORITIES - 1, &ble_task_handle);
    if(ble_task_handle == NULL)
    {
    	printf("Error: could not create BLE task.\r\n");
    	CY_ASSERT(0);
    }

    /* Start Thermal Sensor Task */
    xTaskCreate(ThermalSensorTask, "thermal task", configMINIMAL_STACK_SIZE*2, NULL, configMAX_PRIORITIES - 4, &ThermalSensorTaskHandle);
    if(ThermalSensorTaskHandle == NULL)
    {
    	printf("Error: could not create thermal task.\r\n");
    	CY_ASSERT(0);
    }

    /* Start Radar Sensor BGT60TR13 Task */
    xTaskCreate(RadarSensorTask, "radar task", configMINIMAL_STACK_SIZE*2, NULL, configMAX_PRIORITIES - 5, &RadarSensorTaskHandle);
    if(RadarSensorTaskHandle == NULL)
    {
    	printf("Error: could not create radar task.\r\n");
    	CY_ASSERT(0);
    }

    /* Start Radar Module NJR4652F2S1 Task */
    xTaskCreate(RadarModuleTask, "presence task", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES - 3, &RadarModuleTaskHandle);
    if(RadarModuleTaskHandle == NULL)
    {
    	printf("Error: could not create presence task.\r\n");
    	CY_ASSERT(0);
    }

    /* Start Gesture Sensor VCNL4035X01 Task */
    xTaskCreate(GestureSensorTask, "gesture task", configMINIMAL_STACK_SIZE*2, NULL, configMAX_PRIORITIES - 2, &GestureSensorTaskHandle);
    if(GestureSensorTaskHandle == NULL)
    {
    	printf("Error: could not create gesture task.\r\n");
    	CY_ASSERT(0);
    }

    /* Start UART Smart Display Task */
    xTaskCreate(SmartDisplayTask, "display task", configMINIMAL_STACK_SIZE*2, NULL, configMAX_PRIORITIES - 2, &SmartDisplayTaskHandle);
    if(SmartDisplayTaskHandle == NULL)
    {
    	printf("Error: could not create display task.\r\n");
    	CY_ASSERT(0);
    }

    vTaskStartScheduler();
    /* RTOS scheduler exited */
    /* Halt the CPU if scheduler exits */
    CY_ASSERT(0);
}

cy_rslt_t hardware_init (void)
{
	cy_rslt_t result;
	charge_stat_t status = CHRG_READY;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
    	return result;
    }

    /*Enable QSPI PSRAM in XIP Mode*/
    psram_init();

    __enable_irq();

    /*Initialize LEDs*/
    result = cyhal_gpio_init( LED1, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    if (result != CY_RSLT_SUCCESS)
    {
    	return result;
    }
    result = cyhal_gpio_init( LED2, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    if (result != CY_RSLT_SUCCESS)
    {
    	return result;
    }
    result = cyhal_gpio_init( LED3, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    if (result != CY_RSLT_SUCCESS)
    {
    	return result;
    }

    /*Initialize Buttons*/
    result = cyhal_gpio_init(USER_BTN, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, false);
    if (result != CY_RSLT_SUCCESS)
    {
    	return result;
    }

    /*Initialize Display RESET pin*/
    result = cyhal_gpio_init(ARDU_IO8, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, false);
    if (result != CY_RSLT_SUCCESS)
    {
    	return result;
    }

    /* Heap Type 5 Initialization */
    FreeRTOSHeapInit();

    /*Enable debug output via KitProg UART*/
    result = cy_retarget_io_init( KITPROG_TX, KITPROG_RX, CY_RETARGET_IO_BAUDRATE);
    if (result != CY_RSLT_SUCCESS)
    {
    	return result;
    }

    /*Initialize ARDUINO I2C Master*/
    result = cyhal_i2c_init(&I2C_scb3, ARDU_SDA, ARDU_SCL, NULL);
    if (result != CY_RSLT_SUCCESS)
    {
    	return result;
    }
    result = cyhal_i2c_configure(&I2C_scb3, &i2c_scb3_cfg);
    if (result != CY_RSLT_SUCCESS)
    {
    	return result;
    }

    /* DIO59020 Charger Setup*/
    if(dio_online())
    {
    	/*Disable the charger*/
		dio_charger_disable();

		/*Set the input current limit*/
		dio_set_current_lim(LIMIT_INF);

		/*Set the battery charge current*/
		dio_set_batt_current(CURR_VREF_101_8);

		/*Set the battery charge termination current*/
		dio_set_batt_term_curr(CURR_VREF_12_5);

		/*Configure OTG pin*/
		dio_otg_pin_config(true, false);

		/*Enable the charger*/
		dio_charger_enable();

		CyDelay(500);

		/*Check if battery is connected*/
		dio_get_status(&status);
		if(status == CHRG_FAULT)
		{
	    	/*Disable the charger*/
			dio_charger_disable();
		}
		else //check if we need to engage the booster
		{
	    	if(!cyhal_gpio_read(USER_BTN))
	    	{
	    		CyDelay(500);
	    		/*Check it once more*/
	    		if(!cyhal_gpio_read(USER_BTN))
	    		{
	    			/*The button is pressed*/
	    			/*Unload the DIO59020 and turn on the +5V booster*/
	    			cyhal_gpio_write(ARDU_IO8, true);
	    			CyDelay(10);
	    			dio_booster_enable();

	    			/*Load the DIO59020 now*/
	    			CyDelay(100);
	    			cyhal_gpio_write(ARDU_IO8, false);
	    		}
	    	}
		}
    }


    printf("\x1b[2J\x1b[;H");
    printf("hardware initialization PASS.\r\n");
	return result;
}

/* [] END OF FILE */

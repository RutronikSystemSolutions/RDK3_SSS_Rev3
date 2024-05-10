/*
 * BLETask.c
 *
 *  Created on: 2024-03-28
 *      Author: Gintaras
 */

#include <stdio.h>
#include <stdlib.h>
#include "BLETask.h"
#include "cycfg_ble.h"
#include "cybsp.h"
#include "cyhal.h"
#include "cy_pdl.h"

#define ENABLE      (1u)
#define DISABLE     (0u)

#define DEBUG_BLE_ENABLE    	ENABLE
#define DEBUG_BLE_DATA_ENABLE	ENABLE

#if (DEBUG_BLE_ENABLE || DEBUG_BLE_LOGIC_ENABLE)
#include <stdio.h>
#endif

#if DEBUG_BLE_ENABLE
#define DEBUG_BLE       printf
#else
#define DEBUG_BLE(...)
#endif

#if DEBUG_BLE_DATA_ENABLE
#define DEBUG_BLE_LOGIC       printf
#else
#define DEBUG_BLE_LOGIC(...)
#endif

#define CUSTOM_DESCR_HANDLE						cy_ble_customsConfig.attrInfo[0].customServInfo[0].customServCharDesc[0]
#define CUSTOM_CHAR_HANDLE						cy_ble_customsConfig.attrInfo[0].customServInfo[0].customServCharHandle
#define TARGET_BDADDR       					{{0xFF, 0xBB, 0xAA, 0x50, 0xA0, 0x00}, 0}
#define BLE_INTERRUPT_PRIORITY                  (3u)
#define BLE_PROCESS_EVENTS                      (3u)
#define CONN_INTERVAL_MULTIPLIER                (1.25f)
#define MIN_CONN_INTERVAL                       (54)
#define MAX_CONN_INTERVAL                       (60)
#define SUPERVISION_TIMEOUT                     (400)

static void ble_init(void);
static void bless_interrupt_handler(void);
static void ble_controller_interrupt_handler(void);
static void ble_stack_event_handler(uint32_t event, void *eventParam);
static void update_gatt_db_notif(cy_stc_ble_gatt_write_param_t *write_param);
static void update_gatt_db_write(cy_stc_ble_gatt_write_param_t *write_param);

TaskHandle_t ble_task_handle = NULL;
QueueHandle_t ble_cmdQ = NULL;

/* Variables to hold GATT notification byte count or GATT write byte count */
static uint32_t gatt_write_rx_bytes;
static uint32_t notif_tx_bytes;

/* Variable to store latest connection interval for the BLE connection */
static float conn_interval;

/* Flags */
/* To indicate if GATT Client has subscribed for notifications or not */
static bool notify_flag;
/* To indicate if BLE stack is busy or free */
static bool stack_free = true;
/* To indicate display task that the device has disconnected */
bool device_disconnect_flag = false;

cy_stc_ble_gap_bd_addr_t local_addr = TARGET_BDADDR;

/* Variable to store connection parameters after GAP connection */
static cy_stc_ble_gap_connected_param_t conn_param;

/* Connection handle to identify the connected peer device */
static cy_stc_ble_conn_handle_t conn_handle;

/* Pointer to store a single attribute information present in the GATT DB of the server */
static cy_stc_ble_gatt_write_param_t *write_req_param;

/* Variable to store information about custom data to be sent as notification to GATT Client */
static cy_stc_ble_gatt_handle_value_pair_t custom_data;

/* Variable to store MTU size for active BLE connection */
static uint16_t att_mtu_size = CY_BLE_GATT_MTU;

static uint8_t test_byte = 0;

void BLETask(void *pvParameters)
{
    /* Remove compiler warning for unused variable */
    (void)pvParameters;

    /* Variable to store return value from FreeRTOS APIs */
    BaseType_t rtos_api_result;

    /* Variable to keep track of the BLE API result */
    static cy_en_ble_api_result_t ble_api_result;

    /* Variable to store BLE command received from BLE queue */
    uint8_t bleCmd = BLE_PROCESS_EVENTS;

    /* Initialize BLE and process any stack events */
    ble_init();

    /* Repeatedly running part of the task */
    for(;;)
    {
        /* Block until a BLE command has been received over ble_cmdQ */
        rtos_api_result = xQueueReceive(ble_cmdQ, &bleCmd, portMAX_DELAY);

        /* Command has been received from ble_cmdQ */
        if(rtos_api_result == pdTRUE)
        {
            /* Process the ble stack events */
            if(bleCmd == BLE_PROCESS_EVENTS)
            {
                Cy_BLE_ProcessEvents();
            }
        }

        /* notify_flag is used to indicate if notifications are enabled by GATT client */
        if(notify_flag)
        {
            if(stack_free)
            {
                /* Send notification */
                ble_api_result = Cy_BLE_GATTS_SendNotification(&conn_handle, &custom_data);

                if(ble_api_result == CY_BLE_SUCCESS)
                {
                	test_byte++;
                }
            }
        }
    }
}

/*******************************************************************************
* Function Name: static void ble_init(void)
********************************************************************************
* Summary:
*  This function initializes the BLE Host and Controller, configures BLE
*  interrupt, and registers Application Host callbacks.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void ble_init(void)
{
    cy_en_ble_api_result_t ble_api_result = CY_BLE_SUCCESS;

    /* BLESS interrupt configuration structure */
    const cy_stc_sysint_t bless_isr_config =
    {
       /* The BLESS interrupt */
      .intrSrc = bless_interrupt_IRQn,

       /* The interrupt priority number */
      .intrPriority = BLE_INTERRUPT_PRIORITY
    };

    /* Store the pointer to blessIsrCfg in the BLE configuration structure */
    cy_ble_config.hw->blessIsrConfig = &bless_isr_config;

    /* Hook interrupt service routines for BLESS */
    (void) Cy_SysInt_Init(&bless_isr_config, bless_interrupt_handler);

    /* Register the generic callback functions */
    Cy_BLE_RegisterEventCallback(ble_stack_event_handler);

    /* Register the application Host callback */
    Cy_BLE_RegisterAppHostCallback(ble_controller_interrupt_handler);

    /* Initialize the BLE */
    ble_api_result = Cy_BLE_Init(&cy_ble_config);
    if(ble_api_result != CY_BLE_SUCCESS)
    {
        /* BLE stack initialization failed, check configuration, notify error
         * and halt CPU in debug mode
         */
    	DEBUG_BLE("Cy_BLE_Init API, errorcode: 0x%X\r\n", ble_api_result);
        vTaskSuspend(NULL);
    }

    /* Enable BLE */
    ble_api_result = Cy_BLE_Enable();
    if(ble_api_result != CY_BLE_SUCCESS)
    {
        /* BLE stack initialization failed, check configuration, notify error
         * and halt CPU in debug mode
         */
    	DEBUG_BLE("Cy_BLE_Enable API errorcode: 0x%X\r\n", ble_api_result);
        vTaskSuspend(NULL);
    }
    /* Process BLE events after enabling BLE */
    Cy_BLE_ProcessEvents();
}

/*******************************************************************************
* Function Name: void ble_stack_event_handler (uint32_t event, void *eventParam)
********************************************************************************
* Summary: Call back event function to handle various events from the BLE stack.
*
* Parameters:
*  uint32_t event        :    Event from BLE stack
*  void eventParam       :    Pointer to the value of event specific parameters
*
* Return:
*  None
*
*******************************************************************************/
static void ble_stack_event_handler (uint32_t event, void *eventParam)
{
    /* Variable used to store the return values of BLE APIs */
    cy_en_ble_api_result_t ble_api_result;

    /* Take an action based on the current event */
    switch(event)
    {
        /***********************************************************************
        *                       General Events                                 *
        ***********************************************************************/
        /* This event is received when the BLE stack is Started */
        case CY_BLE_EVT_STACK_ON:
        {
        	DEBUG_BLE("BLE Stack Event : CY_BLE_EVT_STACK_ON\r\n");

            /* Set the address */
            ble_api_result = Cy_BLE_GAP_SetBdAddress((cy_stc_ble_gap_bd_addr_t  *)&local_addr);
            if(ble_api_result != CY_BLE_SUCCESS)
            {
            	DEBUG_BLE("BLE Address Set Failure.\r\n");
            }

            /* Start Advertisement and enter discoverable mode */
            Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST, CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);
            break;
        }

        /* This event indicates BLE stack status. This event is used to handle
         * data throttling which may occur due to continuous notification being
         * sent */
        case CY_BLE_EVT_STACK_BUSY_STATUS:
        {
            /* Variable to store status of the stack */
            cy_stc_ble_l2cap_state_info_t stack_status;
            stack_status = *( cy_stc_ble_l2cap_state_info_t*)eventParam;

            if(stack_status.flowState == CY_BLE_STACK_STATE_BUSY)
            {
                /* If stack is busy, stop notifications */
                stack_free = false;
            }
            else
            {
                /* If stack is free, start notifications */
                stack_free = true;
            }

            DEBUG_BLE("CY_BLE_EVT_STACK_BUSY_STATUS: %x\r\n", *(uint8 *)eventParam);
            break;
        }

        /* This event is received when there is a timeout */
        case CY_BLE_EVT_TIMEOUT:
        {
        	DEBUG_BLE("BLE Stack Event - Event timeout.\r\n");
            break;
        }

        /* This event indicates completion of the Set LE event mask. */
        case  CY_BLE_EVT_LE_SET_EVENT_MASK_COMPLETE:
        {
        	DEBUG_BLE("BLE Stack Event : CY_BLE_EVT_LE_SET_EVENT_MASK_COMPLETE\r\n");
            break;
        }

        /* This event indicates completion of Set suggested data length command. */
        case  CY_BLE_EVT_SET_SUGGESTED_DATA_LENGTH_COMPLETE:
        {
        	DEBUG_BLE("BLE Stack Event : CY_BLE_EVT_SET_SUGGESTED_DATA_LENGTH_COMPLETE\r\n");
            break;
        }

        /* This event indicates that the set device address command has completed. */
        case CY_BLE_EVT_SET_DEVICE_ADDR_COMPLETE:
        {
            cy_stc_ble_events_param_generic_t *param = (cy_stc_ble_events_param_generic_t *) eventParam;

            DEBUG_BLE("CY_BLE_EVT_SET_DEVICE_ADDR_COMPLETE \r\n");

            if( param->status != CY_BLE_SUCCESS)
            {
                DEBUG_BLE("Failed to Set Local BDAddress [Status 0x%02X]\r\n",\
                    param->status);

            }
            else
            {
                DEBUG_BLE("Local Address Set successfully \r\n");
                DEBUG_BLE("BdAddress set to: %02X:%02X:%02X:%02X:%02X:%02X \r\n",\
                    local_addr.bdAddr[5],local_addr.bdAddr[4], local_addr.bdAddr[3],\
                    local_addr.bdAddr[2], local_addr.bdAddr[1], local_addr.bdAddr[0]);

                Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST,\
				CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);
            }

            break;
        }

        /* This event indicates get device address command completed successfully. */
        case CY_BLE_EVT_GET_DEVICE_ADDR_COMPLETE:
        {
			DEBUG_BLE("CY_BLE_EVT_GET_DEVICE_ADDR_COMPLETE: ");
            DEBUG_BLE("\n\rAdvertising with Address: ");
            for(uint8_t i = CY_BLE_GAP_BD_ADDR_SIZE; i > 0u; i--)
            {
                DEBUG_BLE("%2.2X ", ((cy_stc_ble_bd_addrs_t *)((cy_stc_ble_events_param_generic_t *)eventParam)->eventParams)->publicBdAddr[i-1]);
            }
            DEBUG_BLE("\r\n");
            break;
		}

        /* This event indicates that the set Tx Power command has completed. */
        case   CY_BLE_EVT_SET_TX_PWR_COMPLETE:
        {
        	DEBUG_BLE("BLE Stack Event : CY_BLE_EVT_SET_TX_PWR_COMPLETE\r\n");
            break;
        }

        /* This event indicates completion of the Cy_BLE_SetPhy() function. */
        case   CY_BLE_EVT_SET_PHY_COMPLETE:
        {
        	DEBUG_BLE("BLE Stack Event : CY_BLE_EVT_SET_PHY_COMPLETE\r\n");
            break;
        }

        /**********************************************************************
        *                       GAP Events                                    *
        **********************************************************************/

        /* This event indicates that the peripheral device has started/stopped
         * advertising */
        case CY_BLE_EVT_GAPP_ADVERTISEMENT_START_STOP:
        {
            cy_en_ble_adv_state_t adv_state;
            DEBUG_BLE("BLE Stack Event : CY_BLE_EVT_GAPP_ADVERTISEMENT_START_STOP\r\n");
            adv_state = Cy_BLE_GetAdvertisementState();
            if(adv_state == CY_BLE_ADV_STATE_ADVERTISING)
            {
            	DEBUG_BLE("Advertising.....\r\n");
            }
            break;
        }

        /* This event is generated at the GAP Peripheral end after connection
         * is completed with peer Central device */
        case CY_BLE_EVT_GAP_DEVICE_CONNECTED:
        {
        	DEBUG_BLE("BLE Stack Event : CY_BLE_EVT_GAP_DEVICE_CONNECTED\r\n");
            conn_param = *((cy_stc_ble_gap_connected_param_t*)eventParam);

            /* Variable to store values to update PHY to 2M */
            cy_stc_ble_set_phy_info_t phy_param;
            phy_param.allPhyMask = CY_BLE_PHY_NO_PREF_MASK_NONE;
            phy_param.bdHandle = conn_handle.bdHandle;
            phy_param.rxPhyMask = CY_BLE_PHY_MASK_LE_2M;
            phy_param.txPhyMask = CY_BLE_PHY_MASK_LE_2M;

            /* Reset the connection status flag upon reconnection */
            device_disconnect_flag = false;

            /* Function call to set PHY to 2M */
            ble_api_result = Cy_BLE_SetPhy(&phy_param);
            if(ble_api_result == CY_BLE_SUCCESS)
            {
            	DEBUG_BLE("Set PHY to 2M successfull\r\n");
            	DEBUG_BLE("Request sent to switch PHY to 2M\r\n");
            }
            else
            {
            	DEBUG_BLE("Set PHY to 2M API failure, errorcode = 0x%X\r\n", ble_api_result);
            }
            conn_interval = (conn_param.connIntv) * CONN_INTERVAL_MULTIPLIER;
            DEBUG_BLE("Connection Interval is: %f ms\r\n", conn_interval);
            break;
        }

        /* This event is generated when the device is disconnected from remote
         * device or fails to establish connection */
        case CY_BLE_EVT_GAP_DEVICE_DISCONNECTED:
        {
        	DEBUG_BLE("BLE Stack Event : CY_BLE_EVT_GAP_DEVICE_DISCONNECTED\r\n");
        	DEBUG_BLE("Disconnected from peer!!!\r\n");

            /* Set the device disconnected flag */
            device_disconnect_flag = true;

            /* Reset the notify flag and stack_free flag after disconnection */
            notify_flag = false;
            stack_free = true;
            /* Clear the byte count */
            gatt_write_rx_bytes = 0u;
            notif_tx_bytes = 0u;
            /* Start undirected advertisement again */
            ble_api_result = Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST, CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);
            if(ble_api_result == CY_BLE_SUCCESS)
            {
            	DEBUG_BLE("BLE Advertisement API successfull\r\n");
            }
            else
            {
            	DEBUG_BLE("BLE Advertisement API failure, errorcode = 0x%X\r\n", ble_api_result);
            }

            cyhal_gpio_write((cyhal_gpio_t)LED1, CYBSP_LED_STATE_OFF);

            break;
        }

        /* This event is generated when PHY is updated during an active connection */
        case CY_BLE_EVT_PHY_UPDATE_COMPLETE:
        {
        	DEBUG_BLE("BLE Stack Event : CY_BLE_EVT_PHY_UPDATE_COMPLETE\r\n");

            cy_stc_ble_events_param_generic_t *genericParam;
            cy_stc_ble_phy_param_t *currentPHY;
            genericParam = (cy_stc_ble_events_param_generic_t*)eventParam;

            /* GenericParam has to be cast to cy_stc_ble_phy_param_t to get TX
             * and RX PHY */
            currentPHY = (cy_stc_ble_phy_param_t*)(genericParam->eventParams);

            /* Print the RX PHY selected on UART terminal */
            switch(currentPHY->rxPhyMask)
            {
                case CY_BLE_PHY_MASK_LE_1M:
                DEBUG_BLE("Selected Rx PHY: 1M\r\n");
                break;

                case CY_BLE_PHY_MASK_LE_2M:
                DEBUG_BLE("Selected Rx PHY: 2M\r\n");
                break;

                case CY_BLE_PHY_MASK_LE_CODED:
                DEBUG_BLE("Selected Rx PHY: LE Coded\r\n");
                break;
            }

            /* Print the TX PHY selected on UART terminal */
            switch(currentPHY->txPhyMask)
            {
                case CY_BLE_PHY_MASK_LE_1M:
                DEBUG_BLE("Selected Tx PHY: 1M\r\n");
                break;

                case CY_BLE_PHY_MASK_LE_2M:
                DEBUG_BLE("Selected Tx PHY: 2M\r\n");
                break;

                case CY_BLE_PHY_MASK_LE_CODED:
                DEBUG_BLE("Selected Tx PHY: LE Coded\r\n");
                break;
            }

            /* Variable to hold data for new connection parameters request */
            cy_stc_ble_gap_conn_update_param_info_t conn_params = {MIN_CONN_INTERVAL,
                                                                   MAX_CONN_INTERVAL,
                                                                   0u,
                                                                   SUPERVISION_TIMEOUT,
                                                                   conn_handle.bdHandle,
                                                                   0u};
            /* Check if connection parameters decided by Client is in the desired range */
            if((conn_param.connIntv <= MIN_CONN_INTERVAL) || (conn_param.connIntv >= MAX_CONN_INTERVAL))
            {
                /* Request for new connection parameters */
                ble_api_result = Cy_BLE_L2CAP_LeConnectionParamUpdateRequest(&conn_params);
                if(ble_api_result == CY_BLE_SUCCESS)
                {
                	DEBUG_BLE("L2CAP Connection Parameter update request sent successfully\r\n");
                }
                else
                {
                	DEBUG_BLE("FAILURE : L2CAP Connection Parameter update request failed, errorcode = 0x%X\r\n", ble_api_result);
                }
            }
            break;
        }

        /* This event is generated when connection parameter update is
         * requested. If the request is accepted by the Central, this event is
         * generated on both the devices. If the request is rejected, this event
         * is not generated */
        case CY_BLE_EVT_GAP_CONNECTION_UPDATE_COMPLETE:
        {
        	DEBUG_BLE("BLE Stack Event : CY_BLE_EVT_GAP_CONNECTION_UPDATE_COMPLETE\r\n");
            cy_stc_ble_gap_conn_param_updated_in_controller_t *new_conn_params;
            new_conn_params = (cy_stc_ble_gap_conn_param_updated_in_controller_t*)eventParam;

            /* Store the new connection interval value */
            conn_interval = (new_conn_params->connIntv) * CONN_INTERVAL_MULTIPLIER;
            DEBUG_BLE("Updated Connection Interval: %f ms\r\n", conn_interval);
            break;
        }

        /* This event is triggered instead of 'CY_BLE_EVT_GAP_DEVICE_CONNECTED',
        if Link Layer Privacy is enabled in component customizer. */
        case CY_BLE_EVT_GAP_ENHANCE_CONN_COMPLETE:
        {
            /* BLE link is established */
            /* This event will be triggered since link layer privacy is enabled */
            DEBUG_BLE("CY_BLE_EVT_GAP_ENHANCE_CONN_COMPLETE \r\n");

#if (DEBUG_BLE_ENABLE || DEBUG_BLE_LOGIC_ENABLE)
            cy_stc_ble_gap_enhance_conn_complete_param_t *param = \
            (cy_stc_ble_gap_enhance_conn_complete_param_t *)eventParam;

            DEBUG_BLE_LOGIC("Connected to Device ");

			DEBUG_BLE_LOGIC("%02X:%02X:%02X:%02X:%02X:%02X\r\n\n",param->peerBdAddr[5],\
					param->peerBdAddr[4], param->peerBdAddr[3], param->peerBdAddr[2],\
					param->peerBdAddr[1], param->peerBdAddr[0]);

			DEBUG_BLE("\r\nBDhandle : 0x%02X\r\n", param->bdHandle);
#endif
            cyhal_gpio_write((cyhal_gpio_t)LED1, CYBSP_LED_STATE_ON);
            break;
        }

        /* This event is triggered when there is a change to either the maximum Payload
        length or the maximum transmission time of Data Channel PDUs in either direction */
        case CY_BLE_EVT_DATA_LENGTH_CHANGE:
        {
            DEBUG_BLE("CY_BLE_EVT_DATA_LENGTH_CHANGE \r\n");
            cy_stc_ble_set_phy_info_t phyParam;

            /* Configure the BLE Component for 2Mbps data rate */
            phyParam.bdHandle = conn_handle.bdHandle;
            phyParam.allPhyMask = CY_BLE_PHY_NO_PREF_MASK_NONE;
            phyParam.phyOption = 0;
            phyParam.rxPhyMask = CY_BLE_PHY_MASK_LE_2M;
            phyParam.txPhyMask = CY_BLE_PHY_MASK_LE_2M;

            Cy_BLE_EnablePhyUpdateFeature();
            ble_api_result = Cy_BLE_SetPhy(&phyParam);
            if(ble_api_result != CY_BLE_SUCCESS)
            {
                DEBUG_BLE("Failed to set PHY..[bdHandle 0x%02X] : 0x%4x\r\n", phyParam.bdHandle, ble_api_result);
            }
            else
            {
                DEBUG_BLE("Setting PHY.[bdHandle 0x%02X] \r\n", phyParam.bdHandle);
            }

            break;
        }

        /***********************************************************************
        *                          GATT Events                                 *
        ***********************************************************************/
        /* This event is generated at the GAP Peripheral end after connection
         * is completed with peer Central device */
        case CY_BLE_EVT_GATT_CONNECT_IND:
        {
            conn_handle = *((cy_stc_ble_conn_handle_t*)eventParam);
            DEBUG_BLE("BLE Stack Event : CY_BLE_EVT_GATT_CONNECT_IND\r\n");
            DEBUG_BLE("GATT connected\r\n");
            break;
        }

        /* This event indicates that the GATT is disconnected.*/
        case CY_BLE_EVT_GATT_DISCONNECT_IND:
        {
        	DEBUG_BLE("BLE Stack Event : CY_BLE_EVT_GATT_DISCONNECT_IND\r\n");
            break;
        }

        /* This event is generated when a 'write request' is received from GATT
         * Client device */
        case CY_BLE_EVT_GATTS_WRITE_REQ:
        {
        	DEBUG_BLE("BLE Stack Event : CY_BLE_EVT_GATTS_WRITE_REQ\r\n");
            write_req_param = (cy_stc_ble_gatt_write_param_t*)eventParam;

            /* Clear both tx and rx data bytes count */
            gatt_write_rx_bytes = 0u;
            notif_tx_bytes = 0u;

            /* Send response to GATT Client device */
            if(Cy_BLE_GATTS_WriteRsp(write_req_param->connHandle) != CY_BLE_SUCCESS)
            {
                DEBUG_BLE("Failed to send write response \r\n");
                DEBUG_BLE_LOGIC("Error... \r\n");
            }
            else
            {
                /* Update the attribute with new values received from the client
                 * device i.e., In this case, enable or disable notification  */
                update_gatt_db_notif(write_req_param);
            }

            break;
        }

        /* This event is generated when a 'write command' is received from GATT
         * client device */
        case CY_BLE_EVT_GATTS_WRITE_CMD_REQ:
        {
        	DEBUG_BLE("CY_BLE_EVT_GATTS_WRITE_REQ \r\n");
            write_req_param = (cy_stc_ble_gatt_write_param_t*)eventParam;
            DEBUG_BLE("Received GATT Write Request [bdHandle %02X]\r\n",write_req_param->connHandle.bdHandle);

            /* Write the value received from GATT Client device into GATT Server
             * database */
            update_gatt_db_write(write_req_param);
            break;
        }

         /* This event is triggered when a 'read request' is received from GATT
          * client device */
        case CY_BLE_EVT_GATTS_READ_CHAR_VAL_ACCESS_REQ:
        {
        	DEBUG_BLE("BLE Stack Event : CY_BLE_EVT_GATTS_READ_CHAR_VAL_ACCESS_REQ\r\n");
            break;
        }

         /* This event is triggered when 'GATT MTU Exchange Request' is received
          * from GATT client device
          */
        case CY_BLE_EVT_GATTS_XCNHG_MTU_REQ:
        {
        	DEBUG_BLE("BLE Stack Event : CY_BLE_EVT_GATTS_XCNHG_MTU_REQ\r\n");
            cy_stc_ble_gatt_xchg_mtu_param_t mtuParam;
            mtuParam = *(cy_stc_ble_gatt_xchg_mtu_param_t*)eventParam;
            DEBUG_BLE("Client MTU Size: %d \r\n", mtuParam.mtu);
            if(mtuParam.mtu > CY_BLE_GATT_MTU)
            {
                att_mtu_size = CY_BLE_GATT_MTU;
            }
            else
            {
                att_mtu_size = mtuParam.mtu;
            }
            break;
        }

        /***********************************************************************
        *                          L2CAP Events                                 *
        ***********************************************************************/
        /* This event is generated when there is response from host device for
         * connection parameter update request */
        case CY_BLE_EVT_L2CAP_CONN_PARAM_UPDATE_RSP:
        {
            cy_stc_ble_l2cap_conn_update_rsp_param_t conn_param_rsp;
            conn_param_rsp = *((cy_stc_ble_l2cap_conn_update_rsp_param_t*)eventParam);

            /* Print the response on UART terminal*/
            if(conn_param_rsp.result == 0u)
            {
            	DEBUG_BLE("Connection Interval update request accepted\r\n");
            }
            else
            {
            	DEBUG_BLE("Connection Interval update request rejected\r\n");
            }
            break;
        }

        /***********************************************************************
        *                           Other Events                               *
        ***********************************************************************/
        default:
        {
        	DEBUG_BLE("Other BLE event: 0x%X\r\n", (unsigned int)event);
            break;
        }
    }
}

/*******************************************************************************
* Function Name: static void bless_interrupt_handler(void)
********************************************************************************
* Summary:
*  Wrapper function for BLESS interrupt
*
* Parameters:
*  None
*
* Return:
*  None
*******************************************************************************/
static void bless_interrupt_handler(void)
{
    /* Process interrupt events generated by the BLE sub-system */
    Cy_BLE_BlessIsrHandler();
}

/*******************************************************************************
* Function Name: static void ble_controller_interrupt_handler(void)
********************************************************************************
* Summary:
*  Call back event function to handle interrupts from BLE Controller
*
* Parameters:
*  None
*
* Return:
*  None
*******************************************************************************/
static void ble_controller_interrupt_handler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* Send command to process BLE events  */
    uint8_t bleCommand = BLE_PROCESS_EVENTS;
    xQueueSendFromISR(ble_cmdQ, &bleCommand, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**********************************************************************************
* Function Name: void update_gatt_db_notif(cy_stc_ble_gatt_write_param_t *write_param)
***********************************************************************************
* Summary: This function updates the CCCD attribute of GATT Notify
*          characteristic present in the custom service
*
* Parameters:
*  cy_stc_ble_gatt_write_param_t *write_param: parameter which holds information
*                                    about attribute handle and attribute value
*
* Return:
*  None
**********************************************************************************/
static void update_gatt_db_notif(cy_stc_ble_gatt_write_param_t *write_param)
{
    cy_en_ble_gatt_err_code_t gattErrorCode;

    DEBUG_BLE_LOGIC("Info: Attribute handle: 0x%X\r\n", write_param->handleValPair.attrHandle);
    DEBUG_BLE_LOGIC("Info: Attribute Value: 0x%X\r\n", write_param->handleValPair.value.val[0]);

    notif_tx_bytes = 0u;
    gatt_write_rx_bytes = 0u;

    /* Write into the identified attribute in GATT database */
    gattErrorCode = Cy_BLE_GATTS_WriteAttributeValuePeer(&conn_handle, &(write_param->handleValPair));
    if(gattErrorCode == CY_BLE_GATT_ERR_NONE)
    {
        /* If the attribute is CCCD for notification characteristic */
        if(write_param->handleValPair.attrHandle == CUSTOM_DESCR_HANDLE)
        {
            /* If notifications are enabled */
            if(write_param->handleValPair.value.val[0])
            {
                /* Custom data value for BLE notification. */
                DEBUG_BLE("Notifications Enabled\r\n");
                notify_flag = true;
                /* Set the custom notification data using the MTU size */
                /* Packet length = (ATT_MTU_SIZE - ATT_OPCODE(1 byte) - ATT_HEADER(2 bytes))
                 *               = (ATT_MTU_SIZE - 3)*/
                custom_data.value.len = sizeof(test_byte);
                custom_data.value.val = &test_byte;
                custom_data.attrHandle = CUSTOM_CHAR_HANDLE;
            }
            /* If notifications are disabled */
            else
            {
            	DEBUG_BLE("Notifications Disabled\r\n");
                notify_flag = false;
            }
        }
    }
    /* If the operation is not successful */
    else
    {
    	DEBUG_BLE("GATT Write API, errorcode = 0x%X\r\n", gattErrorCode);
    }
}

/**********************************************************************************
* Function Name: void update_gatt_db_write(cy_stc_ble_gatt_write_param_t *write_param)
***********************************************************************************
* Summary: This function updates the value attribute of GATT Write
*          characteristic present in the custom service
*
* Parameters:
*  cy_stc_ble_gatt_write_param_t *write_param: Parameter which holds information
*                              about attribute handle and value.
* Return:
*  None
**********************************************************************************/
static void update_gatt_db_write(cy_stc_ble_gatt_write_param_t *write_param)
{
    cy_en_ble_gatt_err_code_t gattErrorCode;

    /* Write the attribute value into GATT database of server */
    gattErrorCode = Cy_BLE_GATTS_WriteAttributeValuePeer(&conn_handle, &(write_param->handleValPair));

    /* Check for successful write operation */
    if(gattErrorCode == CY_BLE_GATT_ERR_NONE)
    {
        /* If data is written into value attribute of GATT write characteristic */
        if(write_param->handleValPair.attrHandle == CUSTOM_DESCR_HANDLE)
        {
            /* Copy the data to a variable for further processing in application
             * layer if required */
            memcpy(custom_data.value.val, write_param->handleValPair.value.val, write_param->handleValPair.value.len);
            /* Increment count of gatt write bytes received */
            gatt_write_rx_bytes += write_param->handleValPair.value.len;
        }
    }
    else
    {
    	DEBUG_BLE("GATT Write API, errorcode = 0x%X\r\n", gattErrorCode);
    }
}

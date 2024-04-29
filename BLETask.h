/*
 * BLETask.h
 *
 *  Created on: 2024-03-28
 *      Author: Gintaras
 */

#ifndef BLETASK_H_
#define BLETASK_H_

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

#define BLE_CMD_Q_LEN           (10u)

extern TaskHandle_t ble_task_handle;
extern QueueHandle_t ble_cmdQ;

void BLETask(void *pvParameters);

#endif /* BLETASK_H_ */

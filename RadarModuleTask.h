/*
 * RadarModuleTask.h
 *
 *  Created on: 2024-04-09
 *      Author: Gintaras
 */

#ifndef RADARMODULETASK_H_
#define RADARMODULETASK_H_

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

extern TaskHandle_t RadarModuleTaskHandle;
void RadarModuleTask(void *param);

#endif /* RADARMODULETASK_H_ */

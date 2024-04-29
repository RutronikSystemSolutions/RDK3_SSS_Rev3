/*
 * RadarSensorTask.h
 *
 *  Created on: 2024-03-26
 *      Author: Gintaras
 */

#ifndef RADARSENSORTASK_H_
#define RADARSENSORTASK_H_

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

extern TaskHandle_t RadarSensorTaskHandle;
void RadarSensorTask(void *param);
extern float32_t peak_distance;
extern float32_t max_peak;
extern int32_t max_peak_arg;

#endif /* RADARSENSORTASK_H_ */

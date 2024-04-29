/*
 * ThermalSensorTask.h
 *
 *  Created on: 2024-03-25
 *      Author: Gintaras
 */

#ifndef THERMALSENSORTASK_H_
#define THERMALSENSORTASK_H_

extern TaskHandle_t ThermalSensorTaskHandle;
void ThermalSensorTask(void *param);
extern float pix_data[];

#endif /* THERMALSENSORTASK_H_ */

/*
 * GestureSensorTask.h
 *
 *  Created on: 2024-04-03
 *      Author: Gintaras
 */

#ifndef GESTURESENSORTASK_H_
#define GESTURESENSORTASK_H_

#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/*Gesture Samples Frame Size*/
#define GESTURE_FRAME_SIZE	10

/*Gesture Frames Skipping*/
#define GESTURE_FRAME_SKIP	40

/*Gesture Hold Time in milliseconds*/
#define GESTURE_HOLD_MS		400

/*The Moving Average of the Gesture Samples*/
#define MOV_AVERAGE			2

/*Gesture Debugging Information Enable/Disable*/
#define STD_DEBUG_EN		0
#define LFT_RHT_DEBUG_EN	0
#define LFT_UP_DEBUG_EN		0
#define RHT_UP_DEBUG_EN		0

/*Standard Deviation & Signal Delay Thresholds*/
#define STD_THRESHOLD		50
#define LFT_RHT_DELAY_TH	0
#define LFT_UP_DELAY_TH		0
#define RHT_UP_DELAY_TH		0

extern cyhal_i2c_t I2C_scb3;
extern TaskHandle_t GestureSensorTaskHandle;
void GestureSensorTask(void *param);

typedef enum
{
	GESTURE_NONE = 0,
	GESTURE_RIGHT,
	GESTURE_LEFT,
	GESTURE_UP,
	GESTURE_DOWN,
	GESTURE_INIT = 0xFF,

}gesture_t;

typedef struct gesture_data
{
	float sensor1[GESTURE_FRAME_SIZE];
	float sensor2[GESTURE_FRAME_SIZE];
	float sensor3[GESTURE_FRAME_SIZE];
	int buff_pos;
	float std_left;
	float std_up;
	float std_right;
	float crosscorr[GESTURE_FRAME_SIZE * 2 - 1];
	gesture_t gesture; //currently recognized gesture
	TickType_t timestamp;
}gesture_data_t;

extern gesture_data_t gesture_data;

#endif /* GESTURESENSORTASK_H_ */

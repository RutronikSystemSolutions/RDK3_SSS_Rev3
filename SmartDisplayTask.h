/*
 * SmartDisplayTask.h
 *
 *  Created on: 2024-04-03
 *      Author: Gintaras
 */

#ifndef SMARTDISPLAYTASK_H_
#define SMARTDISPLAYTASK_H_

#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#define ARDU_BAUD_RATE       		1093750
#define STARTUP_DELAY_MS			1000
#define TH_IMG_POSLEFT				UINT16_C (60)
#define TH_IMG_POSTOP				UINT16_C (200)
#define RULER_POSLEFT				UINT16_C (TH_IMG_POSLEFT - 50)
#define RULER_POSTOP				UINT16_C (TH_IMG_POSTOP + 20)
#define CHEVUP_POSLEFT				UINT16_C (650)
#define CHEVUP_POSTOP				UINT16_C (130)
#define CHEVDOWN_POSLEFT			UINT16_C (650)
#define CHEVDOWN_POSTOP				UINT16_C (130)
#define CHEVRIGHT_POSLEFT			UINT16_C (650)
#define CHEVRIGHT_POSTOP			UINT16_C (130)
#define CHEVLEFT_POSLEFT			UINT16_C (650)
#define CHEVLEFT_POSTOP				UINT16_C (130)
#define CHEVLEFT_POSLEFT			UINT16_C (650)
#define CHEVLEFT_POSTOP				UINT16_C (130)
#define DISTANCE_POSLEFT			UINT16_C (650)
#define DISTANCE_POSTOP				UINT16_C (290)
#define SLIDER_POSLEFT				UINT16_C (650)
#define SLIDER_POSTOP				UINT16_C (DISTANCE_POSTOP - 20)
#define SLIDER_LAST_POS				UINT8_C (16)
#define SLIDER_MAX_DIST				UINT8_C (4)
#define NJR_IND_POSLEFT				UINT16_C (650)
#define NJR_IND_POSTOP				UINT16_C (390)
#define DUMMY_CMD					0
#define COLOUR_CMD					249
#define POSLEFT_CMD					251
#define POSTOP_CMD					252
#define BITS_UINT8					UINT8_C  (255)
#define THERMAL_SENSORS				UINT16_C (768)
#define BUFF_OVF_TOUT_MS			100

enum disp_notify_enum
{
	sigTHERMO 		= 	( 1UL << 0UL ),
	sigDISTANCE 	= 	( 1UL << 1UL ),
	sigGESTURE	 	= 	( 1UL << 2UL ),
	sigABSENCE	 	= 	( 1UL << 3UL ),
	sigPRESENCE	 	= 	( 1UL << 4UL ),

};

extern TaskHandle_t SmartDisplayTaskHandle;
void SmartDisplayTask(void *param);

#endif /* SMARTDISPLAYTASK_H_ */

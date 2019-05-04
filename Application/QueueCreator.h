/*
 * QueueCreator.h
 *
 *  Created on: 4 мая 2019 г.
 *      Author: Taras.Melnik
 */

#ifndef APPLICATION_QUEUECREATOR_H_
#define APPLICATION_QUEUECREATOR_H_

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "stm32f4xx.h"
#include "queue.h"

	static QueueHandle_t xRengefindersHighQueue;						// Distance for highLevel
	static QueueHandle_t xRengefindersLowQueue;							// Value for collision Handler
	static SemaphoreHandle_t xRengefindersMutex;
	static QueueHandle_t xCollisionAvoidanceQueue;						// Rangefinders values for motors
	static SemaphoreHandle_t xCollisionAvoidanceMutex;
	static QueueHandle_t xAngleQueue;									// Data from motors
	static SemaphoreHandle_t xAngleMutex;
	static QueueHandle_t xHighLvlQueue;								// Commands to motors
	static SemaphoreHandle_t xHighLvlMutex;
	static QueueHandle_t xBatteryChargeQueue;
	static SemaphoreHandle_t xBatteryChargeQueueMutex;
	static QueueHandle_t xLightColorQueue;

#endif /* APPLICATION_QUEUECREATOR_H_ */

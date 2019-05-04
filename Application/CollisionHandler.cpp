/*
 * CollisionHandler.cpp
 *
 *  Created on: 4 мая 2019 г.
 *      Author: Taras.Melnik
 */

#include "CollisionHandler.h"

CollisionHandler* collisionHandler;

CollisionHandler::CollisionHandler() {
//	xCollisionAvoidanceQueue = xQueueCreate(2, sizeof(uint8_t));
//	xCollisionAvoidanceMutex = xSemaphoreCreateMutex();
	xTaskToNotify = 0;
	rColFlag = false; 				// no collision
}

CollisionHandler::~CollisionHandler() {
}

bool CollisionHandler::getStatus()
{
	return rColFlag;
}

void CollisionHandler::run() {
	this->xTaskToNotify= xTaskGetCurrentTaskHandle();
	uint8_t distArr[RANGEFINDERS_NUMBER] = {0};

	while(1) {
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		xSemaphoreTake(xCollisionAvoidanceMutex, portMAX_DELAY);
		for (uint8_t i = 0; i < RANGEFINDERS_NUMBER; i++) {
			xQueueReceive(xRengefindersLowQueue, &distArr[i], portMAX_DELAY);
			if (distArr[i] < 10) {
				this->rColFlag = true;
				break;
			} else
				this->rColFlag = false;
		}
		xSemaphoreGive(xCollisionAvoidanceMutex);
	}
}

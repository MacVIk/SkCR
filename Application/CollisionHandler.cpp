/*
 * CollisionHandler.cpp
 *
 *  Created on: 4 мая 2019 г.
 *      Author: Taras.Melnik
 */

#include "CollisionHandler.h"

CollisionHandler* collisionHandler;

CollisionHandler::CollisionHandler() {
	rColFlag = false; 				// no collision
}

CollisionHandler::~CollisionHandler() {
}

bool CollisionHandler::getStatus()
{
	return rColFlag;
}

void CollisionHandler::run() {
	sensorTask->xTaskToNotify= xTaskGetCurrentTaskHandle();
	uint8_t distArr[RANGEFINDERS_NUMBER] = {0};

	while(1) {
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		sensorTask->getDistance(distArr);
		for (uint8_t i = 0; i < RANGEFINDERS_NUMBER; i++) {
			if (distArr[i] < 15) {
				this->rColFlag = true;
				break;
			} else
				this->rColFlag = false;
		}
	}
}

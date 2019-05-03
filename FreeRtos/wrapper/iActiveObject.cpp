/*
 * iActiveObject.cpp
 *
 *  Created on: 21.09.2015
 *      Author: Tata
 */

#include "iActiveObject.h"

BaseType_t iActiveObject::taskCreate(const tU16 stackDepth, tU32 priority,
		const char * const name) {
	BaseType_t state =
			xTaskCreate((TaskFunction_t) iActiveObject::taskFunction, name, stackDepth,
					this, priority, &taskHandle);
	if (state == pdPASS) {
		cRTOS::addTask(this);
	}
	return state;
}

void iActiveObject::taskDelete() {
	vTaskDelete(this->taskHandle);
}

void iActiveObject::taskResume() {
	vTaskResume(this->taskHandle);
}

void iActiveObject::taskSuspend() {
	vTaskSuspend(this->taskHandle);
}

void iActiveObject::taskDelay(TickType_t timeIncrement) {
	vTaskDelay(timeIncrement);
}

void iActiveObject::taskDelayUntil(TickType_t *previousWakeTime, TickType_t timeIncrement) {
	vTaskDelayUntil(previousWakeTime, timeIncrement);
}

void iActiveObject::prepareForSleep(void) {
	this->taskSuspend();
	this->sleeping = true;
}

iActiveObject::iActiveObject() {
	this->sleeping = false;
}

void iActiveObject::wakeup() {
	this->sleeping = false;
	this->taskResume();
}

bool iActiveObject::isSleeping() {
	return this->sleeping;
}

void iActiveObject::taskFunction(const void *parameters) {
	((iActiveObject*) parameters)->run();
}

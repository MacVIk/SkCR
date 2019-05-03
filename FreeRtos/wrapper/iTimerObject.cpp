/*
 * iTimerObject.cpp
 *
 *  Created on: 19.09.2015
 *      Author: Tata
 */

#include "iTimerObject.h"

BaseType_t iTimerObject::timerStart(TickType_t blockTpe) {
	return xTimerStart((TimerHandle_t)this->timerHandle, blockTpe);
}

BaseType_t iTimerObject::timerStartFromISR(
		BaseType_t* xHigherPriorityTaskWoken) {
	return xTimerStartFromISR((TimerHandle_t) this->timerHandle,
			xHigherPriorityTaskWoken);
}

BaseType_t iTimerObject::timerCreate(const char * const name,
		const TickType_t xTimerPeriodInTicks, const UBaseType_t uxAutoReload) {
	this->timerHandle = xTimerCreate(name, xTimerPeriodInTicks, uxAutoReload,
			this, (TimerCallbackFunction_t) iTimerObject::timerFunction);

	if (this->timerHandle != NULL) {
		return pdPASS;
	} else {
		return pdFALSE;
	}
}

BaseType_t iTimerObject::timerDelete(TickType_t blockTime) {
	return xTimerDelete(this->timerHandle, blockTime);
}

void iTimerObject::timerFunction(TimerHandle_t parameters) {
	iTimerObject* tmp = (iTimerObject*) pvTimerGetTimerID(parameters);
	tmp->timerCallbackFunction();
}

BaseType_t iTimerObject::timerChangePeriod(TickType_t newPeriod,
		TickType_t blockTime) {
	return xTimerChangePeriod(this->timerHandle, newPeriod, blockTime);
}

BaseType_t iTimerObject::timerStop(TickType_t blockTime) {
	return xTimerStop(this->timerHandle, blockTime);
}

BaseType_t iTimerObject::timerReset(TickType_t blockTime) {
	return xTimerReset(this->timerHandle, blockTime);
}


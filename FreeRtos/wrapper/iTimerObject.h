/*
 * iTimerObject.h
 *
 *  Created on: 19.09.2015
 *      Author: Tata
 */

#ifndef ITIMEROBJECT_H_
#define ITIMEROBJECT_H_

#define DEFAULT_WAIT_TIME 10

#include <FreeRTOS.h>
#include "freertos/wrapper/frtoswrapper.h"
#include "timers.h"

class iTimerObject {
public:
	virtual void timerCallbackFunction(void) = 0;

	BaseType_t timerStart(TickType_t blockTime);
	BaseType_t timerStartFromISR(BaseType_t* xHigherPriorityTaskWoken);
	BaseType_t timerCreate(const char * const name,
			const TickType_t xTimerPeriodInTicks,
			const UBaseType_t uxAutoReload);
	BaseType_t timerDelete(TickType_t blockTime);
	BaseType_t timerChangePeriod(TickType_t newPeriod, TickType_t blockTime);
	BaseType_t timerStop(TickType_t blockTime);
	BaseType_t timerReset(TickType_t blockTime);

	void *timerHandle;

private:
	static void timerFunction(TimerHandle_t parameters);
};

#endif /* ITIMEROBJECT_H_ */

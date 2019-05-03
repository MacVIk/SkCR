/*******************************************************************************
 *  FILENAME: iActiveObject.h
 *
 *  DESCRIPTION: Интерфейс для активных объектов. Каждый активный объект должен
 *               наследовать этот интерфейс
 *
 *  Copyright (c) 2015 by SUSU
 *
 *	Modifyed by adozzer
 *******************************************************************************/
#ifndef IACTIVEOBJECT_H
#define IACTIVEOBJECT_H

#include <FreeRTOS.h>
#include "freertos/wrapper/frtoswrapper.h"
#include "Application/types.h"
#include "task.h"

class iActiveObject {
public:
	virtual void run(void) = 0;
	virtual void prepareForSleep(void);
	virtual void wakeup(void);
	BaseType_t taskCreate(const tU16 stackDepth, tU32 priority,
			const char * const name);
	void taskDelete();
	void taskResume();
	void taskSuspend();
	void taskDelay(TickType_t timeIncrement);
	void taskDelayUntil(TickType_t *previousWakeTime, TickType_t timeIncrement);
	void *taskHandle;
	bool isSleeping();

protected:
	iActiveObject();
	bool sleeping;

private:
	static void taskFunction(const void *parameters);
};

#endif //IACTIVEOBJECT_H

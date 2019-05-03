/*******************************************************************************
 *  FILENAME: frtosWrapper.h
 *
 *  DESCRIPTION: Обертка для FreeRTOS
 *
 *  Copyright (c) 2015 by SUSU
 *  Author: Sergey Sikharulidze and Sergey Kolody
 *******************************************************************************/
#ifndef FRTOSWRAPPER_H
#define FRTOSWRAPPER_H

#include <FreeRTOS.h>      //Препроцессорные директивы для настройки компиляции RTOS
#include <task.h>          //Планировщик, задачи
#include <event_groups.h>  //События
#include <queue.h>         //Очереди
#include "Application/types.h"         //Стандартные типы проекта
#include "arm_math.h"
#include "iTimerObject.h"

class iActiveObject;

#define NO_TICKS_WAIT (tU32)0
//Создаем типы, соответствующие кодинг стандарту, которые заменяют стандартные
//типы FreeRTOS                                       
typedef EventGroupHandle_t tEventGroupHandle;
typedef QueueHandle_t tQueueHandle;
typedef TaskHandle_t tTaskHandle;
typedef eNotifyAction tNotifyAction;

typedef enum {
	RS_pass = (tU8) 0, RS_fail = (tU8) 1
} tRtosStatus;

class cRTOS {
public:
	explicit cRTOS(void);
	void startScheduler(void) const;
	//lint -save -e971 Unqualified char types are allowed for strings and single characters only
	//lint -save -e970 matches library prototype
//	tRtosStatus taskCreate(iActiveObject *pActiveObject, const tU16 stackDepth,
//			tU32 priority, const char * const name) const;

//	void timerCreate(const char * const name,
//			const TickType_t xTimerPeriodInTicks,
//			const UBaseType_t uxAutoReload, iTimerObject* timerObject) const;

	//lint -restore
//	void taskDelay(const tU32 timeIncrement) const;
	void schedulerDisable(void) const;
	tRtosStatus schedulerEnable(void) const;
	tEventGroupHandle eventGroupCreate(void) const;
	tU32 eventGroupSetBits(tEventGroupHandle eventGroup,
			const tU32 bitsToSet) const;
	tRtosStatus eventGroupSetBitsFromISR(tEventGroupHandle eventGroup,
			const tU32 bitsToSet) const;
	tU32 eventGroupClearBits(tEventGroupHandle eventGroup,
			const tU32 bitsToClear) const;
	tU32 eventGroupClearBitsFromISR(tEventGroupHandle eventGroup,
			const tU32 bitsToClear) const;
	tU32 eventGroupWaitBits(const tEventGroupHandle eventGroup,
			const tU32 bitsToWaitFor, const tBoolean clearOnExit,
			const tBoolean waitForAllBits, const tU32 ticksToWait) const;
	tQueueHandle queueCreate(const tU32 queueLength, const tU32 itemSize) const;
	tRtosStatus queueSend(const tQueueHandle queue, const void * pItemToQueue,
			const tU32 ticksToWait) const;
	tRtosStatus queueSendToFront(const tQueueHandle queue,
			const void * pItemToQueue, const tU32 ticksToWait) const;
	tRtosStatus queueSendToBack(const tQueueHandle queue,
			const void * pItemToQueue, const tU32 ticksToWait) const;
	tBoolean queueReceive(const tQueueHandle queue, void * pBuffer,
			const tU32 ticksToWait) const;
	tRtosStatus taskNotify(const tTaskHandle taskToNotify, const tU32 value,
			const tNotifyAction eAction) const;
	tBoolean taskNotifyWait(const tU32 bitsToClearOnEntry,
			const tU32 BitsToClearOnExit, const tU32 *value,
			const tU32 ticksToWait) const;

	static UBaseType_t getTasks(iActiveObject** tasks);
	static iActiveObject** getTasks();
	static void addTask(iActiveObject* task);
	static uint8_t getTasksCount();

	static TickType_t fromMsToTick(float32_t ms);
	static TickType_t fromSecToTick(float32_t sec);
	static float32_t fromTickToMs(TickType_t ticks);
	static float32_t fromTickToSec(TickType_t ticks);

private:
	static iActiveObject* tasks[30];
	static uint8_t tasksCount;
};

extern cRTOS oRTOS;

#endif //FRTOSWRAPPER_H

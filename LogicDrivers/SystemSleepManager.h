/*
 * SystemSleepManager.h
 *
 *  Created on: Dec 21, 2015
 *      Author: asquad
 */

#ifndef SYSTEMSLEEPMANAGER_H_
#define SYSTEMSLEEPMANAGER_H_

#include "stm32f4xx.h"
#include "portmacro.h"
#include "iActiveObject.h"
#include "PeripheralDrivers/RTCManager.h"

enum SystemStatus {
	systemActive, systemSleepAndScanning, systemLightSleep, systemDeepSleep, systemWakeupFromStandby
};

class SystemSleepManager: public iTimerObject {
public:
	virtual ~SystemSleepManager();
	static SystemSleepManager* getInstance();
	void wakeup();
	void enterScan();
	void exitScan();
	void addTaskForScan(iActiveObject* task);
	SystemStatus getStatus();
	bool checkFromStandby();
	static void enterStandbyMode(FunctionalState wakeUpOnTimer,
			FunctionalState wakeUpOnInterrupt);

private:
	SystemSleepManager();
	bool fromStandby;
	static SystemSleepManager* SystemSleepManagerInstance;
	SystemStatus status;
	void timerCallbackFunction();

	TickType_t scanPeriod;
	TickType_t activeTime;
	TickType_t scanTime;

	iActiveObject* tasksForScan[3];
	uint8_t tasksForScanIterator;
};

#endif /* SYSTEMSLEEPMANAGER_H_ */

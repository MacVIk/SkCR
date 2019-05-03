/*
 * SystemSleepManager.cpp
 *
 *  Created on: Dec 21, 2015
 *      Author: asquad
 */

#include "SystemSleepManager.h"

SystemSleepManager* SystemSleepManager::SystemSleepManagerInstance = 0;

SystemSleepManager::SystemSleepManager() {
	this->status = systemActive;
	activeTime = cRTOS::fromSecToTick(10);
	scanPeriod = cRTOS::fromSecToTick(1);
	scanTime = cRTOS::fromMsToTick(15);
	this->timerCreate("SystemSleepT", activeTime, pdTRUE);
	this->timerStart(10);
	tasksForScanIterator = 0;

	this->fromStandby = false;
}

SystemSleepManager* SystemSleepManager::getInstance() {
	if (!SystemSleepManagerInstance) {
		SystemSleepManagerInstance = new SystemSleepManager();
	}
	return SystemSleepManager::SystemSleepManagerInstance;
}

SystemSleepManager::~SystemSleepManager() {
	// TODO Auto-generated destructor stub
}

void SystemSleepManager::enterScan() {

	for (uint8_t i = 0; i < tasksForScanIterator; i++) {
		this->tasksForScan[i]->wakeup();
	}
	this->timerChangePeriod(scanTime, 10);
	status = systemSleepAndScanning;
}

void SystemSleepManager::exitScan() {

	//System::getInstance()->beep(2);
	for (uint8_t i = 0; i < tasksForScanIterator; i++) {
		this->tasksForScan[i]->prepareForSleep();
	}
	this->timerChangePeriod(scanPeriod, 10);
	status = systemLightSleep;
	if (this->fromStandby) {
		this->enterStandbyMode(ENABLE, ENABLE);
	}
}

void SystemSleepManager::addTaskForScan(iActiveObject* task) {
	this->tasksForScan[tasksForScanIterator] = task;
	tasksForScanIterator++;
}

SystemStatus SystemSleepManager::getStatus() {
	return this->status;
}

bool SystemSleepManager::checkFromStandby() {
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE); // Enable the PWR clock
	if ((PWR_GetFlagStatus(PWR_FLAG_SB) == SET)
			&& (PWR_GetFlagStatus(PWR_FLAG_WU) == SET)) { // wake-up (WKUP pin) from standby
		this->fromStandby = true;
	}
	PWR_ClearFlag(PWR_FLAG_SB);
	PWR_ClearFlag(PWR_FLAG_WU);
//	this->fromStandby = true;
	return this->fromStandby;
}

void SystemSleepManager::enterStandbyMode(FunctionalState wakeUpOnTimer,
		FunctionalState wakeUpOnInterrupt) {
//	DBGMCU_Config(DBGMCU_STANDBY, ENABLE);
	RTCManager::initWakeUp(wakeUpOnTimer, wakeUpOnInterrupt);
	DBGMCU_Config(DBGMCU_STANDBY, DISABLE);
	PWR_EnterSTANDBYMode();
}

void SystemSleepManager::timerCallbackFunction() {
	if (SystemSleepManagerInstance->status == systemActive) {
//		lightSleep();
	} else if (SystemSleepManagerInstance->status == systemLightSleep) {
		enterScan();
	} else if (SystemSleepManagerInstance->status == systemSleepAndScanning) {
		exitScan();
	}
}

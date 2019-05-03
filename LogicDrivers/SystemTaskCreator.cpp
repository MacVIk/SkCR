	/*
 * SystemTaskCreator.cpp
 *
 *  Created on: 26.12.2015
 *      Author: adozzer
 */

#include "SystemTaskCreator.h"

MovementControl* moveTask;
ReadEncoders* readEncoders;

SystemTaskCreator* SystemTaskCreator::SystemTaskCreatorInstance = 0;

SystemTaskCreator* SystemTaskCreator::getInstance()
{
	if (!SystemTaskCreatorInstance)
		SystemTaskCreatorInstance = new SystemTaskCreator();
	return
		SystemTaskCreator::SystemTaskCreatorInstance;
}

SystemTaskCreator::SystemTaskCreator() {
}

SystemTaskCreator::~SystemTaskCreator() {
}

//INDEPENDED WATCHDOG
//void iwdg_init(void) {
//	// включаем LSI
//	RCC_LSICmd(ENABLE);
//	while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET);
//	// разрешается доступ к регистрам IWDG
//	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
//	// устанавливаем предделитель
//	IWDG_SetPrescaler(IWDG_Prescaler_256);
//	// значение для перезагрузки (~1.5s)
//	IWDG_SetReload(0xEA);
//	// перезагрузим значение
//	IWDG_ReloadCounter();
//	// LSI должен быть включен
//	IWDG_Enable();
//}

void SystemTaskCreator::run() {
	vTaskSuspendAll();

	setLEDTask = new LEDStrip();
	getBatChargeTask = new BatteryChargeAsker();
	sensorTask = new CollisionAvoidance();
	usbUserInterface = new USBUserInterface();
//	moveTask = new MovementControl();
//	readEncoders = new ReadEncoders();
	hyroMotor = new HyroMotor();

	InitUser.GPIOPinInit();
	InitUser.PWMInit();
//	InitUser.EncoderInit();
//	InitUser.OnePulseModeInit();
	InitUser.ADCInit();
	InitUser.InterruptInit();
	InitUser.TIMInit();

	setLEDTask->taskCreate(256, 2, "setLED");
	getBatChargeTask->taskCreate(256, 2, "getBatCharge");
	sensorTask->taskCreate(256, 3, "sensorTask");
	hyroMotor->taskCreate(1025, 4, "hyroMotorTask");
	usbUserInterface->taskCreate(1024, 4, "UserUARTtoUSB");
//	moveTask->taskCreate(512,3,"moveTask");
//	readEncoders->taskCreate(512,3,"readEncoders");
//	setMotTask->taskCreate(600, 3, "setMotTask");

	xTaskResumeAll();
	while (true)
		this->taskSuspend();
}




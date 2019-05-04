
#define HSE_VALUE ((uint32_t)8000000)

#include "core_cm4.h"
#include "iActiveObject.h"
#include "ReadEncoders.h"
#include "LEDStrip.h"
#include "BatteryChargeAsker.h"
#include "USBUserInterface.h"
#include "CollisionAvoidance.h"
#include "MovementControl.h"
#include "InitialisationList.h"
#include "UARTtoRS485.h"
#include "HyroMotor.h"

cRTOS oRTOS;

int main(void)
{
	SystemInit();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

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

	oRTOS.startScheduler();

	return 0;
}

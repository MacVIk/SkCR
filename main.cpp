#include "SystemTaskCreator.h"

//#define LEDS_TASK_HANDLE_INDEX          0
//#define LEDSDIRECTOR_STACK_SIZE configMINIMAL_STACK_SIZE
//#define LEDSDIRECTOR_PRIORITY (tU32)2
//#define TIMEOUT 1000000UL;

cRTOS oRTOS;
SystemTaskCreator* taskCreator;

int main(void)
{
	SystemInit();
//	InitUser.GPIOPinInit();
//	InitUser.PWMInit();
//	InitUser.EncoderInit();
//	InitUser.OnePulseModeInit();
//	InitUser.ADCInit();
//	InitUser.InterruptInit();
//	InitUser.TIMInit();

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	taskCreator = SystemTaskCreator::getInstance();
	taskCreator->taskCreate(1000, 2, "creator");
	oRTOS.startScheduler();
	return 0;
}

/*
 * RTCManager.cpp
 *
 *  Created on: 05.01.2016
 *      Author: adozzer
 */

#include "RTCManager.h"

#ifdef __cplusplus
extern "C" {
#endif
void RTC_WKUP_IRQHandler(void) {
	if (RTC_GetITStatus(RTC_IT_WUT) != RESET) {

		RTC_ClearITPendingBit(RTC_IT_WUT);
		EXTI_ClearITPendingBit(EXTI_Line22);
	}
}
#ifdef __cplusplus
}
#endif

void RTCManager::init() {
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

	// Enable the PWR clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

	// Allow access to RTC
	PWR_BackupAccessCmd(ENABLE);

	// Reset RTC Domain
	//RCC_BackupResetCmd(ENABLE);
	//RCC_BackupResetCmd(DISABLE);

	// Enable the LSE OSC
	RCC_LSEConfig(RCC_LSE_ON);

	// Wait till LSE is ready
	while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET) {
	}

	// Select the RTC Clock Source
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

	// Enable the RTC Clock
	RCC_RTCCLKCmd(ENABLE);

	// Enable the RTC Wakeup Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = RTC_WKUP_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// EXTI configuration *******************************************************
	EXTI_ClearITPendingBit(EXTI_Line22);
	EXTI_InitStructure.EXTI_Line = EXTI_Line22;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	// Wait for RTC APB registers synchronisation
	RTC_WaitForSynchro();
	RTC_ClearITPendingBit(RTC_IT_WUT);
	EXTI_ClearITPendingBit(EXTI_Line22);

	RTC_WakeUpClockConfig(RTC_WakeUpClock_RTCCLK_Div16); // 32768/16 = 2048 Hz - clock of wake-up timer

	RTC_WakeUpCmd(DISABLE);
//	RTC_SetWakeUpCounter(20480); // Setup wake-up frequency

	// Enable the Wakeup Interrupt
	RTC_ITConfig(RTC_IT_WUT, ENABLE);

	// Enable Wakeup Counter
	RTC_WakeUpCmd(ENABLE);

	// Enable BKPSRAM Clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_BKPSRAM, ENABLE);

	// Enable the Backup SRAM low power Regulator to retain it's content in VBAT mode
	PWR_BackupRegulatorCmd(ENABLE);

	// Wait until the Backup SRAM low power Regulator is ready
	while (PWR_GetFlagStatus(PWR_FLAG_BRR) == RESET) {
	}

//	RTC_WakeUpCmd(DISABLE);
//	RTC_SetWakeUpCounter(2048 * 10); // Setup wake-up frequency (10 sec)

//	// Enable the Wakeup Interrupt
//	RTC_ITConfig(RTC_IT_WUT, ENABLE);
//
//	RTC_ClearITPendingBit(RTC_IT_WUT);
//	EXTI_ClearITPendingBit(EXTI_Line22);

	// Enable Wakeup Counter
//	PWR_WakeUpPinCmd(ENABLE); // Enable standby leave by WKUP pin
//	RTC_WakeUpCmd(ENABLE);
	//RTC_AlarmCmd(RTC_Alarm_A, DISABLE);
}

void RTCManager::initWakeUp(FunctionalState wakeUpOnTimer,
		FunctionalState wakeUpOnInterrupt) {
	RTC_SetWakeUpCounter(2048 * 10); // Setup wake-up frequency (10 sec)

	// Enable the Wakeup Interrupt
	RTC_ITConfig(RTC_IT_WUT, ENABLE);

	RTC_ClearITPendingBit(RTC_IT_WUT);
	EXTI_ClearITPendingBit(EXTI_Line22);

	PWR_WakeUpPinCmd(wakeUpOnInterrupt); // Enable standby leave by WKUP pin
	RTC_WakeUpCmd(wakeUpOnTimer);
}

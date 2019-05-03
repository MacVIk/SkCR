/*
 * libPWM.h
 *
 *  Created on: 01.10.2018
 *      Author: Taras.Melnik
 */

#ifndef INITIALISATIONLIST_H_
#define INITIALISATIONLIST_H_

#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "defines.h"
#include "USBUserInterface.h"

class InitialisationList {
public:
	InitialisationList();
	~InitialisationList();

	void GPIOPinInit();
	void TIMInit();
	void PWMInit();
	void OnePulseModeInit();
	void EncoderInit();
	void USARTInit();
	void ADCInit();
	void InterruptInit();
};

extern InitialisationList InitUser;

#endif /* INITIALISATIONLIST_H_ */

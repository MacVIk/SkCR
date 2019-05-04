/*
 * USBUserInterface.h
 *
 *  Created on: 29.10.2018
 *      Author: Taras.Melnik
 */

#ifndef USBUSERINTERFACE_H_
#define USBUSERINTERFACE_H_

#include "iActiveObject.h"
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "defines.h"
#include "QueueCreator.h"
#include "LEDStrip.h"
#include "BatteryChargeAsker.h"
#include "CollisionAvoidance.h"
#include "task.h"
#include "HyroMotor.h"
#include "UARTuserInit.h"


class USBUserInterface: public iActiveObject {
public:
	USBUserInterface();
	virtual ~USBUserInterface();
	void init();
	void run();
	TaskHandle_t xTaskToNotify;
};

extern USBUserInterface* usbUserInterface;


#endif /* USBUSERINTERFACE_H_ */

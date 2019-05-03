/*
 * RTCManager.h
 *
 *  Created on: 05.01.2016
 *      Author: adozzer
 */

#ifndef RTCMANAGER_H_
#define RTCMANAGER_H_

#include "stm32f4xx.h"

class RTCManager {
public:
	static void init();
	static void initWakeUp(FunctionalState wakUpOnTimer,
			FunctionalState wakeUpOnInterrupt);

};

#endif /* RTCMANAGER_H_ */

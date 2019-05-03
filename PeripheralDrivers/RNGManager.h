/*
 * RNGManager.h
 *
 *  Created on: 04.09.2015
 *      Author: Tata
 */

#ifndef RNGMANAGER_H_
#define RNGMANAGER_H_

#include "stm32f4xx.h"
#include "FreeRTOS.h"

class RNGManager {
public:
	static void init();
	static uint32_t getRandom32();
	static uint32_t getRandom(uint32_t maxValue);
};

#endif /* RNGMANAGER_H_ */

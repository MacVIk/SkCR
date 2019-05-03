/*
 * RNGManager.cpp
 *
 *  Created on: 04.09.2015
 *      Author: Tata
 */

#include "RNGManager.h"

void RNGManager::init() {
	RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);
	// RNG Peripheral enable
	RNG_Cmd(ENABLE);
}

uint32_t RNGManager::getRandom32() {
	while (RNG_GetFlagStatus(RNG_FLAG_DRDY) == RESET) {
	}
	return RNG_GetRandomNumber();
}

uint32_t RNGManager::getRandom(uint32_t maxValue) {
	if (maxValue == 1) {
		return 0;
	}
	return (getRandom32() % maxValue);
}


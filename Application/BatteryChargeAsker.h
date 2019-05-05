/*
 * BatteryChargeAsker.h
 *
 *  Created on: 31.10.2018
 *      Author: Taras.Melnik
 */

#ifndef BATTERYCHARGEASKER_H_
#define BATTERYCHARGEASKER_H_

#include "iActiveObject.h"
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "semphr.h"
#include "QueueCreator.h"

#define MAX_BATTERY_VOLTAGE 	((uint16_t)  255)	// max battery charge in ADC units (get empirically)
#define STOP_BATTERY_VOLTAGE 	((uint16_t)  155)	// 5% of battery
#define MIN_BATTERY_VOLTAGE 	((uint16_t)  151)	// discharged

class BatteryChargeAsker: public iActiveObject
{
public:
	BatteryChargeAsker();
	virtual ~BatteryChargeAsker();
	uint8_t getCharge();
	void run();
private:
	uint8_t ChargeVal_P;
};

extern BatteryChargeAsker* getBatChargeTask;

#endif /* BATTERYCHARGEASKER_H_ */

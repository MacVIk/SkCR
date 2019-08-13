/*
 * BatteryChargeAsker.h
 *
 *  Created on: 31.10.2018
 *      Author: Taras.Melnik
 */

#ifndef CODE_BATTERYMANAGER_H_
#define CODE_BATTERYMANAGER_H_

#include "stm32f4xx.h"

#define MAX_BATTERY_VOLTAGE 	((uint16_t) 255)	// max battery charge in ADC units (get empirically)
#define STOP_BATTERY_VOLTAGE 	((uint16_t) 155)	// 5% of battery
#define MIN_BATTERY_VOLTAGE 	((uint16_t) 151)	// discharged

#define ADC_RANK                ((uint8_t) 1)

class BatteryManager {
public:
        BatteryManager();
	virtual ~BatteryManager();

	void init_adc();
	uint8_t get_charge();
	static void run(void *parameters);
private:
	static uint8_t ChargeVal_P;
};

extern BatteryManager bat_manager;

#endif /* CODE_BATTERYMANAGER_H_ */

/*
 * ADCDirty.h
 *
 *  Created on: 19.08.2015
 *      Author: Tata
 */

#ifndef ADCDIRTY_H_
#define ADCDIRTY_H_

#include "stm32f4xx.h"
#include "PeripheralDrivers/PortManager.h"

class ADCDirty {
private:
	ADC_TypeDef* ADCx;
	uint8_t channel;
	GPIO_TypeDef* port;
	uint8_t pinNumber;

public:
	void init();
	uint16_t sample(uint8_t ADC_SampleTime);
	ADCDirty(ADC_TypeDef* ADCx, uint8_t channel, GPIO_TypeDef* port,
			uint8_t pinNumber);
	virtual ~ADCDirty();
};

#endif /* ADCDIRTY_H_ */

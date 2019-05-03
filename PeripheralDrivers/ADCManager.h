/*
 * ADCManager.h
 *
 *  Created on: 22.08.2015
 *      Author: Tata
 */

#ifndef ADCMANAGER_H_
#define ADCMANAGER_H_
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "queue.h"

struct ADCGlobalMessage {
	ADC_TypeDef* ADCx;
	TickType_t timeStamp;
};

struct ADCMessage {
	uint16_t value;
	TickType_t timeStamp;
};

class ADCManager {

public:
	virtual QueueHandle_t initChannel(ADC_TypeDef* ADCx, uint8_t channel,
			uint8_t ADC_SampleTime) = 0; //init in order of rank
	virtual void sample(ADC_TypeDef* ADCx) = 0;
	virtual void start(ADC_TypeDef* ADCx) = 0;

	virtual void init() = 0;
};

#endif /* ADCMANAGER_H_ */

/*
 * ADCDaemonOnInterrupts.h
 *
 *  Created on: 20.08.2015
 *      Author: Tata
 */

#ifndef ADCMANAGERONINTERRUPTS_H_
#define ADCMANAGERONINTERRUPTS_H_

#include "PeripheralDrivers/PortManager.h"
#include "PeripheralDrivers/ADCManager.h"
#include "FreeRtos/wrapper/iactiveobject.h"
#include "semphr.h"
#include "stm32f4xx.h"
#include "task.h"

class ADCManagerOnInterrupts {
private:
	static ADCManagerOnInterrupts* ADCManagerOnInterruptsInstance;
	ADCManagerOnInterrupts();
	ADCManagerOnInterrupts(const ADCManagerOnInterrupts&);
	ADCManagerOnInterrupts& operator=(ADCManagerOnInterrupts&);

public:
	QueueHandle_t ADCGlobalQueue;

	uint8_t ADCInitedChannelCounter[3];
	QueueHandle_t ADCChannelQueues[3][16];
	uint16_t ADCBuffer[3][16];

	void sample(ADC_TypeDef* ADCx);

	void start(ADC_TypeDef* ADCx); //execute after all initChannel() calls
	QueueHandle_t initChannel(ADC_TypeDef* ADCx, uint8_t channel,
			uint8_t ADC_SampleTime); //init in order of rank

	static ADCManagerOnInterrupts* getInstance();
	virtual ~ADCManagerOnInterrupts();
};

#endif /* ADCMANAGERONINTERRUPTS_H_ */


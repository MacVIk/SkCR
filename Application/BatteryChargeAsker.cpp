/*
 * BatteryChargeAsker.cpp
 *
 *  Created on: 31.10.2018
 *      Author: Taras.Melnik
 */

#include "BatteryChargeAsker.h"

BatteryChargeAsker* getBatChargeTask;

BatteryChargeAsker::BatteryChargeAsker() {
	xBatteryChargeQueue = xQueueCreate(1, sizeof(uint8_t));
	xBatteryChargeQueueMutex = xSemaphoreCreateMutex();
	ADC_SoftwareStartConv(ADC1); 									//**Start primary ADC calculation
}

BatteryChargeAsker::~BatteryChargeAsker() {
}

void BatteryChargeAsker::writeToQueue(uint8_t &data)
{
	xSemaphoreTake(xBatteryChargeQueueMutex, portMAX_DELAY);
	xQueueOverwrite(xBatteryChargeQueue, &data);
	xSemaphoreGive(xBatteryChargeQueueMutex);
}

void BatteryChargeAsker::run()
{
	uint16_t ChargeVal = 0;
	uint8_t ChargeVal_P = 0;
	uint16_t hBorder = MAX_BATTERY_VOLTAGE;
	uint16_t sBorder = STOP_BATTERY_VOLTAGE;
	uint16_t lBorder = MIN_BATTERY_VOLTAGE;
	uint16_t buffArray[10];
	uint16_t buffAverege = 0;
	uint8_t k = 0;
	float perKoef = (hBorder - lBorder)/100.f;
	buffAverege = ADC_GetConversionValue(ADC1);
	for (uint8_t i = 0; i < 10; i++){
		buffArray[i] = buffAverege;
	}
	while(1) {
		buffAverege = 0;
		for (uint8_t i = 9; i > 0; i--) {
			buffArray[i] = buffArray[i-1];
			buffAverege += buffArray[i];
		}
		buffArray[0] = ADC_GetConversionValue(ADC1); 		//**Get ADC Calculation
		buffAverege += buffArray[0];
		ChargeVal = buffAverege / 10;
		if (ChargeVal >= hBorder)
			ChargeVal = hBorder;
		else if (ChargeVal > lBorder) {
			if (hBorder - ChargeVal > 3 && k <= 15)
				k++;
			else {
				k = 0;
				hBorder = ChargeVal;
			}
		} else
			ChargeVal = lBorder;
		ChargeVal_P = uint8_t (ChargeVal - lBorder)/perKoef;
		writeToQueue(ChargeVal_P);
		ADC_SoftwareStartConv(ADC1);								//** Start ADC
		taskDelay(oRTOS.fromMsToTick(100));
	}
}








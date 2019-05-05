/*
 * CollisionAvoidance.cpp
 *
 *  Created on: 15.01.2019
 *      Author: Taras.Melnik
 */

#include "CollisionAvoidance.h"

CollisionAvoidance *sensorTask;

uint16_t sensTimeArr[RANGEFINDERS_NUMBER];

CollisionAvoidance::CollisionAvoidance()
{
//	xRengefindersHighQueue = xQueueCreate(RANGEFINDERS_NUMBER, sizeof(uint8_t));
//	xRengefindersLowQueue = xQueueCreate(RANGEFINDERS_NUMBER, sizeof(uint8_t));
	xTaskToNotify = 0;
	this->xRengefindersMutex = xSemaphoreCreateMutex();
	for (uint8_t i = 0; i < RANGEFINDERS_NUMBER; i++)
		this->sensDistArr[i] = new uint8_t[15];					//create array for distance history
}

CollisionAvoidance::~CollisionAvoidance(){
}

uint8_t CollisionAvoidance::partition(uint8_t* input, uint8_t p, uint8_t r)
{
	uint8_t pivot = input[r];
	while (p < r) {
		while (input[p] < pivot)
			p++;
		while (input[r] > pivot)
			r--;
		if (input[p] == input[r])
			p++;
		else if (p < r) {
			uint8_t tmp = input[p];
			input[p] = input[r];
			input[r] = tmp;
		}
	}
	return r;
}

uint8_t CollisionAvoidance::quick_select(uint8_t* input, uint8_t p, uint8_t r, uint8_t k)
{
	if (p == r)
		return input[p];
	uint8_t j = partition(input, p, r);
	uint8_t length = j - p + 1;
	if (length == k)
		return input[j];
	else if (k < length )
		return quick_select(input, p, j - 1, k);
	else
		return quick_select(input, j + 1, r, k - length);
}

void CollisionAvoidance::calculateDistanceQsort(uint8_t* distArr, uint8_t i)
{
	uint16_t timeBuff = 0;
	uint8_t median = 0;
	if (sensTimeArr[i] == 0xff)
		distArr[0] = 0xff;
	else
		distArr[0] = (sensTimeArr[i] - 500) / 56;
	uint8_t sortArr[15];
	for (uint8_t j = 0; j < 15; j++)
		sortArr[j] = distArr[j];
	median = quick_select(sortArr, 0, 15, 7);
	for (uint8_t j = 14; j > 0; j--)
		distArr[j] = distArr[j - 1];
	if ((median - distArr[0] > 7) || (median - distArr[0] < -7))
		distArr[0] = median;
}

void CollisionAvoidance::getDistance(uint8_t* distArr)
{
	for (uint8_t i = 0; i < RANGEFINDERS_NUMBER; i++)
		distArr[i] = this->outDistArr[i];
}

void CollisionAvoidance::run()
{
	InitUser.OnePulseModeInit();
	TickType_t xLastWakeTime;

	while(1) {
		xLastWakeTime = xTaskGetTickCount();

		for (uint8_t i = 0; i < RANGEFINDERS_NUMBER; i++)
			calculateDistanceQsort(sensDistArr[i], i);

		xSemaphoreTake(xRengefindersMutex, portMAX_DELAY);
		for (uint8_t i = 0; i < RANGEFINDERS_NUMBER; i++)
			this->outDistArr[i] = sensDistArr[i][0];
		xSemaphoreGive(xRengefindersMutex);

		TIM_SetCounter(TIM10, 0);								//  Reload timer to generate trigger signal.
		TIM_SetCounter(TIM6, 0);
		xTaskNotifyGive(this->xTaskToNotify);
		taskDelayUntil(&xLastWakeTime, oRTOS.fromMsToTick(MIN_RESPONCE_TIME));
	}
}

extern "C"
{
void EXTI15_10_IRQHandler(){
	if (EXTI_GetITStatus(EXTI_Line10)) {
		EXTI_ClearITPendingBit(EXTI_Line10);
		if (TIM_GetCounter(TIM6) > 500 && TIM_GetCounter(TIM6) < 3250)
			sensTimeArr[0] = TIM_GetCounter(TIM6);
		else
			sensTimeArr[0] = 0xff;
	} else if (EXTI_GetITStatus(EXTI_Line11) == 1) {
		EXTI_ClearITPendingBit(EXTI_Line11);
		if (TIM_GetCounter(TIM6) > 500 && TIM_GetCounter(TIM6) < 3250)
			sensTimeArr[1] = TIM_GetCounter(TIM6);
		else
			sensTimeArr[1] = 0xff;
	} else if (EXTI_GetITStatus(EXTI_Line12) == 1) {
		EXTI_ClearITPendingBit(EXTI_Line12);
		if (TIM_GetCounter(TIM6) > 500 && TIM_GetCounter(TIM6) < 3250)
			sensTimeArr[2] = TIM_GetCounter(TIM6);
		else
			sensTimeArr[2] = 0xff;
	} else if (EXTI_GetITStatus(EXTI_Line13) == 1) {
		EXTI_ClearITPendingBit(EXTI_Line13);
		if (TIM_GetCounter(TIM6) > 500 && TIM_GetCounter(TIM6) < 3250)
			sensTimeArr[3] = TIM_GetCounter(TIM6);
		else
			sensTimeArr[3] = 0xff;
	} else if (EXTI_GetITStatus(EXTI_Line14) == 1) {
		EXTI_ClearITPendingBit(EXTI_Line14);
		if (TIM_GetCounter(TIM6) > 500 && TIM_GetCounter(TIM6) < 3250)
			sensTimeArr[4] = TIM_GetCounter(TIM6);
		else
			sensTimeArr[4] = 0xff;
	} else if (EXTI_GetITStatus(EXTI_Line15) == 1) {
		EXTI_ClearITPendingBit(EXTI_Line15);
		if (TIM_GetCounter(TIM6) > 500 && TIM_GetCounter(TIM6) < 3250)
			sensTimeArr[5] = TIM_GetCounter(TIM6);
		else
			sensTimeArr[5] = 0xff;
	}
  }
}

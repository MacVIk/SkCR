/*
 * CollisionAvoidance.cpp
 *
 *  Created on: 15.01.2019
 *      Author: Taras.Melnik
 */

#include "CollisionAvoidance.h"

CollisionAvoidance *sensorTask;

uint8_t  interruptFlag[RANGEFINDERS_NUMBER];
uint16_t sensTimeArr[2][RANGEFINDERS_NUMBER];

CollisionAvoidance::CollisionAvoidance()
{
	xCollisionAvoidanceQueue = xQueueCreate(RANGEFINDERS_NUMBER, sizeof(uint8_t));
	xCollisionAvoidanceMutex = xSemaphoreCreateMutex();
	for (uint8_t i = 0; i < RANGEFINDERS_NUMBER; i++) {
		interruptFlag[i] = (ECHO_RISING_EDGE);
		this->sensDistArr[i] = new uint8_t[15];					//create array for distance history
	}
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
	timeBuff = (sensTimeArr[1][i] - sensTimeArr[0][i]);
	distArr[0] = (uint8_t) (timeBuff / 56);
	distArr[0] = (distArr[0] >= MAX_RANGEFINDERS_DISTANCE) ? (MAX_RANGEFINDERS_DISTANCE) : (distArr[0]);
	uint8_t sortArr[15];
	for (uint8_t j = 0; j < 15; j++)
		sortArr[j] = distArr[j];
	median = quick_select(sortArr, 0, 15, 7);
	for (uint8_t j = 14; j > 0; j--)
		distArr[j] = distArr[j - 1];
	if ((median - distArr[0] > 7) || (median - distArr[0] < -7))
		distArr[0] = median;
}

void CollisionAvoidance::run()
{
	InitUser.OnePulseModeInit();
	TickType_t xLastWakeTime;

	while(1) {
		GPIO_SetBits(GPIOD, GPIO_PinSource12);
		GPIO_ResetBits(GPIOD, GPIO_PinSource14);
		xLastWakeTime = xTaskGetTickCount();
		xSemaphoreTake(xCollisionAvoidanceMutex, portMAX_DELAY);
		if (uxQueueMessagesWaiting(xCollisionAvoidanceQueue) != 0)
					xQueueReset(xCollisionAvoidanceQueue);
		for (uint8_t i = 0; i < RANGEFINDERS_NUMBER; i++) {
			calculateDistanceQsort(sensDistArr[i], i);
			xQueueSend(xCollisionAvoidanceQueue, &sensDistArr[i][0], portMAX_DELAY);
			interruptFlag[i] = (ECHO_RISING_EDGE);				// Reset the interrupt state flag
		}														// for on fly connection.
		xSemaphoreGive(xCollisionAvoidanceMutex);
		TIM_SetCounter(TIM10, 0);								//  Reload timer to generate trigger signal.
		TIM_SetCounter(TIM6, 0);
		GPIO_SetBits(GPIOD, GPIO_PinSource14);
		GPIO_ResetBits(GPIOD, GPIO_PinSource12);
		taskDelayUntil(&xLastWakeTime, oRTOS.fromMsToTick(MIN_RESPONCE_TIME));
	}
}

extern "C"
{
void EXTI15_10_IRQHandler(){
	if (EXTI_GetITStatus(EXTI_Line10)) {
		EXTI_ClearITPendingBit(EXTI_Line10);
		sensTimeArr[interruptFlag[0]][0] = TIM_GetCounter(TIM6);
		interruptFlag[0] = (interruptFlag[0] == ECHO_RISING_EDGE) ? (ECHO_FALLING_EDGE) : (ECHO_RISING_EDGE);
	} else if (EXTI_GetITStatus(EXTI_Line11)) {
		EXTI_ClearITPendingBit(EXTI_Line11);
		sensTimeArr[interruptFlag[1]][1] = TIM_GetCounter(TIM6);
		interruptFlag[1] = (interruptFlag[1] == ECHO_RISING_EDGE) ? (ECHO_FALLING_EDGE) : (ECHO_RISING_EDGE);
	} else if (EXTI_GetITStatus(EXTI_Line12)) {
		EXTI_ClearITPendingBit(EXTI_Line12);
		sensTimeArr[interruptFlag[2]][2] = TIM_GetCounter(TIM6);
		interruptFlag[2] = (interruptFlag[2] == ECHO_RISING_EDGE) ? (ECHO_FALLING_EDGE) : (ECHO_RISING_EDGE);
	} else if (EXTI_GetITStatus(EXTI_Line13)) {
		EXTI_ClearITPendingBit(EXTI_Line13);
		sensTimeArr[interruptFlag[3]][3] = TIM_GetCounter(TIM6);
		interruptFlag[3] = (interruptFlag[3] == ECHO_RISING_EDGE) ? (ECHO_FALLING_EDGE) : (ECHO_RISING_EDGE);
	} else if (EXTI_GetITStatus(EXTI_Line14)) {
		EXTI_ClearITPendingBit(EXTI_Line14);
		sensTimeArr[interruptFlag[4]][4] = TIM_GetCounter(TIM6);
		interruptFlag[4] = (interruptFlag[4] == ECHO_RISING_EDGE) ? (ECHO_FALLING_EDGE) : (ECHO_RISING_EDGE);
	} else if (EXTI_GetITStatus(EXTI_Line15)) {
		EXTI_ClearITPendingBit(EXTI_Line15);
		sensTimeArr[interruptFlag[5]][5] = TIM_GetCounter(TIM6);
		interruptFlag[5] = (interruptFlag[5] == ECHO_RISING_EDGE) ? (ECHO_FALLING_EDGE) : (ECHO_RISING_EDGE);
	}
  }
}

/*
 * ReadEncoders.cpp
 *
 *  Created on: 02.07.2018
 *      Author: Taras.Melnik
 */

#include "ReadEncoders.h"

static QueueHandle_t xEncodersQueue;
static SemaphoreHandle_t xEncodersMutex;

ReadEncoders::ReadEncoders(){

	xEncodersQueue = xQueueCreate(2, sizeof(float));
	xEncodersMutex = xSemaphoreCreateMutex();

}

ReadEncoders::~ReadEncoders(){
}

void ReadEncoders::writeToQueue(float* inputArr)
{
	uint8_t i = 0;
//	xSemaphoreTake(xEncodersMutex, portMAX_DELAY);
	if (uxQueueMessagesWaiting(xEncodersQueue) != 0)
	{
		xQueueReset(xEncodersQueue);
	}
	for (i = 0; i < 2; i++)
	{
		xQueueSend(xEncodersQueue, &inputArr[i], 2);
	}
//	xSemaphoreGive(xEncodersMutex);
}

void ReadEncoders::readFromQueue(float &outputData)
{
//	xSemaphoreTake(xEncodersMutex, portMAX_DELAY);
	xQueueReceive(xEncodersQueue, &outputData, portMAX_DELAY);
//	xSemaphoreGive(xEncodersMutex);
}

float ReadEncoders::bubleSortMid(float velEnc[][10], uint8_t pos)
{
	for (uint8_t j = 0; j < 5; j++)
	{
		uint8_t buffPoint = 0;
		float buffArr = velEnc[pos][j];
		for ( uint8_t k = j; k < 10; k++)
		{
			buffArr = (buffArr <= velEnc[pos][j]) ?  velEnc[pos][j] : buffArr;
			buffPoint = k;
		}
		velEnc[pos][buffPoint] = velEnc[pos][j];
		velEnc[pos][j] = buffArr;
	}
	return velEnc[pos][4];
}

void ReadEncoders::run(){

	TickType_t xLastWakeTime;
	float T0 = 0;
	uint16_t ticksCur = 0;
	float delTick = 0;
	float delTime = 0;
	float velEnc[2][ENC_HISTORY_ARR];
	uint8_t i, j = 0;
	TIM_TypeDef* timArr[2] = { TIM3, TIM4 };

	float alf[ENC_HISTORY_ARR];
	float vel_f[2];
	float testArr[ENC_HISTORY_ARR];

	alf[0] = 1;
//	alf[1] = OPPOSIT_SMOOTHING_FACTOR;
	for ( i = 1; i < ENC_HISTORY_ARR; i++)
	{
		alf[i] = alf[i - 1] * OPPOSIT_SMOOTHING_FACTOR;
	}
//	float midVelEnc;
	for (i = 0; i < ENC_HISTORY_ARR; i++)
	{
		velEnc[0][i] = 0;
		velEnc[1][i] = 0;
		testArr[i] = 0;
	}

	while (1)
	{
//		GPIO_SetBits(GPIOC, GPIO_Pin_14);
		xLastWakeTime = xTaskGetTickCount();

		for (i = 0; i < 1; i++)
		{
			T0 = TIM7->CNT;
			delTime = T0 / TIMER_RESOLUTION;
			ticksCur = timArr[i]->CNT;
			delTick = (float) (ticksCur - PRIMARY_COUNTER_VALUE);
			velEnc[i][0] = ((delTick /delTime) * ENCODER_RESOLUTION);

//			midVelEnc = bubleSortMid( velEnc, i);
//			midVelEnc = (midVelEnc > 0) ? (midVelEnc) : (-midVelEnc);
//			velEnc[i][0] = (velEnc[i][0] > 0) ? (velEnc[i][0]) : (-velEnc[i][0]);
			vel_f[i] = velEnc[i][0];

			if ((velEnc[i][1] != 0) && (velEnc[i][2] != 0) && (velEnc[i][3] != 0))
			{
				for ( j = 1; j < ENC_HISTORY_ARR - 1; j++)
				{
					vel_f[i] += velEnc[i][j] * alf[j];
				}
				vel_f[i] = vel_f[i] * (1 - OPPOSIT_SMOOTHING_FACTOR) + velEnc[i][ENC_HISTORY_ARR - 1] * alf[ENC_HISTORY_ARR - 1];
			}
			velEnc[i][0] = vel_f[i];
			testArr[0] = vel_f[i];
			for ( j = ENC_HISTORY_ARR - 1; j > 0; j--)
			{
				velEnc[i][j] = velEnc[i][j - 1];
				testArr[j] = testArr[j-1];
			}
		}
		TIM4->CNT = PRIMARY_COUNTER_VALUE;
		TIM3->CNT = PRIMARY_COUNTER_VALUE;
		TIM7->CNT = 0x00;

		this->writeToQueue(vel_f);
//		GPIO_ResetBits(GPIOC, GPIO_Pin_14);
//		this->taskDelay(oRTOS.fromMsToTick(100));
		this->taskDelayUntil(&xLastWakeTime, oRTOS.fromMsToTick(100));
	}
}





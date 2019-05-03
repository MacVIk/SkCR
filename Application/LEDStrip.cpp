/*
 * LEDStrip.cpp
 *
 *  Created on: 28.10.2018
 *      Author: Taras.Melnik
 */

#include "LEDStrip.h"

LEDStrip* setLEDTask;

LEDStrip::LEDStrip() {
	uint8_t movementFlag = SELF_DRIVING;
	uint8_t color = 3;
	TIM5->CCR3 = 0;
	GPIOA->ODR |= GPIO_ODR_ODR_3;
	GPIOA->ODR &= ~GPIO_ODR_ODR_15;
//	GPIOD->ODR |= GPIO_ODR_ODR_5;
	xLightColorQueue = xQueueCreate(1, sizeof(uint8_t));
	xQueueOverwrite(xLightColorQueue, &color);
}

LEDStrip::~LEDStrip() {
}

void LEDStrip::writeToQueue(uint8_t data)
{
	xQueueOverwrite(xLightColorQueue, &data);
}

void LEDStrip::setColor(uint8_t color)
{
	if (color == 1) {
		TIM5->CCR3 = 2800;
		GPIOA->ODR &= ~GPIO_ODR_ODR_3;
		GPIOA->ODR &= ~GPIO_ODR_ODR_15;
	} else if (color == 2) {
		TIM5->CCR3 = 0;
		GPIOA->ODR &= ~GPIO_ODR_ODR_3;
		GPIOA->ODR |= GPIO_ODR_ODR_15;
	} else if (color == 3) {
		TIM5->CCR3 = 0;
		GPIOA->ODR |= GPIO_ODR_ODR_3;
		GPIOA->ODR &= ~GPIO_ODR_ODR_15;
	} else if (color == 4) {
		TIM5->CCR3 = 2800;
		GPIOA->ODR |= GPIO_ODR_ODR_3;
		GPIOA->ODR |= GPIO_ODR_ODR_15;
	} else {
		TIM5->CCR3 = 0;
		GPIOA->ODR &= ~GPIO_ODR_ODR_3;
		GPIOA->ODR &= ~GPIO_ODR_ODR_15;
	}
}

void LEDStrip::run()
{
	uint8_t color = 0;
	uint8_t movementFlag = SELF_DRIVING;
	while (1) {
		if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_15) == Bit_SET) {
			if (uxQueueMessagesWaiting(xLightColorQueue) != 0) {
				xQueueReceive(xLightColorQueue, &color, 0);
				setColor(color);
			} else if (movementFlag == MANUAL_CONTROL) {
				setColor(color);
				movementFlag = SELF_DRIVING;
//				xTaskNotifyGive(hyroMotor->taskHandle);    // Give notification(HyroMotTask) EmergencyButt
			}
		} else {
			if (movementFlag == SELF_DRIVING) {
				GPIOA->ODR &= ~GPIO_ODR_ODR_15;
				GPIOA->ODR &= ~GPIO_ODR_ODR_3;
				TIM5->CCR3 = 1400;
				movementFlag = MANUAL_CONTROL;
			}
		}
		this->taskDelay(oRTOS.fromMsToTick(1));
	}
}



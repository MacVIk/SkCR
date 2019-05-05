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
	uint8_t color = BLUE;
	TIM5->CCR3 = 0;
	GPIOA->ODR |= GPIO_ODR_ODR_3;
	GPIOA->ODR &= ~GPIO_ODR_ODR_15;
//	GPIOD->ODR |= GPIO_ODR_ODR_5;
}

LEDStrip::~LEDStrip() {
}

void LEDStrip::setColor(uint8_t color)
{
	if (color == RED) {
		TIM5->CCR3 = 2800;
		GPIOA->ODR &= ~GPIO_ODR_ODR_3;
		GPIOA->ODR &= ~GPIO_ODR_ODR_15;
	} else if (color == GREEN) {
		TIM5->CCR3 = 0;
		GPIOA->ODR &= ~GPIO_ODR_ODR_3;
		GPIOA->ODR |= GPIO_ODR_ODR_15;
	} else if (color == BLUE) {
		TIM5->CCR3 = 0;
		GPIOA->ODR |= GPIO_ODR_ODR_3;
		GPIOA->ODR &= ~GPIO_ODR_ODR_15;
	} else if (color == WHITE) {
		TIM5->CCR3 = 2800;
		GPIOA->ODR |= GPIO_ODR_ODR_3;
		GPIOA->ODR |= GPIO_ODR_ODR_15;
	} else if (color == RED_FLASH) {
		TIM5->CCR3 = 1400;
		GPIOA->ODR &= ~GPIO_ODR_ODR_3;
		GPIOA->ODR &= ~GPIO_ODR_ODR_15;
	} else {
		TIM5->CCR3 = 0;
		GPIOA->ODR &= ~GPIO_ODR_ODR_3;
		GPIOA->ODR &= ~GPIO_ODR_ODR_15;
	}
}




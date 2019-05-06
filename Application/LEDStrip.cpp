/*
 * LEDStrip.cpp
 *
 *  Created on: 28.10.2018
 *      Author: Taras.Melnik
 */

#include "LEDStrip.h"

LEDStrip* setLEDTask;

LEDStrip::LEDStrip() {
	setColor(BLUE);
	this->errFlag = false;
}

LEDStrip::~LEDStrip() {
}
void LEDStrip::setError()
{
	setColor(RED_FLASH);
	this->errFlag = true;
}

void LEDStrip::clearError() {
	this->errFlag = false;
	setColor(BLUE);
}

void LEDStrip::setColor(uint8_t color)
{
	if(errFlag != true) {
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
}




/*
 * RGB.cpp
 *
 *  Created on: 03.08.2018
 *      Author: Fazli
 */

#include "RGB.h"

RGB::RGB(float32_t maxValue, float32_t minValue) :
		pwmr(GPIOA, 15, TIM2, 1), pwmg(GPIOA, 3, TIM2, 4), pwmb(GPIOA, 2, TIM2, 3) {

	this->minValue = minValue;
	this->maxValue = maxValue;

	this->pwmr.init(1209, 1100);
	this->pwmg.init(1209, 1100);
	this->pwmb.init(1209, 1100);

	setRed(10);
	setGreen(10);
	setBlue(10);
}

RGB::~RGB() {
	// TODO Auto-generated destructor stub
}

void RGB::setRed(float32_t redPercents) {
	float32_t tmp = (redPercents) * (this->maxValue - this->minValue) / 100.0
			+ this->minValue;
	pwmr.setWidthPercents(tmp);
	pwmr.start();
}

void RGB::setGreen(float32_t greenPercents) {
	float32_t tmp = (greenPercents) * (this->maxValue - this->minValue) / 100.0
			+ this->minValue;
	pwmg.setWidthPercents(tmp);
	pwmg.start();
}

void RGB::setBlue(float32_t bluePercents) {
	float32_t tmp = (bluePercents) * (this->maxValue - this->minValue) / 100.0
			+ this->minValue;
	pwmb.setWidthPercents(tmp);
	pwmb.start();
}


/*
 * RGB.h
 *
 *  Created on: 03.08.2018
 *      Author: Fazli
 */

#ifndef RGB_H_
#define RGB_H_

#include "PeripheralDrivers/PWM.h"

class RGB {
private:
	float32_t minValue;
	float32_t maxValue;

	PWM pwmr;
	PWM pwmg;
	PWM pwmb;
public:
	RGB(float32_t maxValue = 0, float32_t minValue = 100);
	virtual ~RGB();

	void setRed(float32_t intencityPercents);
	void setGreen(float32_t intencityPercents);
	void setBlue(float32_t intencityPercents);
};

#endif /* RGB_H_ */

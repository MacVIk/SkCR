/*
 * LEDStrip.h
 *
 *  Created on: 28.10.2018
 *      Author: Taras.Melnik
 */

#ifndef LEDSTRIP_H_
#define LEDSTRIP_H_

#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"

#define MANUAL_CONTROL	((uint8_t)0xff)
#define SELF_DRIVING	((uint8_t)0x00)

enum {
	RED = 0x01,
	GREEN = 0X02,
	BLUE = 0x03,
	WHITE = 0x04,
	TURN_OFF = 0x05,
	RED_FLASH = 0x06
};

class LEDStrip {
public:
	LEDStrip();
	virtual ~LEDStrip();
	void setColor(uint8_t color);
	void run();
};

extern LEDStrip* setLEDTask;

#endif /* LEDSTRIP_H_ */

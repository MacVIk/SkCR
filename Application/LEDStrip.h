/*
 * LEDStrip.h
 *
 *  Created on: 28.10.2018
 *      Author: Taras.Melnik
 */

#ifndef LEDSTRIP_H_
#define LEDSTRIP_H_

#include "iActiveObject.h"
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "semphr.h"
#include "queue.h"
#include "CollisionHandler.h"

#define MANUAL_CONTROL	((uint8_t)0xff)
#define SELF_DRIVING	((uint8_t)0x00)

#define RED				((uint8_t)0x01)
#define GREEN			((uint8_t)0x02)
#define BLUE			((uint8_t)0x03)
#define WHITE			((uint8_t)0x04)

class LEDStrip: public iActiveObject {
public:
	LEDStrip();
	virtual ~LEDStrip();
	void writeToQueue(uint8_t data);
	void setColor(uint8_t color);
	void run();
};

extern LEDStrip* setLEDTask;

#endif /* LEDSTRIP_H_ */

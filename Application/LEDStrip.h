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

#define MANUAL_CONTROL	((uint8_t)0xff)
#define SELF_DRIVING	((uint8_t)0x00)

class LEDStrip: public iActiveObject {
public:
	LEDStrip();
	virtual ~LEDStrip();
	void writeToQueue(uint8_t data);
	void setColor(uint8_t color);
	void run();
private:
	QueueHandle_t xLightColorQueue;
};

extern LEDStrip* setLEDTask;

#endif /* LEDSTRIP_H_ */

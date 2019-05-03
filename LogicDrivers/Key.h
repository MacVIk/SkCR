/*
 * Key.h
 *
 *  Created on: 13.01.2016
 *      Author: Tata
 */

#ifndef KEY_H_
#define KEY_H_

#include "PeripheralDrivers/PortManager.h"

class Key {
protected:
	GPIO_TypeDef* port;
	uint16_t pinMask;
	uint8_t state;
public:
	Key(GPIO_TypeDef* port, uint16_t pinMask, bool isHighActive);
	virtual ~Key();

	void turnOn();
	void turnOff();
	void setState(bool newState);
	bool getState();

	bool isActive();
	void setActive(bool newActive);

	bool isHighActive();
	void setHighActive();
};

#endif /* KEY_H_ */

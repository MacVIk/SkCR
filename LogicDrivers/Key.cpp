/*
 * Key.cpp
 *
 *  Created on: 13.01.2016
 *      Author: Tata
 */

#include "Key.h"

Key::Key(GPIO_TypeDef* _port, uint16_t _pinMask, bool highActive) {
	PortManager::initPins(_port, _pinMask, GPIO_Mode_OUT);

	state = 0;
	port = _port;
	pinMask = _pinMask;

	if (highActive) {
		this->setHighActive();
	}

	this->turnOff();
}

Key::~Key() {

}

bool Key::isActive() {
	if (state & 1) {
		return true;
	} else {
		return false;
	}
}

void Key::setActive(bool newActive) {
	if (newActive) {
		state |= 0b1;
	} else {
		turnOff();
		state &= ~(0b1);
	}
}

bool Key::isHighActive() {
	if (state & 0b1000) {
		return true;
	} else {
		return false;
	}
}

void Key::turnOn() {
	if (isActive()) {
		if (isHighActive()) {
			PortManager::setPins(port, pinMask);
		} else {
			PortManager::resetPins(port, pinMask);
		}
	}
}

void Key::turnOff() {
	if (isActive()) {
		if (isHighActive()) {
			PortManager::resetPins(port, pinMask);
		} else {
			PortManager::setPins(port, pinMask);
		}
	}
}

void Key::setState(bool newState) {
	if (newState) {
		turnOn();
	} else {
		turnOff();
	}
}

bool Key::getState() {
	return PortManager::readPinsOutput(port, pinMask) != 0;
}

void Key::setHighActive() {
	state |= 0b1000;
}

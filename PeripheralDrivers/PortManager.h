/*
 * PortManager.h
 *
 *  Created on: 12.08.2015
 *      Author: Tata
 */

#ifndef PORTMANAGER_H_
#define PORTMANAGER_H_

#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"

class PortManager {
public:
	static void initPort(GPIO_TypeDef *port, GPIOMode_TypeDef mode,
			GPIOSpeed_TypeDef speed, GPIOPuPd_TypeDef pupd,
			GPIOOType_TypeDef oType);
	static void initPort(GPIO_TypeDef *port, GPIOMode_TypeDef mode);
	static void initPin(GPIO_TypeDef *port, uint8_t pinNumber,
			GPIOMode_TypeDef mode, GPIOSpeed_TypeDef speed,
			GPIOPuPd_TypeDef pupd, GPIOOType_TypeDef oType);
	static void initPin(GPIO_TypeDef *port, uint8_t pinNumber,
			GPIOMode_TypeDef mode);
	static void initPins(GPIO_TypeDef *port, uint32_t pinMask,
			GPIOMode_TypeDef mode, GPIOSpeed_TypeDef speed,
			GPIOPuPd_TypeDef pupd, GPIOOType_TypeDef oType);
	static void initPins(GPIO_TypeDef *port, uint32_t pinMask,
			GPIOMode_TypeDef mode);
	static void pinAFConfig(GPIO_TypeDef *port, uint8_t pin, uint8_t altFunc);

	static void setPort(GPIO_TypeDef *port);
	static void resetPort(GPIO_TypeDef *port);
	static void togglePort(GPIO_TypeDef *port);

	static void setPin(GPIO_TypeDef *port, uint8_t pin);
	static void resetPin(GPIO_TypeDef *port, uint8_t pin);
	static void togglePin(GPIO_TypeDef *port, uint8_t pin);

	static void setPins(GPIO_TypeDef *port, uint32_t pins);
	static void resetPins(GPIO_TypeDef *port, uint32_t pins);
	static void togglePins(GPIO_TypeDef *port, uint32_t pins);

	static void writePort(GPIO_TypeDef *port, uint32_t value);
	static void writePin(GPIO_TypeDef *port, uint8_t pinNumber, uint8_t value);

	static uint8_t readPinInput(GPIO_TypeDef *port, uint8_t pin);
	static uint8_t readPinOutput(GPIO_TypeDef *port, uint8_t pin);
	static uint32_t readPinsOutput(GPIO_TypeDef *port, uint32_t pinMask);
	static uint16_t readPortInput(GPIO_TypeDef *port);
	static uint16_t readPortOutput(GPIO_TypeDef *port);

	static void initPinInterrupt(GPIO_TypeDef* port, uint8_t pin,
			EXTITrigger_TypeDef triggerType = EXTI_Trigger_Rising,
			uint8_t priority = 4);
};

#endif /* PORTMANAGER_H_ */

/*
 * PortManager.cpp
 *
 *  Created on: 12.08.2015
 *      Author: Tata
 */

#include "PortManager.h"

void PortManager::initPort(GPIO_TypeDef *port, GPIOMode_TypeDef mode,
		GPIOSpeed_TypeDef speed, GPIOPuPd_TypeDef pupd,
		GPIOOType_TypeDef oType) {
	uint32_t pinpos = 0x00, pos = 0x00, currentpin = 0x00;

	/* -------------------------Configure the port pins---------------- */
	/*-- GPIO Mode Configuration --*/
	for (pinpos = 0x00; pinpos < 0x10; pinpos++) {
		pos = ((uint32_t) 0x01) << pinpos;
		/* Get the port pins position */
		currentpin = (0xffff) & pos;

		if (currentpin == pos) {
			port->MODER &= ~(GPIO_MODER_MODER0 << (pinpos * 2));
			port->MODER |= (((uint32_t) mode) << (pinpos * 2));

			if ((mode == GPIO_Mode_OUT) || (mode == GPIO_Mode_AF)) {

				/* Speed mode configuration */
				port->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << (pinpos * 2));
				port->OSPEEDR |= ((uint32_t) (speed) << (pinpos * 2));

				/* Output mode configuration*/port->OTYPER &=
						~((GPIO_OTYPER_OT_0) << ((uint16_t) pinpos));
				port->OTYPER |= (uint16_t) (((uint16_t) oType)
						<< ((uint16_t) pinpos));
			}

			/* Pull-up Pull down resistor configuration*/port->PUPDR &=
					~(GPIO_PUPDR_PUPDR0 << ((uint16_t) pinpos * 2));
			port->PUPDR |= (((uint32_t) pupd) << (pinpos * 2));
		}
	}
}

void PortManager::initPort(GPIO_TypeDef *port, GPIOMode_TypeDef mode) {
	initPort(port, mode, GPIO_Speed_25MHz, GPIO_PuPd_NOPULL, GPIO_OType_PP);
}

void PortManager::initPin(GPIO_TypeDef *port, uint8_t pinNumber,
		GPIOMode_TypeDef mode, GPIOSpeed_TypeDef speed, GPIOPuPd_TypeDef pupd,
		GPIOOType_TypeDef oType) {
	uint32_t pinpos = 0x00, pos = 0x00, currentpin = 0x00;

	uint32_t pin = (1 << pinNumber);
	/* -------------------------Configure the port pins---------------- */
	/*-- GPIO Mode Configuration --*/
	for (pinpos = 0x00; pinpos < 0x10; pinpos++) {
		pos = ((uint32_t) 0x01) << pinpos;
		/* Get the port pins position */
		currentpin = (pin) & pos;

		if (currentpin == pos) {
			port->MODER &= ~(GPIO_MODER_MODER0 << (pinpos * 2));
			port->MODER |= (((uint32_t) mode) << (pinpos * 2));

			if ((mode == GPIO_Mode_OUT) || (mode == GPIO_Mode_AF)) {

				/* Speed mode configuration */
				port->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << (pinpos * 2));
				port->OSPEEDR |= ((uint32_t) (speed) << (pinpos * 2));

				/* Output mode configuration*/port->OTYPER &=
						~((GPIO_OTYPER_OT_0) << ((uint16_t) pinpos));
				port->OTYPER |= (uint16_t) (((uint16_t) oType)
						<< ((uint16_t) pinpos));
			}

			/* Pull-up Pull down resistor configuration*/port->PUPDR &=
					~(GPIO_PUPDR_PUPDR0 << ((uint16_t) pinpos * 2));
			port->PUPDR |= (((uint32_t) pupd) << (pinpos * 2));
		}
	}
}

void PortManager::initPin(GPIO_TypeDef *port, uint8_t pinNumber,
		GPIOMode_TypeDef mode) {
	initPin(port, pinNumber, mode, GPIO_Speed_25MHz, GPIO_PuPd_NOPULL,
			GPIO_OType_PP);
}

void PortManager::initPins(GPIO_TypeDef *port, uint32_t pinMask,
		GPIOMode_TypeDef mode, GPIOSpeed_TypeDef speed, GPIOPuPd_TypeDef pupd,
		GPIOOType_TypeDef oType) {
	taskENTER_CRITICAL();
	uint32_t pinpos = 0x00, pos = 0x00, currentpin = 0x00;

	uint32_t pin = pinMask;
	uint8_t counter = 0;
	;
	/* -------------------------Configure the port pins---------------- */
	/*-- GPIO Mode Configuration --*/
	for (pinpos = 0x00; pinpos < 0x10; pinpos++) {
		pos = ((uint32_t) 0x01) << pinpos;
		/* Get the port pins position */
		currentpin = (pin) & pos;

		if (currentpin == pos) {
			counter++;
			port->MODER &= ~(GPIO_MODER_MODER0 << (pinpos * 2));
			port->MODER |= (((uint32_t) mode) << (pinpos * 2));

			if ((mode == GPIO_Mode_OUT) || (mode == GPIO_Mode_AF)) {

				/* Speed mode configuration */
				port->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << (pinpos * 2));
				port->OSPEEDR |= ((uint32_t) (speed) << (pinpos * 2));

				/* Output mode configuration*/port->OTYPER &=
						~((GPIO_OTYPER_OT_0) << ((uint16_t) pinpos));
				port->OTYPER |= (uint16_t) (((uint16_t) oType)
						<< ((uint16_t) pinpos));
			}

			/* Pull-up Pull down resistor configuration*/port->PUPDR &=
					~(GPIO_PUPDR_PUPDR0 << ((uint16_t) pinpos * 2));
			port->PUPDR |= (((uint32_t) pupd) << (pinpos * 2));
		}
	}
	taskEXIT_CRITICAL();
}

void PortManager::initPins(GPIO_TypeDef *port, uint32_t pinMask,
		GPIOMode_TypeDef mode) {
	initPins(port, pinMask, mode, GPIO_Speed_25MHz, GPIO_PuPd_NOPULL,
			GPIO_OType_PP);
}

void PortManager::pinAFConfig(GPIO_TypeDef *port, uint8_t pinNumber,
		uint8_t altFunc) {
	if (pinNumber < 16) {
		if (altFunc < 15) {
			uint32_t temp = 0x00;
			uint32_t temp_2 = 0x00;

			temp =
					((uint32_t) (altFunc)
							<< ((uint32_t) ((uint32_t) pinNumber
									& (uint32_t) 0x07) * 4));
			port->AFR[pinNumber >> 0x03] &=
					~((uint32_t) 0xF
							<< ((uint32_t) ((uint32_t) pinNumber
									& (uint32_t) 0x07) * 4));
			temp_2 = port->AFR[pinNumber >> 0x03] | temp;
			port->AFR[pinNumber >> 0x03] = temp_2;
		}
	}
}

void PortManager::setPin(GPIO_TypeDef *port, uint8_t pinNumber) {
	if (pinNumber < 16) {
		uint32_t pin = (1 << pinNumber);
		port->BSRRL = pin;
	}
}

void PortManager::setPort(GPIO_TypeDef *port) {
	uint16_t pin = 0xffff;
	port->BSRRL = pin;
}

void PortManager::resetPort(GPIO_TypeDef *port) {
	uint16_t pin = 0x0000;
	port->BSRRL = pin;
}

void PortManager::resetPin(GPIO_TypeDef *port, uint8_t pinNumber) {
	if (pinNumber < 16) {
		uint32_t pin = (1 << pinNumber);
		port->BSRRH = pin;
	}
}

void PortManager::writePort(GPIO_TypeDef *port, uint32_t value) {
	if (value < 0x10000) {
		port->ODR = value;
	}
}

void PortManager::writePin(GPIO_TypeDef *port, uint8_t pinNumber,
		uint8_t value) {
	if (pinNumber < 16) {
		uint32_t pin = (1 << pinNumber);
		if (value == 0) {
			port->ODR &= ~pin;
		} else {
			port->ODR |= pin;
		}
	}
}

void PortManager::togglePin(GPIO_TypeDef *port, uint8_t pinNumber) {
	if (pinNumber < 16) {
		uint32_t pin = (1 << pinNumber);
		port->ODR ^= pin;
	}
}

void PortManager::setPins(GPIO_TypeDef *port, uint32_t pins) {
	if (pins < 0x10000) {
		port->BSRRL |= pins;
	}
}

void PortManager::resetPins(GPIO_TypeDef *port, uint32_t pins) {
	if (pins < 0x10000) {
		port->BSRRH |= pins;
	}
}

void PortManager::togglePins(GPIO_TypeDef *port, uint32_t pins) {
	if (pins < 0x10000) {
		port->ODR ^= pins;
	}
}

uint16_t PortManager::readPortInput(GPIO_TypeDef *port) {
	return ((uint16_t) port->IDR);
}

uint16_t PortManager::readPortOutput(GPIO_TypeDef *port) {
	return ((uint16_t) port->ODR);
}

uint8_t PortManager::readPinInput(GPIO_TypeDef *port, uint8_t pinNumber) {
	if (pinNumber < 16) {
		if ((port->IDR & (1 << pinNumber)) != 0) {
			return 1;
		}
	}

	return 0;
}

uint8_t PortManager::readPinOutput(GPIO_TypeDef *port, uint8_t pinNumber) {
	if (pinNumber < 16) {
		if ((port->ODR & (1 << pinNumber)) != 0) {
			return 1;
		}
	}

	return 0;
}

uint32_t PortManager::readPinsOutput(GPIO_TypeDef *port, uint32_t pinMask) {
	if (pinMask < 0x10000) {
		return ((port->ODR) & pinMask);
	}

	return 0;
}

void PortManager::initPinInterrupt(GPIO_TypeDef* port, uint8_t pin,
		EXTITrigger_TypeDef triggerType, uint8_t priority) {
	uint8_t extiPort = 0;
	uint8_t extiPin = 0;
	uint32_t extiLine = 0;
	uint8_t nvicChannel = 0;

	if (port == GPIOA) {
		extiPort = EXTI_PortSourceGPIOA;
	} else if (port == GPIOB) {
		extiPort = EXTI_PortSourceGPIOB;
	} else if (port == GPIOC) {
		extiPort = EXTI_PortSourceGPIOC;
	} else if (port == GPIOD) {
		extiPort = EXTI_PortSourceGPIOD;
	} else if (port == GPIOE) {
		extiPort = EXTI_PortSourceGPIOE;
	} else if (port == GPIOF) {
		extiPort = EXTI_PortSourceGPIOF;
	} else if (port == GPIOG) {
		extiPort = EXTI_PortSourceGPIOG;
	} else if (port == GPIOH) {
		extiPort = EXTI_PortSourceGPIOH;
	} else if (port == GPIOI) {
		extiPort = EXTI_PortSourceGPIOI;
	}

	switch (pin) {
	case 0:
		extiPin = EXTI_PinSource0;
		extiLine = EXTI_Line0;
		nvicChannel = EXTI0_IRQn;
		break;
	case 1:
		extiPin = EXTI_PinSource1;
		extiLine = EXTI_Line1;
		nvicChannel = EXTI1_IRQn;
		break;
	case 2:
		extiPin = EXTI_PinSource2;
		extiLine = EXTI_Line2;
		nvicChannel = EXTI2_IRQn;
		break;
	case 3:
		extiPin = EXTI_PinSource3;
		extiLine = EXTI_Line3;
		nvicChannel = EXTI3_IRQn;
		break;
	case 4:
		extiPin = EXTI_PinSource4;
		extiLine = EXTI_Line4;
		nvicChannel = EXTI4_IRQn;
		break;
	case 5:
		extiPin = EXTI_PinSource5;
		extiLine = EXTI_Line5;
		nvicChannel = EXTI9_5_IRQn;
		break;
	case 6:
		extiPin = EXTI_PinSource6;
		extiLine = EXTI_Line6;
		nvicChannel = EXTI9_5_IRQn;
		break;
	case 7:
		extiPin = EXTI_PinSource7;
		extiLine = EXTI_Line7;
		nvicChannel = EXTI9_5_IRQn;
		break;
	case 8:
		extiPin = EXTI_PinSource8;
		extiLine = EXTI_Line8;
		nvicChannel = EXTI9_5_IRQn;
		break;
	case 9:
		extiPin = EXTI_PinSource9;
		extiLine = EXTI_Line9;
		nvicChannel = EXTI9_5_IRQn;
		break;
	case 10:
		extiPin = EXTI_PinSource10;
		extiLine = EXTI_Line10;
		nvicChannel = EXTI15_10_IRQn;
		break;
	case 11:
		extiPin = EXTI_PinSource11;
		extiLine = EXTI_Line11;
		nvicChannel = EXTI15_10_IRQn;
		break;
	case 12:
		extiPin = EXTI_PinSource12;
		extiLine = EXTI_Line12;
		nvicChannel = EXTI15_10_IRQn;
		break;
	case 13:
		extiPin = EXTI_PinSource13;
		extiLine = EXTI_Line13;
		nvicChannel = EXTI15_10_IRQn;
		break;
	case 14:
		extiPin = EXTI_PinSource14;
		extiLine = EXTI_Line14;
		nvicChannel = EXTI15_10_IRQn;
		break;
	case 15:
		extiPin = EXTI_PinSource15;
		extiLine = EXTI_Line15;
		nvicChannel = EXTI15_10_IRQn;
		break;
	}

	SYSCFG_EXTILineConfig(extiPort, extiPin);

	EXTI_InitTypeDef EXTI_InitStruct;

	EXTI_InitStruct.EXTI_Line = extiLine;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = triggerType;
	EXTI_Init(&EXTI_InitStruct);

	NVIC_InitTypeDef NVIC_InitStruct;
	NVIC_InitStruct.NVIC_IRQChannel = nvicChannel;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = priority;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
}

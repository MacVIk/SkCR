/*
 * UARTuserInit.h
 *
 *  Created on: 03.04.2019
 *      Author: Taras.Melnik
 */

#ifndef UARTUSERINIT_H_
#define UARTUSERINIT_H_

#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"

class UARTuserInit {
public:
	UARTuserInit();
	virtual ~UARTuserInit();
	void gpioInit(GPIO_TypeDef* GPIOx, uint16_t rxPin, uint16_t txPin, uint8_t af);
	void dmaInit(USART_TypeDef* USARTx);
	void uartInit(GPIO_TypeDef* GPIOx, USART_TypeDef* USARTx, bool rsStatus);
	void gpioSwitchInit(GPIO_TypeDef* GPIOx, uint16_t swPin);
//	void UARTx_IRQnFunc();
	void send(uint8_t* sendArr, uint8_t length);
	uint8_t receiveByte(uint8_t &byte);
	void resetPointer();
	uint8_t usartRxArr[20];
	uint8_t usartTxArr[20];
private:
//	USART_TypeDef* nvicUart;
	GPIO_TypeDef* nvicPort;
	uint16_t nvicPin;
	DMA_Stream_TypeDef* txStream;
	uint8_t pointer;

};

#endif /* UARTUSERINIT_H_ */

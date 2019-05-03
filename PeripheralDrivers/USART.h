/*
 * USART.h
 *
 *  Created on: 08.08.2017
 *      Author: adozzer
 */

#ifndef USART_H_
#define USART_H_

#include "stm32f4xx_usart.h"
#include "FreeRtos/wrapper/iTimerObject.h"
#include "FreeRtos/wrapper/iActiveObject.h"
#include "FreeRtos/include/semphr.h"

#include "PeripheralDrivers/PortManager.h"

struct USARTMessage {
	uint8_t length;
	uint8_t curPos;
	uint8_t data[255];
	TickType_t timestamp;
};

class USART: public iTimerObject, public iActiveObject {
public:
	USARTMessage receiveMessage;
	USARTMessage transmittMessage;
	SemaphoreHandle_t transmitterMutex;
	SemaphoreHandle_t transmitterSemaphore;

	QueueHandle_t queueSend;
	QueueHandle_t queueReceive;

private:
	GPIO_TypeDef *portRX;
	GPIO_TypeDef* portTX;
	uint8_t pinNumberTX;
	uint8_t pinNumberRX;
	USART_TypeDef* USARTx;
	bool isBurst;
public:
	void IRQHandler();

	void init(uint32_t baudRate, uint16_t wordLength = USART_WordLength_8b,
			uint16_t stopBits = USART_StopBits_1, uint16_t parity =
					USART_Parity_No,
			uint16_t mode = USART_Mode_Tx | USART_Mode_Rx,
			uint16_t hardwareFlowControl = USART_HardwareFlowControl_None);

	USART(USART_TypeDef* USARTx, GPIO_TypeDef *portRX = 0, uint8_t pinNumberRX =
			0, GPIO_TypeDef *portTX = 0, uint8_t pinNumberTX = 0);

	void configurePorts(GPIO_TypeDef *portRX, uint8_t pinNumberRX,
			GPIO_TypeDef *portTX, uint8_t pinNumberTX);

	void startTransmittion(USARTMessage message, TickType_t maxBlockTime);

	void send(USARTMessage* umessage, bool isBurst);

	void setReceive(FunctionalState state);

	void timerCallbackFunction();
	void run();

	virtual ~USART();
};

#endif /* USART_H_ */

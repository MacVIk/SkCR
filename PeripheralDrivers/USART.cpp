/*
 * USART.cpp
 *
 *  Created on: 08.08.2017
 *      Author: adozzer
 *
 *      !!! added GPIOB,12 for MAX485 send/receive control !!!
 */

#include "USART.h"

USART* usart3;

void USART::IRQHandler() {
	BaseType_t xHigherPriorityTaskWoken = 0;

	if (USART_GetITStatus(USARTx, USART_IT_TC) != RESET) {
		if (transmittMessage.curPos == transmittMessage.length) {
			xSemaphoreGiveFromISR(transmitterSemaphore,
					&xHigherPriorityTaskWoken);
			if (this->isBurst) {
				this->setReceive(ENABLE);
				this->isBurst = false;
			}
			PortManager::resetPin(GPIOB,12);
		} else {
			USART_SendData(USARTx,
					transmittMessage.data[transmittMessage.curPos]);
			transmittMessage.curPos++;
		}
		USART_ClearITPendingBit(USARTx, USART_IT_TC);
	}
	if (USART_GetITStatus(USARTx, USART_IT_RXNE) != RESET) {
		USART_ClearITPendingBit(USARTx, USART_IT_RXNE);
		uint16_t tmpr = USART_ReceiveData(USARTx);
		receiveMessage.data[receiveMessage.curPos] = (uint8_t) tmpr;
		receiveMessage.curPos++;
		receiveMessage.timestamp = xTaskGetTickCountFromISR();
		this->timerStartFromISR(&xHigherPriorityTaskWoken);
	}

}

void USART::init(uint32_t baudRate, uint16_t wordLength, uint16_t stopBits,
		uint16_t parity, uint16_t mode, uint16_t hardwareFlowControl) {

	PortManager::initPin(GPIOB, 12, GPIO_Mode_OUT);/////////////////////////////////////

	USART_OverSampling8Cmd(USARTx, DISABLE);

	PortManager::initPin(portRX, pinNumberRX, GPIO_Mode_AF, GPIO_Speed_50MHz,
			GPIO_PuPd_UP, GPIO_OType_OD);
	PortManager::initPin(portTX, pinNumberTX, GPIO_Mode_AF, GPIO_Speed_50MHz,
			GPIO_PuPd_UP, GPIO_OType_PP);//----------------------------------HERE WAS GPIO_OType_OD);

	if (USARTx == USART3) {
		PortManager::pinAFConfig(portRX, pinNumberRX, GPIO_AF_USART3);
		PortManager::pinAFConfig(portTX, pinNumberTX, GPIO_AF_USART3);
	}

	USART_InitTypeDef initStruct;
	initStruct.USART_BaudRate = baudRate;
	initStruct.USART_HardwareFlowControl = hardwareFlowControl;
	initStruct.USART_Mode = mode;
	initStruct.USART_Parity = parity;
	initStruct.USART_StopBits = stopBits;
	initStruct.USART_WordLength = wordLength;

	USART_Init(this->USARTx, &initStruct);

	USART_ClearITPendingBit(USARTx, USART_IT_TC);
	USART_ClearITPendingBit(USARTx, USART_IT_RXNE);

	USART_ITConfig(USARTx, USART_IT_TC, ENABLE);
	USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);

	USART_Cmd(USARTx, ENABLE);
}

USART::USART(USART_TypeDef* USARTx, GPIO_TypeDef *portRX, uint8_t pinNumberRX,
		GPIO_TypeDef *portTX, uint8_t pinNumberTX) :
		portRX(portRX), pinNumberRX(pinNumberRX), portTX(portTX), pinNumberTX(
				pinNumberTX), USARTx(USARTx), isBurst(false) {

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	transmitterMutex = xSemaphoreCreateMutex();
	transmitterSemaphore = xSemaphoreCreateBinary();

	this->timerCreate("USART_Rec", 6, pdFALSE);

	vQueueAddToRegistry(transmitterSemaphore, "U2Sem");
	queueSend = xQueueCreate(10, sizeof(USARTMessage));
	queueReceive = xQueueCreate(10, sizeof(USARTMessage));
	vQueueAddToRegistry(queueReceive, "U2Rec");

	xSemaphoreGive(transmitterSemaphore);
	receiveMessage.curPos = 0;

	NVIC_SetPriorityGrouping(0);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	NVIC_InitTypeDef NVIC_InitStructure;
	if (USARTx == USART3) {
		NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	}

	NVIC_Init(&NVIC_InitStructure);

	for (uint8_t i = 0; i < 255; i++) {
		receiveMessage.data[i] = 0;
	}
}

void USART::configurePorts(GPIO_TypeDef* portRX, uint8_t pinNumberRX,
		GPIO_TypeDef* portTX, uint8_t pinNumberTX) {
	this->portRX = portRX;
	this->portTX = portTX;
	this->pinNumberRX = pinNumberRX;
	this->pinNumberTX = pinNumberTX;
}

void USART::startTransmittion(USARTMessage message, TickType_t maxBlockTime) {
	if (xSemaphoreTake(transmitterSemaphore, maxBlockTime)) {
//		PortManager::setPin(GPIOB,12);
		transmittMessage = message;
		transmittMessage.curPos = 0;
		USART_SendData(USARTx, transmittMessage.data[0]);
		transmittMessage.curPos++;
//		PortManager::resetPin(GPIOB,12);
	}
}

void USART::send(USARTMessage* umessage, bool isBurst) {
	PortManager::setPin(GPIOB,12);
	xQueueSend(queueSend, umessage, 10);
	this->isBurst = isBurst;
	if (this->isBurst) {
		this->setReceive(DISABLE);
	}
//	PortManager::resetPin(GPIOB,12);
}

void USART::timerCallbackFunction() {
	taskDISABLE_INTERRUPTS();
	receiveMessage.length = receiveMessage.curPos;

	USARTMessage queueTmp;
	queueTmp.length = receiveMessage.curPos;
	queueTmp.curPos = 0;
	for (uint8_t i = 0; i < queueTmp.length; i++) {
		queueTmp.data[i] = receiveMessage.data[i];
	}
	receiveMessage.curPos = 0;
	queueTmp.timestamp = receiveMessage.timestamp;
	taskENABLE_INTERRUPTS();
	xQueueSend(queueReceive, &(queueTmp), 100);
}

void USART::run() {
	USARTMessage messageSend;
	USARTMessage messageReceive;
	while (1) {
		if (xQueueReceive(this->queueSend, &messageSend, 100)) {
//			PortManager::setPin(GPIOB,12);
			startTransmittion(messageSend, 100);
			this->taskDelay(2);
//			PortManager::resetPin(GPIOB,12);
		}
		if (xQueuePeek(this->queueReceive, &messageReceive, 0)) {
			if (xTaskGetTickCount() - messageReceive.timestamp > 1000) {
				xQueueReceive(this->queueReceive, &messageReceive, 0);
			}
		}
	}
}

void USART::setReceive(FunctionalState state) {
	USART_ReceiveData(USARTx);
	USART_ITConfig(USARTx, USART_IT_RXNE, state);
}

USART::~USART() {
	// TODO Auto-generated destructor stub
}

//extern "C" {
//void USART3_IRQHandler() {
//	BaseType_t xHigherPriorityTaskWoken = 0;
//
//	usart3->IRQHandler();
//
//	if (xHigherPriorityTaskWoken) {
//		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//	}
//}
//}


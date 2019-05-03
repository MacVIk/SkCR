/*
 * SPI.cpp
 *
 *  Created on: 12.02.2018
 *      Author: Fazli
 */

#include "SPI.h"

SPI* spi2;

void delay_1us(void) {
	volatile uint8_t i;

	switch (hclk_div_coef) {
	case 1: {
		for (i = 0; i < 18; i++) {
		};
		break;
	}

	case 2: {
		for (i = 0; i < 8; i++) {
		};
		break;
	}

	case 4: {
		for (i = 0; i < 4; i++) {
		};
		break;
	}

	default: {
		break;
	}

	};
} // delay_1us

void SPI::IRQHandler() {
	BaseType_t xHigherPriorityTaskWoken = 0;

	if (SPI_I2S_GetITStatus(SPIx, SPI_I2S_IT_TXE) != RESET) {
		if (transmittMessageSPI.curPos == transmittMessageSPI.length) {

			SPI_I2S_ITConfig(SPIx, SPI_IT_TXE, DISABLE);

#ifdef LYNXMOTION_JOYSTICK_STYLE
			for (uint8_t i = 0; i < 13; i++) { //hardcoded 13 microseconds waiting needed for CS signal ONLY
				delay_1us();
			}
			PortManager::initPin(portMOSI, pinNumberMOSI, GPIO_Mode_OUT,
					GPIO_Speed_50MHz, GPIO_PuPd_UP, GPIO_OType_PP);	//hardcoded for LYNXMOTION ONLY
			PortManager::setPin(portMOSI, pinNumberMOSI); //hardcoded ONY FOR LYXMOTION
#endif

#ifdef CS_PIN_EABLE
			setCS();
#endif
			xSemaphoreGiveFromISR(transmitterSemaphoreSPI,
					&xHigherPriorityTaskWoken);
		} else {

#ifdef LYNXMOTION_JOYSTICK_STYLE
			for (uint8_t i = 0; i < 22; i++) { //hardcoded 22 microseconds waiting
				delay_1us();
			}
#endif
			SPI_I2S_SendData(SPIx,
					transmittMessageSPI.data[transmittMessageSPI.curPos]);
			transmittMessageSPI.curPos++;
		}
		SPI_I2S_ClearITPendingBit(SPIx, SPI_I2S_IT_TXE);
	}
	if (SPI_I2S_GetITStatus(SPIx, SPI_I2S_IT_RXNE) != RESET) {
		SPI_I2S_ClearITPendingBit(SPIx, SPI_I2S_IT_RXNE);
		uint16_t tmpr = SPI_I2S_ReceiveData(SPIx);
		receiveMessageSPI.data[receiveMessageSPI.curPos] = (uint8_t) tmpr;
		receiveMessageSPI.curPos++;
		receiveMessageSPI.timestamp = xTaskGetTickCountFromISR();
		this->timerStartFromISR(&xHigherPriorityTaskWoken);
	}
}

void SPI::init(uint16_t direction, uint16_t mode, uint16_t dataSize,
		uint16_t clockPolarity, uint16_t clockEdge, uint16_t slaveSelect,
		uint16_t baudRatePrescaler, uint16_t firstBit, uint16_t crcPoly) {

//	SPI_CalculateCRC(SPIx, DISABLE);
	SPI_NSSInternalSoftwareConfig(SPIx, SPI_NSSInternalSoft_Set);

	PortManager::initPin(portMOSI, pinNumberMOSI, GPIO_Mode_AF,
			GPIO_Speed_50MHz, GPIO_PuPd_UP, GPIO_OType_PP);
	PortManager::initPin(portMISO, pinNumberMISO, GPIO_Mode_AF,
			GPIO_Speed_50MHz, GPIO_PuPd_UP, GPIO_OType_PP);
	PortManager::initPin(portSCK, pinNumberSCK, GPIO_Mode_AF, GPIO_Speed_50MHz,
			GPIO_PuPd_NOPULL, GPIO_OType_PP);

#ifdef CS_PIN_EABLE
	PortManager::initPin(portCS, pinNumberCS, GPIO_Mode_OUT, GPIO_Speed_50MHz,
			GPIO_PuPd_UP, GPIO_OType_PP); //CS
	setCS();
#endif

	if (SPIx == SPI2) {
		PortManager::pinAFConfig(portMOSI, pinNumberMOSI, GPIO_AF_SPI2);
		PortManager::pinAFConfig(portMISO, pinNumberMISO, GPIO_AF_SPI2);
		PortManager::pinAFConfig(portSCK, pinNumberSCK, GPIO_AF_SPI2);
	}

	if (SPIx == SPI3) {
		PortManager::pinAFConfig(portMOSI, pinNumberMOSI, GPIO_AF_SPI3);
		PortManager::pinAFConfig(portMISO, pinNumberMISO, GPIO_AF_SPI3);
		PortManager::pinAFConfig(portSCK, pinNumberSCK, GPIO_AF_SPI3);
	}

	SPI_InitTypeDef initStruct;
	initStruct.SPI_Direction = direction;
	initStruct.SPI_Mode = mode;
	initStruct.SPI_DataSize = dataSize;
	initStruct.SPI_CPOL = clockPolarity;
	initStruct.SPI_CPHA = clockEdge;
	initStruct.SPI_NSS = slaveSelect;
	initStruct.SPI_BaudRatePrescaler = baudRatePrescaler;
	initStruct.SPI_FirstBit = firstBit;
	initStruct.SPI_CRCPolynomial = crcPoly;

	SPI_Init(this->SPIx, &initStruct);

	SPI_I2S_ClearITPendingBit(SPIx, SPI_I2S_IT_TXE);
	SPI_I2S_ClearITPendingBit(SPIx, SPI_I2S_IT_RXNE);

//	SPI_I2S_ITConfig(SPIx, SPI_I2S_IT_TXE, ENABLE);
	SPI_I2S_ITConfig(SPIx, SPI_I2S_IT_RXNE, ENABLE);

	SPI_Cmd(SPIx, ENABLE);
}

SPI::SPI(SPI_TypeDef* SPIx, GPIO_TypeDef* portMOSI, uint8_t pinNumberMOSI,
		GPIO_TypeDef* portMISO, uint8_t pinNumberMISO, GPIO_TypeDef* portSCK,
		uint8_t pinNumberSCK, GPIO_TypeDef* portCS, uint8_t pinNumberCS) :
		portMOSI(portMOSI), pinNumberMOSI(pinNumberMOSI), portMISO(portMISO), pinNumberMISO(
				pinNumberMISO), portSCK(portSCK), pinNumberSCK(pinNumberSCK), portCS(
				portCS), pinNumberCS(pinNumberCS), SPIx(SPIx) {

//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);

	transmitterMutexSPI = xSemaphoreCreateMutex();
	transmitterSemaphoreSPI = xSemaphoreCreateBinary();

	this->timerCreate("SPI_Rec", 6, pdFALSE);

	vQueueAddToRegistry(transmitterSemaphoreSPI, "SPI3Sem");
	queueSendSPI = xQueueCreate(10, sizeof(SPIMessage));
	queueReceiveSPI = xQueueCreate(10, sizeof(SPIMessage));
	vQueueAddToRegistry(queueReceiveSPI, "SPI3Rec");

	xSemaphoreGive(transmitterSemaphoreSPI);
	receiveMessageSPI.curPos = 0;

//	NVIC_SetPriorityGrouping(0);
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	NVIC_InitTypeDef NVIC_InitStructure;

	if (SPIx == SPI2) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
		NVIC_InitStructure.NVIC_IRQChannel = SPI2_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	}

	if (SPIx == SPI3) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
		NVIC_InitStructure.NVIC_IRQChannel = SPI3_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	}

	NVIC_Init(&NVIC_InitStructure);

	for (uint8_t i = 0; i < 255; i++) {
		receiveMessageSPI.data[i] = 0;
	}
}

void SPI::configurePorts(GPIO_TypeDef* portMOSI, uint8_t pinNumberMOSI,
		GPIO_TypeDef* portMISO, uint8_t pinNumberMISO, GPIO_TypeDef* portSCK,
		uint8_t pinNumberSCK, GPIO_TypeDef* portCS, uint8_t pinNumberCS) {

	this->portMOSI = portMOSI;
	this->portMISO = portMISO;
	this->portSCK = portSCK;
	this->portCS = portCS;
	this->pinNumberMOSI = pinNumberMOSI;
	this->pinNumberMISO = pinNumberMISO;
	this->pinNumberSCK = pinNumberSCK;
	this->pinNumberCS = pinNumberCS;
}

void SPI::startTransmittion(SPIMessage message, TickType_t maxBlockTime) {
	if (xSemaphoreTake(transmitterSemaphoreSPI, maxBlockTime)) {
		transmittMessageSPI = message;
		transmittMessageSPI.curPos = 0;
#ifdef CS_PIN_EABLE
		resetCS();
#endif
#ifdef LYNXMOTION_JOYSTICK_STYLE
		PortManager::initPin(portMOSI, pinNumberMOSI, GPIO_Mode_AF,
				GPIO_Speed_50MHz, GPIO_PuPd_UP, GPIO_OType_PP);	//hardcoded for LYNXMOTION ONLY
#endif
		SPI_I2S_SendData(SPIx, transmittMessageSPI.data[0]);
		SPI_I2S_ITConfig(SPIx, SPI_I2S_IT_TXE, ENABLE);
		transmittMessageSPI.curPos++;
	}
}

void SPI::send(SPIMessage* umessage) {
	xQueueSend(queueSendSPI, umessage, 10);
}

//void SPI::setReceive(FunctionalState state) {
//}

void SPI::timerCallbackFunction() {
	taskDISABLE_INTERRUPTS();
	receiveMessageSPI.length = receiveMessageSPI.curPos;

	SPIMessage queueTmp;
	queueTmp.length = receiveMessageSPI.curPos;
	queueTmp.curPos = 0;
	for (uint8_t i = 0; i < queueTmp.length; i++) {
		queueTmp.data[i] = receiveMessageSPI.data[i];
	}
	receiveMessageSPI.curPos = 0;
	queueTmp.timestamp = receiveMessageSPI.timestamp;
	taskENABLE_INTERRUPTS();
	xQueueSend(queueReceiveSPI, &(queueTmp), 100);
}

void SPI::setCS() {
	PortManager::setPin(portCS, pinNumberCS);
}

void SPI::resetCS() {
	PortManager::resetPin(portCS, pinNumberCS);
}

void SPI::run() {
	SPIMessage messageSendSPI;
	SPIMessage messageReceiveSPI;
	while (1) {
		if (xQueueReceive(this->queueSendSPI, &messageSendSPI, 100)) {
			startTransmittion(messageSendSPI, 100);
			this->taskDelay(2);
		}
		if (xQueuePeek(this->queueReceiveSPI, &messageReceiveSPI, 0)) {
			if (xTaskGetTickCount() - messageReceiveSPI.timestamp > 1000) {
				xQueueReceive(this->queueReceiveSPI, &messageReceiveSPI, 0);
			}
		}

	}
}

SPI::~SPI() {
	// TODO Auto-generated destructor stub
}

extern "C" {
//void SPI3_IRQHandler() {
//	BaseType_t xHigherPriorityTaskWoken = 0;
//
//	spi3->IRQHandler();
//
//	if (xHigherPriorityTaskWoken) {
//		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//	}
//}

void SPI2_IRQHandler() {
	BaseType_t xHigherPriorityTaskWoken = 0;

	spi2->IRQHandler();

	if (xHigherPriorityTaskWoken) {
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}
}


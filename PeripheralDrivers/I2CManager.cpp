/*
 * I2CManager.cpp
 *
 *  Created on: 13.08.2015
 *      Author: Tata
 */

#include "I2CManager.h"

I2CManager* I2CManager::I2CManagerInstance = 0;
//QueueHandle_t I2CManager::I2C1QueueSend = 0;
QueueHandle_t I2CManager::I2C1QueueReceive = 0;
SemaphoreHandle_t I2CManager::I2C1SemaphoreMutex = 0;
SemaphoreHandle_t I2CManager::I2C1SemaphoreState = 0;

I2CMessage I2CManager::I2C1CurrentMessage;
uint8_t I2CManager::I2C1currentPosition = 0;

I2CManager* I2CManager::getInstance() {
	if (!I2CManagerInstance) {
		I2CManagerInstance = (I2CManager*) pvPortMalloc(sizeof(I2CManager));
	}
	return I2CManagerInstance;
}

BaseType_t I2CManager::captureI2C(I2C_TypeDef* I2Cx) {
	if (I2Cx == I2C1) {
		return xSemaphoreTake(I2CManager::I2C1SemaphoreState, I2CCaptureTicksToWait);
	}

	return 0;
}

void I2CManager::initI2C(I2C_TypeDef* I2Cx, I2C_InitTypeDef* I2C_InitStruct,
		GPIO_TypeDef *portSCL, uint8_t pinNumberSCL, GPIO_TypeDef *portSDA,
		uint8_t pinNumberSDA) {

	I2C_DeInit(I2C1);
	I2C_Init(I2C1, I2C_InitStruct);
//	I2C_AcknowledgeConfig(I2C1, DISABLE);
	this->I2C1QueueSend = 0;

	if (I2Cx == I2C1) {
		I2C1SemaphoreMutex = xSemaphoreCreateMutex();
		I2C1SemaphoreState = xSemaphoreCreateBinary();
		I2C1QueueSend = xQueueCreate(3, sizeof (I2CMessage));
		I2C1QueueReceive = xQueueCreate(3, sizeof (I2CMessage));
		if (I2CManager::I2C1QueueSend == 0) {
			while (1) {

			}
		}

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

		PortManager::initPin(portSCL, pinNumberSCL, GPIO_Mode_AF,
				GPIO_Speed_50MHz, GPIO_PuPd_UP, GPIO_OType_OD);
		PortManager::initPin(portSDA, pinNumberSDA, GPIO_Mode_AF,
				GPIO_Speed_50MHz, GPIO_PuPd_UP, GPIO_OType_OD);

		PortManager::pinAFConfig(portSCL, pinNumberSCL, GPIO_AF_I2C1);
		PortManager::pinAFConfig(portSDA, pinNumberSDA, GPIO_AF_I2C1);

		I2C_Cmd(I2C1, ENABLE);

		NVIC_InitTypeDef NVIC_InitStructure;

		/* Configure the I2C event priority */
		NVIC_SetPriorityGrouping(0);
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

		NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);

		I2C_ITConfig(I2C1, I2C_IT_BUF | I2C_IT_EVT, ENABLE);

		I2C_ClearFlag(I2C1, I2C_SR1_STOPF);

		xSemaphoreGive(I2CManager::I2C1SemaphoreState);

//		I2C_GenerateSTOP(I2C1, ENABLE);

		int i = 0;
		while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY)) {
			i++;
		}
	}
}

BaseType_t I2CManager::sendViaI2C(I2C_TypeDef* I2Cx, I2CMessage* message) {
	if (I2Cx == I2C1) {
		BaseType_t stat = xQueueSend(I2CManager::I2C1QueueSend, message, 10);
		if (stat) {
			I2C_GenerateSTOP(I2C1, DISABLE);
			I2C_GenerateSTART(I2C1, ENABLE);
		}
		return stat;
	}
	return 0;
}

extern "C" {

void I2C1_EV_IRQHandler() {
	PortManager::setPin(GPIOD, 13);
	I2C_ClearFlag(I2C1, I2C_SR1_AF);
	uint32_t lastEvent = I2C_GetLastEvent(I2C1);
	BaseType_t xHigherPriorityTaskWoken = 0;
	switch (lastEvent) {
	case I2C_EVENT_MASTER_MODE_SELECT:
		I2CManager::I2C1currentPosition = 0;
		I2C_Send7bitAddress(I2C1, I2CManager::I2C1CurrentMessage.address,
				I2CManager::I2C1CurrentMessage.direction);
		break;

		//Master Transmitter section
	case I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED:
		I2C_SendData(I2C1,
				I2CManager::I2C1CurrentMessage.data[I2CManager::I2C1currentPosition]);
		I2CManager::I2C1currentPosition++;
		break;
	case I2C_EVENT_MASTER_BYTE_TRANSMITTED:
		if (I2CManager::I2C1currentPosition
				< I2CManager::I2C1CurrentMessage.length) {
			I2C_SendData(I2C1,
					I2CManager::I2C1CurrentMessage.data[I2CManager::I2C1currentPosition]);
			I2CManager::I2C1currentPosition++;
		} else {
			PortManager::resetPin(GPIOD, 13);
			xSemaphoreGiveFromISR(I2CManager::I2C1SemaphoreState,
					&xHigherPriorityTaskWoken);
			I2C_GenerateSTOP(I2C1, ENABLE);
		}
		break;
	case I2C_EVENT_MASTER_BYTE_TRANSMITTING:
		if (I2CManager::I2C1currentPosition
				< I2CManager::I2C1CurrentMessage.length) {
			I2C_SendData(I2C1,
					I2CManager::I2C1CurrentMessage.data[I2CManager::I2C1currentPosition]);
			I2CManager::I2C1currentPosition++;
		} else {
			PortManager::resetPin(GPIOD, 13);
			xSemaphoreGiveFromISR(I2CManager::I2C1SemaphoreState,
					&xHigherPriorityTaskWoken);
			I2C_GenerateSTOP(I2C1, ENABLE);
		}
		break;

		//Master Reciever Section
	case I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED:
		break;
	case I2C_EVENT_MASTER_BYTE_RECEIVED:
		if (I2CManager::I2C1currentPosition
				< (I2CManager::I2C1CurrentMessage.length - 1)) {
			I2CManager::I2C1CurrentMessage.data[I2CManager::I2C1currentPosition] =
					I2C_ReceiveData(I2C1);
			I2CManager::I2C1currentPosition++;
		} else if (I2CManager::I2C1currentPosition
				== (I2CManager::I2C1CurrentMessage.length - 1)) {
			I2CManager::I2C1CurrentMessage.data[I2CManager::I2C1currentPosition] =
					I2C_ReceiveData(I2C1);
			I2CManager::I2C1currentPosition++;
			PortManager::resetPin(GPIOD, 13);
			xQueueSendFromISR(I2CManager::I2C1QueueReceive,
					&I2CManager::I2C1CurrentMessage, &xHigherPriorityTaskWoken);
			xSemaphoreGiveFromISR(I2CManager::I2C1SemaphoreState,
					&xHigherPriorityTaskWoken);
			I2C_GenerateSTOP(I2C1, ENABLE);
		} else {
			PortManager::resetPin(GPIOD, 13);
			I2C_GenerateSTOP(I2C1, ENABLE);
			I2C_ReceiveData(I2C1);
		}
		break;
	case 196676:
		if (I2CManager::I2C1currentPosition
				< (I2CManager::I2C1CurrentMessage.length - 1)) {
			I2CManager::I2C1CurrentMessage.data[I2CManager::I2C1currentPosition] =
					I2C_ReceiveData(I2C1);
			I2CManager::I2C1currentPosition++;
		} else if (I2CManager::I2C1currentPosition
				== (I2CManager::I2C1CurrentMessage.length - 1)) {
			I2CManager::I2C1CurrentMessage.data[I2CManager::I2C1currentPosition] =
					I2C_ReceiveData(I2C1);
			I2CManager::I2C1currentPosition++;
			PortManager::resetPin(GPIOD, 13);
			xQueueSendFromISR(I2CManager::I2C1QueueReceive,
					&I2CManager::I2C1CurrentMessage, &xHigherPriorityTaskWoken);
			xSemaphoreGiveFromISR(I2CManager::I2C1SemaphoreState,
					&xHigherPriorityTaskWoken);
			I2C_GenerateSTOP(I2C1, ENABLE);
		} else {
			PortManager::resetPin(GPIOD, 13);
			I2C_ReceiveData(I2C1);
			I2C_GenerateSTOP(I2C1, ENABLE);
		}
		break;
	default:
		PortManager::resetPin(GPIOD, 13);
		I2C_ReceiveData(I2C1);
//		I2C_ClearFlag(I2C1, I2C_FLAG_RXNE);
		I2C_GenerateSTOP(I2C1, ENABLE);
		break;
	}

	if (xHigherPriorityTaskWoken) {
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

}


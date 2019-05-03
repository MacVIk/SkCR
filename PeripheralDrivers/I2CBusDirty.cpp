/*
 * I2CManagerDirty.cpp
 *
 *  Created on: 22.10.2015
 *      Author: Tata
 */

#include "I2CBusDirty.h"

I2CBus::I2CBus(GPIO_TypeDef* portSCL, uint8_t pinSCL, GPIO_TypeDef* portSDA,
		uint8_t pinSDA, I2C_TypeDef* I2Cx) {
	this->portSCL = portSCL;
	this->pinSCL = pinSCL;
	this->portSDA = portSDA;
	this->pinSDA = pinSDA;
	this->I2Cx = I2Cx;

	queueRequest = xQueueCreate(10, sizeof(I2CMessageNew));
	queueAnswer = xQueueCreate(10, sizeof(I2CMessageNew));
	failTimeWait = 0;

	this->init();
}

void I2CBus::init() {
	i2c_timer_init();

	PortManager::initPin(portSCL, pinSCL, GPIO_Mode_AF, GPIO_Speed_50MHz,
			GPIO_PuPd_UP, GPIO_OType_OD);
	PortManager::initPin(portSDA, pinSDA, GPIO_Mode_AF, GPIO_Speed_50MHz,
			GPIO_PuPd_UP, GPIO_OType_OD);

	PortManager::pinAFConfig(portSCL, pinSCL, GPIO_AF_I2C1);
	PortManager::pinAFConfig(portSDA, pinSDA, GPIO_AF_I2C1);

	I2C_InitTypeDef I2C_InitStruct;

	I2C_DeInit(I2Cx);

	I2C_InitStruct.I2C_ClockSpeed = 400000; // 400kHz
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C; // controller I2C mode
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2; // 50% duty cycle --> standard
	I2C_InitStruct.I2C_OwnAddress1 = 0x00; // own address, not relevant in master mode
	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable; // enable acknowledge when reading (can be changed later on)
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // set address length to 7 bit addresses
	I2C_Init(I2Cx, &I2C_InitStruct); // init I2Cx

	// run i2c
	I2C_Cmd(I2Cx, ENABLE);
}

I2C_status_t I2CBus::readReg(uint8_t adress, uint8_t* reg, uint8_t regSize,
		uint8_t* data, uint8_t dataSize) {
	I2CMessageNew messageSend;
//	I2CMessageNew messageReceive;
	messageSend.adr = adress;
	messageSend.reg = reg;
	messageSend.data = data;
	messageSend.direction = I2C_Direction_Receiver;
	messageSend.dataSize = dataSize;
	messageSend.regSize = regSize;

	vTaskSuspendAll();
	I2CBus::sendToBus(&messageSend);
	xTaskResumeAll();
//	xQueueSend(queueRequest, &messageSend, 10);
//	uint8_t status = xQueueReceive(queueAnswer, &messageReceive, 100);
//	if (messageSend.adr != messageReceive.adr) {
//		int k = 0;
//	}
	data = messageSend.data;
	if (messageSend.status == 0) {
		failTimeWait = 10;
	}
	return messageSend.status;
}

I2C_status_t I2CBus::readReg(uint8_t adress, uint8_t reg, uint8_t* data,
		uint8_t dataSize) {
	return this->readReg(adress, &reg, 1, data, dataSize);
}

I2C_status_t I2CBus::writeReg(uint8_t adress, uint8_t* reg, uint8_t regSize,
		uint8_t* data, uint8_t dataSize) {
	I2CMessageNew messageSend;
//	I2CMessageNew messageReceive;
	messageSend.adr = adress;
	messageSend.reg = reg;
	messageSend.data = data;
	messageSend.direction = I2C_Direction_Transmitter;
	messageSend.dataSize = dataSize;
	messageSend.regSize = regSize;

	vTaskSuspendAll();
	I2CBus::sendToBus(&messageSend);
	xTaskResumeAll();
	//	xQueueSend(queueRequest, &messageSend, 10);
	//	uint8_t status = xQueueReceive(queueAnswer, &messageReceive, 100);
	//	if (messageSend.adr != messageReceive.adr) {
	//		int k = 0;
	//	}
	data = messageSend.data;
	if (messageSend.status == 0) {
		failTimeWait = 1000;
	}
	return messageSend.status;
}

I2C_status_t I2CBus::writeReg(uint8_t adress, uint8_t reg, uint8_t* data,
		uint8_t dataSize) {
	return this->writeReg(adress, &reg, 1, data, dataSize);
}

uint8_t I2CBus::waitOnFlag(uint32_t temp_flag) {
	uint32_t timeout = 0x1000;
	uint8_t forReturn = 0;
	i2c_timer_start_timeout(timeout);
	while (I2C_GetFlagStatus(I2C1, temp_flag)) {
		if (i2c_timer_timeout_status()) {
			forReturn = 1; //TODO: wtf?
			break;
		}
	}
	i2c_timer_stop_timeout();
	return forReturn;
}

QueueHandle_t I2CBus::getQueueAnswer() {
	return queueAnswer;
}

void I2CBus::setFailTimeWait(uint16_t failTimeWait) {
	this->failTimeWait = failTimeWait;
}

uint16_t I2CBus::getFailTimeWait() const {
	return failTimeWait;
}

QueueHandle_t I2CBus::getQueueRequest() {
	return queueRequest;
}

uint8_t I2CBus::waitOnEvent(uint32_t temp_event) {
	uint32_t timeout = 0x2000;
	uint8_t forReturn = 0;
	i2c_timer_start_timeout(timeout);
	while (!I2C_CheckEvent(I2C1, temp_event)) {
		if (i2c_timer_timeout_status()) {
			forReturn = 1; //TODO:wtf?
			break;
		}
	}
	i2c_timer_stop_timeout();
	return forReturn;
}

uint8_t I2CBus::sendToBus(I2CMessageNew* message) {
	uint8_t tmp;
	if (message->direction == I2C_Direction_Transmitter) {
		tmp = waitOnFlag(I2C_FLAG_BUSY);
		if (tmp) {
			message->status = I2C_ERROR;
			return failReturn();
		}

		I2C_GenerateSTART(I2C1, ENABLE);
		tmp = waitOnEvent(I2C_EVENT_MASTER_MODE_SELECT);
		if (tmp) {
			message->status = I2C_ERROR;
			return failReturn();
		}

		I2C_Send7bitAddress(I2C1, message->adr, I2C_Direction_Transmitter);
		tmp = waitOnEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
		if (tmp) {
			message->status = I2C_ERROR;
			return failReturn();
		}

		for (uint8_t i = 0; i < message->regSize; i++) {
			I2C_SendData(I2C1, message->reg[i]);
			tmp = waitOnEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED);
			if (tmp) {
				message->status = I2C_ERROR;
				return failReturn();
			}
		}
//		I2C_SendData(I2C1, (uint8_t) *(message->reg));
//		tmp = waitOnEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED);
//		if (tmp) {
//			message->status = I2C_ERROR;
//			return failReturn();
//		}

		/* Write data to TXDR */

		for (uint8_t i = 0; i < message->dataSize; i++) {

			I2C_SendData(I2C1, (message->data)[i]);
			tmp = waitOnEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED);
			if (tmp) {
				message->status = I2C_ERROR;
				return failReturn();
			}
		}

		I2C_GenerateSTOP(I2C1, ENABLE);
		message->status = I2C_SUCCESS;
		return I2C_SUCCESS;
	} else {
		tmp = waitOnFlag(I2C_FLAG_BUSY);
		if (tmp) {
			message->status = I2C_ERROR;
			return failReturn();
		}

		I2C_GenerateSTART(I2C1, ENABLE);
		tmp = waitOnEvent(I2C_EVENT_MASTER_MODE_SELECT);
		if (tmp) {
			message->status = I2C_ERROR;
			return failReturn();
		}

		I2C_Send7bitAddress(I2C1, message->adr, I2C_Direction_Transmitter);
		tmp = waitOnEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
		if (tmp) {
			message->status = I2C_ERROR;
			return failReturn();
		}

		for (uint8_t i = 0; i < message->regSize; i++) {
			I2C_SendData(I2C1, message->reg[i]);
			tmp = waitOnEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED);
			if (tmp) {
				message->status = I2C_ERROR;
				return failReturn();
			}
		}
//		I2C_SendData(I2C1, (uint8_t) *(message->reg));
//		tmp = waitOnEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED);
//		if (tmp) {
//			message->status = I2C_ERROR;
//			return failReturn();
//		}

		I2C_GenerateSTART(I2C1, ENABLE);
		tmp = waitOnEvent(I2C_EVENT_MASTER_MODE_SELECT);
		if (tmp) {
			message->status = I2C_ERROR;
			return failReturn();
		}

		I2C_Send7bitAddress(I2C1, message->adr, I2C_Direction_Receiver);
		tmp = waitOnEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);
		if (tmp) {
			message->status = I2C_ERROR;
			return failReturn();
		}

		for (uint8_t i = 0; i < message->dataSize; i++) {

			if (i == message->dataSize - 1) {
				I2C_AcknowledgeConfig(I2C1, DISABLE);
			}

			tmp = waitOnEvent(I2C_EVENT_MASTER_BYTE_RECEIVED);
			if (tmp) {
				message->status = I2C_ERROR;
				I2C_AcknowledgeConfig(I2C1, ENABLE);
				return failReturn();
			}

			// Read data from RXDR
			(message->data)[i] = I2C_ReceiveData(I2C1);
		}

		I2C_GenerateSTOP(I2C1, ENABLE);
		// If all operations OK
		message->status = I2C_SUCCESS;
		I2C_AcknowledgeConfig(I2C1, ENABLE);
		return I2C_SUCCESS;
	}

}

uint8_t I2CBus::failReturn() {
	I2C_GenerateSTOP(I2C1, ENABLE);

	return 0x00;
}

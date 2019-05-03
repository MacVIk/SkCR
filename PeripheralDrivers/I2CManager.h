/*
 * I2CManager.h
 *
 *  Created on: 13.08.2015
 *      Author: Tata
 */

#ifndef I2CMANAGER_H_
#define I2CMANAGER_H_

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "PortManager.h"
#include "stm32f4xx.h"

#define I2CQueueLength	10
#define I2CCaptureTicksToWait 100

struct I2CMessage {
	uint8_t address;
	uint8_t direction;
	uint8_t length;
	uint8_t data[10];
};

enum I2CStatus {
	SENDING_ADRESS, SENDING_DATA, FAILED, SUCCES, WAITING
};

class I2CManager {
private:
	static I2CManager* I2CManagerInstance;
	I2CManager() {
	}
	I2CManager(const I2CManager&);
	I2CManager& operator=(I2CManager&);

public:
	static SemaphoreHandle_t I2C1SemaphoreMutex;
	static SemaphoreHandle_t I2C1SemaphoreState;
	QueueHandle_t I2C1QueueSend;
	static QueueHandle_t I2C1QueueReceive;

	static I2CMessage I2C1CurrentMessage;
	static I2CStatus I2C1currentStatus;
	static uint8_t I2C1currentPosition;

	static I2CManager* getInstance();
	void initI2C(I2C_TypeDef* I2Cx, I2C_InitTypeDef* I2C_InitStruct,
			GPIO_TypeDef *portSCL, uint8_t pinNumberSCL, GPIO_TypeDef *portSDA,
			uint8_t pinNumberSDA);
	void initI2C(I2C_TypeDef* I2Cx);

	BaseType_t captureI2C(I2C_TypeDef* I2Cx);
	BaseType_t sendViaI2C(I2C_TypeDef* I2Cx, I2CMessage* message);
	BaseType_t recieveViaI2C(I2C_TypeDef* I2Cx, I2CMessage* message);

//	I2CMessage getI2C1currentMessage() const {
//		return I2C1CurrentMessage;
//	}
//
//	void setI2C1currentMessage(I2CMessage i2C1currentMessage) {
//		I2C1CurrentMessage = i2C1currentMessage;
//	}
//
//	I2CStatus getI2C1currentStatus() const {
//		return I2C1currentStatus;
//	}
//
//	void setI2C1currentStatus(I2CStatus i2C1currentStatus) {
//		I2C1currentStatus = i2C1currentStatus;
//	}

};

#endif /* I2CMANAGER_H_ */

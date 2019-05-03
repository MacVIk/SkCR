/*
 * I2CManagerDirty.h
 *
 *  Created on: 22.10.2015
 *      Author: Tata
 */

#ifndef I2CMANAGERDIRTY_H_
#define I2CMANAGERDIRTY_H_

#include "FreeRTOS.h"
#include "queue.h"
#include "i2c_timer.h"
#include "stm32f4xx.h"
#include "PeripheralDrivers/PortManager.h"
#include "FreeRTOS/wrapper/iActiveObject.h"

typedef enum {
	I2C_SUCCESS = 0x01, I2C_ERROR = 0x00
} I2C_status_t;

struct I2CMessageNew {
	uint8_t* reg;
	uint8_t* data;
	uint8_t regSize;
	uint8_t dataSize;
	uint8_t direction;
	uint8_t adr;
	I2C_status_t status;
};

class I2CBus {
private:
	GPIO_TypeDef* portSCL;
	uint8_t pinSCL;
	GPIO_TypeDef* portSDA;
	uint8_t pinSDA;
	I2C_TypeDef* I2Cx;

	QueueHandle_t queueRequest;
	QueueHandle_t queueAnswer;
	uint16_t failTimeWait;
public:
	I2CBus(GPIO_TypeDef* portSCL, uint8_t pinSCL, GPIO_TypeDef* portSDA,
			uint8_t pinSDA, I2C_TypeDef* I2Cx);
	void init();
	I2C_status_t readReg(uint8_t adress, uint8_t* reg, uint8_t regSize,
			uint8_t* data, uint8_t dataSize);
	I2C_status_t readReg(uint8_t adress, uint8_t reg, uint8_t* data,
			uint8_t dataSize);
	I2C_status_t writeReg(uint8_t adress, uint8_t* reg, uint8_t regSize,
			uint8_t* data, uint8_t dataSize);
	I2C_status_t writeReg(uint8_t adress, uint8_t reg, uint8_t* data,
			uint8_t dataSize);
	static uint8_t waitOnFlag(uint32_t temp_flag);
	static uint8_t waitOnEvent(uint32_t temp_event);
	static uint8_t sendToBus(I2CMessageNew* message);
	QueueHandle_t getQueueAnswer();
	QueueHandle_t getQueueRequest();
	uint16_t getFailTimeWait() const;
	void setFailTimeWait(uint16_t failTimeWait);
	static uint8_t failReturn();
};

#endif /* I2CMANAGERDIRTY_H_ */

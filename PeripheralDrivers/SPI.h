/*
 * SPI.h
 *
 *  Created on: 12.02.2018
 *      Author: Fazli
 */

#ifndef SPI_H_
#define SPI_H_

#include "stm32f4xx_spi.h"
#include "FreeRTOS/wrapper/iActiveObject.h"
#include "FreeRTOS/wrapper/iTimerObject.h"
#include "FreeRtos/include/semphr.h"

#include "PeripheralDrivers/PortManager.h"

struct SPIMessage {
	uint8_t length;
	uint8_t curPos;
	uint8_t data[255];
	TickType_t timestamp;
};

#define hclk_div_coef			(2)//hardcoded
#define CS_PIN_EABLE				// COMMENT THIS IF YOU DON'T NEED CS PIN
#define LYNXMOTION_JOYSTICK_STYLE	// COMMENT THIS IF YOU NEED ORDINARY SPI

/******* INIT SETTINGS FOR LYXMOTION *******
 * 	direction = SPI_Direction_2Lines_FullDuplex,
 * 	mode = SPI_Mode_Master,
 * 	dataSize = SPI_DataSize_8b,
 * 	clockPolarity = SPI_CPOL_High,
 * 	clockEdge = SPI_CPHA_2Edge,
 * 	slaveSelect = SPI_NSS_Soft,
 * 	baudRatePrescaler = SPI_BaudRatePrescaler_128,
 * 	firstBit =	SPI_FirstBit_LSB,
 * 	crcPoly = 7
 *******************************************/

class SPI: public iActiveObject, public iTimerObject {
public:
	SPIMessage receiveMessageSPI;
	SPIMessage transmittMessageSPI;
	SemaphoreHandle_t transmitterMutexSPI;
	SemaphoreHandle_t transmitterSemaphoreSPI;

	QueueHandle_t queueSendSPI;
	QueueHandle_t queueReceiveSPI;

private:
	GPIO_TypeDef *portMOSI;
	GPIO_TypeDef* portMISO;
	GPIO_TypeDef* portSCK;
	GPIO_TypeDef* portCS;
	uint8_t pinNumberMOSI;
	uint8_t pinNumberMISO;
	uint8_t pinNumberSCK;
	uint8_t pinNumberCS;
	SPI_TypeDef* SPIx;

public:
	void IRQHandler();

	void init(uint16_t direction = SPI_Direction_2Lines_FullDuplex,
			uint16_t mode = SPI_Mode_Master,
			uint16_t dataSize = SPI_DataSize_8b, uint16_t clockPolarity =
					SPI_CPOL_High, uint16_t clockEdge = SPI_CPHA_2Edge,
			uint16_t slaveSelect = SPI_NSS_Soft, uint16_t baudRatePrescaler =
					SPI_BaudRatePrescaler_128, uint16_t firstBit =
					SPI_FirstBit_LSB, uint16_t crcPoly = 7);

	SPI(SPI_TypeDef* SPIx, GPIO_TypeDef *portMOSI = 0,
			uint8_t pinNumberMOSI = 0, GPIO_TypeDef *portMISO = 0,
			uint8_t pinNumberMISO = 0, GPIO_TypeDef *portSCK = 0,
			uint8_t pinNumberSCK = 0, GPIO_TypeDef *portCS = 0,
			uint8_t pinNumberCS = 0);

	void configurePorts(GPIO_TypeDef *portMOSI, uint8_t pinNumberMOSI,
			GPIO_TypeDef *portMISO, uint8_t pinNumberMISO,
			GPIO_TypeDef *portSCK = 0, uint8_t pinNumberSCK = 0,
			GPIO_TypeDef *portCS = 0, uint8_t pinNumberCS = 0);

	void startTransmittion(SPIMessage message, TickType_t maxBlockTime);

	void send(SPIMessage* umessage);

//	void setReceive(FunctionalState state);

	void timerCallbackFunction();

	void setCS();
	void resetCS();

	void run();

	virtual ~SPI();
};

#endif /* SPI_H_ */

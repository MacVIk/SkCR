/*
 * ImuSensor.cpp
 *
 *  Created on: 10 θών. 2019 γ.
 *      Author: Taras.Melnik
 */

#include "ImuSensor.h"

ImuSensor* imuSensor;

ImuSensor::ImuSensor() {
	i2cInit();
	for (uint8_t i = 0; i < 12; i++)
		tByteArr[i] = 0;
}

ImuSensor::~ImuSensor() {
	// TODO Auto-generated destructor stub
}

void ImuSensor::i2cInit()
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

	GPIO_InitTypeDef i2cImuPin;
	i2cImuPin.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	i2cImuPin.GPIO_Mode = GPIO_Mode_AF;
	i2cImuPin.GPIO_OType = GPIO_OType_OD;
	i2cImuPin.GPIO_Speed = GPIO_Speed_100MHz;
	i2cImuPin.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &i2cImuPin);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1);

	I2C_InitTypeDef i2cImu;
	i2cImu.I2C_Mode = I2C_Mode_I2C;
	i2cImu.I2C_ClockSpeed =  100000;
	i2cImu.I2C_Ack = I2C_Ack_Enable;
	i2cImu.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	i2cImu.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_Init(I2C1, &i2cImu);
//	I2C_ITConfig(I2C1, I2C_IT_EVT, ENABLE);
	I2C_Cmd(I2C1, ENABLE);

//	DMA_InitTypeDef dmaRxI2c;
//	NVIC_InitTypeDef dmaRxI2cNvic;
//	DMA_StructInit(&dmaRxI2c);
//	dmaRxI2c.DMA_PeripheralBaseAddr = (uint32_t)&(I2C1->DR);
//	dmaRxI2c.DMA_Memory0BaseAddr = (uint32_t) &(this->i2cRxArr[0]);
//	dmaRxI2c.DMA_Channel = DMA_Channel_1;
//	dmaRxI2c.DMA_DIR = DMA_DIR_PeripheralToMemory;
//	dmaRxI2c.DMA_BufferSize = 20;
//	dmaRxI2c.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//	dmaRxI2c.DMA_MemoryInc = DMA_MemoryInc_Enable;
//	dmaRxI2c.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//	dmaRxI2c.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
//	dmaRxI2c.DMA_Mode = DMA_Mode_Normal;
//	DMA_Init(DMA1_Stream5, &dmaRxI2c);
//	DMA_Cmd(DMA1_Stream5, ENABLE);
//
//	DMA_ITConfig(DMA1_Stream5, DMA_IT_TC, ENABLE);
//	dmaRxI2cNvic.NVIC_IRQChannel = I2C1_EV_IRQn;
//	dmaRxI2cNvic.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&dmaRxI2cNvic);
//	NVIC_SetPriority(I2C1_EV_IRQn, 10);

}

void ImuSensor::getOdometry(uint8_t* byteArr)
{
	for (uint8_t i = 0; i < 12; i++)
		 byteArr[i] = this->tByteArr[i];
}

void ImuSensor::i2cRead(uint8_t slaveAdr, uint8_t subRegAdr, uint8_t regNumb, uint8_t* data)
{
	 uint8_t i = 0;

	 while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
	 I2C_GenerateSTART(I2C1, ENABLE);
	 while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
	 I2C_Send7bitAddress(I2C1, slaveAdr, I2C_Direction_Transmitter);
	 while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
//	 I2C_Cmd(I2C1, ENABLE);
	 I2C_SendData(I2C1, subRegAdr);
	 while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	 I2C_AcknowledgeConfig(I2C1, ENABLE);
	 I2C_GenerateSTART(I2C1, ENABLE);
	 while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
	 I2C_Send7bitAddress(I2C1, slaveAdr, I2C_Direction_Receiver);
	 for (i = 0; i < regNumb - 1; i++) {
		 while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
		 data[i] = I2C_ReceiveData(I2C1);
	 }
	 I2C_AcknowledgeConfig(I2C1, DISABLE);
	 while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
	 data[i++] = I2C_ReceiveData(I2C1);
	 I2C_GenerateSTOP(I2C1, ENABLE);
}

void ImuSensor::i2cWrite(uint8_t slaveAdr, uint8_t subRegAdr, uint8_t data)
{
	 while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
	 I2C_GenerateSTART(I2C1, ENABLE);
	 while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
	 I2C_Send7bitAddress(I2C1, slaveAdr, I2C_Direction_Transmitter);
	 while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	 I2C_Cmd(I2C1, ENABLE);
	 I2C_SendData(I2C1, subRegAdr);
	 while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	 I2C_SendData(I2C1, data);
	 while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	 I2C_GenerateSTOP(I2C1, ENABLE);
}

void ImuSensor::accelInit()
{
	uint8_t cr1Reg = 0b00101111;
	this->i2cWrite(ACCELEROMETER_ADRR, ACCELEROMETER_CTRL_REG1, cr1Reg);
}

int16_t ImuSensor::uint8toInt16(uint8_t* arr)
{
	int16_t dtb = 0;
	dtb |= ((int16_t) arr[0]) << 8;
	dtb |= 0xffff & arr[1];
	return dtb;
}

void ImuSensor::run() {
	uint8_t dataI2c = 0;
	int16_t aXYZ[3] = {0};
	float32_t aXYZf[3] = {0	};
	float32_t multConst = 2.f / 32767.f;
	this->accelInit();

	 while (1) {
		 this->i2cRead(ACCELEROMETER_ADRR, ACCELEROMETER_OUT_X_L | 1 << 7, 6, i2cRxArr);
		 memcpy(aXYZ, i2cRxArr, sizeof(aXYZ));

		 vTaskDelay(10);
	 }
}

extern "C" {

void I2C1_IRQHandler(void)
	{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;			// Notify task about interrupt
		if (USART_GetITStatus(USART3, USART_IT_IDLE)) {			// Clear IDLE flag step 1
			DMA_Cmd(DMA1_Stream1, DISABLE);						// DMA turn off to clear DMA1 counter
			USART_ReceiveData(USART3);							// Clear IDLE flag step 2
		}
		if (USART_GetITStatus(USART3, USART_IT_TC)) {
			USART_ClearITPendingBit(USART3, USART_IT_TC);
		}
		if (xHigherPriorityTaskWoken)
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);		// Run Higher priority task if exist													// between ticks
	}
//***********************DMA_RECEIVE_INTERRUPT******************//
	void DMA1_Stream5_IRQHandler(void)
	{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;			// Notify task about interrupt
		DMA_ClearITPendingBit(DMA1_Stream1, DMA_IT_TCIF1);		// Clear DMA "transmitting complete" interrupt
		DMA_Cmd(DMA1_Stream1, ENABLE);							// Reset DMA
		if (xHigherPriorityTaskWoken)
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);		// Run Higher priority task if exist													// between ticks
	}

}







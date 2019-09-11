/*
 * DriverI2C.cpp
 *
 *  Created on: 10 сент. 2019 г.
 *      Author: Taras.Melnik
 */

#include "DriverI2C.h"
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_i2c.h"
#include "misc.h"

DriverI2C::DriverI2C() {
        // TODO Auto-generated constructor stub

}

DriverI2C::~DriverI2C() {
        // TODO Auto-generated destructor stub
}

void DriverI2C::init_i2c()
{
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

        GPIO_InitTypeDef pin_i2c;
        GPIO_StructInit(&pin_i2c);
        pin_i2c.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
        pin_i2c.GPIO_Mode = GPIO_Mode_AF;
        pin_i2c.GPIO_OType = GPIO_OType_OD;
        pin_i2c.GPIO_Speed = GPIO_Speed_100MHz;
        pin_i2c.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(GPIOB, &pin_i2c);
        GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);
        GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1);

        I2C_InitTypeDef line_1_i2c;
        I2C_StructInit(&line_1_i2c);
        line_1_i2c.I2C_Mode = I2C_Mode_I2C;
        line_1_i2c.I2C_ClockSpeed =  100000;
        line_1_i2c.I2C_Ack = I2C_Ack_Enable;
        line_1_i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
        line_1_i2c.I2C_DutyCycle = I2C_DutyCycle_2;
        I2C_Init(I2C1, &line_1_i2c);
//      I2C_ITConfig(I2C1, I2C_IT_EVT, ENABLE);
        I2C_Cmd(I2C1, ENABLE);

        //      DMA_InitTypeDef dmaRxI2c;
        //      NVIC_InitTypeDef dmaRxI2cNvic;
        //      DMA_StructInit(&dmaRxI2c);
        //      dmaRxI2c.DMA_PeripheralBaseAddr = (uint32_t)&(I2C1->DR);
        //      dmaRxI2c.DMA_Memory0BaseAddr = (uint32_t) &(this->i2cRxArr[0]);
        //      dmaRxI2c.DMA_Channel = DMA_Channel_1;
        //      dmaRxI2c.DMA_DIR = DMA_DIR_PeripheralToMemory;
        //      dmaRxI2c.DMA_BufferSize = 20;
        //      dmaRxI2c.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        //      dmaRxI2c.DMA_MemoryInc = DMA_MemoryInc_Enable;
        //      dmaRxI2c.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
        //      dmaRxI2c.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
        //      dmaRxI2c.DMA_Mode = DMA_Mode_Normal;
        //      DMA_Init(DMA1_Stream5, &dmaRxI2c);
        //      DMA_Cmd(DMA1_Stream5, ENABLE);
        //
        //      DMA_ITConfig(DMA1_Stream5, DMA_IT_TC, ENABLE);
        //      dmaRxI2cNvic.NVIC_IRQChannel = I2C1_EV_IRQn;
        //      dmaRxI2cNvic.NVIC_IRQChannelCmd = ENABLE;
        //      NVIC_Init(&dmaRxI2cNvic);
        //      NVIC_SetPriority(I2C1_EV_IRQn, 10);
}

void DriverI2C::read_i2c(uint8_t slaveAdr, uint8_t subRegAdr, uint8_t regNumb, uint8_t* data)
{
         uint8_t i = 0;

         while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
         I2C_GenerateSTART(I2C1, ENABLE);
         while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
         I2C_Send7bitAddress(I2C1, slaveAdr, I2C_Direction_Transmitter);
         while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
//       I2C_Cmd(I2C1, ENABLE);
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

void DriverI2C::write_i2c(uint8_t slaveAdr, uint8_t subRegAdr, uint8_t data)
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

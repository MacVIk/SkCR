/*
 * DriverUsart.cpp
 *
 *  Created on: 03.04.2019
 *      Author: Taras.Melnik
 */

#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_usart.h"
#include "misc.h"

#include "DriverUsart.h"

DriverUsart::DriverUsart() {
}

DriverUsart::~DriverUsart() {
	// TODO Auto-generated destructor stub
}

 uint8_t DriverUsart::usart_3_Rx_Arr[MAX_MESSEGE_LENGTH_RX];
 uint8_t DriverUsart::usart_3_Tx_Arr[MAX_MESSEGE_LENGTH_RX];
 uint8_t DriverUsart::uart_4_Rx_Arr[MAX_MESSEGE_LENGTH_RX];
 uint8_t DriverUsart::uart_4_Tx_Arr[MAX_MESSEGE_LENGTH_RX];
 uint8_t DriverUsart::usart_6_Rx_Arr[MAX_MESSEGE_LENGTH_RX];
 uint8_t DriverUsart::usart_6_Tx_Arr[MAX_MESSEGE_LENGTH_RX];

void DriverUsart::init_gpio(GPIO_TypeDef* GPIOx, uint16_t rxPin, uint16_t txPin, uint8_t af)
{
	uint8_t rxPinSrs;
	uint8_t txPinSrs;
	if (GPIOx == GPIOA) {
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
		txPinSrs = GPIO_PinSource0;
		rxPinSrs = GPIO_PinSource1;
	} else if (GPIOx == GPIOB) {
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		txPinSrs = GPIO_PinSource10;
		rxPinSrs = GPIO_PinSource11;
	} else if (GPIOx == GPIOC) {
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
		txPinSrs = GPIO_PinSource6;
		rxPinSrs = GPIO_PinSource7;
	}

	GPIO_InitTypeDef UartTxPin;
	GPIO_InitTypeDef UartRxPin;
	//Rx-Tx_Pins//
	UartTxPin.GPIO_Pin = txPin;
	UartTxPin.GPIO_Mode = GPIO_Mode_AF;
	UartTxPin.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOx,&UartTxPin);
	UartRxPin.GPIO_Pin = rxPin;
	UartRxPin.GPIO_Mode  = GPIO_Mode_AF;
	UartRxPin.GPIO_OType = GPIO_OType_OD;
	UartRxPin.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(GPIOx,&UartRxPin);
	GPIO_PinAFConfig(GPIOx, rxPinSrs, af);
	GPIO_PinAFConfig(GPIOx, txPinSrs, af);
}

void DriverUsart::init_dma(USART_TypeDef* USARTx)
{
	DMA_Stream_TypeDef* rxStream;
	DMA_Stream_TypeDef* txStream;
	DMA_InitTypeDef DmaRxUart, DmaTxUart;
	NVIC_InitTypeDef DmaRxNvic, DmaTxNvic;
	IRQn_Type rxNvic;
	IRQn_Type txNvic;
	uint32_t DMA_Chanel;

	if (USARTx == USART3) {
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
		rxStream = DMA1_Stream1;
		rxNvic = DMA1_Stream1_IRQn;
		txStream = DMA1_Stream3;
		txNvic = DMA1_Stream3_IRQn;
		DMA_Chanel = DMA_Channel_4;
	} else if (USARTx == UART4) {
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
		rxStream = DMA1_Stream2;
		rxNvic = DMA1_Stream2_IRQn;
		txStream = DMA1_Stream4;
		txNvic = DMA1_Stream4_IRQn;
		DMA_Chanel = DMA_Channel_4;
	} else if (USARTx == USART6) {
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
		rxStream = DMA2_Stream1;
		rxNvic = DMA2_Stream1_IRQn;
		txStream = DMA2_Stream6;
		txNvic = DMA2_Stream6_IRQn;
		DMA_Chanel = DMA_Channel_5;
	}
	usart.txStream = txStream;

	DMA_StructInit(&DmaRxUart);
	DmaRxUart.DMA_PeripheralBaseAddr = (uint32_t)&(USARTx->DR);
	DmaRxUart.DMA_Memory0BaseAddr = (uint32_t) usart.RxArr;
	DmaRxUart.DMA_Channel = DMA_Chanel;
	DmaRxUart.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DmaRxUart.DMA_BufferSize = MAX_MESSEGE_LENGTH_RX;
	DmaRxUart.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DmaRxUart.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DmaRxUart.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DmaRxUart.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
	DmaRxUart.DMA_Mode = DMA_Mode_Normal;
	DMA_Init(rxStream, &DmaRxUart);
	DMA_Cmd(rxStream, ENABLE);

	DMA_StructInit(&DmaTxUart);
	DmaTxUart.DMA_PeripheralBaseAddr = (uint32_t)&(USARTx->DR);
	DmaTxUart.DMA_Memory0BaseAddr = (uint32_t) usart.TxArr;
	DmaTxUart.DMA_Channel = DMA_Chanel;
	DmaTxUart.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DmaTxUart.DMA_BufferSize = MAX_MESSEGE_LENGTH_TX;
	DmaTxUart.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DmaTxUart.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DmaTxUart.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DmaTxUart.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
	DmaTxUart.DMA_Mode = DMA_Mode_Normal;
	DMA_Init(txStream, &DmaTxUart);
	//DMA_Cmd(txStream, ENABLE);   				// enable in a function

	DMA_ITConfig(rxStream, DMA_IT_TC, ENABLE);
	DmaRxNvic.NVIC_IRQChannel = rxNvic;
	DmaRxNvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&DmaRxNvic);
	NVIC_SetPriority(rxNvic, 10);

	DMA_ITConfig(txStream, DMA_IT_TC, ENABLE);
	DmaTxNvic.NVIC_IRQChannel = txNvic;
	DmaTxNvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&DmaTxNvic);
	NVIC_SetPriority(txNvic, 10);
}

void DriverUsart::init_usart(GPIO_TypeDef* GPIOx, USART_TypeDef* USARTx, bool rsStatus)
{
	USART_InitTypeDef USART;
	NVIC_InitTypeDef USARTnvic;
	IRQn_Type USARTx_IRQn;

//	this->nvicUart = USARTx;

	if (GPIOx == GPIOB && USARTx == USART3) {
		init_gpio(GPIOx, GPIO_Pin_11, GPIO_Pin_10, GPIO_AF_USART3);
	} else if (GPIOx == GPIOC && USARTx == USART6) {
		init_gpio(GPIOx, GPIO_Pin_7, GPIO_Pin_6, GPIO_AF_USART6);
	} else if (GPIOx == GPIOA && USARTx == UART4)
		init_gpio(GPIOx, GPIO_Pin_1, GPIO_Pin_0, GPIO_AF_UART4);

	if (USARTx == USART3) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
		usart.RxArr = &usart_3_Rx_Arr[0];
		usart.TxArr = &usart_3_Tx_Arr[0];
		USARTx_IRQn = USART3_IRQn;
	} else if (USARTx == USART6) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
		usart.RxArr = &usart_6_Rx_Arr[0];
                usart.TxArr = &usart_6_Tx_Arr[0];
		USARTx_IRQn = USART6_IRQn;
	} else if (USARTx == UART4) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
		usart.RxArr = &uart_4_Rx_Arr[0];
		usart.TxArr = &uart_4_Tx_Arr[0];
		USARTx_IRQn = UART4_IRQn;
	}

        init_dma(USARTx);

	USART.USART_BaudRate = 115200;
	USART.USART_StopBits = USART_StopBits_1;
	USART.USART_WordLength = USART_WordLength_8b;
	USART.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART.USART_Parity = USART_Parity_No;
	USART.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USARTx, &USART);
	USART_Cmd(USARTx, ENABLE);

	NVIC_SetPriority(USARTx_IRQn, 10);
	USARTnvic.NVIC_IRQChannel = USARTx_IRQn;
	USARTnvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&USARTnvic);

	USART_ClearITPendingBit(USARTx, USART_IT_TC);
	USART_ClearITPendingBit(USARTx, USART_IT_TXE);
	USART_ClearITPendingBit(USARTx, USART_IT_RXNE);
	USART_ClearITPendingBit(USARTx, USART_IT_IDLE);

	USART_ITConfig(USARTx, USART_IT_IDLE, ENABLE);
	if (rsStatus == true) {
		USART_ITConfig(USARTx, USART_IT_TC, ENABLE);
	}

	USART_DMACmd(USARTx, USART_DMAReq_Rx, ENABLE);
	USART_DMACmd(USARTx, USART_DMAReq_Tx, ENABLE);
}

void DriverUsart::gpioSwitchInit(GPIO_TypeDef* GPIOx, uint16_t swPin)
{
	GPIO_InitTypeDef RxTxPin;
	usart.nvicPin = swPin;
	usart.nvicPort = GPIOx;
	if (GPIOx == GPIOB)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RxTxPin.GPIO_Pin = swPin;
	RxTxPin.GPIO_Mode = GPIO_Mode_OUT;
	RxTxPin.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOx, &RxTxPin);
}

void DriverUsart::usart_send(uint8_t* sendArr, uint8_t length)
{
	if (DMA_GetCmdStatus(usart.txStream))
		DMA_Cmd(usart.txStream, DISABLE);
	GPIO_SetBits(usart.nvicPort, usart.nvicPin);
	for (uint8_t i = 0; i < length; ++i)
	        usart.TxArr[i] = sendArr[i];
	DMA_SetCurrDataCounter(usart.txStream, length);
	DMA_Cmd(usart.txStream, ENABLE);
}
//
//uint8_t DriverUsart::usart_receive_byte()
//{
//	return usart.RxArr[usart.pointer++];
//}
//
//uint8_t DriverUsart::usart_get_recieved_size()
//{
//        return usart.pointer;
//}
//
//void DriverUsart::usart_stop_receiving()
//{
//        for (uint8_t i = 0; i < usart.pointer; ++i)
//                usart.RxArr[usart.pointer] = 0;
//        usart.pointer = 0;
//}







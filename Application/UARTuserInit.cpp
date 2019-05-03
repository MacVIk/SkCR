/*
 * UARTuserInit.cpp
 *
 *  Created on: 03.04.2019
 *      Author: Taras.Melnik
 */

#include "UARTuserInit.h"

UARTuserInit::UARTuserInit() {
	pointer = 0;
}

UARTuserInit::~UARTuserInit() {
	// TODO Auto-generated destructor stub
}

void UARTuserInit::gpioInit(GPIO_TypeDef* GPIOx, uint16_t rxPin, uint16_t txPin, uint8_t af)
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
	UartTxPin.GPIO_Pin 	= txPin;
	UartTxPin.GPIO_Mode = GPIO_Mode_AF;
	UartTxPin.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOx,&UartTxPin);
	UartRxPin.GPIO_Pin	 = rxPin;
	UartRxPin.GPIO_Mode  = GPIO_Mode_AF;
	UartRxPin.GPIO_OType = GPIO_OType_OD;
	UartRxPin.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(GPIOx,&UartRxPin);
	GPIO_PinAFConfig(GPIOx, rxPinSrs, af);
	GPIO_PinAFConfig(GPIOx, txPinSrs, af);
}

void UARTuserInit::dmaInit(USART_TypeDef* USARTx)
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
	this->txStream = txStream;

	DMA_StructInit(&DmaRxUart);
	DmaRxUart.DMA_PeripheralBaseAddr = (uint32_t)&(USARTx->DR);
	DmaRxUart.DMA_Memory0BaseAddr = (uint32_t) &(this->usartRxArr[0]);
	DmaRxUart.DMA_Channel = DMA_Chanel;
	DmaRxUart.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DmaRxUart.DMA_BufferSize = 20;
	DmaRxUart.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DmaRxUart.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DmaRxUart.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DmaRxUart.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
	DmaRxUart.DMA_Mode = DMA_Mode_Normal;
	DMA_Init(rxStream, &DmaRxUart);
	DMA_Cmd(rxStream, ENABLE);

	DMA_StructInit(&DmaTxUart);
	DmaTxUart.DMA_PeripheralBaseAddr = (uint32_t)&(USARTx->DR);
	DmaTxUart.DMA_Memory0BaseAddr = (uint32_t)&(this->usartTxArr[0]);
	DmaTxUart.DMA_Channel = DMA_Chanel;
	DmaTxUart.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DmaTxUart.DMA_BufferSize = 20;
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

void UARTuserInit::uartInit(GPIO_TypeDef* GPIOx, USART_TypeDef* USARTx, bool rsStatus)
{
	USART_InitTypeDef USART;
	NVIC_InitTypeDef USARTnvic;
	IRQn_Type USARTx_IRQn;

//	this->nvicUart = USARTx;
	dmaInit(USARTx);

	if (GPIOx == GPIOB && USARTx == USART3) {
		gpioInit(GPIOx, GPIO_Pin_11, GPIO_Pin_10, GPIO_AF_USART3);
	} else if (GPIOx == GPIOC && USARTx == USART6) {
		gpioInit(GPIOx, GPIO_Pin_7, GPIO_Pin_6, GPIO_AF_USART6);
	} else if (GPIOx == GPIOA && USARTx == UART4)
		gpioInit(GPIOx, GPIO_Pin_1, GPIO_Pin_0, GPIO_AF_UART4);

	if (USARTx == USART3) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
		USARTx_IRQn = USART3_IRQn;
	} else if (USARTx == USART6) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
		USARTx_IRQn = USART6_IRQn;
	} else if (USARTx == UART4) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
		USARTx_IRQn = UART4_IRQn;
	}
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
	if (rsStatus == true){
		USART_ITConfig(USARTx, USART_IT_TC, ENABLE);
	}

	USART_DMACmd(USARTx, USART_DMAReq_Rx, ENABLE);
	USART_DMACmd(USARTx, USART_DMAReq_Tx, ENABLE);
}

void UARTuserInit::gpioSwitchInit(GPIO_TypeDef* GPIOx, uint16_t swPin)
{
	GPIO_InitTypeDef RxTxPin;
	this->nvicPin = swPin;
	this->nvicPort = GPIOx;
	if (GPIOx == GPIOB)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RxTxPin.GPIO_Pin = swPin;
	RxTxPin.GPIO_Mode = GPIO_Mode_OUT;
	RxTxPin.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOx, &RxTxPin);
}
//void UARTuserInit::UARTx_IRQnFunc()
//{
//	if (USART_GetITStatus(nvicUart, USART_IT_IDLE)){			// Clear IDLE flag step 1
//		DMA_Cmd(nvicStream, DISABLE);							// DMA turn off to clear DMA1 counter
//		USART_ReceiveData(nvicUart);							// Clear IDLE flag step 2
//	}
//	if (USART_GetITStatus(nvicUart, USART_IT_TC)){
//		USART_ClearITPendingBit(nvicUart, USART_IT_TC);
//		GPIO_ResetBits(nvicPort, nvicPin);
//	}
//}

void UARTuserInit::send(uint8_t* sendArr, uint8_t length)
{
	if (DMA_GetCmdStatus(txStream))
		DMA_Cmd(txStream, DISABLE);
	GPIO_SetBits(nvicPort, nvicPin);
	if (txStream->M0AR !=(uint32_t) sendArr)
		txStream->M0AR = (uint32_t) sendArr;
	DMA_SetCurrDataCounter(txStream, length);
	DMA_Cmd(txStream, ENABLE);
}

uint8_t UARTuserInit::receiveByte(uint8_t &byte)
{
	byte = this->usartRxArr[pointer];
	this->pointer++;
	return pointer;
}

void UARTuserInit::resetPointer()
{
	pointer = 0;
}






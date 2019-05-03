///*
// * ADCDaemonOnInterrupts.cpp
// *
// *  Created on: 20.08.2015
// *      Author: Tata
// */
//
//#include "ADCManagerOnInterrupts.h"
//
//uint16_t ADCBuffer[] = { 0xAAAA, 0xAAAA, 0xAAAA };
//
//ADCManagerOnInterrupts* ADCManagerOnInterrupts::ADCManagerOnInterruptsInstance =
//		0;
//
//ADCManagerOnInterrupts* ADCManagerOnInterrupts::getInstance() {
//	if (!ADCManagerOnInterruptsInstance) {
//		ADCManagerOnInterruptsInstance = new ADCManagerOnInterrupts();
//	}
//	return ADCManagerOnInterrupts::ADCManagerOnInterruptsInstance;
//}
//
//ADCManagerOnInterrupts::ADCManagerOnInterrupts() {
//	ADC_DeInit();
//
//	ADCInitedChannelCounter[0] = 0;
//	ADCInitedChannelCounter[1] = 0;
//	ADCInitedChannelCounter[2] = 0;
//
//	ADCGlobalQueue = xQueueCreate(6, sizeof(ADCGlobalMessage));
//}
//
//ADCManagerOnInterrupts::~ADCManagerOnInterrupts() {
//
//}
//
//QueueHandle_t ADCManagerOnInterrupts::initChannel(ADC_TypeDef* ADCx,
//		uint8_t channel, uint8_t ADC_SampleTime) {
//
//	uint8_t adcNumber = 0;
//
//	if (ADCx == ADC1) {
//		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
//		adcNumber = 0;
//	} else if (ADCx == ADC2) {
//		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
//		adcNumber = 1;
//	} else if (ADCx == ADC3) {
//		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);
//		adcNumber = 2;
//	} else {
//		return NULL;
//	}
//
//	ADCChannelQueues[adcNumber][ADCInitedChannelCounter[adcNumber]] =
//			xQueueCreate(1, sizeof(ADCMessage));
//
//	ADCInitedChannelCounter[adcNumber]++;
//
//	ADC_RegularChannelConfig(ADCx, channel, ADCInitedChannelCounter[adcNumber],
//			ADC_SampleTime);
//
//	return ADCChannelQueues[adcNumber][ADCInitedChannelCounter[adcNumber] - 1];
//}
//
//void ADCManagerOnInterrupts::sample(ADC_TypeDef* ADCx) {
//	ADC_SoftwareStartConv(ADCx);
//}
//
//void ADCManagerOnInterrupts::start(ADC_TypeDef* ADCx) {
//	DMA_InitTypeDef DMA_InitStruct;
//	NVIC_InitTypeDef NVIC_InitStructure;
//	ADC_CommonInitTypeDef ADC_CommonInitStructure;
//	ADC_InitTypeDef std_adc_adjustment;
//
//	if (ADCx == ADC3) {
//		std_adc_adjustment.ADC_NbrOfConversion = ADCInitedChannelCounter[2];
//		NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream1_IRQn;
//		DMA_InitStruct.DMA_Channel = DMA_Channel_2;
//		DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t) &ADC3->DR; //ADC3's data register
//		DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t) &ADCBuffer[2];
//		DMA_InitStruct.DMA_BufferSize = ADCInitedChannelCounter[2];
//	}
//	if (ADCx == ADC2) {
//		std_adc_adjustment.ADC_NbrOfConversion = ADCInitedChannelCounter[1];
//		NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream2_IRQn;
//		DMA_InitStruct.DMA_Channel = DMA_Channel_1;
//		DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t) &ADC2->DR; //ADC3's data register
//		DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t) &ADCBuffer[1];
//		DMA_InitStruct.DMA_BufferSize = ADCInitedChannelCounter[1];
//	}
//	if (ADCx == ADC1) {
//		std_adc_adjustment.ADC_NbrOfConversion = ADCInitedChannelCounter[0];
//		NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
//		DMA_InitStruct.DMA_Channel = DMA_Channel_0;
//		DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t) &ADC1->DR; //ADC3's data register
//		DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t) &ADCBuffer[0];
//		DMA_InitStruct.DMA_BufferSize = ADCInitedChannelCounter[0];
//	}
//
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
//
//	std_adc_adjustment.ADC_Resolution = ADC_Resolution_12b;
//	std_adc_adjustment.ADC_ScanConvMode = ENABLE;
//	std_adc_adjustment.ADC_ContinuousConvMode = DISABLE;
//	std_adc_adjustment.ADC_ExternalTrigConvEdge = 0; // no external trigger
//	std_adc_adjustment.ADC_ExternalTrigConv = 0;
//	std_adc_adjustment.ADC_DataAlign = ADC_DataAlign_Right;
//	ADC_Init(ADCx, &std_adc_adjustment);
//
//	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
//	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2; // ADCCLK= APB2_FREQ/2
//	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
//	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
//	ADC_CommonInit(&ADC_CommonInitStructure);
//
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
//
//	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
//	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
//	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //Reads 16 bit values
//	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; //Stores 16 bit values
//	DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
//	DMA_InitStruct.DMA_Priority = DMA_Priority_High;
//	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
//	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
//	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
//	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//
//	if (ADCx == ADC3) {
//		DMA_Init(DMA2_Stream1, &DMA_InitStruct);
//		DMA_ITConfig(DMA2_Stream1, DMA_IT_TC, ENABLE);
//		DMA_Cmd(DMA2_Stream1, ENABLE);
//	}
//	if (ADCx == ADC2) {
//		DMA_Init(DMA2_Stream2, &DMA_InitStruct);
//		DMA_ITConfig(DMA2_Stream2, DMA_IT_TC, ENABLE);
//		DMA_Cmd(DMA2_Stream2, ENABLE);
//	}
//	if (ADCx == ADC1) {
//		DMA_Init(DMA2_Stream0, &DMA_InitStruct);
//		DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);
//		DMA_Cmd(DMA2_Stream0, ENABLE);
//	}
//
//	ADC_DMARequestAfterLastTransferCmd(ADCx, ENABLE);
//
//	ADC_DMACmd(ADCx, ENABLE);
//	ADC_Cmd(ADCx, ENABLE);
//}
//
//extern "C" {
//
//uint8_t globalCounter = 0;
//uint8_t globalCounter1 = 0;
//uint8_t globalCounter2 = 0;
//uint8_t globalOVR = 0;
//
//void DMA2_Stream0_IRQHandler() {
//	globalCounter++;
//	BaseType_t xHigherPriorityTaskWoken = 0;
//
//	if (DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0)) {
//		DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);
//
//		ADCGlobalMessage tmpMessage;
//		tmpMessage.ADCx = ADC1;
//		tmpMessage.timeStamp = xTaskGetTickCountFromISR();
//
//		xQueueSendFromISR(ADCManagerOnInterrupts::getInstance()->ADCGlobalQueue,
//				&tmpMessage, &xHigherPriorityTaskWoken);
//	}
//
//	if (xHigherPriorityTaskWoken) {
//		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//	}
//}
//
//void DMA2_Stream1_IRQHandler() {
//	globalCounter1++;
//	BaseType_t xHigherPriorityTaskWoken = 0;
//
//	if (DMA_GetITStatus(DMA2_Stream1, DMA_IT_TCIF1)) {
//		DMA_ClearITPendingBit(DMA2_Stream1, DMA_IT_TCIF1);
//
//		ADCGlobalMessage tmpMessage;
//		tmpMessage.ADCx = ADC3;
//		tmpMessage.timeStamp = xTaskGetTickCountFromISR();
//
//		xQueueSendFromISR(ADCManagerOnInterrupts::getInstance()->ADCGlobalQueue,
//				&tmpMessage, &xHigherPriorityTaskWoken);
//	}
//
//	if (xHigherPriorityTaskWoken) {
//		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//	}
//}
//
//void DMA2_Stream2_IRQHandler() {
//	globalCounter2++;
//	BaseType_t xHigherPriorityTaskWoken = 0;
//
//	if (DMA_GetITStatus(DMA2_Stream2, DMA_IT_TCIF2)) {
//		DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2);
//
//		ADCGlobalMessage tmpMessage;
//		tmpMessage.ADCx = ADC2;
//		tmpMessage.timeStamp = xTaskGetTickCountFromISR();
//
//		xQueueSendFromISR(ADCManagerOnInterrupts::getInstance()->ADCGlobalQueue,
//				&tmpMessage, &xHigherPriorityTaskWoken);
//	}
//
//	if (xHigherPriorityTaskWoken) {
//		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//	}
//}
//
//void ADC_IRQHandler() {
//	globalCounter++;
//	BaseType_t xHigherPriorityTaskWoken = 0;
//
//	ADCGlobalMessage tmpMessage;
//
//	if (ADC_GetITStatus(ADC3, ADC_IT_EOC)) {
//		globalOVR = ADC_GetFlagStatus(ADC3, ADC_FLAG_OVR);
//		ADC_ClearITPendingBit(ADC3, ADC_IT_EOC);
//
//		tmpMessage.ADCx = ADC3;
////		tmpMessage.value = ADC_GetConversionValue(ADC3);
//		xQueueSendFromISR(ADCManagerOnInterrupts::getInstance()->ADCGlobalQueue,
//				&tmpMessage, &xHigherPriorityTaskWoken);
//	}
//
////	xSemaphoreGiveFromISR(ADCDaemonOnInterrupts::ADCSemaphore, &xHigherPriorityTaskWoken);
//
//	if (xHigherPriorityTaskWoken) {
//		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//	}
//}
//}
//

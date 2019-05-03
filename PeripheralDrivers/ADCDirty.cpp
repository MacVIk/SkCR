/*
 * ADCDirty.cpp
 *
 *  Created on: 19.08.2015
 *      Author: Tata
 */

#include "ADCDirty.h"

ADCDirty::ADCDirty(ADC_TypeDef* ADCx, uint8_t channel, GPIO_TypeDef* port,
		uint8_t pinNumber) {
	this->ADCx = ADCx;
	this->channel = channel;
	this->port = port;
	this->pinNumber = pinNumber;

}

ADCDirty::~ADCDirty() {
	// TODO Auto-generated destructor stub
}

void ADCDirty::init() {
	PortManager::initPin(this->port, this->pinNumber,
			GPIO_Mode_AN, GPIO_Speed_2MHz, GPIO_PuPd_NOPULL, GPIO_OType_OD);

	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	ADC_InitTypeDef std_adc_adjustment;

//	ADC_DeInit(); // reset ADC peripheral  VERY DIRTY!!

	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2; // ADCCLK= APB2_FREQ/2
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	std_adc_adjustment.ADC_Resolution = ADC_Resolution_12b;
	std_adc_adjustment.ADC_ScanConvMode = ENABLE;
	std_adc_adjustment.ADC_ContinuousConvMode = DISABLE;
	std_adc_adjustment.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None; // no external trigger
	std_adc_adjustment.ADC_ExternalTrigConv = ADC_ExternalTrigConv_Ext_IT11;
	std_adc_adjustment.ADC_DataAlign = ADC_DataAlign_Right;
	std_adc_adjustment.ADC_NbrOfConversion = 1;

	ADC_Init(ADCx, &std_adc_adjustment);

	ADC_DMACmd(ADCx, DISABLE); // Disable DMA
	ADC_Cmd(ADCx, ENABLE); // Enable ADCx module

	ADC_EOCOnEachRegularChannelCmd(ADCx, ENABLE);
	ADC_ITConfig(ADCx, ADC_IT_EOC, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);

}

uint16_t ADCDirty::sample(uint8_t ADC_SampleTime) {
	if (channel == 14)
		ADC_RegularChannelConfig(ADCx, channel, 1, ADC_SampleTime);
	if (channel == 15)
		ADC_RegularChannelConfig(ADCx, channel, 1, ADC_SampleTime);

	// Clear end of conversion flag
	ADC_ClearFlag(ADCx, ADC_FLAG_EOC);

	// Start conversion
	ADC_SoftwareStartConv(ADCx);

//	// wait for end of conversion
//	while (ADC_GetFlagStatus(ADCx, ADC_FLAG_EOC) != SET) {
//
//	}
//
//	// Clear end of conversion flag
//	ADC_ClearFlag(ADCx, ADC_FLAG_EOC);

	return 0;
}

//extern "C" {
//void ADC_IRQHandler() {
//	if (ADC_GetITStatus(ADC3, ADC_IT_EOC)) {
//		ADC_ClearITPendingBit(ADC3, ADC_IT_EOC);
//		uint16_t value = ADC_GetConversionValue(ADC3);
//	}
//}
//}


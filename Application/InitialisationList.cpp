/*
 * libPWM.cpp
 *
 *  Created on: 01.10.2018
 *      Author: Taras.Melnik
 */

#include "InitialisationList.h"

InitialisationList InitUser;

InitialisationList::InitialisationList()
{}

InitialisationList::~InitialisationList()
{}


//******************GPIO_Initialisation*****************//

void InitialisationList::GPIOPinInit()
{
//******************PinInit to catch Stop of motion********//

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	GPIO_InitTypeDef ButtonInterrupt;
	ButtonInterrupt.GPIO_Pin = GPIO_Pin_15;
	ButtonInterrupt.GPIO_Mode = GPIO_Mode_IN;
	ButtonInterrupt.GPIO_OType = GPIO_OType_OD;
	ButtonInterrupt.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &ButtonInterrupt);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_InitTypeDef testLED;
	testLED.GPIO_Pin = GPIO_Pin_12 || GPIO_Pin_14;
	testLED.GPIO_Mode = GPIO_Mode_OUT;
	testLED.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOD, &testLED);

//******************PinsInit_for_LED_Strip_control********//

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	GPIO_InitTypeDef LED;
	LED.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_15;
	LED.GPIO_Mode = GPIO_Mode_OUT;
	LED.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOA, &LED);

//******************PinsInit_for_Motor_rele********//

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	GPIO_InitTypeDef RelePin;
	RelePin.GPIO_Pin = GPIO_Pin_14;
	RelePin.GPIO_Mode = GPIO_Mode_OUT;
	RelePin.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOC, &RelePin);

//******************PinsInit_for_Motors_config********//

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	GPIO_InitTypeDef MotConfPins;
	MotConfPins.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
	MotConfPins.GPIO_Mode = GPIO_Mode_OUT;
	MotConfPins.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOE, &MotConfPins);
}


//*****************Internal_Timer_Initialisation************************//

// time we get in us.
// Used in ultrasonic sensors

void InitialisationList::TIMInit()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
	TIM_TimeBaseInitTypeDef SimpleCounter6;
	TIM_TimeBaseStructInit(&SimpleCounter6);
	SimpleCounter6.TIM_Prescaler = 84-1;
	TIM_TimeBaseInit(TIM6, &SimpleCounter6);
	TIM_Cmd(TIM6, ENABLE);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
	TIM_TimeBaseInitTypeDef SimpleCounter7;
	TIM_TimeBaseStructInit(&SimpleCounter7);
	SimpleCounter7.TIM_Prescaler = 8400-1;
	TIM_TimeBaseInit(TIM7, &SimpleCounter7);
	TIM_Cmd(TIM7, ENABLE);
}

//*****************PWM_Initialisation************************//

void InitialisationList::PWMInit()
{
	// PWM for LED

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

	GPIO_InitTypeDef PWMPinLED;
	PWMPinLED.GPIO_Pin = GPIO_Pin_2;
	PWMPinLED.GPIO_Mode = GPIO_Mode_AF;
	PWMPinLED.GPIO_OType = GPIO_OType_PP;
	PWMPinLED.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA,&PWMPinLED);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM5);

	TIM_TimeBaseInitTypeDef TIM5CH3;
	TIM5CH3.TIM_Prescaler = 50000-1;
	TIM5CH3.TIM_CounterMode = TIM_CounterMode_Up;
	TIM5CH3.TIM_Period = 2800;
	TIM_TimeBaseInit(TIM5, &TIM5CH3);

	TIM5->CR1 |= TIM_CR1_ARPE;

	TIM_OCInitTypeDef TIM5CH3_PWM;
	TIM5CH3_PWM.TIM_OCMode = TIM_OCMode_PWM1;
	TIM5CH3_PWM.TIM_OCNPolarity = TIM_OCPolarity_High;
	TIM5CH3_PWM.TIM_Pulse = 1400;
	TIM5CH3_PWM.TIM_OutputState = ENABLE;
	TIM_OC3Init(TIM5, &TIM5CH3_PWM);

	// PWM for Motors

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);

	GPIO_InitTypeDef PWMPinMot;
	PWMPinMot.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
	PWMPinMot.GPIO_Mode = GPIO_Mode_AF;
	PWMPinMot.GPIO_OType = GPIO_OType_PP;
	PWMPinMot.GPIO_Speed = GPIO_Speed_2MHz;
//	PWMPinMot.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOE,&PWMPinMot);

	GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_TIM9);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource6, GPIO_AF_TIM9);

	TIM_TimeBaseInitTypeDef TIM9Motors;
	TIM9Motors.TIM_Prescaler = 84-1;
	TIM9Motors.TIM_CounterMode = TIM_CounterMode_Up;
	TIM9Motors.TIM_Period = 1000;
	TIM_TimeBaseInit(TIM9, &TIM9Motors);

	TIM9->CR1 |= TIM_CR1_ARPE;

	TIM_OCInitTypeDef TIM9CH1_PWM;
	TIM9CH1_PWM.TIM_OCMode = TIM_OCMode_PWM1;
	TIM9CH1_PWM.TIM_OCNPolarity = TIM_OCPolarity_High;
	TIM9CH1_PWM.TIM_Pulse = 0;
	TIM9CH1_PWM.TIM_OutputState = ENABLE;
	TIM_OC1Init(TIM9, &TIM9CH1_PWM);

	TIM_OCInitTypeDef TIM9CH2_PWM;
	TIM9CH2_PWM.TIM_OCMode = TIM_OCMode_PWM1;
	TIM9CH2_PWM.TIM_OCNPolarity = TIM_OCPolarity_High;
	TIM9CH2_PWM.TIM_Pulse = 0;
	TIM9CH2_PWM.TIM_OutputState = ENABLE;
	TIM_OC2Init(TIM9, &TIM9CH2_PWM);

	TIM_Cmd(TIM9, ENABLE);

}

void InitialisationList::OnePulseModeInit()
{
	TIM_DeInit(TIM10);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);

	GPIO_InitTypeDef OPMPin;
	OPMPin.GPIO_Pin = GPIO_Pin_8;
	OPMPin.GPIO_Mode = GPIO_Mode_AF;
	OPMPin.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOB,&OPMPin);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM10);

	TIM_TimeBaseInitTypeDef TIM10CH1;
	TIM10CH1.TIM_Prescaler = 84 - 1;
	TIM10CH1.TIM_CounterMode = TIM_CounterMode_Up;
	TIM10CH1.TIM_Period = 0xFFFF;
	TIM_TimeBaseInit(TIM10, &TIM10CH1);

	TIM_OCInitTypeDef TIM10CH1_OPM;
	TIM10CH1_OPM.TIM_OCMode = TIM_OCMode_PWM1;
	TIM10CH1_OPM.TIM_OCNPolarity = TIM_OCPolarity_High;
	TIM10CH1_OPM.TIM_Pulse = 10;
	TIM10CH1_OPM.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC1Init(TIM10, &TIM10CH1_OPM);
	TIM_Cmd(TIM10, ENABLE);

}

void InitialisationList::EncoderInit()
{
	TIM_TypeDef * timArr[2] = {TIM3, TIM4};

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	GPIO_InitTypeDef EncMot1Pin;
	EncMot1Pin.GPIO_Pin 	= GPIO_Pin_6 | GPIO_Pin_7;
	EncMot1Pin.GPIO_Mode 	= GPIO_Mode_AF;
	EncMot1Pin.GPIO_OType 	= GPIO_OType_OD;
//	EncMot1Pin.GPIO_PuPd 	= GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA,&EncMot1Pin);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3);

	GPIO_InitTypeDef EncMot2Pin;
	EncMot2Pin.GPIO_Pin 	= GPIO_Pin_12 | GPIO_Pin_13;
	EncMot2Pin.GPIO_Mode 	= GPIO_Mode_AF;
	EncMot2Pin.GPIO_OType 	= GPIO_OType_OD;
//	EncMot2Pin.GPIO_PuPd 	= GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD,&EncMot2Pin);

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);

//	TIM_TimeBaseInitTypeDef TIM3Enc;
//	TIM3Enc.TIM_Prescaler 	= 84 - 1;
//	TIM3Enc.TIM_CounterMode = TIM_CounterMode_Up;
//	TIM3Enc.TIM_Period 		= 0xffff;
//	TIM_TimeBaseInit(TIM3, &TIM3Enc);
//	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
//	TIM_Cmd(TIM3, ENABLE);
//
//	TIM_DeInit(TIM4);
//
//	TIM_TimeBaseInitTypeDef TIM4Enc;
//	TIM4Enc.TIM_Prescaler 	= 84 - 1;
//	TIM4Enc.TIM_CounterMode = TIM_CounterMode_Up;
//	TIM4Enc.TIM_Period 		= 0xffff;
//	TIM_TimeBaseInit(TIM4, &TIM4Enc);
//	TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
//	TIM_Cmd(TIM4, ENABLE);

	for (volatile uint8_t i = 0; i < 2; i++)
	{
		timArr[i]->PSC = 84 - 1;
		// Timer auto reload register
		timArr[i]->ARR = 0xFFFF;
		timArr[i]->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
		timArr[i]->CCMR1 |= TIM_CCMR1_CC2S_0 | TIM_CCMR1_CC2S_0;
		// Set input polarity (non inverted mode rising edge)
		timArr[i]->CCER &= ~TIM_CCER_CC1NP | ~TIM_CCER_CC1P;
		// Encoder mode 3 (SMS = 0x03), counter counts on both rising and falling edge on both TI1FP2 and TI2FP2
		timArr[i]->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;
		// Enable ticks
		timArr[i]->CR1 |= TIM_CR1_CEN;
	}

}

//void InitialisationList:: USARTInit()
//{
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
//
//	//-----------------DMA_INIT---------------------------
//
//	DMA_InitTypeDef DMA1_UART_RX;
//	DMA_InitTypeDef DMA1_UART_TX;
//
//	DMA_StructInit(&DMA1_UART_RX);
//	DMA1_UART_RX.DMA_PeripheralBaseAddr = (uint32_t)&(USART3->DR);
////	DMA1_UART.DMA_PeripheralDataSize = this->dataRxUARTArr;
//	DMA1_UART_RX.DMA_Memory0BaseAddr = (uint32_t) &(usbUserInterface->usartRxArr[0]);
//	DMA1_UART_RX.DMA_Channel = DMA_Channel_4;
//	DMA1_UART_RX.DMA_DIR = DMA_DIR_PeripheralToMemory;
//	DMA1_UART_RX.DMA_BufferSize = 20;
//	DMA1_UART_RX.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//	DMA1_UART_RX.DMA_MemoryInc = DMA_MemoryInc_Enable;
//	DMA1_UART_RX.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//	DMA1_UART_RX.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
//	DMA1_UART_RX.DMA_Mode = DMA_Mode_Normal;
//	DMA_Init(DMA1_Stream1, &DMA1_UART_RX);
//
//	usbUserInterface->usartTxArr[0] = 254;
//	usbUserInterface->usartTxArr[1] = 13;
//
//	DMA_StructInit(&DMA1_UART_TX);
//	DMA1_UART_TX.DMA_PeripheralBaseAddr = (uint32_t)&(USART3->DR);
////	DMA1_UART.DMA_PeripheralDataSize = this->dataRxUARTArr;
//	DMA1_UART_TX.DMA_Memory0BaseAddr = (uint32_t)&(usbUserInterface->usartTxArr[0]);
//	DMA1_UART_TX.DMA_Channel = DMA_Channel_4;
//	DMA1_UART_TX.DMA_DIR = DMA_DIR_MemoryToPeripheral;
//	DMA1_UART_TX.DMA_BufferSize = 20;
//	DMA1_UART_TX.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//	DMA1_UART_TX.DMA_MemoryInc = DMA_MemoryInc_Enable;
//	DMA1_UART_TX.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//	DMA1_UART_TX.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
//	DMA1_UART_TX.DMA_Mode = DMA_Mode_Normal;
//	DMA_Init(DMA1_Stream3, &DMA1_UART_TX);
//
//	DMA_Cmd(DMA1_Stream1, ENABLE);
////	DMA_Cmd(DMA1_Stream3, ENABLE);
//
//	NVIC_InitTypeDef DmaRxNvic;
//	NVIC_InitTypeDef DmaTxNvic;
//
//	DMA_ITConfig(DMA1_Stream1, DMA_IT_TC, ENABLE);
//
//	/* Enable and set EXTI Line0 Interrupt to the lowest priority */
//	DmaRxNvic.NVIC_IRQChannel = DMA1_Stream1_IRQn;
//	DmaRxNvic.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&DmaRxNvic);
//	NVIC_SetPriority(DMA1_Stream1_IRQn, 10);
//
//	DMA_ITConfig(DMA1_Stream3, DMA_IT_TC, ENABLE);
//
//	DmaTxNvic.NVIC_IRQChannel = DMA1_Stream3_IRQn;
//	DmaTxNvic.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&DmaTxNvic);
//	NVIC_SetPriority(DMA1_Stream3_IRQn, 10);
//	//------------------GPIO_INIT-------------------------
//
//	GPIO_InitTypeDef UART3txPin;
//	UART3txPin.GPIO_Pin 	= GPIO_Pin_10;
//	UART3txPin.GPIO_Mode 	= GPIO_Mode_AF;
//	UART3txPin.GPIO_OType 	= GPIO_OType_PP;
//	UART3txPin.GPIO_PuPd 	= GPIO_PuPd_NOPULL;
//	UART3txPin.GPIO_Speed  	= GPIO_Speed_100MHz;
//	GPIO_Init(GPIOB,&UART3txPin);
//
//
//	GPIO_InitTypeDef UART3rxPin;
//	UART3rxPin.GPIO_Pin 	= GPIO_Pin_11;
//	UART3rxPin.GPIO_Mode 	= GPIO_Mode_AF;
//	UART3rxPin.GPIO_OType 	= GPIO_OType_OD;
//	UART3rxPin.GPIO_PuPd 	= GPIO_PuPd_UP;
//	UART3rxPin.GPIO_Speed  	= GPIO_Speed_100MHz;
//	GPIO_Init(GPIOB,&UART3rxPin);
//
//	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
//	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
//
//	//------------------UART_INIT-------------------------
//
//	USART_InitTypeDef UART3Motor;
//	UART3Motor.USART_BaudRate = 115200;
//	UART3Motor.USART_StopBits = USART_StopBits_1;
//	UART3Motor.USART_WordLength = USART_WordLength_8b;
//	UART3Motor.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
//	UART3Motor.USART_Parity = USART_Parity_No;
//	UART3Motor.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//	USART_Init(USART3, &UART3Motor);
//
//	USART_Cmd(USART3, ENABLE);
//
//	NVIC_InitTypeDef USART3nvic;
//	/* Enable and set EXTI Line0 Interrupt to the lowest priority */
//	USART3nvic.NVIC_IRQChannel = USART3_IRQn;
//	USART3nvic.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&USART3nvic);
//
//	NVIC_SetPriority(USART3_IRQn, 10);
//
//
//	USART_ClearITPendingBit(USART3, USART_IT_TC);
//	USART_ClearITPendingBit(USART3, USART_IT_TXE);
//	USART_ClearITPendingBit(USART3, USART_IT_RXNE);
//	USART_ClearITPendingBit(USART3, USART_IT_IDLE);
//
//	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
////	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
//	USART_ITConfig(USART3, USART_IT_TC, ENABLE);
//
//	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
//	USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);

//}

void InitialisationList::ADCInit()
{
//***************** Turn on clocking on Pin and on ADC ****************/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
//***************** Structures initialization (for STDPerifLib)********/
	GPIO_InitTypeDef BatteryPin;
	ADC_CommonInitTypeDef  BatteryPinADCCoMmon;
	ADC_InitTypeDef BatteryPinADC;
//***************** Analog Input Setting ******************************/
	BatteryPin.GPIO_Pin = GPIO_Pin_0;
	BatteryPin.GPIO_OType = GPIO_OType_OD;
	BatteryPin.GPIO_PuPd = GPIO_PuPd_DOWN;
	BatteryPin.GPIO_Mode = GPIO_Mode_AN;
	GPIO_Init(GPIOC, &BatteryPin);
//***************** ADC common register setting(common settings for all channels)/
	ADC_CommonStructInit(&BatteryPinADCCoMmon);
	BatteryPinADCCoMmon.ADC_Prescaler = ADC_Prescaler_Div8;
	ADC_CommonInit(&BatteryPinADCCoMmon);
//***************** ADC1 settings
//	(in this case single measurement on regular channel)***************/
	ADC_StructInit(&BatteryPinADC);
	BatteryPinADC.ADC_Resolution = ADC_Resolution_8b;
	ADC_Init(ADC1, &BatteryPinADC);
//***************** Specific Channel setting **************************/
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_TwoSamplingDelay_10Cycles);
//***************** To Turn on ADC1 ***********************************/
	ADC_Cmd(ADC1, ENABLE);
}

void InitialisationList::InterruptInit()
{
	GPIO_InitTypeDef SensPin;
	EXTI_InitTypeDef SensInterrupt;
	NVIC_InitTypeDef SensNvic;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	SensPin.GPIO_Mode = GPIO_Mode_IN;
	SensPin.GPIO_OType = GPIO_OType_OD;
	SensPin.GPIO_PuPd = GPIO_PuPd_DOWN;
	SensPin.GPIO_Pin =  GPIO_Pin_10 | GPIO_Pin_11 |GPIO_Pin_12 |GPIO_Pin_13 |
			GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_Init(GPIOE, &SensPin);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource10);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource11);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource12);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource13);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource14);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource15);

	SensInterrupt.EXTI_Line = EXTI_Line10 | EXTI_Line11| EXTI_Line12 | EXTI_Line13 |
			EXTI_Line14 | EXTI_Line15;
	SensInterrupt.EXTI_Mode = EXTI_Mode_Interrupt;
	SensInterrupt.EXTI_Trigger = EXTI_Trigger_Falling;
	SensInterrupt.EXTI_LineCmd = ENABLE;
	EXTI_Init(&SensInterrupt);

	SensNvic.NVIC_IRQChannel = EXTI15_10_IRQn;
	SensNvic.NVIC_IRQChannelPreemptionPriority = 0x01;
	SensNvic.NVIC_IRQChannelSubPriority = 0x01;
	SensNvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&SensNvic);

}

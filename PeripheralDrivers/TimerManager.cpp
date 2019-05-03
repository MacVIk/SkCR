/*
 * TimerManager.cpp
 *
 *  Created on: Aug 13, 2015
 *      Author: asquad
 */

#include "TimerManager.h"

uint32_t counter = 0;

extern "C" {
void TIM6_DAC_IRQHandler() {
	if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET) {
		TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
		counter++;
	}
}
}

void TimerManager::init(TIM_TypeDef* timx, uint16_t counterMode,
		uint16_t prescaler, uint32_t period, uint16_t clockDivision,
		uint8_t repetitionCounter) {

	//uint16_t ITstatus = 0x0000;

	uint16_t tmpcr1 = 0;

	tmpcr1 = timx->CR1;

	if ((timx == TIM1) || (timx == TIM8) || (timx == TIM2) || (timx == TIM3)
			|| (timx == TIM4) || (timx == TIM5)) {
		/* Select the Counter Mode */
		tmpcr1 &= (uint16_t) (~(TIM_CR1_DIR | TIM_CR1_CMS));
		tmpcr1 |= (uint32_t) counterMode;
	}

	if ((timx != TIM6) && (timx != TIM7)) {
		/* Set the clock division */
		tmpcr1 &= (uint16_t) (~TIM_CR1_CKD);
		tmpcr1 |= (uint32_t) clockDivision;
	}

	timx->CR1 = tmpcr1;

	/* Set the Autoreload value */
	timx->ARR = period;

	/* Set the Prescaler value */
	timx->PSC = prescaler;

	if ((timx == TIM1) || (timx == TIM8)) {
		/* Set the Repetition Counter value */
		timx->RCR = repetitionCounter;
	}

	/* Generate an update event to reload the Prescaler
	 and the repetition counter(only for TIM1 and TIM8) value immediatly */
	timx->EGR = TIM_PSCReloadMode_Immediate;
}

void TimerManager::init(TIM_TypeDef* timx, uint16_t prescaler,
		uint32_t period) {
	init(timx, TIM_CounterMode_Up, prescaler, period, TIM_CKD_DIV1, 0x0000);
}

void TimerManager::init(TIM_TypeDef* timx, uint32_t period_usec) {

	uint32_t prescaler = (uint32_t) (1 + (period_usec * 168) / (65535 * 2));
	uint32_t length = (uint32_t) ((period_usec * 168) / (2 * prescaler));

	setPrescaler(timx, prescaler - 1);
	setAutoreload(timx, length);

}

void TimerManager::ITenable(TIM_TypeDef* timx, uint16_t IT) {
	timx->DIER |= IT;
}

void TimerManager::ITdisable(TIM_TypeDef* timx, uint16_t IT) {
	timx->DIER &= (uint16_t) ~IT;
}

void TimerManager::clearITbit(TIM_TypeDef* timx, uint16_t IT) {
	timx->SR = (uint16_t) ~IT;
}

uint8_t TimerManager::getITstatus(TIM_TypeDef* timx, uint16_t IT) {
	uint16_t itstatus = 0x0, itenable = 0x0;
	uint8_t bitstatus;

	itstatus = timx->SR & IT;
	itenable = timx->DIER & IT;

	if ((itstatus != (uint16_t) 0x00) && (itenable != (uint16_t) 0x00)) {
		bitstatus = 1;
	} else {
		bitstatus = 0;
	}
	return bitstatus;
}

void TimerManager::OCinit(TIM_TypeDef* timx, uint8_t channel, uint16_t OCMode,
		uint16_t outputState, uint16_t outputNState, uint32_t pulse,
		uint16_t OCPolarity, uint16_t OCNPolarity, uint16_t OCIdleState,
		uint16_t OCNIdleState) {

//	uint8_t timNum;
//	if (timx == TIM1) {
//		timNum = 0;
//	} else if (timx == TIM2) {
//		timNum = 1;
//	} else if (timx == TIM3) {
//		timNum = 2;
//	} else if (timx == TIM4) {
//		timNum = 3;
//	} else if (timx == TIM5) {
//		timNum = 4;
//	} else if (timx == TIM6) {
//		timNum = 5;
//	} else if (timx == TIM7) {
//		timNum = 6;
//	} else if (timx == TIM8) {
//		timNum = 7;
//	} else if (timx == TIM9) {
//		timNum = 8;
//	} else if (timx == TIM10) {
//		timNum = 9;
//	} else if (timx == TIM11) {
//		timNum = 10;
//	} else if (timx == TIM12) {
//		timNum = 11;
//	}

	if (channel == 1) {
		uint16_t tmpccmrx = 0, tmpccer = 0, tmpcr2 = 0;

		/* Disable the Channel 1: Reset the CC1E Bit */
		timx->CCER &= (uint16_t) ~TIM_CCER_CC1E;

		/* Get the timx CCER register value */
		tmpccer = timx->CCER;
		/* Get the timx CR2 register value */
		tmpcr2 = timx->CR2;

		/* Get the timx CCMR1 register value */
		tmpccmrx = timx->CCMR1;

		/* Reset the Output Compare Mode Bits */
		tmpccmrx &= (uint16_t) ~TIM_CCMR1_OC1M;
		tmpccmrx &= (uint16_t) ~TIM_CCMR1_CC1S;
		/* Select the Output Compare Mode */
		tmpccmrx |= OCMode;

		/* Reset the Output Polarity level */
		tmpccer &= (uint16_t) ~TIM_CCER_CC1P;
		/* Set the Output Compare Polarity */
		tmpccer |= OCPolarity;

		/* Set the Output State */
		tmpccer |= outputState;

		if ((timx == TIM1) || (timx == TIM8)) {
			/* Reset the Output N Polarity level */
			tmpccer &= (uint16_t) ~TIM_CCER_CC1NP;
			/* Set the Output N Polarity */
			tmpccer |= OCNPolarity;
			/* Reset the Output N State */
			tmpccer &= (uint16_t) ~TIM_CCER_CC1NE;

			/* Set the Output N State */
			tmpccer |= outputNState;
			/* Reset the Output Compare and Output Compare N IDLE State */
			tmpcr2 &= (uint16_t) ~TIM_CR2_OIS1;
			tmpcr2 &= (uint16_t) ~TIM_CR2_OIS1N;
			/* Set the Output Idle state */
			tmpcr2 |= OCIdleState;
			/* Set the Output N Idle state */
			tmpcr2 |= OCNIdleState;
		}
		/* Write to timx CR2 */
		timx->CR2 = tmpcr2;

		/* Write to timx CCMR1 */
		timx->CCMR1 = tmpccmrx;

		/* Set the Capture Compare Register value */
		timx->CCR1 = pulse;

		/* Write to timx CCER */
		timx->CCER = tmpccer;
//		compareIdeal[timNum][0] = pulse;
	}
	if (channel == 2) {
		uint16_t tmpccmrx = 0, tmpccer = 0, tmpcr2 = 0;

		/* Disable the Channel 1: Reset the CC1E Bit */
		timx->CCER &= (uint16_t) ~TIM_CCER_CC2E;

		/* Get the timx CCER register value */
		tmpccer = timx->CCER;
		/* Get the timx CR2 register value */
		tmpcr2 = timx->CR2;

		/* Get the timx CCMR1 register value */
		tmpccmrx = timx->CCMR1;

		/* Reset the Output Compare Mode Bits */
		tmpccmrx &= (uint16_t) ~TIM_CCMR1_OC2M;
		tmpccmrx &= (uint16_t) ~TIM_CCMR1_CC2S;
		/* Select the Output Compare Mode */
		tmpccmrx |= (uint16_t) (OCMode << 8);

		/* Reset the Output Polarity level */
		tmpccer &= (uint16_t) ~TIM_CCER_CC2P;
		/* Set the Output Compare Polarity */
		tmpccer |= (uint16_t) (OCPolarity << 4);

		/* Set the Output State */
		tmpccer |= (uint16_t) (outputState << 4);

		if ((timx == TIM1) || (timx == TIM8)) {
			/* Reset the Output N Polarity level */
			tmpccer &= (uint16_t) ~TIM_CCER_CC2NP;
			/* Set the Output N Polarity */
			tmpccer |= (uint16_t) (OCNPolarity << 4);
			/* Reset the Output N State */
			tmpccer &= (uint16_t) ~TIM_CCER_CC2NE;

			/* Set the Output N State */
			tmpccer |= (uint16_t) (outputNState << 4);
			/* Reset the Output Compare and Output Compare N IDLE State */
			tmpcr2 &= (uint16_t) ~TIM_CR2_OIS2;
			tmpcr2 &= (uint16_t) ~TIM_CR2_OIS2N;
			/* Set the Output Idle state */
			tmpcr2 |= (uint16_t) (OCIdleState << 2);
			/* Set the Output N Idle state */
			tmpcr2 |= (uint16_t) (OCNIdleState << 2);
		}
		/* Write to timx CR2 */
		timx->CR2 = tmpcr2;

		/* Write to timx CCMR1 */
		timx->CCMR1 = tmpccmrx;

		/* Set the Capture Compare Register value */
		timx->CCR2 = pulse;

		/* Write to timx CCER */
		timx->CCER = tmpccer;
//		compareIdeal[timNum][1] = pulse;
	}
	if (channel == 3) {
		uint16_t tmpccmrx = 0, tmpccer = 0, tmpcr2 = 0;

		/* Disable the Channel 1: Reset the CC1E Bit */
		timx->CCER &= (uint16_t) ~TIM_CCER_CC3E;

		/* Get the timx CCER register value */
		tmpccer = timx->CCER;
		/* Get the timx CR2 register value */
		tmpcr2 = timx->CR2;

		/* Get the timx CCMR1 register value */
		tmpccmrx = timx->CCMR2;

		/* Reset the Output Compare Mode Bits */
		tmpccmrx &= (uint16_t) ~TIM_CCMR2_OC3M;
		tmpccmrx &= (uint16_t) ~TIM_CCMR2_CC3S;
		/* Select the Output Compare Mode */
		tmpccmrx |= OCMode;

		/* Reset the Output Polarity level */
		tmpccer &= (uint16_t) ~TIM_CCER_CC3P;
		/* Set the Output Compare Polarity */
		tmpccer |= (uint16_t) (OCPolarity << 8);

		/* Set the Output State */
		tmpccer |= (uint16_t) (outputState << 8);

		if ((timx == TIM1) || (timx == TIM8)) {
			/* Reset the Output N Polarity level */
			tmpccer &= (uint16_t) ~TIM_CCER_CC3NP;
			/* Set the Output N Polarity */
			tmpccer |= (uint16_t) (OCNPolarity << 8);
			/* Reset the Output N State */
			tmpccer &= (uint16_t) ~TIM_CCER_CC3NE;

			/* Set the Output N State */
			tmpccer |= (uint16_t) (outputNState << 8);
			/* Reset the Output Compare and Output Compare N IDLE State */
			tmpcr2 &= (uint16_t) ~TIM_CR2_OIS3;
			tmpcr2 &= (uint16_t) ~TIM_CR2_OIS3N;
			/* Set the Output Idle state */
			tmpcr2 |= (uint16_t) (OCIdleState << 4);
			/* Set the Output N Idle state */
			tmpcr2 |= (uint16_t) (OCNIdleState << 4);
		}
		/* Write to timx CR2 */
		timx->CR2 = tmpcr2;

		/* Write to timx CCMR1 */
		timx->CCMR2 = tmpccmrx;

		/* Set the Capture Compare Register value */
		timx->CCR3 = pulse;

		/* Write to timx CCER */
		timx->CCER = tmpccer;
//		compareIdeal[timNum][2] = pulse;
	}
	if (channel == 4) {
		uint16_t tmpccmrx = 0, tmpccer = 0, tmpcr2 = 0;

		/* Disable the Channel 1: Reset the CC1E Bit */
		timx->CCER &= (uint16_t) ~TIM_CCER_CC4E;

		/* Get the timx CCER register value */
		tmpccer = timx->CCER;
		/* Get the timx CR2 register value */
		tmpcr2 = timx->CR2;

		/* Get the timx CCMR1 register value */
		tmpccmrx = timx->CCMR2;

		/* Reset the Output Compare Mode Bits */
		tmpccmrx &= (uint16_t) ~TIM_CCMR2_OC4M;
		tmpccmrx &= (uint16_t) ~TIM_CCMR2_CC4S;
		/* Select the Output Compare Mode */
		tmpccmrx |= (uint16_t) (OCMode << 8);

		/* Reset the Output Polarity level */
		tmpccer &= (uint16_t) ~TIM_CCER_CC4P;
		/* Set the Output Compare Polarity */
		tmpccer |= (uint16_t) (OCPolarity << 12);

		/* Set the Output State */
		tmpccer |= (uint16_t) (outputState << 12);

		if ((timx == TIM1) || (timx == TIM8)) {
			tmpcr2 &= (uint16_t) ~TIM_CR2_OIS4;
			/* Set the Output Idle state */
			tmpcr2 |= (uint16_t) (OCIdleState << 6);
		}
		/* Write to timx CR2 */
		timx->CR2 = tmpcr2;

		/* Write to timx CCMR1 */
		timx->CCMR2 = tmpccmrx;

		/* Set the Capture Compare Register value */
		timx->CCR4 = pulse;

		/* Write to timx CCER */
		timx->CCER = tmpccer;
//		compareIdeal[timNum][3] = pulse;
	}
}

void TimerManager::OCinit(TIM_TypeDef* timx, uint8_t channel, uint16_t OCMode,
		uint16_t OutputState) {
	OCinit(timx, channel, OCMode, OutputState, TIM_OutputNState_Disable,
			0x00000000, TIM_OCPolarity_High, TIM_OCPolarity_High,
			TIM_OCIdleState_Reset, TIM_OCNIdleState_Reset);
}

void TimerManager::OCinit(TIM_TypeDef* timx, uint8_t channel, uint16_t OCMode,
		uint16_t outputState, uint32_t pulse) {
	OCinit(timx, channel, OCMode, outputState, TIM_OutputNState_Disable, pulse,
			TIM_OCPolarity_High, TIM_OCPolarity_High, TIM_OCIdleState_Reset,
			TIM_OCNIdleState_Reset);
}

void TimerManager::prescalerConfig(TIM_TypeDef* timx, uint16_t prescaler,
		uint16_t PSCreloadMode) {
	/* Set the Prescaler value */
	timx->PSC = prescaler;
	/* Set or reset the UG Bit */
	timx->EGR = PSCreloadMode;
}

void TimerManager::setCounterMode(TIM_TypeDef* timx, uint16_t counterMode) {
	if (counterMode == TIM_CounterMode_Up || counterMode == TIM_CounterMode_Down
			|| counterMode == TIM_CounterMode_CenterAligned1
			|| counterMode == TIM_CounterMode_CenterAligned2
			|| counterMode == TIM_CounterMode_CenterAligned3) {
		uint16_t tmpcr1 = 0;

		tmpcr1 = timx->CR1;

		/* Reset the CMS and DIR Bits */
		tmpcr1 &= (uint16_t) ~(TIM_CR1_DIR | TIM_CR1_CMS);

		/* Set the Counter Mode */
		tmpcr1 |= counterMode;

		/* Write to TIMx CR1 register */
		timx->CR1 = tmpcr1;
	}
}

void TimerManager::setCounter(TIM_TypeDef* timx, uint32_t counter) {
	timx->CNT = counter;
}

void TimerManager::setPrescaler(TIM_TypeDef* timx, uint16_t prescaler) {
	timx->PSC = prescaler;
}

void TimerManager::setAutoreload(TIM_TypeDef* timx, uint32_t arr) {
	timx->ARR = arr;
}

uint32_t TimerManager::getCounter(TIM_TypeDef* timx) {
	return timx->CNT;
}

uint16_t TimerManager::getPrescaler(TIM_TypeDef* timx) {
	return timx->PSC;
}

uint32_t TimerManager::getAutoreload(TIM_TypeDef* timx) {
	return timx->ARR;
}

void TimerManager::setCompare(TIM_TypeDef* timx, uint8_t channel,
		uint32_t value) {
	if (channel == 1) {
		timx->CCR1 = value;
	}
	if (channel == 2) {
		timx->CCR2 = value;
	}
	if (channel == 3) {
		timx->CCR3 = value;
	}
	if (channel == 4) {
		timx->CCR4 = value;
	}
}

void TimerManager::enable(TIM_TypeDef* timx) {
	timx->EGR = TIM_PSCReloadMode_Immediate;
	timx->CR1 |= TIM_CR1_CEN;
}

void TimerManager::disable(TIM_TypeDef* timx) {
	timx->CR1 &= (uint16_t) ~TIM_CR1_CEN;
}

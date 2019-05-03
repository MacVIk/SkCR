/*
 * i2c_timer.c
 *
 *  Created on: Apr 9, 2013
 *      Author: Admin
 */
#include "i2c_timer.h"
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"

// I2C timer selection
#define RMOD_I2C_TIMER TIM4
#define RMOD_I2C_TIMER_RCC RCC_APB1Periph_TIM4

//////////////////////////////////////////////////////////////////////////////////////////////

// Initialize timer2
void i2c_timer_init (void) {

	/* Timer Periph clock enable */
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(RMOD_I2C_TIMER_RCC, ENABLE);

	/* Time base configuration */
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_Prescaler = 0xf;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV4;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(RMOD_I2C_TIMER, &TIM_TimeBaseStructure);

	TIM_SelectOnePulseMode(RMOD_I2C_TIMER, TIM_OPMode_Single);
}//i2c_timer_init

//////////////////////////////////////////////////////////////////////////////////////////////

// Wait timeout t
void i2c_timer_timeout (unsigned int t) {

	RMOD_I2C_TIMER->SR &= ~0x01;
	RMOD_I2C_TIMER->ARR = t;
	RMOD_I2C_TIMER->CNT = 0;
	TIM_Cmd(RMOD_I2C_TIMER, ENABLE);
	while ((RMOD_I2C_TIMER->SR & 0x01) == 0);
	TIM_Cmd(RMOD_I2C_TIMER, DISABLE);
	RMOD_I2C_TIMER->SR &= ~0x01;
}//i2c_timer_timeout

//////////////////////////////////////////////////////////////////////////////////////////////

// Start timeout t
void i2c_timer_start_timeout (unsigned int t) {

	RMOD_I2C_TIMER->SR &= ~0x01;
	RMOD_I2C_TIMER->ARR = t;
	RMOD_I2C_TIMER->CNT = 0;
	TIM_Cmd(RMOD_I2C_TIMER, ENABLE);
}//i2c_timer_start_timeout

//////////////////////////////////////////////////////////////////////////////////////////////

// Stop timeout t
void i2c_timer_stop_timeout (void) {

	TIM_Cmd(RMOD_I2C_TIMER, DISABLE);
	RMOD_I2C_TIMER->SR &= ~0x01;
}//i2c_timer_stop_timeout

//////////////////////////////////////////////////////////////////////////////////////////////

// Read timeout status 1 - owerflow, 0 - not owerflow
unsigned char i2c_timer_timeout_status (void) {

	return (RMOD_I2C_TIMER->SR & 0x01);
}//i2c_timer_timeout_status


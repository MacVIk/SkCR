/*
 * TimerManager.h
 *
 *  Created on: Aug 13, 2015
 *      Author: asquad
 */

#ifndef TIMERMANAGER_H_
#define TIMERMANAGER_H_

#include "stm32f4xx.h"
#include "FreeRTOS.h"

#define TIM_CounterMode_Up                 ((uint16_t)0x0000)
#define TIM_CounterMode_Down               ((uint16_t)0x0010)
#define TIM_CounterMode_CenterAligned1     ((uint16_t)0x0020)
#define TIM_CounterMode_CenterAligned2     ((uint16_t)0x0040)
#define TIM_CounterMode_CenterAligned3     ((uint16_t)0x0060)

#define TIM_IT_Update                      ((uint16_t)0x0001)
#define TIM_IT_CC1                         ((uint16_t)0x0002)
#define TIM_IT_CC2                         ((uint16_t)0x0004)
#define TIM_IT_CC3                         ((uint16_t)0x0008)
#define TIM_IT_CC4                         ((uint16_t)0x0010)
#define TIM_IT_COM                         ((uint16_t)0x0020)
#define TIM_IT_Trigger                     ((uint16_t)0x0040)
#define TIM_IT_Break                       ((uint16_t)0x0080)

#define TIM_CKD_DIV1                       ((uint16_t)0x0000)
#define TIM_CKD_DIV2                       ((uint16_t)0x0100)
#define TIM_CKD_DIV4                       ((uint16_t)0x0200)

#define TIM_PSCReloadMode_Update           ((uint16_t)0x0000)
#define TIM_PSCReloadMode_Immediate        ((uint16_t)0x0001)

#define TIM_OCMode_Timing                  ((uint16_t)0x0000)
#define TIM_OCMode_Active                  ((uint16_t)0x0010)
#define TIM_OCMode_Inactive                ((uint16_t)0x0020)
#define TIM_OCMode_Toggle                  ((uint16_t)0x0030)
#define TIM_OCMode_PWM1                    ((uint16_t)0x0060)
#define TIM_OCMode_PWM2                    ((uint16_t)0x0070)

#define TIM_OCPolarity_High                ((uint16_t)0x0000)
#define TIM_OCPolarity_Low                 ((uint16_t)0x0002)

#define TIM_OutputState_Disable            ((uint16_t)0x0000)
#define TIM_OutputState_Enable             ((uint16_t)0x0001)

#define TIM_OutputNState_Disable           ((uint16_t)0x0000)
#define TIM_OutputNState_Enable            ((uint16_t)0x0004)

#define TIM_OCIdleState_Set                ((uint16_t)0x0100)
#define TIM_OCIdleState_Reset              ((uint16_t)0x0000)

#define TIM_OCNIdleState_Set               ((uint16_t)0x0200)
#define TIM_OCNIdleState_Reset             ((uint16_t)0x0000)

#define TIM_CounterMode_Up                 ((uint16_t)0x0000)
#define TIM_CounterMode_Down               ((uint16_t)0x0010)
#define TIM_CounterMode_CenterAligned1     ((uint16_t)0x0020)
#define TIM_CounterMode_CenterAligned2     ((uint16_t)0x0040)
#define TIM_CounterMode_CenterAligned3     ((uint16_t)0x0060)

typedef enum {
	ITprescaler = 0,
	ITcounter = 1,
	ITautoreload = 2,
	ITcompare1 = 3,
	ITcompare2 = 4,
	ITcompare3 = 5,
	ITcompare4 = 6,
	ITcompareSBS1 = 7,
	ITcompareSBS2 = 8,
	ITcompareSBS3 = 9,
	ITcompareSBS4 = 10,
//	ITmodbus = 11,
//	ITmovement = 12,
} ITtype;

class TimerManager {
private:
//	TimerManager() {
//	}
//	TimerManager(const TimerManager&);
//	TimerManager& operator=(TimerManager&);

//	float compareIdeal[12][4];

public:
	static void init(TIM_TypeDef* timx, uint16_t counterMode,
			uint16_t prescaler, uint32_t period, uint16_t clockDivision,
			uint8_t repetitionCounter);
	static void init(TIM_TypeDef* timx, uint16_t prescaler, uint32_t period);
	static void init(TIM_TypeDef* timx, uint32_t period);

	static void ITenable(TIM_TypeDef* tim, uint16_t IT);
	static void ITdisable(TIM_TypeDef* tim, uint16_t IT);
	static void clearITbit(TIM_TypeDef* tim, uint16_t IT);
	static uint8_t getITstatus(TIM_TypeDef* tim, uint16_t IT);

	//void ITUpdate(TIM_TypeDef* tim);???????????????

	static void OCinit(TIM_TypeDef* tim, uint8_t channel, uint16_t OCMode,
			uint16_t outputState, uint16_t outputNState, uint32_t pulse,
			uint16_t OCPolarity, uint16_t OCNPolarity, uint16_t OCIdleState,
			uint16_t OCNIdleState);
	static void OCinit(TIM_TypeDef* tim, uint8_t channel, uint16_t TIM_OCMode,
			uint16_t TIM_OutputState);
	static void OCinit(TIM_TypeDef* tim, uint8_t channel, uint16_t OCMode,
			uint16_t outputState, uint32_t pulse);

	static void prescalerConfig(TIM_TypeDef* tim, uint16_t prescaler,
			uint16_t PSCreloadMode);
	static void setCounterMode(TIM_TypeDef* tim, uint16_t counterMode);
	static void setCounter(TIM_TypeDef* tim, uint32_t counter);
	static void setPrescaler(TIM_TypeDef* tim, uint16_t prescaler);
	static void setAutoreload(TIM_TypeDef* tim, uint32_t arr);
	static void setCompare(TIM_TypeDef* tim, uint8_t channel, uint32_t value);

	static uint32_t getCounter(TIM_TypeDef* tim);
	static uint16_t getPrescaler(TIM_TypeDef* tim);
	static uint32_t getAutoreload(TIM_TypeDef* tim);
	static uint32_t getCompare(TIM_TypeDef* tim);

	static void enable(TIM_TypeDef* tim);
	static void disable(TIM_TypeDef* tim);
};

#endif /* TIMERMANAGER_H_ */

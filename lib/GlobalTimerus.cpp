/*
 * GlobalTimerus.cpp
 *
 *  Created on: 25 сент. 2019 г.
 *      Author: Taras.Melnik
 */

#include "GlobalTimerus.h"
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "misc.h"

/* For fast interrupt increment */
static uint64_t highByte {0};

GlobalTimer_us* GlobalTimer_us::mClassPointer = {nullptr};

GlobalTimer_us::GlobalTimer_us() : tickValue(0)
{
        timer_init();
        interrupt_init();
}

GlobalTimer_us::~GlobalTimer_us()
{
       // ToDo is it correct?
        delete this;
}

GlobalTimer_us* GlobalTimer_us::get_instance()
{
        if (mClassPointer == nullptr) {
                mClassPointer = new GlobalTimer_us();
        }
        return mClassPointer;
}

uint64_t GlobalTimer_us::get_time()
{
        static uint64_t tickBuff {0};

        //ToDO check is it rvalue?
        tickBuff = highByte << 16;
        tickBuff |= TIM6->CNT;
        tickValue = tickBuff;

        return tickValue;
}

void GlobalTimer_us::timer_init()
{
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

        TIM_TimeBaseInitTypeDef tim6;
        TIM_TimeBaseStructInit(&tim6);

        /* Tick every 10 us */
        tim6.TIM_Prescaler = 840 - 1;

        /* Max ticks number */
        tim6.TIM_Period = 0xffff;
        TIM_TimeBaseInit(TIM6, &tim6);
        TIM_Cmd(TIM6, ENABLE);
}

void GlobalTimer_us::interrupt_init()
{
        NVIC_InitTypeDef interruptTim6;

        interruptTim6.NVIC_IRQChannel = TIM6_DAC_IRQn;

        /* The interrupt level is higher then FreeRtos
         * The interrupt body should be as small as possible */
        interruptTim6.NVIC_IRQChannelPreemptionPriority = (uint8_t) 2;
        interruptTim6.NVIC_IRQChannelSubPriority = (uint8_t) 1;
        interruptTim6.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&interruptTim6);
}

extern "C" {
void TIM6_DAC_IRQHandler()
{
        //ToDo Check is it enough time to clean
        TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
        ++highByte;
}
}



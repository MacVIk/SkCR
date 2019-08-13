/*
 * CollisionAvoidance.cpp
 *
 *  Created on: 15.01.2019
 *      Author: Taras.Melnik
 */

#include "RangefinderManager.h"

#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "misc.h"
#include "stm32f4xx_syscfg.h"
#include "stm32f4xx_exti.h"

#include "FreeRTOS.h"
#include "task.h"

RangefinderManager rf_manager;

static uint16_t sensTimeArr[RANGEFINDERS_NUMBER];

uint8_t RangefinderManager::outDistArr[RANGEFINDERS_NUMBER];
uint8_t* RangefinderManager::sensDistArr[RANGEFINDERS_NUMBER];

RangefinderManager::RangefinderManager()
{
        //      xRengefindersHighQueue = xQueueCreate(RANGEFINDERS_NUMBER, sizeof(uint8_t));
        //      xRengefindersLowQueue = xQueueCreate(RANGEFINDERS_NUMBER, sizeof(uint8_t));
        //      this->xRengefindersMutex = xSemaphoreCreateMutex();
        //create array for distance history
        for (uint8_t i = 0; i < RANGEFINDERS_NUMBER; i++)
                sensDistArr[i] = new uint8_t[10];
}

RangefinderManager::~RangefinderManager() {
}

void RangefinderManager::init_pwm()
{
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);

        GPIO_InitTypeDef pin_trig;
        GPIO_StructInit(&pin_trig);
        pin_trig.GPIO_Pin = GPIO_Pin_8;
        pin_trig.GPIO_Mode = GPIO_Mode_AF;
        GPIO_Init(GPIOB,&pin_trig);
        GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM10);

        /* Set the period equal to 1 us*/
        TIM_TimeBaseInitTypeDef tim10;
        TIM_TimeBaseStructInit(&tim10);
        tim10.TIM_Prescaler = 168 - 1;
        tim10.TIM_Period = 0xffff;
        TIM_TimeBaseInit(TIM10, &tim10);

        /*
         * Set the pulse width equal to 10 us
         * Time for rangefinder trigger
         */
        TIM_OCInitTypeDef tim10_pwm;
        TIM_OCStructInit(&tim10_pwm);
        tim10_pwm.TIM_OCMode = TIM_OCMode_PWM1;
        tim10_pwm.TIM_Pulse = 10;
        tim10_pwm.TIM_OutputState = TIM_OutputState_Enable;
        TIM_OC1Init(TIM10, &tim10_pwm);
        TIM_Cmd(TIM10, ENABLE);
}

void RangefinderManager::init_interrupt()
{
        /* Create structures for peripheral initialization */
        GPIO_InitTypeDef rf_pin;
        EXTI_InitTypeDef rf_interrupt;
        NVIC_InitTypeDef rf_nvic;

        /* Clocking on port lines and interrupts */
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

        /* Configuration of port E pins */
        GPIO_StructInit(&rf_pin);
        rf_pin.GPIO_OType = GPIO_OType_OD;
        rf_pin.GPIO_PuPd = GPIO_PuPd_DOWN;
        rf_pin.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 |GPIO_Pin_12 |
                        GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
        GPIO_Init(GPIOE, &rf_pin);

        /* Connect EXTI Lines to pins */
        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource10);
        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource11);
        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource12);
        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource13);
        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource14);
        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource15);

        /* Set EXTI lines */
        EXTI_StructInit(&rf_interrupt);
        rf_interrupt.EXTI_Line =  EXTI_Line10 | EXTI_Line11| EXTI_Line12 |
                        EXTI_Line13 | EXTI_Line14 | EXTI_Line15;
        rf_interrupt.EXTI_Mode = EXTI_Mode_Interrupt;
        rf_interrupt.EXTI_Trigger = EXTI_Trigger_Falling;
        rf_interrupt.EXTI_LineCmd = ENABLE;
        EXTI_Init(&rf_interrupt);

        rf_nvic.NVIC_IRQChannel = EXTI15_10_IRQn;
        rf_nvic.NVIC_IRQChannelPreemptionPriority = 0x01;
        rf_nvic.NVIC_IRQChannelSubPriority = 0x01;
        rf_nvic.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&rf_nvic);
}

void RangefinderManager::init_timer()
{
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

        TIM_TimeBaseInitTypeDef rf_tim6;
        TIM_TimeBaseStructInit(&rf_tim6);
        rf_tim6.TIM_Prescaler = 84 - 1;
        rf_tim6.TIM_Period = 0xffff;
        TIM_TimeBaseInit(TIM6, &rf_tim6);
        TIM_Cmd(TIM6, ENABLE);
}

uint8_t RangefinderManager::partition(uint8_t* input, uint8_t p, uint8_t r)
{
        uint8_t pivot = input[r];
        while (p < r) {
                while (input[p] < pivot)
                        p++;
                while (input[r] > pivot)
                        r--;
                if (input[p] == input[r])
                        p++;
                else if (p < r) {
                        uint8_t tmp = input[p];
                        input[p] = input[r];
                        input[r] = tmp;
                }
        }
        return r;
}

uint8_t RangefinderManager::quick_select(uint8_t* input, uint8_t p, uint8_t r, uint8_t k)
{
        if (p == r)
                return input[p];
        uint8_t j = partition(input, p, r);
        uint8_t length = j - p + 1;
        if (length == k)
                return input[j];
        else if (k < length )
                return quick_select(input, p, j - 1, k);
        else
                return quick_select(input, j + 1, r, k - length);
}

void RangefinderManager::calculateDistanceQsort(uint8_t* distArr, uint8_t i)
{
        uint16_t timeBuff = 0;
        uint8_t median = 0;
        if (sensTimeArr[i] == 0xff)
                distArr[0] = 0xff;
        else
                distArr[0] = (sensTimeArr[i] - 500) / 56;
        uint8_t sortArr[10];
        for (uint8_t j = 0; j < 10; j++)
                sortArr[j] = distArr[j];
        median = quick_select(sortArr, 0, 10, 5);
        for (uint8_t j = 9; j > 0; j--)
                distArr[j] = distArr[j - 1];
        if ((median - distArr[0] > 5) || (median - distArr[0] < -5))
                distArr[0] = median;
}

void RangefinderManager::get_distance(uint8_t* distArr)
{
        for (uint8_t i = 0; i < RANGEFINDERS_NUMBER; i++)
                distArr[i] = outDistArr[i];
}

void RangefinderManager::run(void *parameters)
{
        TickType_t xLastWakeTime;
        init_pwm();
        init_interrupt();
        init_timer();

        while(1) {
                xLastWakeTime = xTaskGetTickCount();

                for (uint8_t i = 0; i < RANGEFINDERS_NUMBER; i++)
                        calculateDistanceQsort(sensDistArr[i], i);

                //		xSemaphoreTake(xRengefindersMutex, portMAX_DELAY);
                for (uint8_t i = 0; i < RANGEFINDERS_NUMBER; i++)
                        outDistArr[i] = sensDistArr[i][0];
                //		xSemaphoreGive(xRengefindersMutex);

                /* Reload timer to generate trigger signal */
                TIM_SetCounter(TIM10, 0);

                TIM_SetCounter(TIM6, 0);
                vTaskDelayUntil(&xLastWakeTime, MIN_RESPONCE_TIME);
        }
}

extern "C"
{
void EXTI15_10_IRQHandler(){
        if (EXTI_GetITStatus(EXTI_Line10)) {
                EXTI_ClearITPendingBit(EXTI_Line10);
                if (TIM_GetCounter(TIM6) > 500 && TIM_GetCounter(TIM6) < 3250)
                        sensTimeArr[0] = TIM_GetCounter(TIM6);
                else
                        sensTimeArr[0] = 0xff;
        } else if (EXTI_GetITStatus(EXTI_Line11) == 1) {
                EXTI_ClearITPendingBit(EXTI_Line11);
                if (TIM_GetCounter(TIM6) > 500 && TIM_GetCounter(TIM6) < 3250)
                        sensTimeArr[1] = TIM_GetCounter(TIM6);
                else
                        sensTimeArr[1] = 0xff;
        } else if (EXTI_GetITStatus(EXTI_Line12) == 1) {
                EXTI_ClearITPendingBit(EXTI_Line12);
                if (TIM_GetCounter(TIM6) > 500 && TIM_GetCounter(TIM6) < 3250)
                        sensTimeArr[2] = TIM_GetCounter(TIM6);
                else
                        sensTimeArr[2] = 0xff;
        } else if (EXTI_GetITStatus(EXTI_Line13) == 1) {
                EXTI_ClearITPendingBit(EXTI_Line13);
                if (TIM_GetCounter(TIM6) > 500 && TIM_GetCounter(TIM6) < 3250)
                        sensTimeArr[3] = TIM_GetCounter(TIM6);
                else
                        sensTimeArr[3] = 0xff;
        } else if (EXTI_GetITStatus(EXTI_Line14) == 1) {
                EXTI_ClearITPendingBit(EXTI_Line14);
                if (TIM_GetCounter(TIM6) > 500 && TIM_GetCounter(TIM6) < 3250)
                        sensTimeArr[4] = TIM_GetCounter(TIM6);
                else
                        sensTimeArr[4] = 0xff;
        } else if (EXTI_GetITStatus(EXTI_Line15) == 1) {
                EXTI_ClearITPendingBit(EXTI_Line15);
                if (TIM_GetCounter(TIM6) > 500 && TIM_GetCounter(TIM6) < 3250)
                        sensTimeArr[5] = TIM_GetCounter(TIM6);
                else
                        sensTimeArr[5] = 0xff;
        }
}
}

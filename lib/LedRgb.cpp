/*
 * Led_rgb.cpp
 *
 *  Created on: 8 рту. 2019 у.
 *      Author: Taras.Melnik
 */

#include <LedRgb.h>
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"

LedRgb ledRgb;
static const uint32_t color_red_contin = 2800;
static const uint32_t color_red_flash = 1400;

LedRgb::LedRgb() {
    buttonMutex = false;
}

LedRgb::~LedRgb() {
    // TODO Auto-generated destructor stub
}

void LedRgb::init_led()
{
    init_gpio();
    set_color(BLUE);

}

void LedRgb::init_gpio()
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

    GPIO_InitTypeDef led_pin;
    GPIO_StructInit(&led_pin);
    led_pin.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_15;
    led_pin.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_Init(GPIOA, &led_pin);

    GPIO_InitTypeDef pwm_led_pin;
    GPIO_StructInit(&pwm_led_pin);
    pwm_led_pin.GPIO_Pin = GPIO_Pin_2;
    pwm_led_pin.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(GPIOA,&pwm_led_pin);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM5);

    TIM_TimeBaseInitTypeDef tim5_ch3;
    TIM_TimeBaseStructInit(&tim5_ch3);
    tim5_ch3.TIM_Prescaler = 50000 - 1;
    tim5_ch3.TIM_Period = 2800;
    TIM_TimeBaseInit(TIM5, &tim5_ch3);
    TIM_Cmd(TIM5, ENABLE);

    TIM5->CR1 |= TIM_CR1_ARPE;

    TIM_OCInitTypeDef tim5_ch3_pwm;
    tim5_ch3_pwm.TIM_OCMode = TIM_OCMode_PWM1;
    tim5_ch3_pwm.TIM_OCNPolarity = TIM_OCPolarity_High;
    tim5_ch3_pwm.TIM_Pulse = 1400;
    tim5_ch3_pwm.TIM_OutputState = ENABLE;
    TIM_OC3Init(TIM5, &tim5_ch3_pwm);

}

void LedRgb::mutex_take(const cl color)
{
    if (!buttonMutex)
        set_color(color);
    this->buttonMutex = true;
}

void LedRgb::mutex_take()
{
    this->buttonMutex = true;
}

void LedRgb::mutex_give()
{
    this->buttonMutex = false;
}

void LedRgb::set_color(const cl color)
{
    /* Check button status */
    if (!buttonMutex) {
        if (color == RED) {
            TIM_SetCompare3(TIM5, color_red_contin);
            GPIO_ResetBits(GPIOA, GPIO_Pin_3);
            GPIO_ResetBits(GPIOA, GPIO_Pin_15);
        } else if (color == GREEN) {
            TIM_SetCompare3(TIM5, 0);
            GPIO_ResetBits(GPIOA, GPIO_Pin_3);
            GPIO_SetBits(GPIOA, GPIO_Pin_15);
        } else if (color == BLUE) {
            TIM_SetCompare3(TIM5, 0);
            GPIO_SetBits(GPIOA, GPIO_Pin_3);
            GPIO_ResetBits(GPIOA, GPIO_Pin_15);
        } else if (color == WHITE) {
            TIM_SetCompare3(TIM5, color_red_contin);
            GPIO_SetBits(GPIOA, GPIO_Pin_3);
            GPIO_SetBits(GPIOA, GPIO_Pin_15);
        } else if (color == RED_FLASH) {
            TIM_SetCompare3(TIM5, color_red_flash);
            GPIO_ResetBits(GPIOA, GPIO_Pin_3);
            GPIO_ResetBits(GPIOA, GPIO_Pin_15);
        } else {
            TIM_SetCompare3(TIM5, 0);
            GPIO_ResetBits(GPIOA, GPIO_Pin_3);
            GPIO_ResetBits(GPIOA, GPIO_Pin_15);
        }
    }
}


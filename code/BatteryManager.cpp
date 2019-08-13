/*
 * BatteryChargeAsker.cpp
 *
 *  Created on: 31.10.2018
 *      Author: Taras.Melnik
 */

#include "BatteryManager.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_adc.h"
#include "FreeRTOS.h"
#include "task.h"


BatteryManager bat_manager;

BatteryManager::BatteryManager() {
        /* Start primary ADC calculation */
        ADC_SoftwareStartConv(ADC1);
}

BatteryManager::~BatteryManager() {
}

uint8_t BatteryManager::ChargeVal_P;

void BatteryManager::init_adc()
{
        /* Turn on clocking on Pin and on ADC */
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

        /* GPIO setting for ADC */
        GPIO_InitTypeDef pin_batt;
        GPIO_StructInit(&pin_batt);
        pin_batt.GPIO_Pin = GPIO_Pin_0;
        pin_batt.GPIO_OType = GPIO_OType_OD;
        pin_batt.GPIO_PuPd = GPIO_PuPd_DOWN;
        pin_batt.GPIO_Mode = GPIO_Mode_AN;
        GPIO_Init(GPIOC, &pin_batt);

        /* ADC common register setting (for all channels) */
        ADC_CommonInitTypeDef  adc_common;
        ADC_CommonStructInit(&adc_common);
        adc_common.ADC_Prescaler = ADC_Prescaler_Div8;
        ADC_CommonInit(&adc_common);

        /* ADC1 settings */
        ADC_InitTypeDef adc_1;
        ADC_StructInit(&adc_1);
        adc_1.ADC_Resolution = ADC_Resolution_12b;
        ADC_Init(ADC1, &adc_1);

        /* Specific Channel setting */
        ADC_RegularChannelConfig(ADC1, ADC_Channel_10, ADC_RANK, ADC_SampleTime_84Cycles);

        /* To Turn on ADC1 */
        ADC_Cmd(ADC1, ENABLE);
}

uint8_t BatteryManager::get_charge()
{
        return ChargeVal_P;
}

void BatteryManager::run(void *parameters)
{
        uint16_t ChargeVal = 0;
        uint16_t hBorder = MAX_BATTERY_VOLTAGE;
        uint16_t sBorder = STOP_BATTERY_VOLTAGE;
        uint16_t lBorder = MIN_BATTERY_VOLTAGE;
        uint16_t buffArray[10];
        uint16_t buffAverege = 0;
        uint8_t k = 0;
        float perKoef = (hBorder - lBorder)/100.f;
        buffAverege = ADC_GetConversionValue(ADC1);
        for (uint8_t i = 0; i < 10; i++){
                buffArray[i] = buffAverege;
        }

        while(1) {

                /* Mean value filter */
                buffAverege = 0;
                for (uint8_t i = 9; i > 0; i--) {
                        buffArray[i] = buffArray[i-1];
                        buffAverege += buffArray[i];
                }
                buffArray[0] = ADC_GetConversionValue(ADC1);
                buffAverege += buffArray[0];
                ChargeVal = buffAverege / 10;

                /* The next value cannot be greater than the previous */
                if (ChargeVal >= hBorder)
                        ChargeVal = hBorder;
                else if (ChargeVal > lBorder) {
                        if (hBorder - ChargeVal > 3 && k <= 15)
                                k++;
                        else {
                                k = 0;
                                hBorder = ChargeVal;
                        }
                } else
                        ChargeVal = lBorder;
                ChargeVal_P = uint8_t (ChargeVal - lBorder)/perKoef;

                ADC_SoftwareStartConv(ADC1);
                vTaskDelay(5000);
        }
}








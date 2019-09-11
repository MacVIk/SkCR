#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_flash.h"

/*
 *   System Clock Configuration
 *   The system Clock is configured as follow :
 *   System Clock source            = PLL (HSE)
 *   SYSCLK(Hz)                     = 168000000
 *   HCLK(Hz)                       = 168000000
 *   AHB Prescaler                  = 1
 *   APB1 Prescaler                 = 4
 *   APB2 Prescaler                 = 2
 *   HSE Frequency(Hz)              = 8000000
 *   PLL_M                          = 8
 *   PLL_N                          = 336
 *   PLL_P                          = 2
 *   PLL_Q                          = 7
 *   VDD(V)                         = 3.3
 *   Main regulator output voltage  = Scale1 mode
 *   Flash Latency(WS)              = 5
 */

/*
 * Do not delete this function It provides
 * The correct System Clock settings
 */
void system_clock_config()
{
        /* Enable HSE oscillator */
        RCC_HSEConfig(RCC_HSE_ON);
        while (!RCC_GetFlagStatus(RCC_FLAG_HSERDY));

        /* Set FLASH latency */
        FLASH_SetLatency(FLASH_Latency_5);

        /* Set AHB (system bus) clock frequency */
        RCC_HCLKConfig(RCC_SYSCLK_Div1);

        /* Set APB1 and APB2 clock frequency */
        RCC_PCLK1Config(RCC_HCLK_Div4);
        RCC_PCLK2Config(RCC_HCLK_Div2);

         /* Set HSE as source for PLL
         * Set divider (M, N, P, Q)
         * Enable PLL
         */
        RCC_PLLConfig(RCC_PLLSource_HSE, 8, 336, 2, 7);
        RCC_PLLCmd(ENABLE);
        while (!RCC_GetFlagStatus(RCC_FLAG_PLLRDY));

        /* SysClk activation on the main PLL */
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
        while (!RCC_GetFlagStatus(RCC_FLAG_PLLRDY));

        /* Update CMSIS variable */
        SystemCoreClock = 168000000;
}

/* FreeRtos core includes */
#include <stdio.h>
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

/* User libraries includes */
#include "terminal.h"
#include "RangefinderManager.h"
#include "BatteryManager.h"
#include "MotorManager.h"
#include "I2c1Manager.h"
#include "LedRgb.h"

int main(void) {

        /* Set clocking frequency */
        system_clock_config();

        terminal.task_create(256, 2, "Terminal");
//        xTaskCreate(terminal.run, "run", 256, NULL, 1, NULL);

        bat_manager.task_create(256, 1, "Battery");
//        xTaskCreate(rf_manager.run, "run", 256, NULL, 1, NULL);

        rf_manager.task_create(512, 1, "Rangefinder");
//        xTaskCreate(bat_manager.run, "run", 512, NULL, 1, NULL);

        mot_manager.task_create(2048, 1, "Motors");

        i2c_manager.task_create(1024, 1, "I2C");

        /* Robot LED initialisatin */
        ledRgb.init_led();

        /* Start tasks */
        vTaskStartScheduler();

        return 1;
}


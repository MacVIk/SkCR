#ifndef __ADJUST_SYS_H_
#define __ADJUST_SYS_H_

// System adjustment constants

///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
// Clock adjustment

/* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLL_M) * PLL_N */
/* Warning ! PLL_VCO must be between 192 and 432 MHz ! */
#define PLL_M      (HSE_VALUE/1000000)
#define PLL_N      336

/* SYSCLK = PLL_VCO / PLL_P */
#define PLL_P      2

/* USB OTG FS, SDIO and RNG Clock =  PLL_VCO / PLLQ */
#define PLL_Q      7

#define SYSCLK_FREQ_MAX (((HSE_VALUE/PLL_M)*PLL_N)/PLL_P)
// !!! Set APB prescalers in "SetSysClock" function in system_stm32f4xx.h
#define APB1_PRESCALER 4
#define APB2_PRESCALER 2
#define APB1_FREQ_MAX (SYSCLK_FREQ_MAX/APB1_PRESCALER)
#define APB2_FREQ_MAX (SYSCLK_FREQ_MAX/APB2_PRESCALER)

#define HSI_VALUE            ((uint32_t)16000000) /*!< Value of the Internal oscillator in Hz*/

// Target device family (used in stm32f4xx.h)
#define STM32F40XX

///////////////////////////////////////////////////////////////////////

// Voltage reference (VREF+)
#define VREF_VOLTAGE_TYPICAL (3.3)
#define VREF_12BIT_SAMPLES (4095)

// Internal reference (Vrefint)
#define VREFINT_VOLTAGE (1.21)

#endif// __ADJUST_SYS_H_

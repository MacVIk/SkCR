/*
 * gpio_map.h
 *
 *  Created on: 8 дек. 2019 г.
 *      Author: Taras.Melnik
 */

#ifndef LIB_GPIO_MAP_H_
#define LIB_GPIO_MAP_H_

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"

/*
 * Terminal UART6 pinout
 */
#define TERM_UART_PORT              GPIOC
#define TERM_UART_RX_PIN            GPIO_Pin_7
#define TERM_UART_TX_PIN            GPIO_Pin_6

/*
 * Motor-wheel 1 interface - UART3 pinout
 */
#define MOTOR_1_UART_PORT           GPIOB
#define MOTOR_1_UART_RX_PIN         GPIO_Pin_11
#define MOTOR_1_UART_TX_PIN         GPIO_Pin_10
#define MOTOR_1_UART_SWITCH_PORT    GPIOB
#define MOTOR_1_UART_SWITCH_PIN     GPIO_Pin_12

/*
 * Motor-wheel 2 interface - UART4 pinout
 */
#define MOTOR_2_UART_PORT           GPIOA
#define MOTOR_2_UART_RX_PIN         GPIO_Pin_1
#define MOTOR_2_UART_TX_PIN         GPIO_Pin_0
#define MOTOR_2_UART_SWITCH_PORT    GPIOA
#define MOTOR_2_UART_SWITCH_PIN     GPIO_Pin_4

/*
 * Emergency stop button pin
 */
#define EMERGEBCY_STOP_BUTTON_PORT  GPIOC
#define EMERGEBCY_STOP_BUTTON_PIN   GPIO_Pin_15

/*
 * IMU sensor I2S1 pinout
 */
#define I2C_1_IMU_PORT              GPIOB
#define I2C_1_IMU_SCL_PIN           GPIO_Pin_6
#define I2C_1_IMU_SDA_PIN           GPIO_Pin_7

/*
 * Ultrasonic rangefinders pinout
 */
#define RANGEFINDERS_PORT           GPIOE
#define RANGEFINDER_1_PIN           GPIO_Pin_10
#define RANGEFINDER_2_PIN           GPIO_Pin_11
#define RANGEFINDER_3_PIN           GPIO_Pin_12
#define RANGEFINDER_4_PIN           GPIO_Pin_13
#define RANGEFINDER_5_PIN           GPIO_Pin_14
#define RANGEFINDER_6_PIN           GPIO_Pin_15

/*
 * Battery charge pin
 */
#define BATTERY_CHARGE_PORT         GPIOC
#define BATTERY_CHARGE_PIN          GPIO_Pin_0

/*
 * RGB LED pinout
 */
#define RGB_LED_PORT                GPIOC
#define RGB_LED_RED_PIN             GPIO_Pin_2
#define RGB_LED_BLUE_PIN            GPIO_Pin_3
#define RGB_LED_GREEN_PIN           GPIO_Pin_15




#endif /* LIB_GPIO_MAP_H_ */

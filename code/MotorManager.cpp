/*
 * MotorManager.cpp
 *
 *  Created on: 11.03.2019
 *      Author: Taras.Melnik
 */

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_dma.h"
#include <math.h>
#include "arm_math.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "MotorManager.h"
#include "LedRgb.h"
#include "MotorWheel.h"

MotorManager mot_manager;

static MotorWheel motor_wheel_1(1);
static MotorWheel motor_wheel_2(2);

/* ----------------------------------------------------------
 * High level commands
 */
void MotorManager::set_robot_speed(uint8_t* byte_arr)
{
    for (uint8_t i = 0; i < 8; i++)
        speed_byte_arr[i]= byte_arr[i];
    terminalRxFlag = true;
}

void MotorManager::get_robot_position(uint8_t* byte_arr)
{
    for (uint8_t i = 0; i < 12; i++)
        byte_arr[i] = position_byte_arr[i];
}

bool MotorManager::get_robot_button_status()
{
    bool buff {robot.button_status};
    robot.button_status = false;
    return buff;
}

bool MotorManager::get_robot_current_status()
{
    bool buff {robot.current_status};
    robot.current_status = false;
    return buff;
}

/* ------------------------------------------------------------
 * Motors data processing functions
 */
void MotorManager::set_robot_speed()
{
    float robot_speed[2] = {};

    /* Get the command */
    terminalRxFlag = false;

    /* Convert from byte array into linear and angular robot velocity */
    memcpy(robot_speed, speed_byte_arr, sizeof(robot_speed));

    /* If status was not processed */
    if (!robot.current_status)
        set_robot_speed(robot_speed[0], robot_speed[1]);
}

void MotorManager::set_robot_speed(float lin_vel, float ang_vel )
{
    float robot_speed[2] {lin_vel, ang_vel};
    int16_t wheel_speed[2] {};

    /* Safety limitation on the robot speed */
    if (lin_vel > MAX_ROBOT_SPEED)
        robot.linear_velocity = MAX_ROBOT_SPEED;
    convert_robot_to_wheel_speed(robot_speed, wheel_speed);

    bool ms1 {motor_wheel_1.set_wheel_speed(wheel_speed[0])};
    bool ms2 {motor_wheel_2.set_wheel_speed(wheel_speed[1])};
    if (!ms1 || !ms2)
        process_robot_error();
}

void MotorManager::process_angle()
{
    float robot_position[3];
    int32_t wh_arr[2];

    bool ms1 {motor_wheel_1.request_angle(wh_arr[0])};
    bool ms2 {motor_wheel_2.request_angle(wh_arr[1])};
    if (!ms1 || !ms2)
        process_robot_error();

    calculate_position(wh_arr);
    robot_position[0] = robot.x;
    robot_position[1] = robot.y;
    robot_position[2] = robot.theta;

    memcpy(position_byte_arr, robot_position, sizeof(position_byte_arr));
}

void MotorManager::process_speed()
{
    int16_t r_vel_arr[2];

    //        convert_wheel_to_robot_speed();
    motor_wheel_1.request_wheel_speed(r_vel_arr[0]);
    motor_wheel_2.request_wheel_speed(r_vel_arr[1]);


    //incorrect
    //        memcpy(speed_byte_arr, r_vel_arr,sizeof(speed_byte_arr));
}

void MotorManager::process_current()
{
    int16_t cur_val[2];
    bool ms1 {motor_wheel_1.request_current(cur_val[0])};
    bool ms2 {motor_wheel_2.request_current(cur_val[1])};
    if (!ms1 || !ms2)
        process_robot_error();

    if (cur_val[0] > CURRENT_COLLISION_VALUE ||
            cur_val[2] > CURRENT_COLLISION_VALUE)
    {
        robot.current_status = true;
        set_robot_speed(STOP_MOTION, STOP_MOTION);

    }
}

void MotorManager::process_motor_error()
{
    bool err_val[2];

    bool ms1 {motor_wheel_1.request_error(err_val[0])};
    bool ms2 {motor_wheel_2.request_error(err_val[1])};
    if (!ms1 || !ms2)
        process_robot_error();

    if (err_val[0] || err_val[1]) {
        motor_wheel_1.clear_error();
        motor_wheel_2.clear_error();
        vTaskDelay(500);
        motor_wheel_1.init_wheel();
        motor_wheel_2.init_wheel();
    }
}
/* ------------------------------------------------------------
 * Internal options
 */

inline void MotorManager::convert_robot_to_wheel_speed(float* robArr, int16_t* wheel_arr)
{
    float32_t buff[2];
    buff[0] = (robArr[0] + robArr[1] * L_CENTER) * CONST_WHEEL_1 / R_WHEEL;
    buff[1] = (robArr[0] - robArr[1] * L_CENTER) * CONST_WHEEL_2 / R_WHEEL;
    for (uint8_t i = 0; i < 2; i++) {
        wheel_arr[i] = (int16_t) buff[i];

        /* More accurate rounding */
        if (buff[i] - 0.5 >= wheel_arr[i]) {
            ++wheel_arr[i];
        } else if (buff[i] + 0.5 <= wheel_arr[i])
            --wheel_arr[i];
    }
}

void MotorManager::convert_wheel_to_robot_speed()
{
    //        robot.linear_velocity = motor_wheel_1.options.speed;
}

inline void MotorManager::fetch_angle()
{
    static constexpr float CONST_2_PI = 2 * PI;

    if (robot.theta >= CONST_2_PI) {
        robot.theta -= CONST_2_PI;
    } else if (robot.theta <= -CONST_2_PI)
        robot.theta += CONST_2_PI;
}

void MotorManager::calculate_position(int32_t* wheel_arr)
{
    static int32_t wheel_arr_prev[] = {0, 0};

    float d_wheel_angle[2] = {0};
    float d_theta = 0;
    float d_dist = 0;

    for (uint8_t i = 0; i < 2; i++) {
        /*
         * Protection against random initial
         * values on motors after switching on
         */
        //		if (wheel_arr_prev[i] == 0)
        //		        wheel_arr_prev[i] = wheel_arr[i];

        d_wheel_angle[i] = (wheel_arr[i] - wheel_arr_prev[i]) * W_CONST;
        wheel_arr_prev[i] = wheel_arr[i];
    }
    d_theta = (d_wheel_angle[0] - d_wheel_angle[1]) * ROTATION_CONST;

    /* Fix an incorrect answer during shutdown */
    if (d_theta < 10 && d_theta > -10) {
        d_dist = (d_wheel_angle[0] + d_wheel_angle[1]) * R_WHEEL / 2;

        robot.theta += d_theta;

        /* The angle should be in 2pi range (for ARM tables) */
        fetch_angle();

        /* Use arm_math table functions for high speed calculation */
        robot.y += d_dist * arm_sin_f32(robot.theta);
        robot.x += d_dist * arm_cos_f32(robot.theta);
    }
}

void MotorManager::process_robot_error()
{
    if (!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_15)) {
        while (!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_15)){
            robot.button_status = true;
            ledRgb.mutex_take(LedColor::RED_FLASH);
            vTaskDelay(5);
        }
        /* Init motors after power reset */
        ledRgb.mutex_give();
        ledRgb.mutex_take(LedColor::RED);
        vTaskDelay(PAUSE_AFTER_POWER_ON);
        motor_wheel_1.init_wheel();
        motor_wheel_2.init_wheel();
        set_robot_speed(STOP_MOTION, STOP_MOTION);
        ledRgb.mutex_give();
    } else {
        //ToDo Add processing of each wheel
        robot.error_status = true;
    }
    vTaskDelay(5);
}

/* ------------------------------------------------------------
 * Task body
 */
void MotorManager::run()
{
    TickType_t pxPreviousWakeTime;

    /* Peripheral initialization */
    motor_wheel_1.init_usart(GPIOB, USART3, true);
    //	motor_wheel_2.init_usart(GPIOB, USART3, true);
    motor_wheel_2.init_usart(GPIOA, UART4, true);

    /* Switch pin for rs485 driver */
    motor_wheel_1.gpioSwitchInit(GPIOB, GPIO_Pin_12);
    //	motor_wheel_2.gpioSwitchInit(GPIOB, GPIO_Pin_12);
    motor_wheel_2.gpioSwitchInit(GPIOA, GPIO_Pin_4);

    /* Should be after peripheral initialization */
    motor_wheel_1.init_wheel();
    motor_wheel_2.init_wheel();
    set_robot_speed(STOP_MOTION, STOP_MOTION);

    while (1) {
        /* If command from high level has come set speed */
        if (terminalRxFlag)
            set_robot_speed();
        process_angle();
        if (terminalRxFlag)
            set_robot_speed();
        process_current();
        if (terminalRxFlag)
            set_robot_speed();
        process_motor_error();
    }
}

/* ------------------------------------------------------------
 * Peripheral interrupts
 */
extern "C"
{
//---------------------------WHEEL_1---------------------------------//

//***********************UART_RECEIVE_INTERRUPT******************//
void USART3_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;			// Notify task about interrupt
    if (USART_GetITStatus(USART3, USART_IT_IDLE)) {			// Clear IDLE flag step 1
        DMA_Cmd(DMA1_Stream1, DISABLE);				// DMA turn off to clear DMA1 counter
        USART_ReceiveData(USART3);				// Clear IDLE flag step 2
    }
    if (USART_GetITStatus(USART3, USART_IT_TC)) {
        USART_ClearITPendingBit(USART3, USART_IT_TC);
        GPIO_ResetBits(GPIOB, GPIO_Pin_12);
    }
    if (xHigherPriorityTaskWoken)
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);		// Run Higher priority task if exist													// between ticks
}
//***********************DMA_RECEIVE_INTERRUPT******************//
void DMA1_Stream1_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    DMA_ClearITPendingBit(DMA1_Stream1, DMA_IT_TCIF1);		// Clear DMA "transmitting complete" interrupt
    DMA_Cmd(DMA1_Stream1, ENABLE);					// Reset DMA

    /* Set flag into motor constant */
    motor_wheel_1.confirm_motor_answer();

    vTaskNotifyGiveFromISR(mot_manager.task_handle,
            &xHigherPriorityTaskWoken);                     // Notify task about interrupt
    if (xHigherPriorityTaskWoken)
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);		// Run Higher priority task if exist													// between ticks
}

//***********************DMA_TRANSMIT_INTERRUPT*****************//
void DMA1_Stream3_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;			// Notify task about interrupt
    DMA_ClearITPendingBit(DMA1_Stream3, DMA_IT_TCIF3);		// Clear DMA "transmission complete" interrupt
    DMA_Cmd(DMA1_Stream3, DISABLE);					// Stop DMA transmitting
    if (xHigherPriorityTaskWoken)
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);		// Run Higher priority task if exist													// between ticks
}

//---------------------------WHEEL_2---------------------------------//

//***********************UART_RECEIVE_INTERRUPT******************//
void UART4_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;			// Notify task about interrupt
    if (USART_GetITStatus(UART4, USART_IT_IDLE)){			// Clear IDLE flag step 1
        DMA_Cmd(DMA1_Stream2, DISABLE);				// DMA turn off to clear DMA1 counter
        USART_ReceiveData(UART4);				// Clear IDLE flag step 2
    }
    if (USART_GetITStatus(UART4, USART_IT_TC)){
        USART_ClearITPendingBit(UART4, USART_IT_TC);
        GPIO_ResetBits(GPIOA, GPIO_Pin_4);
    }
    if (xHigherPriorityTaskWoken)					// Run Higher priority task if exist
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);		// between ticks
}
//***********************DMA_RECEIVE_INTERRUPT******************//
void DMA1_Stream2_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;			// Notify task about interrupt
    DMA_ClearITPendingBit(DMA1_Stream2, DMA_IT_TCIF2);		// Clear DMA "transmitting complete" interrupt
    motor_wheel_2.confirm_motor_answer();

    vTaskNotifyGiveFromISR(mot_manager.task_handle,
            &xHigherPriorityTaskWoken);                     // Notify task about interrupt
    DMA_Cmd(DMA1_Stream2, ENABLE);				        // Reset DMA
    if (xHigherPriorityTaskWoken)				        // Run Higher priority task if exist
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);		// between ticks
}
//***********************DMA_TRANSMIT_INTERRUPT*****************//
void DMA1_Stream4_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;			// Notify task about interrupt
    DMA_ClearITPendingBit(DMA1_Stream4, DMA_IT_TCIF4);		// Clear DMA "transmission complete" interrupt
    DMA_Cmd(DMA1_Stream4, DISABLE);					// Stop DMA transmitting
    if (xHigherPriorityTaskWoken)					// Run Higher priority task if exist
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);		// between ticks
}
}

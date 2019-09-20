/*
 * USBUserInterface.cpp
 *
 *  Created on: 29.10.2018
 *      Author: Taras.Melnik
 */

#include "stm32f4xx.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_dma.h"

#include "FreeRTOS.h"
#include "task.h"

#include "DriverUsart.h"
#include "defines.h"
#include "terminal.h"
#include "LedRgb.h"
#include "BatteryManager.h"
#include "RangefinderManager.h"
#include "MotorManager.h"
#include "I2c1Manager.h"

Terminal terminal;
static DriverUsart uart_6;
static uint8_t uart_rx_arr[24];
static uint8_t uart_tx_arr[24];

inline void Terminal::add_error_byte(uint8_t& answerLength)
{
        uint8_t err_byte = 0;
        if (mot_manager.get_robot_button_status()) {
                err_byte |= 1;
        }
        if (mot_manager.get_robot_current_status()) {
                err_byte |= 1 << 1;
        }

        //        if (collisionHandler->getStatus())
        //                err_byte |= 1 << 2;
        uart_tx_arr[answerLength++] = err_byte;
        //       uart_tx_arr[++answerLength] = 0;
}

inline void Terminal::calculate_checksum(uint8_t answerLength)
{
        uart_tx_arr[answerLength] = 0;
        for (uint8_t i = 0; i < answerLength; ++i)
                uart_tx_arr[answerLength] +=uart_tx_arr[i];
}

void Terminal::run()
{
        uint8_t answerLength = 0;
        uint8_t errByte = 0;

        uart_6.init_usart(GPIOC, USART6, false);

        while(1) {
                /* Task is suspended until notification */
                ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

                uint8_t commad = uart_6.usart_receive_byte();

                if (commad == HighLvlCommand::ECHO) {
                        uart_tx_arr[0] = 0x11;
                        answerLength = 1;

                } else if (commad == HighLvlCommand::SET_COLOR) {
                        uart_rx_arr[0] = uart_6.usart_receive_byte();
                        ledRgb.set_color((cl) uart_rx_arr[0]);
                        answerLength = 1;

                } else if (commad == HighLvlCommand::GET_BATTERY_CHARGE) {
                        uart_tx_arr[0] = bat_manager.get_charge();
                        answerLength = 1;

                } else if (commad == HighLvlCommand::GET_DISTANCE) {
                        rf_manager.get_distance(uart_tx_arr);
                        answerLength = RANGEFINDERS_NUMBER;

                } else if (commad == HighLvlCommand::SET_ROBOT_SPEED) {
                        for (uint8_t i = 0; i < 8; i++)
                                uart_rx_arr[i] = uart_6.usart_receive_byte();
                        mot_manager.set_robot_speed(uart_rx_arr);
                        answerLength = 1;

                        /* Receive from RS485 ----- (x, y, theta) */
                } else if (commad == HighLvlCommand::GET_ROBOT_POSITION) {
                        mot_manager.get_robot_position(uart_tx_arr);
                        answerLength = 12;

                        /* Receive from IMU ----- (x, y, theta) */
                } else if (commad == HighLvlCommand::GET_ACCELERATION) {
                        i2c_manager.get_acceleration(uart_tx_arr);
                        answerLength = 12;

                } else if (commad == HighLvlCommand::GET_ANG_VEL) {
                        i2c_manager.get_angular_velocity(uart_tx_arr);
                        answerLength = 12;

                } else {
                        uart_tx_arr[0] = 0xff;
                        answerLength = 1;
                }

                add_error_byte(answerLength);
                calculate_checksum(answerLength);

                uart_6.usart_send(uart_tx_arr, ++answerLength);

                //ToDo replace it into interrupt
                uart_6.usart_stop_receiving();

                /* Finish the task before next tick */
                taskYIELD();
        }
}

extern "C"
{
// ***********************UART_RECEIVE_INTERRUPT******************//
void USART6_IRQHandler(void)
{
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;			// Notify task about interrupt
        if (USART_GetITStatus(USART6, USART_IT_IDLE)) {			// Clear IDLE flag step 1
                DMA_Cmd(DMA2_Stream1, DISABLE);				// DMA turn off to clear DMA1 counter
                USART_ReceiveData(USART6);				// Clear IDLE flag step 2
        }
        if (xHigherPriorityTaskWoken)					// Run Higher priority task if exist
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);		// between ticks
}
//***********************DMA_RECEIVE_INTERRUPT******************//
void DMA2_Stream1_IRQHandler(void)
{
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;			// Notify task about interrupt
        vTaskNotifyGiveFromISR(terminal.task_handle,	                // Notify USART task about receiving
                        &xHigherPriorityTaskWoken);
        DMA_ClearITPendingBit(DMA2_Stream1, DMA_IT_TCIF1);		// Clear DMA "transmitting complete" interrupt
        DMA_Cmd(DMA2_Stream1, ENABLE);					// Reset DMA
        if (xHigherPriorityTaskWoken)					// Run Higher priority task if exist
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);		// between ticks
}
//***********************DMA_TRANSMIT_INTERRUPT*****************//
void DMA2_Stream6_IRQHandler(void)
{
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;			// Notify task about interrupt
        DMA_ClearITPendingBit(DMA2_Stream6, DMA_IT_TCIF6);		// Clear DMA "transmission complete" interrupt
        DMA_Cmd(DMA2_Stream6, DISABLE);					// Stop DMA transmitting
        if (xHigherPriorityTaskWoken)					// Run Higher priority task if exist
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);		// between ticks
}
}

//extern "C"
//{
////***********************UART_WITHOUT_DMA**********************//
//	void USART3_IRQHandler(void)
//	{
//		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//		vTaskNotifyGiveFromISR(xTaskGetCurrentTaskHandle(),&xHigherPriorityTaskWoken);
//		if (USART_GetITStatus(USART3, USART_IT_RXNE)) {
//			USART_ClearITPendingBit(USART3, USART_IT_RXNE);
//			usartRxArr[usartRxPoiter++] = (uint8_t) USART_ReceiveData(USART3);
//		}
//		usartActiveFlag = true;
//		if (xHigherPriorityTaskWoken)
//			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//	}
//}

/*
 * USBUserInterface.cpp
 *
 *  Created on: 29.10.2018
 *      Author: Taras.Melnik
 */

#include "USBUserInterface.h"

USBUserInterface* usbUserInterface;
UARTuserInit uart6;

USBUserInterface::USBUserInterface() {
}

USBUserInterface::~USBUserInterface() {
}

void USBUserInterface::run()
{
	uint8_t answerLength;
	uint8_t i = 0;
	uint8_t histColArr[RANGEFINDERS_NUMBER + 1] = {0};
	uint8_t histAngArr[12] = {0};

	this->xTaskToNotify = xTaskGetCurrentTaskHandle();
	uart6.uartInit(GPIOC, USART6, false);

	while(1) {
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		if (uart6.usartRxArr[0] == ECHO) {
			uart6.usartTxArr[0] = 0x11;
			uart6.usartTxArr[1] = uart6.usartTxArr[0];
			answerLength = 2;

		} else if (uart6.usartRxArr[0] == SET_COLOR_NUMBER) {
			setLEDTask->writeToQueue(uart6.usartRxArr[1]);
			uart6.usartTxArr[0] = 0;
			uart6.usartTxArr[1] = uart6.usartTxArr[0];
			answerLength = 2;

		} else if (uart6.usartRxArr[0] == GET_BATTERY_CHARGE) {
			getBatChargeTask->readFromQueue(uart6.usartTxArr[0]);
			uart6.usartTxArr[1] = uart6.usartTxArr[0];
			answerLength = 2;

		} else if (uart6.usartRxArr[0] == GET_COLLISION_STATUS) {
			uart6.usartTxArr[RANGEFINDERS_NUMBER + 1] = 0;
			if (uxQueueMessagesWaiting(sensorTask->xCollisionAvoidanceQueue))
				for (i = 0; i < RANGEFINDERS_NUMBER; i++)
					xQueueReceive(sensorTask->xCollisionAvoidanceQueue, &histColArr[i], portMAX_DELAY);
			for (i = 0; i < RANGEFINDERS_NUMBER; i++) {
				uart6.usartTxArr[i] = histColArr[i];
				uart6.usartTxArr[RANGEFINDERS_NUMBER + 1] += uart6.usartTxArr[i];
			}
			uart6.usartTxArr[RANGEFINDERS_NUMBER] = hyroMotor->getWheelStatus();
			uart6.usartTxArr[RANGEFINDERS_NUMBER + 1] += uart6.usartTxArr[RANGEFINDERS_NUMBER];
			answerLength = RANGEFINDERS_NUMBER + 2;
			hyroMotor->clearWheelStatus();

		} else if (uart6.usartRxArr[0] == SEND_RS485) {			// Send to RS485 -----
			if (uxQueueMessagesWaiting(hyroMotor->xHighLvlQueue))
				xQueueReset(hyroMotor->xHighLvlQueue);
			for (i = 1; i < 9; i++)
				xQueueSend(hyroMotor->xHighLvlQueue, &uart6.usartRxArr[i], portMAX_DELAY);
			uart6.usartTxArr[0] = SEND_RS485;
			uart6.usartTxArr[1] = uart6.usartTxArr[0];
			answerLength = 2;

		} else if (uart6.usartRxArr[0] == RECEIVE_RS485) {		// Receive from RS485 -----
			uart6.usartTxArr[12] = 0;
			if (uxQueueMessagesWaiting(hyroMotor->xAngleQueue))
				for (i = 0; i < 12; i++)
					xQueueReceive(hyroMotor->xAngleQueue, &histAngArr[i], portMAX_DELAY);
			for (i = 0; i < 12; i++) {
				uart6.usartTxArr[i] = histAngArr[i];
				uart6.usartTxArr[12] += uart6.usartTxArr[i];
			}
			answerLength = 13;

		} else {
			uart6.usartTxArr[0] = 0xff;
			uart6.usartTxArr[1] = uart6.usartTxArr[0];
			answerLength = 2;
		}
		uart6.send(uart6.usartTxArr, answerLength);
	}
}

extern "C"
{
// ***********************UART_RECEIVE_INTERRUPT******************//
	void USART6_IRQHandler(void)
	{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;			// Notify task about interrupt
		if (USART_GetITStatus(USART6, USART_IT_IDLE)) {			// Clear IDLE flag step 1
			DMA_Cmd(DMA2_Stream1, DISABLE);						// DMA turn off to clear DMA1 counter
			USART_ReceiveData(USART6);							// Clear IDLE flag step 2
		}
		if (xHigherPriorityTaskWoken)							// Run Higher priority task if exist
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);		// between ticks
	}
//***********************DMA_RECEIVE_INTERRUPT******************//
	void DMA2_Stream1_IRQHandler(void)
	{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;			// Notify task about interrupt
		vTaskNotifyGiveFromISR(usbUserInterface->xTaskToNotify,	// Notify USART task about receiving
				&xHigherPriorityTaskWoken);
		DMA_ClearITPendingBit(DMA2_Stream1, DMA_IT_TCIF1);		// Clear DMA "transmitting complete" interrupt
		DMA_Cmd(DMA2_Stream1, ENABLE);							// Reset DMA
		if (xHigherPriorityTaskWoken)							// Run Higher priority task if exist
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);		// between ticks
	}
//***********************DMA_TRANSMIT_INTERRUPT*****************//
	void DMA2_Stream6_IRQHandler(void)
	{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;			// Notify task about interrupt
		DMA_ClearITPendingBit(DMA2_Stream6, DMA_IT_TCIF6);		// Clear DMA "transmission complete" interrupt
		DMA_Cmd(DMA2_Stream6, DISABLE);							// Stop DMA transmitting
		if (xHigherPriorityTaskWoken)							// Run Higher priority task if exist
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

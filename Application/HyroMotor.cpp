/*
 * HyroMotor.cpp
 *
 *  Created on: 11.03.2019
 *      Author: Taras.Melnik
 */

#include "HyroMotor.h"

HyroMotor* hyroMotor;
UARTuserInit uart3;
UARTuserInit uart4;
UARTtoRS485	motor1;
UARTtoRS485 motor2;

HyroMotor::HyroMotor() {
	xAngleQueue = xQueueCreate(12, sizeof(uint8_t));
	xAngleMutex = xSemaphoreCreateMutex();
	xHighLvlQueue = xQueueCreate(8, sizeof(uint8_t));
	xHighLvlMutex = xSemaphoreCreateMutex();
	motArr[0] = &motor1;
	motArr[1] = &motor2;
	for (uint8_t i = 0; i < 2; i++)
		this->rxRsDataArr[i] = new uint8_t[4];
}

HyroMotor::~HyroMotor() {
}

void HyroMotor::delayPort(uint32_t ticks)
{
	txFlag = true;
	rxFlag = true;
	ulTaskNotifyTake(pdTRUE, oRTOS.fromMsToTick(ticks));
}

void HyroMotor::taskNotifyFromISR(BaseType_t xHigherPriorityTaskWoken)
{
	if (rxFlag) {
		if (xTaskToNotify != 0) {
			vTaskNotifyGiveFromISR(xTaskToNotify, &xHigherPriorityTaskWoken);
			rxFlag = false;
			aknFlag = true;
		}
	} else
		rxFlag = true;
}

void HyroMotor::switchPin()
{
	if (txFlag) {
		GPIO_ResetBits(GPIOB, GPIO_Pin_12);
		txFlag = false;
	} else
		txFlag = true;
}

void HyroMotor::robotSpeed2WheelSpeed(float* robArr, int16_t* whArr)
{
	float32_t buff[2];
	buff[0] = (robArr[0] + robArr[1] * L_CENTER) * CONST_WHEEL_1 / R_WHEEL;
	buff[1] = (robArr[0] - robArr[1] * L_CENTER) * CONST_WHEEL_2 / R_WHEEL;
	for (uint8_t i = 0; i < 2; i ++) {
		whArr[i] = (int16_t) buff[i];
		if (buff[i] - 0.5 >= whArr[i]) {						// more accurate rounding speed
			whArr[i]++;
		} else if (buff[i] + 0.5 <= whArr[i])
			whArr[i]--;
	}
}

void HyroMotor::calculateXYAlf(int32_t* whArr, int32_t* whHistArr, float32_t* xyalfArr)
{
	static float32_t deltAng[2] = {0};
	static float32_t deltAlf = 0;
	static float32_t deltDist = 0;
	static float32_t cBuff = 2 * PI;

	for (uint8_t i = 0; i < 2; i++) {
		if (whHistArr[i] == 0)
			whHistArr[i] = whArr[i];
		deltAng[i] = (whArr[i] - whHistArr[i]) * W_CONST;
		whHistArr[i] = whArr[i];
	}
	deltAlf = (deltAng[0] - deltAng[1]) * ROTATION_CONST;
	if (deltAlf < 10 && deltAlf > -10) { 						// fix an incorrect answer during shutdown
		deltDist = (deltAng[0] + deltAng[1]) * R_WHEEL / 2;
		xyalfArr[2] += deltAlf;
		if (xyalfArr[2] >= cBuff){								//lead to the 2pi range (for ARM tables)
			xyalfArr[2] -= cBuff;
		} else if (xyalfArr[2] <= -cBuff)
			xyalfArr[2] += cBuff;
		xyalfArr[1] += deltDist * arm_sin_f32(xyalfArr[2]);
		xyalfArr[0] += deltDist * arm_cos_f32(xyalfArr[2]);
	}
}

uint8_t HyroMotor::getWheelStatus()
{
	return errStatus;
}

void HyroMotor::clearWheelStatus()
{
	if (this->errStatus == WHEEL_COLLISION)
		errStatus = WHEEL_OK;
}

void HyroMotor::motorInit(uint8_t id2set)
{
	motArr[id2set - 1]->modbusWriteReg(id2set, REG_MODE, MODE_SPEED);
	delayPort(4);
	motArr[id2set - 1]->modbusWriteReg(id2set, REG_S_PID_P, 1);
	delayPort(4);
	motArr[id2set - 1]->modbusWriteReg(id2set, REG_S_PID_I, 200);
	delayPort(4);
	motArr[id2set - 1]->modbusWriteReg(id2set, REG_S_PID_D, 0);
	delayPort(4);
	motArr[id2set - 1]->modbusWriteReg(id2set, REG_I_CUTOFF, 1500);
	delayPort(4);
	motArr[id2set - 1]->modbusWriteReg(id2set, REG_V_CUTOFF, 10000);
	delayPort(4);
	motArr[id2set - 1]->modbusWriteReg(id2set, REG_T_CUTOFF, 100);
	delayPort(4);
	motArr[id2set - 1]->modbusWriteReg(id2set, REG_PWM_LIMIT, 230);
	delayPort(4);
	motArr[id2set - 1]->modbusWriteReg(id2set, REG_S_PID_I_LIMIT, 2500);
	delayPort(4);
	motArr[id2set - 1]->modbusWriteReg(id2set, REG_A_PID_I_LIMIT, 2000);
	delayPort(4);
	motArr[id2set - 1]->modbusWriteReg(id2set, REG_ERROR_CODE, 0);
	delayPort(4);
	motArr[id2set - 1]->modbusWriteReg(id2set, REG_SET_SPEED, 0);
	delayPort(4);
	motArr[id2set - 1]->modbusReadReg(id2set, REG_READ_ANGLE_LOW, 2);
	delayPort(4);

}

void HyroMotor::run()
{
	uint8_t i, j = 0;
	uint8_t length = 0;
	uint8_t speedByteArr[8] = {0};
	uint8_t angleByteArr[12] = {0};
	uint8_t curCom = REG_READ_ANGLE_LOW;
	int16_t wheelSpeed[2] = {0};
	int32_t whAngleArr[2] = {0};
	int32_t whAngleHistArr[2] = {0};
	float32_t xyalfArr[3] = {0};
	float32_t robotSpeed[2];
	uint8_t color = 3;

	uart3.uartInit(GPIOB, USART3, true);
	uart3.gpioSwitchInit(GPIOB, GPIO_Pin_12);
	uart4.uartInit(GPIOA, UART4, true);
	uart4.gpioSwitchInit(GPIOB, GPIO_Pin_12);
	motor1.init(&uart3);
	motor2.init(&uart4);
	xTaskToNotify = xTaskGetCurrentTaskHandle();
	vTaskDelay(oRTOS.fromMsToTick(5000));
	this->motorInit(1);
	this->motorInit(2);
	this->rxFlag = false;
	this->txFlag = false;
	this->aknFlag = true;
	this->errStatus = WHEEL_POWER_OFF;

	while (1) {
		ulTaskNotifyTake(pdTRUE, oRTOS.fromMsToTick(4));
		if (aknFlag) {
			if (uxQueueMessagesWaiting(xHighLvlQueue)) {
				xSemaphoreTake(xHighLvlMutex, portMAX_DELAY);
				for (j = 0; j < 8; j++)
					xQueueReceive(xHighLvlQueue, &speedByteArr[j], portMAX_DELAY);
				xSemaphoreGive(xHighLvlMutex);
				memcpy(robotSpeed, speedByteArr, sizeof(robotSpeed));
				if (collisionHandler->getStatus() && robotSpeed[0] > 0) {
					color = 1; 										//red
					robotSpeed[0] = 0;
				} else if (robotSpeed[0] || robotSpeed[1]) {
					color = 3;
				} else
					color = 2;
				robotSpeed2WheelSpeed(robotSpeed, wheelSpeed);
				motor1.modbusWriteReg(1, REG_SET_SPEED, wheelSpeed[0]);
				motor2.modbusWriteReg(2, REG_SET_SPEED, wheelSpeed[1]);
				curCom = REG_ERROR_CODE;
				xQueueOverwrite(xLightColorQueue, &color);
			} else {
				motor1.modbusReadData(rxRsDataArr[0], length);
				motor2.modbusReadData(rxRsDataArr[1], length);
				aknFlag = false;								// flag that both acknowledges got
				txFlag = false;
				rxFlag = false;

				if (curCom == REG_ERROR_CODE) {
					int16_t errBuff1 = motor1.uint8toInt16(rxRsDataArr[0]);
					int16_t errBuff2 = motor1.uint8toInt16(rxRsDataArr[1]);
					if (errBuff1 != 0 || errBuff2 != 0) {
						for (i = 0; i < 2; i++) {
							motArr[i]->modbusWriteReg(i + 1, REG_SET_SPEED, STOP_MOTION);
							ulTaskNotifyTake(pdTRUE, oRTOS.fromMsToTick(4));
						}
						this->errStatus = WHEEL_COLLISION;
						vTaskDelay(oRTOS.fromMsToTick(500));
						motorInit(1);
						motorInit(2);
					}
					this->errStatus = WHEEL_OK;
					curCom = REG_READ_ANGLE_LOW;
					for (i = 0; i < 2; i++)
						motArr[i]->modbusReadReg(i + 1, curCom, 2);

				} else if (curCom == REG_READ_ANGLE_LOW) {
					for (i = 0; i < 2; i++) {							// low register comes first,
						uint8_t anglBuff[4];							// convert to correct sequence.
						for (j = 0; j < 2; j++) {
							anglBuff[j] = rxRsDataArr[i][j + 2];
							anglBuff[j + 2] = rxRsDataArr[i][j];
						}
						whAngleArr[i] = motArr[i]->uint8toInt32(anglBuff);
						if (i == 0)											// convert first wheel to
							whAngleArr[i] = -whAngleArr[i];					// right dir
					}
					calculateXYAlf(whAngleArr, whAngleHistArr, xyalfArr);	// from speed on wheels to
					memcpy(angleByteArr, xyalfArr, sizeof(angleByteArr));	// x, y, alf in global system
					xSemaphoreTake(xAngleMutex, portMAX_DELAY);
					if (uxQueueMessagesWaiting(xAngleQueue))
						xQueueReset(xAngleQueue);
					for (j = 0; j < 12; j++)
						xQueueSend(xAngleQueue, &angleByteArr[j], portMAX_DELAY);
					xSemaphoreGive(xAngleMutex);
					curCom = REG_ERROR_CODE;
					for (i = 0; i < 2; i++)
						motArr[i]->modbusReadReg(i + 1, curCom, 1);
				}
			}
		} else {
			this->errStatus = WHEEL_POWER_OFF;
			ulTaskNotifyTake(pdTRUE, oRTOS.fromMsToTick(1000));
			if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_15)) {				// check if stopButtom is pressed
				for (i = 0; i < 2; i++)
					motArr[i]->modbusWriteReg(i + 1, REG_SET_SPEED, STOP_MOTION);
			} else {
				while(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_15) == 0) {
					vTaskDelay(oRTOS.fromMsToTick(100));
					this->errStatus = WHEEL_POWER_OFF;
				}
				vTaskDelay(oRTOS.fromMsToTick(2000));
				for (i = 0; i < 2; i++) {
					whAngleHistArr[i] = 0;									// for correct calculation
					motorInit(i + 1);
				}
				this->errStatus = WHEEL_OK;
			}
		}
	}
//	GPIO_ResetBits(GPIOC, GPIO_Pin_14);
}

extern "C"
{
//---------------------------WHEEL_1---------------------------------//

//***********************UART_RECEIVE_INTERRUPT******************//
	void USART3_IRQHandler(void)
	{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;			// Notify task about interrupt
		if (USART_GetITStatus(USART3, USART_IT_IDLE)) {			// Clear IDLE flag step 1
			DMA_Cmd(DMA1_Stream1, DISABLE);						// DMA turn off to clear DMA1 counter
			USART_ReceiveData(USART3);							// Clear IDLE flag step 2
		}
		if (USART_GetITStatus(USART3, USART_IT_TC)) {
			USART_ClearITPendingBit(USART3, USART_IT_TC);
			hyroMotor->switchPin();
		}
		if (xHigherPriorityTaskWoken)
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);		// Run Higher priority task if exist													// between ticks
	}
//***********************DMA_RECEIVE_INTERRUPT******************//
	void DMA1_Stream1_IRQHandler(void)
	{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;			// Notify task about interrupt
		hyroMotor->taskNotifyFromISR(xHigherPriorityTaskWoken);
		DMA_ClearITPendingBit(DMA1_Stream1, DMA_IT_TCIF1);		// Clear DMA "transmitting complete" interrupt
		DMA_Cmd(DMA1_Stream1, ENABLE);							// Reset DMA
		if (xHigherPriorityTaskWoken)
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);		// Run Higher priority task if exist													// between ticks
	}

//***********************DMA_TRANSMIT_INTERRUPT*****************//
	void DMA1_Stream3_IRQHandler(void)
	{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;			// Notify task about interrupt
		DMA_ClearITPendingBit(DMA1_Stream3, DMA_IT_TCIF3);		// Clear DMA "transmission complete" interrupt
		DMA_Cmd(DMA1_Stream3, DISABLE);							// Stop DMA transmitting
		if (xHigherPriorityTaskWoken)
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);		// Run Higher priority task if exist													// between ticks
	}

//---------------------------WHEEL_2---------------------------------//

	//***********************UART_RECEIVE_INTERRUPT******************//
	void UART4_IRQHandler(void)
	{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;			// Notify task about interrupt
		if (USART_GetITStatus(UART4, USART_IT_IDLE)){			// Clear IDLE flag step 1
			DMA_Cmd(DMA1_Stream2, DISABLE);						// DMA turn off to clear DMA1 counter
			USART_ReceiveData(UART4);							// Clear IDLE flag step 2
		}
		if (USART_GetITStatus(UART4, USART_IT_TC)){
			USART_ClearITPendingBit(UART4, USART_IT_TC);
			hyroMotor->switchPin();
		}
		if (xHigherPriorityTaskWoken)							// Run Higher priority task if exist
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);		// between ticks
	}
	//***********************DMA_RECEIVE_INTERRUPT******************//
	void DMA1_Stream2_IRQHandler(void)
	{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;			// Notify task about interrupt
		hyroMotor->taskNotifyFromISR(xHigherPriorityTaskWoken);
		DMA_ClearITPendingBit(DMA1_Stream2, DMA_IT_TCIF2);		// Clear DMA "transmitting complete" interrupt
		DMA_Cmd(DMA1_Stream2, ENABLE);							// Reset DMA
		if (xHigherPriorityTaskWoken)							// Run Higher priority task if exist
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);		// between ticks
	}
	//***********************DMA_TRANSMIT_INTERRUPT*****************//
	void DMA1_Stream4_IRQHandler(void)
	{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;			// Notify task about interrupt
		DMA_ClearITPendingBit(DMA1_Stream4, DMA_IT_TCIF4);		// Clear DMA "transmission complete" interrupt
		DMA_Cmd(DMA1_Stream4, DISABLE);							// Stop DMA transmitting
		if (xHigherPriorityTaskWoken)							// Run Higher priority task if exist
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);		// between ticks
	}
}

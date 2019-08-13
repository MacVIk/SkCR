///*
// * HyroMotor.cpp
// *
// *  Created on: 11.03.2019
// *      Author: Taras.Melnik
// */
//
//#include "HyroMotor.h"
//
//HyroMotor* hyroMotor;
//UARTuserInit uart3;
//UARTuserInit uart4;
//UARTtoRS485 motor1;
//UARTtoRS485 motor2;
//
//HyroMotor::HyroMotor() {
////	xAngleQueue = xQueueCreate(12, sizeof(uint8_t));
////	xAngleMutex = xSemaphoreCreateMutex();
////	xHighLvlQueue = xQueueCreate(8, sizeof(uint8_t));
//	xHighLvlMutex = xSemaphoreCreateMutex();
//	xTaskToNotify = 0;
//	motArr[0] = &motor1;
//	motArr[1] = &motor2;
//	for (uint8_t i = 0; i < 2; i++)
//		this->rxRsDataArr[i] = new uint8_t[4];
//	this->rxFlag = false;
//	this->txFlag = false;
//	this->aknFlag = true;
//	this->hlFlag = false;
//	this->powFlag = true;
//	this->curColFlag = false;
//}
//
//HyroMotor::~HyroMotor() {
//}
//
//void HyroMotor::delayPort(uint32_t ticks)
//{
//	txFlag = true;
//	rxFlag = true;
//	ulTaskNotifyTake(pdTRUE, oRTOS.fromMsToTick(ticks));
//}
//
//void HyroMotor::taskNotifyFromISR(BaseType_t xHigherPriorityTaskWoken)
//{
//	if (rxFlag) {
//	        if (xTaskToNotify != 0) {
//	                vTaskNotifyGiveFromISR(xTaskToNotify, &xHigherPriorityTaskWoken);
//	                rxFlag = false;
//	                aknFlag = true;
//	        }
//	} else
//	        rxFlag = true;
//}
//
//void HyroMotor::switchPin()
//{
//	if (txFlag) {
//		GPIO_ResetBits(GPIOB, GPIO_Pin_12);
//		txFlag = false;
//	} else
//		txFlag = true;
//}
//
//void HyroMotor::robotSpeed2WheelSpeed(float* robArr, int16_t* whArr)
//{
//	float32_t buff[2];
//	buff[0] = (robArr[0] + robArr[1] * L_CENTER) * CONST_WHEEL_1 / R_WHEEL;
//	buff[1] = (robArr[0] - robArr[1] * L_CENTER) * CONST_WHEEL_2 / R_WHEEL;
//	for (uint8_t i = 0; i < 2; i ++) {
//		whArr[i] = (int16_t) buff[i];
//		if (buff[i] - 0.5 >= whArr[i]) {						// more accurate rounding speed
//			whArr[i]++;
//		} else if (buff[i] + 0.5 <= whArr[i])
//			whArr[i]--;
//	}
//}
//
//void HyroMotor::calculateXYAlf(int32_t* whArr, int32_t* whHistArr, float32_t* xyalfArr)
//{
//	static float32_t deltAng[2] = {0};
//	static float32_t deltAlf = 0;
//	static float32_t deltDist = 0;
//	static float32_t cBuff = 2 * PI;
//
//	for (uint8_t i = 0; i < 2; i++) {
//		if (whHistArr[i] == 0)
//			whHistArr[i] = whArr[i];
//		deltAng[i] = (whArr[i] - whHistArr[i]) * W_CONST;
//		whHistArr[i] = whArr[i];
//	}
//	deltAlf = (deltAng[0] - deltAng[1]) * ROTATION_CONST;
//	if (deltAlf < 10 && deltAlf > -10) { 						// fix an incorrect answer during shutdown
//		deltDist = (deltAng[0] + deltAng[1]) * R_WHEEL / 2;
//		xyalfArr[2] += deltAlf;
//		if (xyalfArr[2] >= cBuff){								//lead to the 2pi range (for ARM tables)
//			xyalfArr[2] -= cBuff;
//		} else if (xyalfArr[2] <= -cBuff)
//			xyalfArr[2] += cBuff;
//		xyalfArr[1] += deltDist * arm_sin_f32(xyalfArr[2]);
//		xyalfArr[0] += deltDist * arm_cos_f32(xyalfArr[2]);
//	}
//}
//
//bool HyroMotor::getWhCurColStatus()
//{
//	bool buff = this->curColFlag;
//	this->curColFlag = false;
//	return buff;
//}
//
//bool HyroMotor::getWhPowStatus()
//{
//	bool buff = this->powFlag;
//	this->powFlag = false;
//	return buff;
//}
//
//void HyroMotor::setSpeed(uint8_t* byteArr)
//{
//	for (uint8_t i = 0; i < 8; i++)
//		this->rByteArr[i] = byteArr[i];
//	this->hlFlag = true;
//}
//
//void HyroMotor::getOdometry(uint8_t* byteArr)
//{
//	for (uint8_t i = 0; i < 12; i++)
//		 byteArr[i] = this->tByteArr[i];
//}
//
//void HyroMotor::motorInit(uint8_t id2set)
//{
//	motArr[id2set - 1]->modbusWriteReg(id2set, REG_MODE, MODE_SPEED);
//	delayPort(4);
//	motArr[id2set - 1]->modbusWriteReg(id2set, REG_S_PID_P, 1);
//	delayPort(4);
//	motArr[id2set - 1]->modbusWriteReg(id2set, REG_S_PID_I, 200);
//	delayPort(4);
//	motArr[id2set - 1]->modbusWriteReg(id2set, REG_S_PID_D, 0);
//	delayPort(4);
//	motArr[id2set - 1]->modbusWriteReg(id2set, REG_I_CUTOFF, 3000);
//	delayPort(4);
//	motArr[id2set - 1]->modbusWriteReg(id2set, REG_V_CUTOFF, 10000);
//	delayPort(4);
//	motArr[id2set - 1]->modbusWriteReg(id2set, REG_T_CUTOFF, 100);
//	delayPort(4);
//	motArr[id2set - 1]->modbusWriteReg(id2set, REG_PWM_LIMIT, 230);
//	delayPort(4);
//	motArr[id2set - 1]->modbusWriteReg(id2set, REG_S_PID_I_LIMIT, 2000);
//	delayPort(4);
//	motArr[id2set - 1]->modbusWriteReg(id2set, REG_A_PID_I_LIMIT, 2000);
//	delayPort(4);
//	motArr[id2set - 1]->modbusWriteReg(id2set, REG_ERROR_CODE, 0);
//	delayPort(4);
//	motArr[id2set - 1]->modbusWriteReg(id2set, REG_SET_SPEED, 0);
//	delayPort(4);
//	motArr[id2set - 1]->modbusReadReg(id2set, REG_READ_ANGLE_LOW, 2);
//	delayPort(4);
//}
//
//void HyroMotor::run()
//{
//	uint8_t i, j = 0;
//	uint8_t length = 0;
//	uint8_t angleByteArr[12] = {0};
//	uint8_t curCom = REG_READ_ANGLE_LOW;
//	int16_t wheelSpeed[2] = {0};
//	int32_t whAngleArr[2] = {0};
//	int32_t whAngleHistArr[2] = {0};
//	float32_t xyalfArr[3] = {0};
//	float32_t robotSpeed[2];
//
//	uart3.uartInit(GPIOB, USART3, true);
//	uart3.gpioSwitchInit(GPIOB, GPIO_Pin_12);
//	uart4.uartInit(GPIOA, UART4, true);
//	uart4.gpioSwitchInit(GPIOB, GPIO_Pin_12);
//	motor1.init(&uart3);
//	motor2.init(&uart4);
//	xTaskToNotify = xTaskGetCurrentTaskHandle();
//	vTaskDelay(oRTOS.fromMsToTick(5000));
//	this->motorInit(1);
//	this->motorInit(2);
//	this->powFlag = false;
//
//	while (1) {
//		ulTaskNotifyTake(pdTRUE, oRTOS.fromMsToTick(4));
//		if (aknFlag) {
//			if (hlFlag) {
//				this->hlFlag = false;
//				xSemaphoreTake(xHighLvlMutex, portMAX_DELAY);
//				memcpy(robotSpeed, rByteArr, sizeof(robotSpeed));
//				xSemaphoreGive(xHighLvlMutex);
//				if (collisionHandler->getStatus() && robotSpeed[0] > 0) {
//					setLEDTask->setColor(RED);
//					robotSpeed[0] = 0;
//				} else if (robotSpeed[0] || robotSpeed[1]) {
//					setLEDTask->setColor(BLUE);
//				} else
//					setLEDTask->setColor(GREEN);
//				robotSpeed2WheelSpeed(robotSpeed, wheelSpeed);
//				motor1.modbusWriteReg(1, REG_SET_SPEED, wheelSpeed[0]);
//				motor2.modbusWriteReg(2, REG_SET_SPEED, wheelSpeed[1]);
//				curCom = REG_SET_SPEED;
//			} else {
//				motor1.modbusReadData(rxRsDataArr[0], length);
//				motor2.modbusReadData(rxRsDataArr[1], length);
//				aknFlag = false;								// flag that both acknowledges got
//				txFlag = false;
//				rxFlag = false;
//
//				if (curCom == REG_SET_SPEED) {
//					curCom = REG_READ_ANGLE_LOW;
//					for (i = 0; i < 2; i++)
//						motArr[i]->modbusReadReg(i + 1, curCom, 2);
//
//				} else if (curCom == REG_ERROR_CODE) {
//					int16_t errBuff1 = motor1.uint8toInt16(rxRsDataArr[0]);
//					int16_t errBuff2 = motor2.uint8toInt16(rxRsDataArr[1]);
//					if (errBuff1 != 0 || errBuff2 != 0) {
//						for (i = 0; i < 2; i++) {
//							motArr[i]->modbusWriteReg(i + 1, REG_SET_SPEED, STOP_MOTION);
//							ulTaskNotifyTake(pdTRUE, oRTOS.fromMsToTick(4));
//						}
//						this->curColFlag = true;
//						setLEDTask->setColor(RED);
//						vTaskDelay(oRTOS.fromMsToTick(1000));
//						motorInit(1);
//						motorInit(2);
//					}
////					this->curColFlag = false;
//					curCom = REG_READ_ANGLE_LOW;
//					for (i = 0; i < 2; i++)
//						motArr[i]->modbusReadReg(i + 1, curCom, 2);
//
//				} else if (curCom == REG_READ_ANGLE_LOW) {
//					for (i = 0; i < 2; i++) {							// low register comes first,
//						uint8_t anglBuff[4];							// convert to correct sequence.
//						for (j = 0; j < 2; j++) {
//							anglBuff[j] = rxRsDataArr[i][j + 2];
//							anglBuff[j + 2] = rxRsDataArr[i][j];
//						}
//						whAngleArr[i] = motArr[i]->uint8toInt32(anglBuff);
//						if (i == 0)											// convert first wheel to
//							whAngleArr[i] = -whAngleArr[i];					// right dir
//					}
//					calculateXYAlf(whAngleArr, whAngleHistArr, xyalfArr);	// from speed on wheels to
//					xSemaphoreTake(xHighLvlMutex, portMAX_DELAY);
//					memcpy(tByteArr, xyalfArr, sizeof(tByteArr));	// x, y, alf in global system
//					xSemaphoreGive(xHighLvlMutex);
//					curCom = REG_READ_CURRENT;
//					for (i = 0; i < 2; i++)
//						motArr[i]->modbusReadReg(i + 1, curCom, 1);
//
//				} else if (curCom == REG_READ_CURRENT) {
//					for (i = 0; i < 2; i++) {
//						int16_t curVal = motor1.uint8toInt16(rxRsDataArr[i]);
//						if (curVal >= 1000 || curColFlag)
//							this->curColFlag = true;
//					}
//					if (curColFlag) {
//						curCom = REG_SET_SPEED;
//						motor1.modbusWriteReg(1, REG_SET_SPEED, 0);
//						motor2.modbusWriteReg(2, REG_SET_SPEED, 0);
//						setLEDTask->setColor(RED);
//					} else {
//						curCom = REG_ERROR_CODE;
//						for (i = 0; i < 2; i++)
//							motArr[i]->modbusReadReg(i + 1, curCom, 1);
//					}
//				}
//			}
//		} else {
//			this->powFlag = true;
//			ulTaskNotifyTake(pdTRUE, oRTOS.fromMsToTick(1000));
//			if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_15)) {				// check if stopButtom is pressed
//				for (i = 0; i < 2; i++)
//					motArr[i]->modbusWriteReg(i + 1, REG_SET_SPEED, STOP_MOTION);
//				setLEDTask->setColor(RED);
//			} else {
//				while(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_15) == 0) {
//					vTaskDelay(oRTOS.fromMsToTick(100));
//					setLEDTask->setError();
//					this->powFlag = true;
//				}
//				vTaskDelay(oRTOS.fromMsToTick(3000));
//				for (i = 0; i < 2; i++) {
//					whAngleHistArr[i] = 0;									// for correct calculation
//					motorInit(i + 1);
//				}
//				setLEDTask->clearError();
////				this->powFlag = false;
////				vTaskDelay(oRTOS.fromMsToTick(1000));
////				setLEDTask->setColor(BLUE);
//			}
//		}
//	}
//}
//
//extern "C"
//{
////---------------------------WHEEL_1---------------------------------//
//
////***********************UART_RECEIVE_INTERRUPT******************//
//	void USART3_IRQHandler(void)
//	{
//		BaseType_t xHigherPriorityTaskWoken = pdFALSE;			// Notify task about interrupt
//		if (USART_GetITStatus(USART3, USART_IT_IDLE)) {			// Clear IDLE flag step 1
//			DMA_Cmd(DMA1_Stream1, DISABLE);						// DMA turn off to clear DMA1 counter
//			USART_ReceiveData(USART3);							// Clear IDLE flag step 2
//		}
//		if (USART_GetITStatus(USART3, USART_IT_TC)) {
//			USART_ClearITPendingBit(USART3, USART_IT_TC);
//			hyroMotor->switchPin();
//		}
//		if (xHigherPriorityTaskWoken)
//			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);		// Run Higher priority task if exist													// between ticks
//	}
////***********************DMA_RECEIVE_INTERRUPT******************//
//	void DMA1_Stream1_IRQHandler(void)
//	{
//		BaseType_t xHigherPriorityTaskWoken = pdFALSE;			// Notify task about interrupt
//		hyroMotor->taskNotifyFromISR(xHigherPriorityTaskWoken);
//		DMA_ClearITPendingBit(DMA1_Stream1, DMA_IT_TCIF1);		// Clear DMA "transmitting complete" interrupt
//		DMA_Cmd(DMA1_Stream1, ENABLE);							// Reset DMA
//		if (xHigherPriorityTaskWoken)
//			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);		// Run Higher priority task if exist													// between ticks
//	}
//
////***********************DMA_TRANSMIT_INTERRUPT*****************//
//	void DMA1_Stream3_IRQHandler(void)
//	{
//		BaseType_t xHigherPriorityTaskWoken = pdFALSE;			// Notify task about interrupt
//		DMA_ClearITPendingBit(DMA1_Stream3, DMA_IT_TCIF3);		// Clear DMA "transmission complete" interrupt
//		DMA_Cmd(DMA1_Stream3, DISABLE);							// Stop DMA transmitting
//		if (xHigherPriorityTaskWoken)
//			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);		// Run Higher priority task if exist													// between ticks
//	}
//
////---------------------------WHEEL_2---------------------------------//
//
//	//***********************UART_RECEIVE_INTERRUPT******************//
//	void UART4_IRQHandler(void)
//	{
//		BaseType_t xHigherPriorityTaskWoken = pdFALSE;			// Notify task about interrupt
//		if (USART_GetITStatus(UART4, USART_IT_IDLE)){			// Clear IDLE flag step 1
//			DMA_Cmd(DMA1_Stream2, DISABLE);						// DMA turn off to clear DMA1 counter
//			USART_ReceiveData(UART4);							// Clear IDLE flag step 2
//		}
//		if (USART_GetITStatus(UART4, USART_IT_TC)){
//			USART_ClearITPendingBit(UART4, USART_IT_TC);
//			hyroMotor->switchPin();
//		}
//		if (xHigherPriorityTaskWoken)							// Run Higher priority task if exist
//			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);		// between ticks
//	}
//	//***********************DMA_RECEIVE_INTERRUPT******************//
//	void DMA1_Stream2_IRQHandler(void)
//	{
//		BaseType_t xHigherPriorityTaskWoken = pdFALSE;			// Notify task about interrupt
//		hyroMotor->taskNotifyFromISR(xHigherPriorityTaskWoken);
//		DMA_ClearITPendingBit(DMA1_Stream2, DMA_IT_TCIF2);		// Clear DMA "transmitting complete" interrupt
//		DMA_Cmd(DMA1_Stream2, ENABLE);							// Reset DMA
//		if (xHigherPriorityTaskWoken)							// Run Higher priority task if exist
//			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);		// between ticks
//	}
//	//***********************DMA_TRANSMIT_INTERRUPT*****************//
//	void DMA1_Stream4_IRQHandler(void)
//	{
//		BaseType_t xHigherPriorityTaskWoken = pdFALSE;			// Notify task about interrupt
//		DMA_ClearITPendingBit(DMA1_Stream4, DMA_IT_TCIF4);		// Clear DMA "transmission complete" interrupt
//		DMA_Cmd(DMA1_Stream4, DISABLE);							// Stop DMA transmitting
//		if (xHigherPriorityTaskWoken)							// Run Higher priority task if exist
//			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);		// between ticks
//	}
//}

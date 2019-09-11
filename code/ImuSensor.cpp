///*
// * ImuSensor.cpp
// *
// *  Created on: 10 θών. 2019 γ.
// *      Author: Taras.Melnik
// */
//
//#include "ImuSensor.h"
//
//void ImuSensor::accelAdjustment(int16_t* aConst)
//{
//	int32_t* accArr[3];
//	int16_t adjArr[3] = {0};
//	for (uint8_t i = 0; i < 3; i++) {
//		accArr[i] = new int32_t[20];
//		for (uint8_t j = 0; j < 20; j++)
//			accArr[i][j] = 0;
//	}
//	for (uint8_t j = 1; j < 20; j++) {
//		this->i2cRead(ACCELEROMETER_ADDR, ACCELEROMETER_OUT_X_L | 1 << 7, 6, i2cRxArr);
//		memcpy(adjArr, i2cRxArr, sizeof(adjArr));
//		for (uint8_t i = 0; i < 3; i++)
//			accArr[i][j] = accArr[i][j - 1] + adjArr[i];
//		vTaskDelay(oRTOS.fromMsToTick(10));
//	}
//	for (uint8_t i = 0; i < 3; i++)
//		aConst[i] = (int16_t) (accArr[i][19] / 19);
//}
//
//void ImuSensor::gyroAdjustment(int16_t* aConst)
//{
//	int32_t* accArr[3];
//	int16_t adjArr[3] = {0};
//	for (uint8_t i = 0; i < 3; i++) {
//		accArr[i] = new int32_t[20];
//		for (uint8_t j = 0; j < 20; j++)
//			accArr[i][j] = 0;
//	}
//	for (uint8_t j = 1; j < 20; j++) {
//		this->i2cRead(GYROSCOPE_ADDR, GYROSCOPE_OUT_X_L | 1 << 7, 6, i2cRxArr);
//		memcpy(adjArr, i2cRxArr, sizeof(adjArr));
//		for (uint8_t i = 0; i < 3; i++)
//			accArr[i][j] = accArr[i][j - 1] + adjArr[i];
//		vTaskDelay(oRTOS.fromMsToTick(10));
//	}
//	for (uint8_t i = 0; i < 3; i++)
//		aConst[i] = (int16_t) (accArr[i][19] / 19);
//}
//
//void ImuSensor::calculateXYTheta(float32_t* vArr, float32_t* xyalfArr)
//{
//	static float32_t deltAlf = 0;
//	static float32_t deltDist = 0;
//	static float32_t cBuff = 2 * PI;
//
//	deltDist = vArr[0] * 0.001f;
//	deltAlf = vArr[1];
//	if (deltAlf < 5 && deltAlf > -5 && deltDist < 0.1 && deltDist > -0.1) { 						// fix an incorrect answer during shutdown
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
//
//
//void ImuSensor::run() {
//	float32_t velArr[2] = {0};
//	float32_t velHistArr[2] = {0};
//	float32_t xyThetArr[3] = {0};
//	float32_t constAcc = 2.f * 9.8f / 32767.f;
//	float32_t constGyro = 0.0875f / 57.3f;
//	int16_t testArr[3] = {0};
//	int16_t cArr[3] = {0};
//	int16_t wArr[3] = {0};
//	this->accelInit();
//	this->gyroInit();
//	this->accelAdjustment(cArr);
//	this->gyroAdjustment(wArr);
//
//	 while (1) {
//		 int16_t bufI2c = 0;
//		 this->i2cRead(ACCELEROMETER_ADDR, ACCELEROMETER_OUT_Y_L | 1 << 7, 2, i2cRxArr);
//		 memcpy(&bufI2c, i2cRxArr, sizeof(bufI2c));
//		 velArr[0] += (bufI2c - cArr[1]) * constAcc * 0.001f;
//		 this->i2cRead(GYROSCOPE_ADDR, GYROSCOPE_OUT_Z_L | 1 << 7, 2, i2cRxArr);
//		 memcpy(&bufI2c, i2cRxArr, sizeof(bufI2c));
//		 velArr[1] = (bufI2c - wArr[2]) * constGyro * 0.001f;
//		 calculateXYTheta(velArr, xyThetArr);
//		 memcpy(tByteArr, xyThetArr, sizeof(tByteArr));
//		 vTaskDelay(oRTOS.fromMsToTick(10));
//
////		 // Test for angularVelocities
////		 this->i2cRead(GYROSCOPE_ADDR, GYROSCOPE_OUT_X_L | 1 << 7, 6, i2cRxArr);
////		 memcpy(testArr, i2cRxArr, sizeof(testArr));
////		 for (uint8_t i = 0; i < 3; i++)
////			 xyThetArr[i] += (testArr[i] - wArr[i]) * constGyro * 0.001f;
////		 memcpy(tByteArr, xyThetArr, sizeof(tByteArr));
////		 vTaskDelay(oRTOS.fromMsToTick(10));
//
////		 // Test for accelerations
////		 int16_t bufI2c = 0;
////		 this->i2cRead(ACCELEROMETER_ADDR, ACCELEROMETER_OUT_X_L | 1 << 7, 6, i2cRxArr);
////		 memcpy(testArr, i2cRxArr, sizeof(testArr));
////		 for (uint8_t i = 0; i < 3; i++)
////			 xyThetArr[i] += (testArr[i] - cArr[i]) * constAcc * 0.001f;
////		 memcpy(tByteArr, xyThetArr, sizeof(tByteArr));
////		 vTaskDelay(oRTOS.fromMsToTick(10));
//	 }
//}
//
//extern "C" {
//
//void I2C1_IRQHandler(void)
//	{
//		BaseType_t xHigherPriorityTaskWoken = pdFALSE;			// Notify task about interrupt
//		if (USART_GetITStatus(USART3, USART_IT_IDLE)) {			// Clear IDLE flag step 1
//			DMA_Cmd(DMA1_Stream1, DISABLE);						// DMA turn off to clear DMA1 counter
//			USART_ReceiveData(USART3);							// Clear IDLE flag step 2
//		}
//		if (USART_GetITStatus(USART3, USART_IT_TC)) {
//			USART_ClearITPendingBit(USART3, USART_IT_TC);
//		}
//		if (xHigherPriorityTaskWoken)
//			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);		// Run Higher priority task if exist													// between ticks
//	}
////***********************DMA_RECEIVE_INTERRUPT******************//
//	void DMA1_Stream5_IRQHandler(void)
//	{
//		BaseType_t xHigherPriorityTaskWoken = pdFALSE;			// Notify task about interrupt
//		DMA_ClearITPendingBit(DMA1_Stream1, DMA_IT_TCIF1);		// Clear DMA "transmitting complete" interrupt
//		DMA_Cmd(DMA1_Stream1, ENABLE);							// Reset DMA
//		if (xHigherPriorityTaskWoken)
//			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);		// Run Higher priority task if exist													// between ticks
//	}
//
//}
//
//
//
//
//
//

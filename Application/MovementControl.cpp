/*
 * MovementControl.cpp
 *
 *  Created on: 06.02.2019
 *      Author: Taras.Melnik
 */

#include "MovementControl.h"

QueueHandle_t xMovementControlQueueRe;
QueueHandle_t xMovementControlQueueTr;
SemaphoreHandle_t xMovementControlMutexRe;
SemaphoreHandle_t xMovementControlMutexTr;
ReadEncoders encData;

MovementControl::MovementControl(){
	xMovementControlQueueRe = xQueueCreate(8, sizeof(uint8_t));
	xMovementControlMutexRe = xSemaphoreCreateMutex();
	xMovementControlQueueTr = xQueueCreate(8, sizeof(uint8_t));
	xMovementControlMutexTr = xSemaphoreCreateMutex();
	this->kp = 0;
	this->ki = 0;
	this->kd = 0;
}

MovementControl::~MovementControl() {
	// TODO Auto-generated destructor stub
}

void MovementControl::writePIDkoef(uint8_t p, uint8_t i, uint8_t d)
{
	this->kp = p;
	this->ki = i;
	this->kd = d;
}

void MovementControl::writeToQueue(uint8_t &data)
{
	xSemaphoreTake(xMovementControlMutexRe, portMAX_DELAY);
	if (uxQueueMessagesWaiting(xMovementControlQueueRe) == 8)
	{
		xQueueReset(xMovementControlQueueRe);
	}
	xQueueSend(xMovementControlQueueRe, &data, portMAX_DELAY);
	xSemaphoreGive(xMovementControlMutexRe);
}

void MovementControl::readFromQueue(uint8_t &data)
{
	xQueueReceive(xMovementControlQueueTr, &data, portMAX_DELAY);
}

void MovementControl::run()
{
	uint8_t inputData[8];
	uint8_t outpuData[8];
	float velValue[2] = { 0, 0 };
	float encVelVal[2] = { 0, 0 };
	float primeArr[10];
	float sortArr1[10];
	float sortArr2[10];

//	float vel_f[2];

	uint32_t motorPWM = 0;

//	arm_pid_instance_f32 PIDmot1;
//	arm_pid_instance_f32 PIDmot2;

	uint8_t i = 0;

	// motion start

	GPIO_SetBits(GPIOE, DIR_MOTOR_1);
	GPIO_SetBits(GPIOE, DIR_MOTOR_2);
	GPIO_ResetBits(GPIOE, START_MOTOR_1);
	GPIO_ResetBits(GPIOE, START_MOTOR_2);
	this->taskDelay(oRTOS.fromMsToTick(500));
	GPIO_SetBits(GPIOC, MOTOR_RELEY);


	while (1)
	{
//		GPIO_SetBits(GPIOC, GPIO_Pin_14);

		if (uxQueueMessagesWaiting(xMovementControlQueueRe) != 0)
		{
			xSemaphoreTake(xMovementControlMutexRe, portMAX_DELAY);
			for (i = 0; i < 8; i++)
			{
				xQueueReceive(xMovementControlQueueRe, &inputData[i], 10);
			}
			xSemaphoreGive(xMovementControlMutexRe);

			memcpy(velValue, inputData, sizeof(velValue));
		}

//		if (uxQueueMessagesWaiting(encData.)) ToDo
		for ( i = 0; i < 2; i++)
		{
			encData.readFromQueue(encVelVal[i]);
		}


//		PIDmot1.Kp = kp;
//		PIDmot1.Kd = kd;
//		PIDmot1.Ki = ki;
//		for (i = 0; i < 3; i++)
//		{
//			PIDmot1.state[i] = velEnc[0][i];
//		}
//		arm_pid_f32

//		arm_pid_init_f32(PIDmot1, );

//		arm_pid_init_f32();
		encVelVal[0] =  (encVelVal[0] >= 0) ? encVelVal[0] : -encVelVal[0];
		float deltVelVal = velValue[0] - encVelVal[0];
//		deltVelVal = (deltVelVal > 0) ? (deltVelVal) : (-deltVelVal);

		if (deltVelVal > VELOCITY_ERROR_DIAPASON)
		{
			motorPWM += 2;
		}
		if (deltVelVal < -VELOCITY_ERROR_DIAPASON)
		{
			motorPWM -= 2;
		}
		motorPWM = (motorPWM <= 0) ? 0 : motorPWM;
		TIM9->CCR1 =  motorPWM;
		TIM9->CCR2 =  motorPWM;
//		TIM9->CCR1 =  (velValue[0] >= 0) ? velValue[0] : -velValue[0];
//		TIM9->CCR2 =  (velValue[1] >= 0) ? velValue[1] : -velValue[1];

		memcpy(outpuData, encVelVal, sizeof(outpuData));

		xSemaphoreTake(xMovementControlMutexTr, portMAX_DELAY);
		if (uxQueueMessagesWaiting(xMovementControlQueueTr) == 8)
		{
			xQueueReset(xMovementControlQueueTr);
		}
		for (i = 0; i < 8; i++)
		{
			xQueueSend(xMovementControlQueueTr, &outpuData[i], 10);
		}
		xSemaphoreGive(xMovementControlMutexTr);

		this->taskDelay(oRTOS.fromMsToTick(50));
	  }
}





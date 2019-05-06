/*
 * HyroMotor.h
 *
 *  Created on: 11.03.2019
 *      Author: Taras.Melnik
 */
#ifndef HYROMOTOR_H_
#define HYROMOTOR_H_

#include "iActiveObject.h"
#include "QueueCreator.h"
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "semphr.h"
#include "UARTtoRS485.h"
#include "UARTuserInit.h"
#include "arm_math.h"
#include <math.h>
#include "LEDStrip.h"
#include "CollisionHandler.h"

//*************Modbus_Functions*************

#define READ_MODBUS_REG 	((uint8_t)  0x03)
#define WRITE_MODBUS_REG 	((uint8_t)  0x06)
#define WRITE_MULTIPLE_REG 	((uint8_t)  0x10)

//*************COMANDS_MACROSES*************
#define MODE_ANGLE 			((uint8_t)  1)
#define MODE_SPEED			((uint8_t)  2)
#define MODE_PWM			((uint8_t)  3)
#define MODE_NONE			((uint8_t)  0)

//**************int16_t_Registers*************
#define REG_MODE			((uint8_t)  0)
#define REG_SET_PWM			((uint8_t)  2)
#define REG_SET_SPEED		((uint8_t)  3)
#define REG_SET_ANGLE_LOW	((uint8_t)  4)
#define REG_SET_ANGLE_HIGH	((uint8_t)  5)
#define REG_SPEED_DIV		((uint8_t)  8)
#define REG_V_CUTOFF		((uint8_t)  9)
#define REG_I_CUTOFF		((uint8_t)  10)
#define REG_T_CUTOFF		((uint8_t)  11)
#define REG_S_PID_I			((uint8_t)  12)
#define REG_S_PID_P			((uint8_t)  13)
#define REG_S_PID_D			((uint8_t)  14)
#define REG_A_PID_I			((uint8_t)  15)
#define REG_A_PID_P			((uint8_t)  16)
#define REG_A_PID_D			((uint8_t)  17)
#define REG_TIME_OUT		((uint8_t)  18)
#define REG_S_PID_I_LIMIT	((uint8_t)  19)
#define REG_A_PID_I_LIMIT	((uint8_t)  20)
#define REG_PWM_LIMIT		((uint8_t)  21)
#define REG_ERROR_CODE		((uint8_t)  29)

//**************RO_Registers*************

#define REG_READ_CURRENT	((uint8_t)  65)
#define REG_READ_TEMP		((uint8_t)  66)
#define REG_READ_ANGLE_LOW	((uint8_t)  67)
#define REG_READ_ANGLE_HIGH	((uint8_t)  68)
#define REG_READ_SPEED		((uint8_t)  69)

//**************RW_Registers*************

#define REG_ID				((int16_t)  129)
#define REG_SAVE_FLASH		((int16_t)  130)

//***************************************

#define STOP_MOTION			((int16_t)  0)
#define ERROR_CURRENT 		((int8_t)  0xff)

//**************Robot_options*************

#define W_CONST				((float32_t)  0.00164f)  	// rad/ticksOfencoders
#define L_CENTER			((float32_t)  0.25f)  		// distance between wheel and center (m)
#define R_WHEEL				((float32_t)  0.0833f)  	// wheel radius (m)
#define CONST_WHEEL_1 		((float32_t)  -3.57f)		// (pointsOfwheel/rad)
#define CONST_WHEEL_2 		((float32_t)  3.57f)		// (pointsOfwheel/rad)
#define ROTATION_CONST		((float32_t)  0.1666f)  	// WheelRadius/CenterRadius

//**************Wheels_status*************
#define WHEEL_OK			((uint8_t)  0x64)
#define WHEEL_COLLISION		((uint8_t)  0x00)
#define WHEEL_POWER_OFF		((uint8_t)  0xff)

class HyroMotor: public iActiveObject
{
public:
	HyroMotor();
	virtual ~HyroMotor();
	void delayPort(uint32_t ticks);
	void taskNotifyFromISR(BaseType_t xHigherPriorityTaskWoken);
	void switchPin();
	void robotSpeed2WheelSpeed(float* hDatArr, int16_t* whArr);
	void calculateXYAlf(int32_t* whArr, int32_t* whHistArr, float32_t* robArr);
	bool getWhCurColStatus();
	bool getWhPowStatus();
	void setSpeed(uint8_t* byteArr);
	void getOdometry(uint8_t* byteArr);
	void motorInit(uint8_t idid2set);
	void run();
	uint8_t* rxRsDataArr[2];
private:
	SemaphoreHandle_t xHighLvlMutex;
	TaskHandle_t xTaskToNotify;
	UARTtoRS485* motArr[2];
	uint8_t rByteArr[8];
	uint8_t tByteArr[12];
	bool hlFlag;
	bool txFlag;
	bool rxFlag;
	bool aknFlag;
	bool powFlag;
	bool curColFlag;
};

extern HyroMotor* hyroMotor;

#endif /* HYROMOTOR_H_ */

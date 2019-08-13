///*
// * HyroMotor.h
// *
// *  Created on: 11.03.2019
// *      Author: Taras.Melnik
// */
//#ifndef HYROMOTOR_H_
//#define HYROMOTOR_H_
//
//#include "stm32f4xx.h"
//#include "stm32f4xx_conf.h"
//#include "semphr.h"
//#include "UARTtoRS485.h"
//#include "UARTuserInit.h"
//#include "arm_math.h"
//#include <math.h>
//#include "LEDStrip.h"
//
//
////***************************************
//
//#define STOP_MOTION		((int16_t) 0)
//#define ERROR_CURRENT 		((int8_t) 0xff)
//
////**************Robot_options*************
//
//#define W_CONST			((float32_t) 0.00164f)  // rad/ticksOfencoders
//#define L_CENTER		((float32_t) 0.25f)  	// distance between wheel and center (m)
//#define R_WHEEL			((float32_t) 0.0833f)  	// wheel radius (m)
//#define CONST_WHEEL_1 		((float32_t) -3.57f)	// (pointsOfwheel/rad)
//#define CONST_WHEEL_2 		((float32_t) 3.57f)	// (pointsOfwheel/rad)
//#define ROTATION_CONST		((float32_t) 0.1666f)  	// WheelRadius/CenterRadius
//
////**************Wheels_status*************
//#define WHEEL_OK		((uint8_t) 0x64)
//#define WHEEL_COLLISION		((uint8_t) 0x00)
//#define WHEEL_POWER_OFF		((uint8_t) 0xff)
//
//class MotorManager {
//public:
//	HyroMotor();
//	virtual ~HyroMotor();
//	void delayPort(uint32_t ticks);
//	void taskNotifyFromISR(BaseType_t xHigherPriorityTaskWoken);
//	void switchPin();
//	void robotSpeed2WheelSpeed(float* hDatArr, int16_t* whArr);
//	void calculateXYAlf(int32_t* whArr, int32_t* whHistArr, float32_t* robArr);
//	bool getWhCurColStatus();
//	bool getWhPowStatus();
//	void setSpeed(uint8_t* byteArr);
//	void getOdometry(uint8_t* byteArr);
//	void motorInit(uint8_t idid2set);
//	void run();
//	uint8_t* rxRsDataArr[2];
//private:
//	SemaphoreHandle_t xHighLvlMutex;
//	TaskHandle_t xTaskToNotify;
//	UARTtoRS485* motArr[2];
//	uint8_t rByteArr[8];
//	uint8_t tByteArr[12];
//	bool hlFlag;
//	bool txFlag;
//	bool rxFlag;
//	bool aknFlag;
//	bool powFlag;
//	bool curColFlag;
//};
//
//extern MotorManager* mot_manager;
//
//#endif /* HYROMOTOR_H_ */

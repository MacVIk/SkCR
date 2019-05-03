/*
 * SetMotors.cpp
 *
 *  Created on: 04.07.2018
 *      Author: Taras.Melnik
 */

#include "SetMotors.h"
#include "PeripheralDrivers/PWM.h"
#include "PeripheralDrivers/PortManager.h"
#include "Application/MovConvert.h"
//#include "USBcmndReceive.h"
//#include "PeripheralDrivers/USART.h"
//#include "Application/USARTreceiveHandler.h"
//#include "Application/USBcmndSend.h"

//#include "Application/RGB.h"

MovConvert* movConvert_set;// = new MovConvert();



PWM* pwmSpeedMot1;
PWM* pwmSpeedMot2;
PWM* pwmSpeedMot3;
PWM* pwmSpeedMot4;

GPIO_TypeDef* gpioDirArray[4] = { GPIOH, GPIOH, gpioDirMot3, gpioDirMot4 };
uint8_t pinDirArray[4] = { 0, 0, DirMot3, DirMot4 };

GPIO_TypeDef* gpioStartArray[4] = { GPIOH, GPIOH, gpioStartMot3, gpioStartMot4 };
uint8_t pinStartArray[4] = { 0, 0, StartMot3, StartMot4 };

GPIO_TypeDef* gpioVelArray[4] = { GPIOH, GPIOH, gpioVelInputMot3,
		gpioVelInputMot4 };
uint8_t pinVelArray[4] = { 0, 0, VelInputMot3, VelInputMot4 };

PWM* pwmArray[4] = { pwmSpeedMot1, pwmSpeedMot2, pwmSpeedMot3, pwmSpeedMot4 };

SetMotors::SetMotors() {
	PortManager::initPin(gpioDirMot3, DirMot3, GPIO_Mode_OUT);
	PortManager::initPin(gpioStartMot3, StartMot3, GPIO_Mode_OUT);

	PortManager::initPin(gpioDirMot4, DirMot4, GPIO_Mode_OUT);
	PortManager::initPin(gpioStartMot4, StartMot4, GPIO_Mode_OUT);



//inverse signals for proper work (power STM first, then controllers)
	PortManager::setPin(gpioStartMot3, StartMot3);
	PortManager::setPin(gpioStartMot4, StartMot4);

	PortManager::setPin(gpioDirMot3, DirMot3);
	PortManager::setPin(gpioDirMot4, DirMot4);

	PortManager::initPin(gpioOnOff, pinOnOff, GPIO_Mode_OUT);
}

SetMotors::~SetMotors() {
}

//extern unsigned char OWI_ComputeCRC8(uint8_t inData, uint8_t seed);

static uint8_t motorFlag[4] = { 0, 0, 0, 0 };
static int8_t motorDirPrev[4] = { 1, 1, 1, 1 };
//static int8_t motorVelPrev[4] = { 0, 0, 0, 0 };
//int8_t motorVel[4];

static float32_t motorVelPrev[4] = { 0, 0, 0, 0 };
float32_t motorVel[4];

void SetMotors::activeChange(GPIO_TypeDef* gpioStartStop, uint8_t StartStop) {

	//inverse signals for proper work (power STM first, then controllers)
//	PortManager::resetPin(gpioStartStop, StartStop);
//	this->taskDelay(oRTOS.fromMsToTick(300));
//	PortManager::setPin(gpioStartStop, StartStop);
//	this->taskDelay(oRTOS.fromMsToTick(300));
	PortManager::writePin(gpioStartStop, StartStop, 0);

}

void SetMotors::toogleDir(GPIO_TypeDef *gpioDir, uint8_t Dir) {

	//inverse signals for proper work (power STM first, then controllers)
	PortManager::resetPin(gpioDir, Dir);
	this->taskDelay(oRTOS.fromMsToTick(250));
	PortManager::setPin(gpioDir, Dir);
//	this->taskDelay(oRTOS.fromMsToTick(100));
}

int8_t SetMotors::sign(float value) {
	if (value == 0) {
		return 0;
	} else if (value < 0) {
		return -1;
	} else {
		return 1;
	}
}

void setDirCW(GPIO_TypeDef *gpioDir, uint8_t Dir) {
	PortManager::setPin(gpioDir, Dir);
}

void setDirCCW(GPIO_TypeDef *gpioDir, uint8_t Dir) { // That is the default direction of rotation
	PortManager::resetPin(gpioDir, Dir);
}

void speedChange(PWM* targetPWM, float32_t speedPercents) {
	targetPWM->setWidthPercents(speedPercents);
	targetPWM->update();
}

//void SetMotors::setSpeed(PWM* targetPWM, GPIO_TypeDef* gpioDir, uint8_t Dir,
//		GPIO_TypeDef* gpioStartStop, uint8_t StartStop, float32_t speed) {
//	static uint8_t flag = 0;
//
//	if ((speed == 0) && (flag == 1)) {
//		activeChange(gpioStartStop, StartStop);
//		flag = 0;
//		speedChange(targetPWM, 100); //-speed);
//	}
//	if (speed != 0) {
//		if (flag == 0) {
//			activeChange(gpioStartStop, StartStop);
//			flag = 1;
//		}
//		if (speed > 0) {
//			setDirCW(gpioDir, Dir);
//			speedChange(targetPWM, 100 - speed);
////			direction = 1;
//		}
//
//		if (speed < 0) {
//			setDirCCW(gpioDir, Dir);
//			speedChange(targetPWM, 100 + speed);
////			direction = 0;
//		}
//
//	}
//}

//void SetMotors::setMotors(int8_t motor1, int8_t motor2,
//							int8_t motor3, int8_t motor4) {
//	uint8_t i = 1;
//	USARTMessage umTX;
//
//	motorVel[0] = motor1;
//	motorVel[1] = motor2;
//	motorVel[2] = motor3;
//	motorVel[3] = motor4;
//


void SetMotors::setMotors(float32_t motor1, float32_t motor2, float32_t motor3,
		float32_t motor4) {
	uint8_t i = 1;

	motorVel[0] = motor1;
	motorVel[1] = motor2;
	motorVel[2] = motor3;
	motorVel[3] = motor4;

//	static uint8_t flag = 0;
//
//	if ((speed == 0) && (flag == 1)) {
//		activeChange(gpioStartStop, StartStop);
//		flag = 0;
//		speedChange(targetPWM, 100); //-speed);
//	}
//	if (speed != 0) {
//		if (flag == 0) {
//			activeChange(gpioStartStop, StartStop);
//			flag = 1;
//		}
//		if (speed > 0) {
//			setDirCW(gpioDir, Dir);
//			speedChange(targetPWM, 100 - speed);
////			direction = 1;
//		}
//
//		if (speed < 0) {
//			setDirCCW(gpioDir, Dir);
//			speedChange(targetPWM, 100 + speed);
////			direction = 0;
//		}
//
//	}

//	if (motorDirPrev[2] * this->sign(motorVel[2]) == -1) {
//		toogleDir(gpioDirArray[2], pinDirArray[2]);
//		motorDirPrev[2] *= -1;
//	}
//	if (motorDirPrev[3] * this->sign(motorVel[3]) == -1) {
//		toogleDir(gpioDirArray[3], pinDirArray[3]);
//		motorDirPrev[3] *= -1;
//	}

	if (motorVelPrev[2] != motorVel[2]) {

		motorVelPrev[2] = motorVel[2];

		//-----__STOP_MOUTION__-----//
		if (motorVel[2] == 0) {
			speedChange(pwmSpeedMot3, 100);
		}
		if (motorVel[2] > 0) {
//			setDirCW(gpioDirArray[2], pinDirArray[2]);
			PortManager::writePin(gpioDirMot3, DirMot3, 1);
			speedChange(pwmSpeedMot3, 100 - motorVel[2]);
		}
		if (motorVel[2] < 0) {
//			setDirCCW(gpioDirArray[2], pinDirArray[2]);
			PortManager::writePin(gpioDirMot3, DirMot3, 0);
			speedChange(pwmSpeedMot3, 100 + motorVel[2]);
		}
	}

	if (motorVelPrev[3] != motorVel[3]) {

		motorVelPrev[3] = motorVel[3];

		//-----__STOP_MOUTION__-----//
		if (motorVel[3] == 0) {
			speedChange(pwmSpeedMot4, 100);
		}
		if (motorVel[3] > 0) {
//			setDirCW(gpioDirArray[3], pinDirArray[3]);
			PortManager::writePin(gpioDirMot4, DirMot4, 1);
			speedChange(pwmSpeedMot4, 100 - motorVel[3]);
		}
		if (motorVel[3] < 0) {
//			setDirCCW(gpioDirArray[3], pinDirArray[3]);
			PortManager::writePin(gpioDirMot4, DirMot4, 0);
			speedChange(pwmSpeedMot4, 100 + motorVel[3]);
		}
	}

}

void SetMotors::run() {
//	uint8_t motor3, motor4 = 0;
//	uint8_t motor3_prev, motor4_prev = 0;
//	float32_t motor3 = 0;
//	float32_t motor4 = 0;
	float32_t motor3_prev = 0;
	float32_t motor4_prev = 0;

	float32_t wheelSpeed[2];
	float32_t robotSpeed[2];

	uint8_t constr = 60; //limit on max PWM on wheels.

	movConvert_set = new MovConvert();

////	pwmArray[3] = new PWM(gpioVelInputMot3, VelInputMot3, TIM8, 3);
////	pwmArray[4] = new PWM(gpioVelInputMot4, VelInputMot4, TIM8, 4);
//	pwmSpeedMot3 = new PWM(gpioVelInputMot3, VelInputMot3, TIM8, 3);
//	pwmSpeedMot4 = new PWM(gpioVelInputMot4, VelInputMot4, TIM8, 4);
//
////	for (int i = 3; i <= 4; i++) {
////		pwmArray[i-1]->init(60000, 20000);
////		pwmArray[i-1]->start();
////	}
//	pwmSpeedMot3->init(60000, 20000);
//	pwmSpeedMot4->init(60000, 20000);
//	pwmSpeedMot3->start();
//	pwmSpeedMot4->start();

//	setMotors(0, 0, 0, -50);

	pwmSpeedMot3 = new PWM(gpioVelInputMot3, VelInputMot3, TIM9, 1);
	pwmSpeedMot4 = new PWM(gpioVelInputMot4, VelInputMot4, TIM9, 2);
	pwmSpeedMot3->init(600, 200);
	pwmSpeedMot4->init(600, 200);
	pwmSpeedMot3->start();
	pwmSpeedMot4->start();

	activeChange(gpioStartArray[2], pinStartArray[2]);
	speedChange(pwmSpeedMot3, 100);
	activeChange(gpioStartArray[3], pinStartArray[3]);
	speedChange(pwmSpeedMot4, 100);

//	for (int i = 3; i <= 4; i++) {
//		activeChange(gpioStartArray[i - 1], pinStartArray[i - 1]);
//	}

	this->taskDelay(20);
	PortManager::setPin(gpioOnOff, pinOnOff);

//	this->taskDelay(oRTOS.fromMsToTick(1000));
	activeChange(gpioStartArray[2], pinStartArray[2]);
	activeChange(gpioStartArray[3], pinStartArray[3]);


	while (true) {

		robotSpeed[0] = this->v;
		robotSpeed[1] = this->w;
		movConvert_set->robotSpeed_2_wheelSpeed(wheelSpeed,robotSpeed,movConvert_set->R,movConvert_set->L);

		wheelSpeed[0] = wheelSpeed[0] * movConvert_set->k_PWM;
		wheelSpeed[1] = wheelSpeed[1] * movConvert_set->k_PWM;

		if (wheelSpeed[0] > constr) {
			wheelSpeed[0] = constr;
		}
		if (wheelSpeed[0] < -constr) {
			wheelSpeed[0] = -constr;
		}
		if (wheelSpeed[1] > constr) {
			wheelSpeed[1] = constr;
		}
		if (wheelSpeed[1] < -constr) {
			wheelSpeed[1] = -constr;
		}

		if ((motor3_prev != wheelSpeed[0]) || (motor4_prev != wheelSpeed[1])) {
			setMotors(0, 0, wheelSpeed[0], -wheelSpeed[1]);
			motor3_prev = wheelSpeed[0];
			motor4_prev = wheelSpeed[1];
		}

		//------------------------------------
		if ((wheelSpeed[0] == 0) && (wheelSpeed[1] == 0)) {
			this->statusLed.setRed(0);
			this->statusLed.setGreen(0);
			this->statusLed.setBlue(100);
		} else {
			this->statusLed.setRed(80);
			this->statusLed.setGreen(0);
			this->statusLed.setBlue(80);
		}

		this->taskDelay(oRTOS.fromMsToTick(1));
	}
}


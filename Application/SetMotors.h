/*
 * SetMotors.h
 *
 *  Created on: 04.07.2018
 *      Author: Taras.Melnik
 */

#ifndef SETMOTORS_H_
#define SETMOTORS_H_

#include "FreeRtos/wrapper/iActiveObject.h"
#include "FreeRtos/include/semphr.h"
#include "PeripheralDrivers/PortManager.h"
#include "PeripheralDrivers/PWM.h"

#include "Application/RGB.h"

#define gpioOnOff GPIOC
#define pinOnOff 14

//motor3:
#define DirMot3 3
#define gpioDirMot3 GPIOE
#define StartMot3 1
#define gpioStartMot3 GPIOE
#define VelInputMot3 5
#define gpioVelInputMot3 GPIOE

//motor4:
#define DirMot4 4
#define gpioDirMot4 GPIOE
#define StartMot4 2
#define gpioStartMot4 GPIOE
#define VelInputMot4 6
#define gpioVelInputMot4 GPIOE

class SetMotors: public iActiveObject {
public:

	RGB statusLed;
	SetMotors();
	virtual ~SetMotors();

	void run();
	void activeChange(GPIO_TypeDef *gpioStartStop, uint8_t StartStop);

	void setMotors(float32_t motor1, float32_t motor2,
			float32_t motor3, float32_t motor4);
	void toogleDir(GPIO_TypeDef *gpioDir, uint8_t Dir);
	int8_t sign(float value);

	float v = 0;
	float w = 0;
};

#endif /* SETMOTORS_H_ */

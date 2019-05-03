/*
 * MovementControl.h
 *
 *  Created on: 06.02.2019
 *      Author: Taras.Melnik
 */

#ifndef MOVEMENTCONTROL_H_
#define MOVEMENTCONTROL_H_

#include "iActiveObject.h"
#include "stm32f4xx_gpio.h"
//#include "InitialisationList.h"
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "semphr.h"
#include "queue.h"
#include "ReadEncoders.h"

#define VELOCITY_ERROR_DIAPASON 	((float) 0.5f)

#define START_MOTOR_1 	GPIO_Pin_2
#define DIR_MOTOR_1 	GPIO_Pin_4
#define START_MOTOR_2 	GPIO_Pin_1
#define DIR_MOTOR_2 	GPIO_Pin_3
#define MOTOR_RELEY		GPIO_Pin_14

class MovementControl: public iActiveObject {
public:
	MovementControl();
	virtual ~MovementControl();

	void writePIDkoef(uint8_t p, uint8_t i, uint8_t d);
	void writeToQueue(uint8_t &data);
	void readFromQueue(uint8_t &data);

	void run();

private:
	float32_t kp, kd, ki;

};

#endif /* MOVEMENTCONTROL_H_ */

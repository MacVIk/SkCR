/*
 * MovConvert.h
 *
 *  Created on: 14.07.2018
 *      Author: Fazli
 */

#ifndef MOVCONVERT_H_
#define MOVCONVERT_H_

#include "Math.h"
#include "FreeRtos/wrapper/iActiveObject.h"

class MovConvert {
public:
	MovConvert();
	virtual ~MovConvert();

//	float32_t R = 0.05; // радиус колеса
//	float32_t L = 0.662; // расстояние между центрами колес
//
//	float32_t R_enc = 0.0635; // радиус колеса энкодера
//	float32_t L_enc = 0.1539; // расстояние между центрами колес энкодера
//
//	float32_t k_robot_speed2wheel_speed = 15.6559;
//	float32_t k_wheel_speed2robot_speed = 0.813;

	float32_t R; // радиус колеса
	float32_t L; // расстояние между центрами колес

	float32_t R_enc; // радиус колеса энкодера
	float32_t L_enc; // расстояние между центрами колес энкодера

	float32_t k_PWM;
	float32_t k_wheel_speed2robot_speed;

	float32_t rps2rad_per_sec(float32_t v);
	void robotSpeed_2_wheelSpeed(float32_t* wheelSpeed, float32_t* robotSpeed,
			float32_t R, float32_t L);
	void wheelSpeed_2_robotSpeed(float32_t* robotSpeed, float32_t* wheelSpeed,
			float32_t R, float32_t L);
	void encoderSpeed_2_wheelSpeed(float32_t* wheelSpeed, float32_t* encoderSpeed);
};

#endif /* MOVCONVERT_H_ */

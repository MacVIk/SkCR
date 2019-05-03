/*
 * MovConvert.cpp
 *
 *  Created on: 14.07.2018
 *      Author: Fazli
 */

#include "MovConvert.h"

MovConvert::MovConvert() {
	this->R = 0.05; // радиус колеса
	this->L = 0.662; // расстояние между центрами колес

	this->R_enc = 0.055; // радиус колеса энкодера
	this->L_enc = 0.662; // расстояние между центрами колес энкодера

	this->k_PWM = 2.4;
	this->k_wheel_speed2robot_speed = 0.138;
}

MovConvert::~MovConvert() {
	// TODO Auto-generated destructor stub
}

float32_t MovConvert::rps2rad_per_sec(float32_t v) {
	return v * 2 * M_PI;
}

void MovConvert::robotSpeed_2_wheelSpeed(float32_t* wheelSpeed,
		float32_t* robotSpeed, float32_t R, float32_t L) {

	//robotSpeed[0] -> v; robotSpeed[1] -> w;
	wheelSpeed[0] = (2 * robotSpeed[0] - robotSpeed[1] * L) / (2. * R);
	wheelSpeed[1] = (2 * robotSpeed[0] + robotSpeed[1] * L) / (2. * R);
//* k_robot_speed2wheel_speed # koeff here, probably, converts to PWM
}

void MovConvert::wheelSpeed_2_robotSpeed(float32_t* robotSpeed,
		float32_t* wheelSpeed, float32_t R, float32_t L) {

	//wheelSpeed[0] -> v_left; wheelSpeed[1] -> v_right
	robotSpeed[0] = (wheelSpeed[1] + wheelSpeed[0]) * R / 2.0 * k_wheel_speed2robot_speed;
	robotSpeed[1] = (wheelSpeed[1] - wheelSpeed[0]) * R / L * k_wheel_speed2robot_speed;
//* k_wheel_speed2robot_speed # koeff here, probably, converts from ticks to rad/s
}

void MovConvert::encoderSpeed_2_wheelSpeed(float32_t* wheelSpeed,
		float32_t* encoderSpeed) {

	//encoderSpeed[] -> v_left_enc; encoderSpeed[1] -> v_right_enc
	float32_t encoderSpeedInRadPerSec[2];
	encoderSpeedInRadPerSec[0] = rps2rad_per_sec(encoderSpeed[0]);
	encoderSpeedInRadPerSec[1] = rps2rad_per_sec(encoderSpeed[1]);

	float32_t robotSpeed[2];
	wheelSpeed_2_robotSpeed(robotSpeed, encoderSpeedInRadPerSec, R_enc, L_enc);
	robotSpeed_2_wheelSpeed(wheelSpeed, robotSpeed, R, L);
}


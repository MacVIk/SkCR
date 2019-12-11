/*
 * HyroMotor.h
 *
 *  Created on: 11.03.2019
 *      Author: Taras.Melnik
 */
#ifndef CODE_MOTOTRMANAGER_H_
#define CODE_MOTOTRMANAGER_H_

#include "stm32f4xx.h"
#include "TaskWrapper.h"

//***************************************

#define STOP_MOTION		((int16_t) 0)
#define ERROR_CURRENT 		((int8_t) 0xff)
#define MAX_ROBOT_SPEED         ((float) 0.5)
#define CURRENT_COLLISION_VALUE ((int16_t) 2000)
#define TIME_FOR_RESPONCE       ((uint64_t) 5)

//**************Robot_options*************

#define W_CONST			((float32_t) 0.00164f)  // rad/ticksOfencoders
#define L_CENTER		((float32_t) 0.25f)  	// distance between wheel and center (m)
#define R_WHEEL			((float32_t) 0.0833f)  	// wheel radius (m)
#define CONST_WHEEL_1 		((float32_t) -3.57f)	// (pointsOfwheel/rad)
#define CONST_WHEEL_2 		((float32_t) 3.57f)	// (pointsOfwheel/rad)
#define ROTATION_CONST		((float32_t) 0.1666f)  	// WheelRadius/CenterRadius

#define PAUSE_AFTER_POWER_ON    ((uint16_t) 3000)       // Need for drivers launch (mc)

class MotorManager: public TaskWrapper {
public:
    /* Terminal interface */
    void set_robot_speed(uint8_t* byteArr);
    void get_robot_position(uint8_t* byteArr);
    bool get_robot_button_status();
    bool get_robot_current_status();
    /* Mandatory task method */
    void run();

private:
    /* Robot parameters */
    struct type_robot {
        float linear_velocity;
        float angular_velocity;
        float x;
        float y;
        float theta;
        uint8_t code;
        bool button_status;
        bool current_status;
        bool error_status;
    } robot;

    /* Working methods */
    void set_robot_speed();
    void set_robot_speed(float lin_vel, float ang_vel);
    void process_angle();
    void process_current();
    void process_speed();
    void process_motor_error();

    /* Terminal communication parameters */
    //ToDO check mutex in FreeRtos
    uint8_t position_byte_arr[12];
    uint8_t speed_byte_arr[8];
    bool terminalRxFlag;

    /* Internal math transforms */
    inline void fetch_angle();
    void convert_robot_to_wheel_speed(float* hDatArr, int16_t* whArr);
    void convert_wheel_to_robot_speed();
    void calculate_position(int32_t* wheel_arr);

    /* Internal interface */
    void process_robot_error();
};

extern MotorManager mot_manager;

#endif /* CODE_MOTOTRMANAGER_H_ */

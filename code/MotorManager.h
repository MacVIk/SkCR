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

enum MotorErrorStatus {
        OK,
        MOTOR_1_DONOT_ANSWER,
        MOTOR_2_DONOT_ANSWER,
        MOTORS_DONOT_ANSWER,
};

/* Requests for motors */
typedef enum Request {
        SET_SPEED,
        GET_POSITION,
        GET_CURRENT_STATUS,
        GET_ERROR_STATUS,

        /* This macros should be the last */
        END_POSITION
} type_motor_request;

class MotorManager: public TaskWrapper {
public:
	/* Terminal interface */
	void set_robot_speed(uint8_t* byteArr);
	void get_robot_position(uint8_t* byteArr);

	/* Interrupt method */
        void switch_to_receive();


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
                bool button_status;
                bool current_status;
        } robot;


        /* Working methods */
        void set_robot_speed();
        void set_robot_speed(float lin_vel, float ang_vel);
        void process_angle();
        void process_current();
        void process_error();

        /* Terminal communication parameters */
        //ToDO check mutex in FreeRtos
        uint8_t position_byte_arr[12];
        uint8_t speed_byte_arr[8];
        bool terminalRxFlag;
        bool firstTransmitedFlag;

	/* Internal interface */
        MotorErrorStatus read_motors_aknowlege(uint8_t &code);
	void send_next_request(uint8_t current_code);

        /* Internal math transforms */
	void fetch_angle();
        void convert_robot_to_wheel_speed(float* hDatArr, int16_t* whArr);
        void calculate_position();




};

extern MotorManager mot_manager;

#endif /* CODE_MOTOTRMANAGER_H_ */

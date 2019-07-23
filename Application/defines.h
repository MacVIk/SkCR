/*
 * defines.h
 *
 *  Created on: 01.11.2018
 *      Author: Taras.Melnik
 */

#ifndef DEFINES_H_
#define DEFINES_H_

//#define CMD_DEF(cmd_num, cmd_name) \
//					cmd_name = (cmd_num),

//enum
//{
//	FLAG_NULL,
////#include "cmd_defs"
//	FLAG_END
//};


enum {
//	FLAG_NULL,
	ECHO 							= 0x01,
	SET_SPEED_ON_MOTORS 			= 0x02,
	GET_SPEED_FROM_MOTORS 			= 0x03,
	CLEAR_COLLISION_STATUS			= 0x04,
	MAX_COMMAND_LENGTH 				= 0x05,

	SET_COLOR_NUMBER				= 0x07,
	GET_BATTERY_CHARGE				= 0x08,
	SEND_RS485						= 0x09,
	GET_DISTANCE					= 0x0A,
	RECEIVE_RS485					= 0x0B,
	RECEIVE_IMU						= 0x0C,
	RECEIVE_ACCELERATION			= 0x0D,
	RECEIVE_ANGULAR_VELOCITY		= 0x0E
//	FLAG
};

#endif /* DEFINES_H_ */

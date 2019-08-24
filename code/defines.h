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


enum HighLvlCommand {
//	FLAG_NULL,
	ECHO 			= 0x01,

	SET_COLOR       	= 0x07,
	GET_BATTERY_CHARGE	= 0x08,
	SET_ROBOT_SPEED		= 0x09,
	GET_DISTANCE		= 0x0A,
	GET_WHEELS_ANGLE	= 0x0B,
	RECEIVE_IMU		= 0x0C
//	FLAG
};

#endif /* DEFINES_H_ */

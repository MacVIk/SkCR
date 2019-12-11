/*
 * defines.h
 *
 *  Created on: 01.11.2018
 *      Author: Taras.Melnik
 */

#ifndef DEFINES_H_
#define DEFINES_H_

enum HighLvlCommand {
//	FLAG_NULL,
	ECHO 			    = 0x01,

	SET_COLOR       	= 0x07,
	GET_BATTERY_CHARGE	= 0x08,
	SET_ROBOT_SPEED		= 0x09,
	GET_DISTANCE		= 0x0A,
	GET_ROBOT_POSITION  = 0x0B,
	GET_ACCELERATION	= 0x0C,
	GET_ANG_VEL         = 0x0D
//	FLAG
};

#endif /* DEFINES_H_ */

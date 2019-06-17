/*
 * ImuSensor.h
 *
 *  Created on: 10 θών. 2019 γ.
 *      Author: Taras.Melnik
 */

#ifndef APPLICATION_IMUSENSOR_H_
#define APPLICATION_IMUSENSOR_H_

#include "iActiveObject.h"

#define ACCELEROMETER_ADRR 		((uint8_t) 0b00110000)
#define ACCELEROMETER_OUT_X_L 	((uint8_t) 0x28)
#define ACCELEROMETER_CTRL_REG1	((uint8_t) 0x20)

class ImuSensor: public iActiveObject {
public:
	ImuSensor();
	virtual ~ImuSensor();
	void i2cInit();
	void getOdometry(uint8_t* byteArr);
	void i2cRead(uint8_t slaveAdr, uint8_t subRegAdr, uint8_t regNumb, uint8_t* data);
	void i2cWrite(uint8_t slaveAdr, uint8_t subRegAdr, uint8_t data);
	void accelInit();
	int16_t uint8toInt16(uint8_t* arr);
	void run();

private:
	uint8_t i2cRxArr[20]; 		// Buffer for data from I2C;
	uint8_t tByteArr[12];		// Buffer for (x, y, theta) transmitting.
};

extern ImuSensor* imuSensor;

#endif /* APPLICATION_IMUSENSOR_H_ */

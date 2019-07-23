/*
 * ImuSensor.h
 *
 *  Created on: 10 θών. 2019 γ.
 *      Author: Taras.Melnik
 */

#ifndef APPLICATION_IMUSENSOR_H_
#define APPLICATION_IMUSENSOR_H_

#include "iActiveObject.h"
#include "arm_math.h"
#include <math.h>

#define ACCELEROMETER_ADDR 				((uint8_t) 0b00110000)
#define ACCELEROMETER_CTRL_REG1_ADDR	((uint8_t) 0x20)
#define ACCELEROMETER_CTRL_REG1_MASK	((uint8_t) 0b00101111)
#define ACCELEROMETER_OUT_X_L 			((uint8_t) 0x28)
#define ACCELEROMETER_OUT_Y_L			((uint8_t) 0x2A)

#define GYROSCOPE_ADDR 					((uint8_t) 0b11010000)
#define GYROSCOPE_CTRL_REG1_ADDR		((uint8_t) 0x20)
#define GYROSCOPE_CTRL_REG1_MASK		((uint8_t) 0b01101111)
#define GYROSCOPE_OUT_X_L 				((uint8_t) 0x28)
#define GYROSCOPE_OUT_Z_L				((uint8_t) 0x2C)

#define MAGNETOMETER_ADDR 				((uint8_t) 0b00111001)
#define MAGNETOMETER_CTRL_REG1_ADDR		((uint8_t) 0x20)
#define MAGNETOMETER_CTRL_REG1_MASK		((uint8_t) 0b01011100)
#define MAGNETOMETER_CTRL_REG2_ADDR		((uint8_t) 0x21)
#define MAGNETOMETER_CTRL_REG2_MASK		((uint8_t) 0b00000000)
#define MAGNETOMETER_CTRL_REG3_ADDR		((uint8_t) 0x22)
#define MAGNETOMETER_CTRL_REG3_MASK		((uint8_t) 0b00000000)
#define MAGNETOMETER_OUT_X_L 			((uint8_t) 0x28)
#define MAGNETOMETER_OUT_Y_L			((uint8_t) 0x2A)

// ------------------Filtering
#define FILTER_K_COFFICIENT				((float32_t) 0.1)

class ImuSensor: public iActiveObject {
public:
	ImuSensor();
	virtual ~ImuSensor();
	void i2cInit();
	void i2cReset();
	void getOdometry(uint8_t* byteArr);
	void getAngularVel(uint8_t* byteArr);
	void getAcceleration(uint8_t* byteArr);
	void i2cRead(uint8_t slaveAdr, uint8_t subRegAdr, uint8_t regNumb, uint8_t* data);
	void i2cWrite(uint8_t slaveAdr, uint8_t subRegAdr, uint8_t data);
	void accelInit();
	void gyroInit();
	void magnetInit();
	void accelAdjustment(int16_t* aConst);
	void gyroAdjustment(int16_t* aConst);
	void calculateXYTheta(float32_t* vArr, float32_t* xyalfArr);
	float32_t runningMean(float32_t* arr);
	void run();

private:
	uint8_t i2cRxArr[20]; 		// Buffer for data from I2C;
	uint8_t tByteArr[12];		// Buffer for (x, y, theta) transmitting.
	uint8_t accArr[24];
	uint8_t gyroArr[24];
};

extern ImuSensor* imuSensor;

#endif /* APPLICATION_IMUSENSOR_H_ */

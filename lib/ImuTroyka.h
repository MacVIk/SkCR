/*
 * ImuTroyka.h
 *
 *  Created on: 11 сент. 2019 г.
 *      Author: Taras.Melnik
 */

#ifndef LIB_IMUTROYKA_H_
#define LIB_IMUTROYKA_H_

#include "DriverI2C.h"

#define ACCELEROMETER_ADDR              ((uint8_t) 0b00110000)
#define ACCELEROMETER_CTRL_REG1_ADDR    ((uint8_t) 0x20)
#define ACCELEROMETER_CTRL_REG1_MASK    ((uint8_t) 0b00101111)
#define ACCELEROMETER_OUT_X_L           ((uint8_t) 0x28)
#define ACCELEROMETER_OUT_Y_L           ((uint8_t) 0x2A)

#define GYROSCOPE_ADDR                  ((uint8_t) 0b11010000)
#define GYROSCOPE_CTRL_REG1_ADDR        ((uint8_t) 0x20)
#define GYROSCOPE_CTRL_REG1_MASK        ((uint8_t) 0b01101111)
#define GYROSCOPE_OUT_X_L               ((uint8_t) 0x28)
#define GYROSCOPE_OUT_Z_L               ((uint8_t) 0x2C)

#define MAGNETOMETER_ADDR               ((uint8_t) 0b00111001)
#define MAGNETOMETER_CTRL_REG1_ADDR     ((uint8_t) 0x20)
#define MAGNETOMETER_CTRL_REG1_MASK     ((uint8_t) 0b01011100)
#define MAGNETOMETER_CTRL_REG2_ADDR     ((uint8_t) 0x21)
#define MAGNETOMETER_CTRL_REG2_MASK     ((uint8_t) 0b00000000)
#define MAGNETOMETER_CTRL_REG3_ADDR     ((uint8_t) 0x22)
#define MAGNETOMETER_CTRL_REG3_MASK     ((uint8_t) 0b00000000)

class ImuTroyka: public DriverI2C {
public:
    void init_imu();
    void read_accelerations(float* acc_arr);
    void read_angular_velocity(float* gyr_arr);
    void read_magnet_field(float* gyr_arr);
private:
    void init_accelerometer();
    void init_gyroscope();
    void init_magnetometer();
};

#endif /* LIB_IMUTROYKA_H_ */

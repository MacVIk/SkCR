/*
 * I2c1Manager.h
 *
 *  Created on: 10 сент. 2019 г.
 *      Author: Taras.Melnik
 */

#ifndef CODE_I2C1MANAGER_H_
#define CODE_I2C1MANAGER_H_

#include "TaskWrapper.h"

#include "DriverI2C.h"

#define ACCELEROMETER_ADDR              ((uint8_t) 0b00110000)
#define ACCELEROMETER_                  ((uint8_t) 0b00110000)
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

struct ImuSensor {
        float acceleration [3];
        float angular_velocity [3];
        float magnet_field[3];
};

class I2c1Manager: public TaskWrapper {
public:
        I2c1Manager();
        virtual ~I2c1Manager();

        void get_acceleration(uint8_t* acc_arr);
        void get_angular_velocity(uint8_t* ang_vel_arr);
        void run();

private:
        void init_accelerometer();
        void init_gyroscope();
        void init_magnetometer();

        ImuSensor imu;
        uint8_t i2c_byte_arr[16];
        uint8_t hl_acc_byte_arr[12];
        uint8_t hl_gyr_byte_arr[12];
        uint8_t hl_magnet_byte_arr[12];
};

extern I2c1Manager i2c_manager;
#endif /* CODE_I2C1MANAGER_H_ */

/*
 * ImuTroyka.cpp
 *
 *  Created on: 11 сент. 2019 г.
 *      Author: Taras.Melnik
 */

#include "ImuTroyka.h"

#include <string.h>

static constexpr float const_acc = 2.f * 9.8f / 32767.f;
static constexpr float const_gyro = 0.0875f / 57.3f;

void ImuTroyka::init_imu()
{
        init_accelerometer();
        init_gyroscope();
        init_magnetometer();
}

void ImuTroyka::init_accelerometer()
{
        write_i2c(ACCELEROMETER_ADDR, ACCELEROMETER_CTRL_REG1_ADDR,
                        ACCELEROMETER_CTRL_REG1_MASK);
}

void ImuTroyka::init_gyroscope()
{
        write_i2c(GYROSCOPE_ADDR, GYROSCOPE_CTRL_REG1_ADDR,
                        GYROSCOPE_CTRL_REG1_MASK);
}

void ImuTroyka::init_magnetometer()
{
        write_i2c(MAGNETOMETER_ADDR, MAGNETOMETER_CTRL_REG1_ADDR,
                        MAGNETOMETER_CTRL_REG1_MASK);
        write_i2c(MAGNETOMETER_ADDR, MAGNETOMETER_CTRL_REG2_ADDR,
                        MAGNETOMETER_CTRL_REG2_MASK);
        write_i2c(MAGNETOMETER_ADDR, MAGNETOMETER_CTRL_REG3_ADDR,
                        MAGNETOMETER_CTRL_REG3_MASK);
}

void ImuTroyka::read_accelerations(float* acc_arr)
{
        int16_t buff_arr[3];
        uint8_t i2c_byte_arr[6];

        read_i2c(ACCELEROMETER_ADDR, ACCELEROMETER_OUT_X_L | 1 << 7, 6, i2c_byte_arr);
        memcpy(buff_arr, i2c_byte_arr, sizeof(buff_arr));
        for (uint8_t i = 0; i < 3; ++i)
                acc_arr[i] = buff_arr[i] * const_acc;
}

void ImuTroyka::read_angular_velocity(float* gyr_arr)
{
        int16_t buff_arr[3];
        uint8_t i2c_byte_arr[6];

        read_i2c(GYROSCOPE_ADDR, GYROSCOPE_OUT_X_L | 1 << 7, 6, i2c_byte_arr);
        memcpy(buff_arr, i2c_byte_arr, sizeof(buff_arr));
        for (uint8_t i = 0; i < 3; ++i)
                gyr_arr[i] = buff_arr[i] * const_gyro;
}

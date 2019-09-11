/*
 * I2c1Manager.cpp
 *
 *  Created on: 10 сент. 2019 г.
 *      Author: Taras.Melnik
 */

#include "I2c1Manager.h"

#include <string.h>

I2c1Manager i2c_manager;
static DriverI2C i2c_1;

I2c1Manager::I2c1Manager() {
        // TODO Auto-generated constructor stub

}

I2c1Manager::~I2c1Manager() {
        // TODO Auto-generated destructor stub
}

void I2c1Manager::get_acceleration(uint8_t* acc_arr)
{
        for (uint8_t i; i < 12; ++i)
                acc_arr[i] = hl_acc_byte_arr[i];
}

void I2c1Manager::get_angular_velocity(uint8_t* ang_vel_arr)
{
        for (uint8_t i; i < 12; ++i)
                ang_vel_arr[i] = hl_gyr_byte_arr[i];

}

void I2c1Manager::init_accelerometer()
{
        i2c_1.write_i2c(ACCELEROMETER_ADDR, ACCELEROMETER_CTRL_REG1_ADDR,
                        ACCELEROMETER_CTRL_REG1_MASK);
}

void I2c1Manager::init_gyroscope()
{
        i2c_1.write_i2c(GYROSCOPE_ADDR, GYROSCOPE_CTRL_REG1_ADDR,
                        GYROSCOPE_CTRL_REG1_MASK);
}

void I2c1Manager::init_magnetometer()
{
        i2c_1.write_i2c(MAGNETOMETER_ADDR, MAGNETOMETER_CTRL_REG1_ADDR,
                        MAGNETOMETER_CTRL_REG1_MASK);
        i2c_1.write_i2c(MAGNETOMETER_ADDR, MAGNETOMETER_CTRL_REG2_ADDR,
                        MAGNETOMETER_CTRL_REG2_MASK);
        i2c_1.write_i2c(MAGNETOMETER_ADDR, MAGNETOMETER_CTRL_REG3_ADDR,
                        MAGNETOMETER_CTRL_REG3_MASK);
}

void I2c1Manager::run()
{
        i2c_1.init_i2c();
        init_accelerometer();
        init_gyroscope();
        init_magnetometer();

//        TickType_t pxPreviousWakeTime;

        static constexpr float const_acc = 2.f * 9.8f / 32767.f;
        static constexpr float const_gyro = 0.0875f / 57.3f;

        int16_t buff_arr[3];

        while(1) {
//                pxPreviousWakeTime = pxPreviousWakeTime = xTaskGetTickCount();

                i2c_1.read_i2c(ACCELEROMETER_ADDR, ACCELEROMETER_OUT_X_L | 1 << 7, 6, i2c_byte_arr);
                memcpy(buff_arr, i2c_byte_arr, sizeof(buff_arr));
                for (uint8_t i = 0; i < 3; ++i)
                        imu.acceleration[i] = buff_arr[i] * const_acc * 0.001f;
                memcpy(hl_acc_byte_arr, imu.acceleration, sizeof(hl_acc_byte_arr));

                i2c_1.read_i2c(GYROSCOPE_ADDR, GYROSCOPE_OUT_X_L | 1 << 7, 6, i2c_byte_arr);
                memcpy(buff_arr, i2c_byte_arr, sizeof(buff_arr));
                for (uint8_t i = 0; i < 3; ++i)
                        imu.angular_velocity[i] = buff_arr[i] * const_gyro * 0.001f;
                memcpy(hl_gyr_byte_arr, imu.angular_velocity, sizeof(hl_gyr_byte_arr));

//                vTaskDelayUntil(&pxPreviousWakeTime, 10);
                vTaskDelay(10);
        }
}

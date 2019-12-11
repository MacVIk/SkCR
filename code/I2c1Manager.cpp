/*
 * I2c1Manager.cpp
 *
 *  Created on: 10 сент. 2019 г.
 *      Author: Taras.Melnik
 */

#include "I2c1Manager.h"
#include "DriverI2C.h"
#include "ImuTroyka.h"

#include <string.h>

I2c1Manager i2c_manager;
static DriverI2C i2c_1;
static ImuTroyka imu;

I2c1Manager::I2c1Manager() {
    // TODO Auto-generated constructor stub

}

I2c1Manager::~I2c1Manager() {
    // TODO Auto-generated destructor stub
}

void I2c1Manager::get_acceleration(uint8_t* acc_arr)
{
    for (uint8_t i = 0; i < 12; ++i)
        acc_arr[i] = hl_acc_byte_arr[i];
}

void I2c1Manager::get_angular_velocity(uint8_t* ang_vel_arr)
{
    for (uint8_t i = 0; i < 12; ++i)
        ang_vel_arr[i] = hl_gyr_byte_arr[i];
}

void I2c1Manager::run()
{
    i2c_1.init_i2c();
    imu.init_imu();
//        TickType_t pxPreviousWakeTime;

    int16_t buff_arr[3];

    while(1) {
//                pxPreviousWakeTime = pxPreviousWakeTime = xTaskGetTickCount();
        imu.read_accelerations(robot.acceleration);
        memcpy(hl_acc_byte_arr, robot.acceleration, sizeof(hl_acc_byte_arr));

        imu.read_angular_velocity(robot.angular_velocity);
        memcpy(hl_gyr_byte_arr, robot.angular_velocity, sizeof(hl_gyr_byte_arr));

//                vTaskDelayUntil(&pxPreviousWakeTime, 10);
        vTaskDelay(10);
    }
}

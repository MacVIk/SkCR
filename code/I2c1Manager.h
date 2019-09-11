/*
 * I2c1Manager.h
 *
 *  Created on: 10 сент. 2019 г.
 *      Author: Taras.Melnik
 */

#ifndef CODE_I2C1MANAGER_H_
#define CODE_I2C1MANAGER_H_

#include "TaskWrapper.h"

struct RobotParameters {
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
        RobotParameters robot;
        uint8_t hl_acc_byte_arr[12];
        uint8_t hl_gyr_byte_arr[12];
        uint8_t hl_magnet_byte_arr[12];
};

extern I2c1Manager i2c_manager;
#endif /* CODE_I2C1MANAGER_H_ */

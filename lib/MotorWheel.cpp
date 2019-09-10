/*
 * MotorWheel.cpp
 *
 *  Created on: 13 рту. 2019 у.
 *      Author: Taras.Melnik
 */
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "MotorWheel.h"

MotorWheel::MotorWheel(uint8_t id) {
        this->id = id;
}

MotorWheel::~MotorWheel(){}

bool MotorWheel::init_wheel()
{
        bool status_init {true};
        write_single_register(id, REG_ERROR_CODE, 0);
        status_init = wait_acknowlege();
        write_single_register(id, REG_MODE, MODE_SPEED);
        status_init = wait_acknowlege();
        write_single_register(id, REG_S_PID_P, 1);
        status_init = wait_acknowlege();
        write_single_register(id, REG_S_PID_I, 200);
        status_init = wait_acknowlege();
        write_single_register(id, REG_S_PID_D, 0);
        status_init = wait_acknowlege();
        write_single_register(id, REG_I_CUTOFF, 3000);
        status_init = wait_acknowlege();
        write_single_register(id, REG_V_CUTOFF, 10000);
        status_init = wait_acknowlege();
        write_single_register(id, REG_T_CUTOFF, 100);
        status_init = wait_acknowlege();
        write_single_register(id, REG_PWM_LIMIT, 230);
        status_init = wait_acknowlege();
        write_single_register(id, REG_S_PID_I_LIMIT, 2000);
        status_init = wait_acknowlege();
        write_single_register(id, REG_A_PID_I_LIMIT, 2000);
        status_init = wait_acknowlege();
        write_single_register(id, REG_ERROR_CODE, 0);
        status_init = wait_acknowlege();
        this->request_angle();
        vTaskDelay(3);

        return status_init;
}

bool MotorWheel::wait_acknowlege()
{
       vTaskDelay(3);
       read_response();
       if (this->options.last_code)
               return true;
       else
               return false;
}

bool MotorWheel::read_response()
{
        uint8_t cap {0};
        int16_t buf_arr[8] = {};
        uint8_t arr_size {0};
        int32_t ang_buff {0};

//        modbus_receive_package(cap, buf_arr, arr_size);

        if (modbus_receive_package(cap, buf_arr, arr_size))
        {
                if (options.last_code == REG_READ_ANGLE_LOW) {
                        memcpy(&ang_buff, buf_arr, sizeof(ang_buff));
                        options.angle = ang_buff;
                } else if (options.last_code == REG_READ_SPEED) {
                        options.speed = buf_arr[0];
                } else if (options.last_code == REG_ERROR_CODE) {
                        options.error_flag = buf_arr[0];
                } else if (options.last_code == REG_READ_CURRENT) {
                        options.current_value = buf_arr[0];
                } else if (options.last_code == REG_SET_SPEED) {
                        // All is Ok, go through
                }
                /* Command is recognized */
                return true;
        } else
                return false;
}

void MotorWheel::set_wheel_speed(int16_t s_val)
{
        options.last_code = REG_SET_SPEED;
        write_single_register(id, REG_SET_SPEED, s_val);
}

void MotorWheel::request_angle()
{
        options.last_code = REG_READ_ANGLE_LOW;
        read_holding_registers(id, REG_READ_ANGLE_LOW, 2);
}

void MotorWheel::request_wheel_speed()
{
        options.last_code = REG_READ_SPEED;
        read_holding_registers(id, REG_READ_SPEED, 1);
}

void MotorWheel::request_current()
{
        options.last_code = REG_READ_CURRENT;
        read_holding_registers(id, REG_READ_CURRENT, 1);
}

void MotorWheel::request_error()
{
        options.last_code = REG_ERROR_CODE;
        read_holding_registers(id, REG_ERROR_CODE, 1);
}

void MotorWheel::clear_error()
{
        options.last_code = REG_ERROR_CODE;
        write_single_register(id, REG_ERROR_CODE, 1);

}




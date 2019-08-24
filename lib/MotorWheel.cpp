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
        bool status_init = true;
        write_single_register(id, REG_MODE, MODE_SPEED);
        wait_acknowlege();
        write_single_register(id, REG_S_PID_P, 1);
        wait_acknowlege();
        write_single_register(id, REG_S_PID_I, 200);
        wait_acknowlege();
        write_single_register(id, REG_S_PID_D, 0);
        wait_acknowlege();
        write_single_register(id, REG_I_CUTOFF, 3000);
        wait_acknowlege();
        write_single_register(id, REG_V_CUTOFF, 10000);
        wait_acknowlege();
        write_single_register(id, REG_T_CUTOFF, 100);
        wait_acknowlege();
        write_single_register(id, REG_PWM_LIMIT, 230);
        wait_acknowlege();
        write_single_register(id, REG_S_PID_I_LIMIT, 2000);
        wait_acknowlege();
        write_single_register(id, REG_A_PID_I_LIMIT, 2000);
        wait_acknowlege();
        write_single_register(id, REG_ERROR_CODE, 0);
        wait_acknowlege();

        return status_init;
}

bool MotorWheel::wait_acknowlege()
{
       vTaskDelay(5);
       read_response();
       if (this->options.last_code)
               return true;
       else
               return false;
}

void MotorWheel::read_response()
{
        uint8_t cap = 0;
        int16_t buf_arr[8] = {0};
        uint8_t arr_size = 0;
        int32_t ang_buff = 0;

        modbus_receive_package(cap, options.last_code, buf_arr, arr_size);

        if (options.last_code == REG_READ_ANGLE_LOW) {
                memcpy(&ang_buff, buf_arr, sizeof(ang_buff));
        } else if (options.last_code == REG_READ_CURRENT)
                options.current_value = buf_arr[0];
}

void MotorWheel::set_wheel_speed(int16_t s_val)
{
        write_single_register(id, REG_SET_SPEED, s_val);
}

void MotorWheel::request_angle()
{
        read_holding_registers(id, REG_READ_ANGLE_LOW, 2);
}

void MotorWheel::request_wheel_speed()
{
        read_holding_registers(id, REG_READ_SPEED, 1);
}

void MotorWheel::request_current()
{
        write_single_register(id, REG_READ_CURRENT, 1);
}

void MotorWheel::request_error()
{
        write_single_register(id, REG_ERROR_CODE, 1);
}




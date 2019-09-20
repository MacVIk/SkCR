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
        options.id = id;
}

MotorWheel::~MotorWheel(){}

static int16_t data_arr[2];
static uint8_t arr_size;

static uint8_t cap_id;

bool MotorWheel::akn_flag = true;

bool MotorWheel::init_wheel()
{
        options.last_code = REG_SET_SPEED;
        options.payload = 0;
        options.rquest_type = RequestType::WRITE;

        bool status_init {true};
        write_single_register(options.id, REG_ERROR_CODE, 0);
        status_init = wait_acknowlege();
        write_single_register(options.id, REG_MODE, MODE_SPEED);
        status_init = wait_acknowlege();
        write_single_register(options.id, REG_S_PID_P, 1);
        status_init = wait_acknowlege();
        write_single_register(options.id, REG_S_PID_I, 200);
        status_init = wait_acknowlege();
        write_single_register(options.id, REG_S_PID_D, 0);
        status_init = wait_acknowlege();
        write_single_register(options.id, REG_I_CUTOFF, 3000);
        status_init = wait_acknowlege();
        write_single_register(options.id, REG_V_CUTOFF, 10000);
        status_init = wait_acknowlege();
        write_single_register(options.id, REG_T_CUTOFF, 100);
        status_init = wait_acknowlege();
        write_single_register(options.id, REG_PWM_LIMIT, 230);
        status_init = wait_acknowlege();
        write_single_register(options.id, REG_S_PID_I_LIMIT, 2000);
        status_init = wait_acknowlege();
        write_single_register(options.id, REG_A_PID_I_LIMIT, 2000);
        status_init = wait_acknowlege();
        write_single_register(options.id, REG_ERROR_CODE, 0);
        status_init = wait_acknowlege();

        return status_init;
}

bool MotorWheel::wait_acknowlege()
{
        static int8_t count {MAX_REQUEST_TRY};
        ulTaskNotifyTake(pdTRUE, 5);
        //ToDO add a function
        if (akn_flag) {
                akn_flag = false;
                count = MAX_REQUEST_TRY;
                return true;
        } else {
                if (options.rquest_type == RequestType::READ) {
                        read_holding_registers(options.id, options.payload, options.payload);
                } else
                        write_single_register(options.id, options.payload, options.payload);
                while ((--count))
                        return wait_acknowlege();
                count = MAX_REQUEST_TRY;
                return false;
        }
}

bool MotorWheel::set_wheel_speed(int16_t s_val)
{
        options.last_code = REG_SET_SPEED;
        options.payload = s_val;
        options.rquest_type = RequestType::WRITE;

        write_single_register(options.id, options.last_code, options.payload);
        if (wait_acknowlege())
                return true;
        else
                return false;
}

bool MotorWheel::request_angle(int32_t& angle)
{
        int32_t ang_buff {0};

        options.last_code = REG_READ_ANGLE_LOW;
        options.payload = 2;
        options.rquest_type = RequestType::READ;

        read_holding_registers(options.id, options.last_code, options.payload);
//        last_request = read_holding_registers;

        if (wait_acknowlege()) {
                modbus_receive_package(cap_id, data_arr, arr_size);
                if (cap_id != options.id) {
                        //ToDo id of request and answer are not the same
                }
                memcpy(&ang_buff, data_arr, sizeof(ang_buff));
                angle = ang_buff;
                return true;
        } else {
                //ToDO error processing
                return false;
        }

}

bool MotorWheel::request_wheel_speed(int16_t& speed)
{
        options.last_code = REG_READ_SPEED;
        options.payload = 1;
        options.rquest_type = RequestType::READ;

        read_holding_registers(options.id, options.last_code, options.payload);
        if (wait_acknowlege()) {
                modbus_receive_package(cap_id, data_arr, arr_size);
                if (cap_id != options.id) {
                        //ToDo id of request and answer are not the same
                }
                speed = data_arr[0];
                return true;
        } else
                return false;

}

bool MotorWheel::request_current(int16_t& current)
{
        options.last_code = REG_READ_CURRENT;
        options.payload = 1;
        options.rquest_type = RequestType::READ;

        read_holding_registers(options.id, options.last_code, options.payload);
        if (wait_acknowlege()) {
                modbus_receive_package(cap_id, data_arr, arr_size);
                if (cap_id != options.id) {
                        //ToDo id of request and answer are not the same
                }
                current = data_arr[0];
                return true;
        } else
                return false;

}

bool MotorWheel::request_error(bool& error_flag)
{
        options.last_code = REG_ERROR_CODE;
        options.payload = 1;
        options.rquest_type = RequestType::READ;

        read_holding_registers(options.id, options.last_code, options.payload);
        if (wait_acknowlege()) {
                modbus_receive_package(cap_id, data_arr, arr_size);
                if (cap_id != options.id) {
                        //ToDo id of request and answer are not the same
                }

                error_flag = data_arr[0];
                return true;
        } else
                return false;
}

bool MotorWheel::clear_error()
{
        options.last_code = REG_ERROR_CODE;
        options.payload = 1;
        options.rquest_type = RequestType::WRITE;

        write_single_register(options.id, options.last_code, options.payload);
        if (wait_acknowlege()) {
                modbus_receive_package(cap_id, data_arr, arr_size);
                return true;
        } else
                return false;
}




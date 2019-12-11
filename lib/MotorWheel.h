/*
 * MotorWheel.h
 *
 *  Created on: 13 рту. 2019 у.
 *      Author: Taras.Melnik
 */

#ifndef LIB_MOTORWHEEL_H_
#define LIB_MOTORWHEEL_H_

#include "stm32f4xx.h"

/* int16_t_Registers */
#define REG_MODE                ((uint8_t)  0)
#define REG_SET_PWM             ((uint8_t)  2)
#define REG_SET_SPEED           ((uint8_t)  3)
#define REG_SET_ANGLE_LOW       ((uint8_t)  4)
#define REG_SET_ANGLE_HIGH      ((uint8_t)  5)
#define REG_V_CUTOFF            ((uint8_t)  9)
#define REG_I_CUTOFF            ((uint8_t)  10)
#define REG_T_CUTOFF            ((uint8_t)  11)
#define REG_S_PID_I             ((uint8_t)  12)
#define REG_S_PID_P             ((uint8_t)  13)
#define REG_S_PID_D             ((uint8_t)  14)
#define REG_A_PID_I             ((uint8_t)  15)
#define REG_A_PID_P             ((uint8_t)  16)
#define REG_A_PID_D             ((uint8_t)  17)
#define REG_TIME_OUT            ((uint8_t)  18)
#define REG_S_PID_I_LIMIT       ((uint8_t)  19)
#define REG_A_PID_I_LIMIT       ((uint8_t)  20)
#define REG_PWM_LIMIT           ((uint8_t)  21)
#define REG_ERROR_CODE          ((uint8_t)  29)

/* RO_Registers */
#define REG_READ_CURRENT        ((uint8_t)  65)
#define REG_READ_TEMP           ((uint8_t)  66)
#define REG_READ_ANGLE_LOW      ((uint8_t)  67)
#define REG_READ_ANGLE_HIGH     ((uint8_t)  68)
#define REG_READ_SPEED          ((uint8_t)  69)

/* RW_Registers */
#define REG_ID                  ((int16_t)  129)
#define REG_SAVE_FLASH          ((int16_t)  130)

/* Working mode */
#define MODE_ANGLE              ((uint8_t)  1)
#define MODE_SPEED              ((uint8_t)  2)
#define MODE_PWM                ((uint8_t)  3)
#define MODE_NONE               ((uint8_t)  0)

/* Internal Flags */

#define MAX_REQUEST_TRY         ((uint8_t) 3)

#include "stm32f4xx.h"

#include "ProtocolModBus.h"

// ToDo fix this crutch
enum RequestType {
    READ,
    WRITE
};

struct RequestOptions {
    int16_t payload;
    uint8_t id;
    uint8_t last_code;
    RequestType rquest_type; //read or write command
};

class MotorWheel: public ProtocolModBus {
public:
    MotorWheel(uint8_t id);
    virtual ~MotorWheel();

    /* Read only access to wheel options */
    bool init_wheel();

    bool set_wheel_speed(int16_t _val);
    bool request_angle(int32_t& angle);
    bool request_wheel_speed(int16_t& speed);
    bool request_current(int16_t& current);
    bool request_error(bool& error_flag);
    bool clear_error();

    inline void confirm_motor_answer()
    {
        akn_flag = true;
    }

private:
    inline bool wait_acknowlege();
    RequestOptions options;
    //        void (*last_request)(uint8_t, int16_t, int16_t);

    static bool akn_flag;
};

#endif /* LIB_MOTORWHEEL_H_ */

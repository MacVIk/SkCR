/*
 * DriverI2C.h
 *
 *  Created on: 10 сент. 2019 г.
 *      Author: Taras.Melnik
 */

#ifndef LIB_DRIVERI2C_H_
#define LIB_DRIVERI2C_H_

#include "stm32f4xx.h"

class DriverI2C {
public:
        DriverI2C();
        virtual ~DriverI2C();

        void init_i2c();
        void read_i2c(uint8_t slaveAdr, uint8_t subRegAdr,
                        uint8_t regNumb, uint8_t* data);
        void write_i2c(uint8_t slaveAdr, uint8_t subRegAdr,
                        uint8_t data);
private:
        void init_i2c_watchdog();

};

#endif /* LIB_DRIVERI2C_H_ */

/*
 * i2c_timer.h
 *
 *  Created on: Apr 9, 2013
 *      Author: Admin
 */

#ifndef I2C_TIMER_H_
#define I2C_TIMER_H_

#ifdef __cplusplus
extern "C" {
#endif

void i2c_timer_init(void); // Initialize timer
void i2c_timer_timeout(unsigned int t); // Wait timeout t
void i2c_timer_start_timeout(unsigned int t); // Start timeout t
void i2c_timer_stop_timeout(void); // Stop timeout t
unsigned char i2c_timer_timeout_status(void);  // Read timeout status

#ifdef __cplusplus
}
#endif

#endif /* I2C_TIMER_H_ */

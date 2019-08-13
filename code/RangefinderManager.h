/*
 * CollisionAvoidance.h
 *
 *  Created on: 15.01.2019
 *      Author: Taras.Melnik
 */

#ifndef CODE_RANGEFINDERMANAGER_H_
#define CODE_RANGEFINDERMANAGER_H_

#include "stm32f4xx.h"
#include "TaskWrapper.h"

#define MIN_RESPONCE_TIME 		((uint8_t) 50)	// ms. got from datasheet for ultrasonic rangefinders
#define MAX_RANGEFINDERS_DISTANCE 	((uint8_t) 50) 	// sm. rangefinders distance for collision (got from Misha)
#define QUEUE_IS_EMPTY			((uint8_t) 0)
#define RANGEFINDERS_NUMBER		((uint8_t) 6)	// number of rangefinders we use

// filtration empiric constant
#define CONST_HISTOR_ARR		((uint8_t) 15)	//size of array for previous collision data

class RangefinderManager: public TaskWrapper {
public:
        RangefinderManager();
	virtual ~RangefinderManager();

	/* Peripheral initialization functions */
	void init_pwm();
	void init_interrupt();
	void init_timer();

	void get_distance(uint8_t* distArr);
protected:
        /* Median filter */
        uint8_t partition(uint8_t* input, uint8_t p, uint8_t r);
        uint8_t quick_select(uint8_t* input, uint8_t p, uint8_t r, uint8_t k);
private:
        void calculate_distance(uint8_t* distArr, uint8_t i);
	void run();

	static uint8_t outDistArr[RANGEFINDERS_NUMBER];
	static uint8_t  *sensDistArr[RANGEFINDERS_NUMBER];
};
extern RangefinderManager rf_manager;

#endif /* CODE_RANGEFINDERMANAGER_H_ */


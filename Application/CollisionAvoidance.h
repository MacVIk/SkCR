/*
 * CollisionAvoidance.h
 *
 *  Created on: 15.01.2019
 *      Author: Taras.Melnik
 */

#ifndef COLLISIONAVOIDANCE_H_
#define COLLISIONAVOIDANCE_H_

#include "iActiveObject.h"
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "semphr.h"
#include "InitialisationList.h"
#include "USBUserInterface.h"

#define MIN_RESPONCE_TIME 					60			// ms. got from datasheet for ultrasonic rangefinders
#define MAX_RANGEFINDERS_DISTANCE 			50 			// sm. rangefinders distance for collision (got from Misha)
#define QUEUE_IS_EMPTY						0
#define RANGEFINDERS_NUMBER		((uint8_t)  6)			// number of rangefinders we use

// filtration empiric constants
#define CONST_HISTOR_ARR		((uint8_t)  15)			//size of array for previous collision data

class CollisionAvoidance: public iActiveObject {
public:
	CollisionAvoidance();
	virtual ~CollisionAvoidance();
	uint8_t partition(uint8_t* input, uint8_t p, uint8_t r);
	uint8_t quick_select(uint8_t* input, uint8_t p, uint8_t r, uint8_t k);
	void calculateDistanceQsort(uint8_t* distArr, uint8_t i);
	void run();

	QueueHandle_t xCollisionAvoidanceQueue;
	SemaphoreHandle_t xCollisionAvoidanceMutex;
private:
	uint8_t  *sensDistArr[RANGEFINDERS_NUMBER];
};
extern CollisionAvoidance *sensorTask;
#endif /* COLLISIONAVOIDANCE_H_ */


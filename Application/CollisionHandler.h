/*
 * CollisionHandler.h
 *
 *  Created on: 4 мая 2019 г.
 *      Author: Taras.Melnik
 */

#ifndef APPLICATION_COLLISIONHANDLER_H_
#define APPLICATION_COLLISIONHANDLER_H_

#include "iActiveObject.h"
#include "stm32f4xx.h"
#include "semphr.h"
#include "QueueCreator.h"
#include "task.h"
#include "CollisionAvoidance.h"

class CollisionHandler: public iActiveObject {
public:
	CollisionHandler();
	virtual ~CollisionHandler();
	bool getStatus();
	void run();
private:
	bool rColFlag;
};

extern CollisionHandler* collisionHandler;

#endif /* APPLICATION_COLLISIONHANDLER_H_ */

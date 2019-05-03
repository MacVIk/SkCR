/*
 * SystemTaskCreator.h
 *
 *  Created on: 26.12.2015
 *      Author: adozzer
 */

#ifndef SYSTEMTASKCREATOR_H_
#define SYSTEMTASKCREATOR_H_

#include "PeripheralDrivers/PortManager.h"
#include "PeripheralDrivers/RTCManager.h"
#include "core_cm4.h"
#include "Application/ReadEncoders.h"
#include "Application/LEDStrip.h"
#include "Application/BatteryChargeAsker.h"
#include "Application/USBUserInterface.h"
#include "Application/CollisionAvoidance.h"
#include "Application/MovementControl.h"
#include "Application/InitialisationList.h"
#include "Application/UARTtoRS485.h"
#include "Application/HyroMotor.h"

class SystemTaskCreator: public iActiveObject {
private:
	SystemTaskCreator();
	virtual ~SystemTaskCreator();
	static SystemTaskCreator* SystemTaskCreatorInstance;

public:
	static SystemTaskCreator* getInstance();
	void run();
};

#endif /* SYSTEMTASKCREATOR_H_ */

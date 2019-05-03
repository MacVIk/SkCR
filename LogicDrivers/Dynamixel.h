/*
 * Dynamixel.h
 *
 *  Created on: 08.08.2017
 *      Author: adozzer
 */

#ifndef DYNAMIXEL_H_
#define DYNAMIXEL_H_

#include "PeripheralDrivers/USART.h"
#include "FreeRtos/wrapper/iActiveObject.h"

enum Instructions {
	NONE = 0x00, PING = 0x01, READ = 0x02, WRITE = 0x03
};

class Dynamixel: public iActiveObject {
private:
	const static uint8_t header[];
	static const float32_t resolution;
	static const float32_t threshold;

	USART* usart;
	uint8_t id;

	float32_t zeroBias;
	float32_t currentAngle;
	uint16_t targetPosition;

	uint16_t modelNumber;
	uint8_t fwVersion;

	Instructions lastInstruction;
	uint16_t lastAddressedReg;
	uint8_t state[60];

	uint16_t update_crc(uint8_t *data_blk_ptr, uint16_t data_blk_size,
			uint16_t crc_accum = 0);
public:
	Dynamixel(USART* usart, uint8_t id = 1, float32_t zeroBias = 2.618f);
	void sendCommand(uint8_t instruction, uint8_t* params, uint8_t paramsCount);
	bool processAnswer();

	void readReg(uint16_t address, uint16_t length);
	void writeReg(uint16_t address, uint16_t length, uint8_t* buffer);
	void act();

	float32_t updateCurrentPosition();
	float32_t getCurrentPositionInUnits();
	void setPosition(float32_t target);
	void enable();
	void emergencyStop();
	void protectiveStop();
	bool isMoving();

	void setLED(uint8_t r, uint8_t g, uint8_t b);

	void run();
	virtual ~Dynamixel();
};

#endif /* DYNAMIXEL_H_ */

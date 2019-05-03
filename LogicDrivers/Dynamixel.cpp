/*
 * Dynamixel.cpp
 *
 *  Created on: 08.08.2017
 *      Author: adozzer
 */

#include "Dynamixel.h"

const uint8_t Dynamixel::header[] = { 0xFF, 0xFF, 0xFD, 0x00 };
const float	Dynamixel::resolution = 300.0f * PI / (1024.0f * 180.0f);
const float	Dynamixel::threshold = 2 * PI / 180.0f;

Dynamixel::Dynamixel(USART* usart, uint8_t id, float32_t zeroBias) :
		usart(usart), id(id), zeroBias(zeroBias), currentAngle(0), targetPosition(
				0), modelNumber(0), fwVersion(0), lastInstruction(NONE), lastAddressedReg(
				0) {
	for (uint8_t i = 0; i < 60; i++) {
		this->state[i] = 0;
	}
	this->setPosition(0.2);
}

void Dynamixel::sendCommand(uint8_t instruction, uint8_t* params,
		uint8_t paramsCount) {
	USARTMessage msg;
	msg.curPos = 0;

	memcpy(&msg.data[0], this->header, 4); //insert header

	msg.data[4] = this->id;

	uint16_t len = 3 + paramsCount;
	memcpy(&msg.data[5], &len, 2);

	msg.data[7] = instruction;

	if (paramsCount > 0) {
		memcpy(&msg.data[8], params, paramsCount);
	}

	uint16_t crc = update_crc(msg.data, 8 + paramsCount);
	memcpy(&msg.data[8 + paramsCount], &crc, 2);

	msg.length = 10 + paramsCount;
	this->lastInstruction = (Instructions) instruction;

	usart->send(&msg, true);
}

uint16_t Dynamixel::update_crc(uint8_t *data_blk_ptr, uint16_t data_blk_size,
		uint16_t crc_accum) {
	uint16_t crc_table[256] = { 0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E,
			0x0014, 0x8011, 0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D,
			0x8027, 0x0022, 0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D,
			0x8077, 0x0072, 0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E,
			0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD,
			0x80D7, 0x00D2, 0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE,
			0x00E4, 0x80E1, 0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE,
			0x00B4, 0x80B1, 0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D,
			0x8087, 0x0082, 0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D,
			0x8197, 0x0192, 0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE,
			0x01A4, 0x81A1, 0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE,
			0x01F4, 0x81F1, 0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD,
			0x81C7, 0x01C2, 0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E,
			0x0154, 0x8151, 0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D,
			0x8167, 0x0162, 0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D,
			0x8137, 0x0132, 0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E,
			0x0104, 0x8101, 0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D,
			0x8317, 0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E,
			0x0324, 0x8321, 0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E,
			0x0374, 0x8371, 0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D,
			0x8347, 0x0342, 0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE,
			0x03D4, 0x83D1, 0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED,
			0x83E7, 0x03E2, 0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD,
			0x83B7, 0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E,
			0x0384, 0x8381, 0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E,
			0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD,
			0x82A7, 0x02A2, 0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD,
			0x82F7, 0x02F2, 0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE,
			0x02C4, 0x82C1, 0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D,
			0x8257, 0x0252, 0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E,
			0x0264, 0x8261, 0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E,
			0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D,
			0x8207, 0x0202 };

	for (uint16_t j = 0; j < data_blk_size; j++) {
		uint16_t i = ((unsigned short) (crc_accum >> 8) ^ data_blk_ptr[j])
				& 0xFF;
		crc_accum = (crc_accum << 8) ^ crc_table[i];
	}

	return crc_accum;
}

bool Dynamixel::processAnswer() {
	USARTMessage msg;
	bool status = xQueuePeek(this->usart->queueReceive, &msg, 1000);
	if (!status) {
		return false;
	}
	uint16_t crc = this->update_crc(msg.data, msg.length - 2);
	uint16_t incomeCRC = 0;
	memcpy(&incomeCRC, &msg.data[msg.length - 2], 2);

	if (incomeCRC != crc) {
		//wrong CRC. We free queue for other messages
		xQueueReceive(this->usart->queueReceive, &msg, 0);
		return false;
	}

	uint8_t incomeID = msg.data[4];
	if (incomeID != this->id) {
		return false;
	} else {
		//this message is for us. Remove this item from the queue
		xQueueReceive(this->usart->queueReceive, &msg, portMAX_DELAY);
	}

	uint8_t instuction = msg.data[7];
	uint16_t length = 0;
	memcpy(&length, &msg.data[5], 2);
	uint16_t paramsCount = length - 4;
	uint8_t params[paramsCount];
	memcpy(params, &msg.data[9], paramsCount);

	switch (this->lastInstruction) {
	case PING:
		memcpy(&this->modelNumber, &params[0], 2);
		this->fwVersion = params[2];
		break;
	case READ:
		memcpy(&this->state[this->lastAddressedReg], &params, paramsCount);
		break;
	}

	return true;
}

void Dynamixel::run() {
	uint8_t pGain = 120;
	this->writeReg(29, 1, &pGain);
	while (true) {
//		this->readReg(49, 1);
		this->updateCurrentPosition();
		uint8_t pos[2];
		memcpy(pos, &this->targetPosition, 2);
		this->writeReg(30, 2, pos);

		this->taskDelay(10);
	}
}

void Dynamixel::readReg(uint16_t address, uint16_t length) {
	uint8_t params[4];
	memcpy(&params[0], &address, 2);
	memcpy(&params[2], &length, 2);
	this->lastAddressedReg = address;
	this->sendCommand(READ, params, 4);
	this->processAnswer();
}

void Dynamixel::writeReg(uint16_t address, uint16_t length, uint8_t* buffer) {
	uint8_t params[length + 2];
	memcpy(&params[0], &address, 2);
	memcpy(&params[2], buffer, length);
	this->sendCommand(WRITE, params, length + 2);
	this->processAnswer();
}

float32_t Dynamixel::updateCurrentPosition() {
	this->readReg(37, 2);
	uint16_t position = 0;
	memcpy(&position, &state[37], 2);
	this->currentAngle = position * this->resolution - 2.618f;
	return this->currentAngle;
}

void Dynamixel::setLED(uint8_t r, uint8_t g, uint8_t b) {
	uint8_t val = (r & 1) | ((g & 1) << 1) | ((b & 1) << 2);
	this->writeReg(25, 1, &val);
}

void Dynamixel::act() {
	this->sendCommand(0x05, 0, 0);
	this->processAnswer();
}

void Dynamixel::setPosition(float32_t target) {
	uint16_t position = ((target + 2.618f) / this->resolution);
	if (position > 1023) {
		position = 1023;
	}
	this->targetPosition = position;
}

bool Dynamixel::isMoving() {
	float32_t dif = fabsf(
			this->targetPosition * this->resolution - 2.618
					- this->currentAngle);
	if (dif < this->threshold) {
		return false;
	} else {
		return true;
	}
}

void Dynamixel::emergencyStop() {
//	this->setLED(1, 0, 0);
}

void Dynamixel::protectiveStop() {
//	this->setLED(1, 1, 0);
}

void Dynamixel::enable() {
//	this->setLED(0, 1, 0);
}

float32_t Dynamixel::getCurrentPositionInUnits() {
	return this->currentAngle;
}

Dynamixel::~Dynamixel() {
	// TODO Auto-generated destructor stub
}


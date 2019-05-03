/*
 * UARTtoRS485.cpp
 *
 *  Created on: 20.03.2019
 *      Author: Taras.Melnik
 */

#include "UARTtoRS485.h"

UARTtoRS485::UARTtoRS485() {
}

UARTtoRS485::~UARTtoRS485() {
	// TODO Auto-generated destructor stub
}

void UARTtoRS485::init(UARTuserInit* uartX)
{
	this->uart = uartX;
}

void UARTtoRS485::modbusReadReg(uint8_t id, uint8_t regAdress, uint8_t regQuantity)
{
	uint16_t crc;
	regArr[0] = (uint8_t) id;
	regArr[1] = (uint8_t) READ_MODBUS_REG;
	regArr[2] = (uint8_t) 0;
	regArr[3] = (uint8_t) regAdress;
	regArr[4] = (uint8_t) 0;
	regArr[5] = (uint8_t) regQuantity;
	crc = Crc16(regArr, 6);
	regArr[6] = (uint8_t) (crc & 0x00FF);
	regArr[7] = (uint8_t) (crc >> 8);
	uart->send(regArr, 8);
}

void UARTtoRS485::modbusWriteReg(uint8_t id, uint8_t regAdress, uint16_t data)
{
	uint16_t crc;
	regArr[0] = (uint8_t) id;
	regArr[1] = (uint8_t) WRITE_MODBUS_REG;
	regArr[2] = (uint8_t) 0;
	regArr[3] = (uint8_t) regAdress;
	regArr[4] = (uint8_t) (data >> 8);
	regArr[5] = (uint8_t) (data & 0x00FF);;
	crc = Crc16(regArr, 6);
	regArr[6] = (uint8_t) (crc & 0x00FF);
	regArr[7] = (uint8_t) (crc >> 8);
	uart->send(regArr, 8);
}

void UARTtoRS485::modbusWriteMultReg(uint8_t id, uint8_t regAdress, uint16_t* daraArr)
{
	uint16_t crc;
	regArr[0] = (uint8_t) id;
	regArr[1] = (uint8_t) WRITE_MULTIPLE_REG;
	regArr[2] = (uint8_t) 0;
	regArr[3] = (uint8_t) regAdress;
	regArr[4] = (uint8_t) (daraArr[0] & 0x00FF);
	regArr[5] = (uint8_t) (daraArr[0] >> 8);
	regArr[6] = (uint8_t) (daraArr[1] & 0x00FF);
	regArr[7] = (uint8_t) (daraArr[1] >> 8);
	crc = Crc16(regArr, 8);
	regArr[8] = (uint8_t) (crc & 0x00FF);
	regArr[9] = (uint8_t) (crc >> 8);
	uart->send(regArr, 10);
}

uint16_t UARTtoRS485::Crc16(uint8_t *pData, uint8_t length)
{
	uint8_t ucCRCHi = 0xFF;
	uint8_t ucCRCLo = 0xFF;
	uint8_t idx;
    while (length--) {
        idx = ucCRCLo ^ *( pData++ );
        ucCRCLo = ( uint8_t )( ucCRCHi ^ aucCRCHi[idx] );
        ucCRCHi = aucCRCLo[idx];
    }
    return (uint16_t )( ucCRCHi << 8 | ucCRCLo );
}

int16_t UARTtoRS485::uint8toInt16(uint8_t* arr)
{
	int16_t dtb = 0;
	dtb |= ((int16_t) arr[0]) << 8;
	dtb |= 0xffff & arr[1];
	return dtb;
}

int32_t UARTtoRS485::uint8toInt32(uint8_t* arr)
{
	int32_t dtb = 0;
	dtb |= ((int32_t) arr[0]) << 24;
	dtb |= ((int32_t) arr[1]) << 16;
	dtb |= ((int32_t) arr[2]) << 8;
	dtb |= arr[3];
	return dtb;
}

void UARTtoRS485::modbusReadData(uint8_t* arr, uint8_t &length)
{
	uint8_t buff = 0;
	uart->receiveByte(buff);
	uart->receiveByte(buff);
	if (buff == READ_MODBUS_REG) {
		uart->receiveByte(length);
		for (uint8_t i = 0; i < length; i++)
			uart->receiveByte(arr[i]);
	} else {
		for (uint8_t i = 0; i < 5; i++)
			arr[i] = 0;
	}
	uart->resetPointer();
}



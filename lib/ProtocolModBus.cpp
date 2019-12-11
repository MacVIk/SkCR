/*
 * ProtocolModBus.cpp
 *
 *  Created on: 20.03.2019
 *      Author: Taras.Melnik
 */

#include <string.h>
#include "stm32f4xx.h"

#include "ProtocolModBus.h"

ProtocolModBus::ProtocolModBus() {
}

ProtocolModBus::~ProtocolModBus() {
    // TODO Auto-generated destructor stub
}

void ProtocolModBus::write_single_register(uint8_t id, int16_t reg_adr,
        int16_t value)
{
    package.id = id;
    package.code = WRITE_SINGLE_REG;
    package.reg_address = reg_adr;
    package.reg_value = value;
    modbus_send_package(1);
}

void ProtocolModBus::write_multiple_registers(uint8_t id, int16_t reg_adr,
        int16_t* p_arr, int16_t size_arr)
{
    package.id = id;
    package.code = WRITE_MULTIPLE_REG;
    package.reg_address = reg_adr;
    package.pData = p_arr;
    modbus_send_package(size_arr);
}

void ProtocolModBus::read_holding_registers(uint8_t id, int16_t reg_adr,
        int16_t reg_number)
{
    package.id = id;
    package.code = READ_HOLDING_REG;
    package.reg_address = reg_adr;
    package.reg_value = reg_number;
    modbus_send_package(1);
}

uint16_t ProtocolModBus::crc_calculate(uint8_t *pData, uint8_t length)
{
    uint8_t ucCRCHi = 0xFF;
    uint8_t ucCRCLo = 0xFF;
    uint8_t idx;
    while (length--) {
        idx = ucCRCLo ^ *(pData++);
        ucCRCLo = (uint8_t) (ucCRCHi ^ aucCRCHi[idx]);
        ucCRCHi = aucCRCLo[idx];
    }
    return (uint16_t) (ucCRCHi << 8 | ucCRCLo);
}


void ProtocolModBus::modbus_send_package(uint8_t size_arr)
{
    uint16_t crc = 0;
    uint8_t pac_size = 0;

    usart.TxArr[0] = (uint8_t) package.id;
    usart.TxArr[1] = (uint8_t) package.code;
    usart.TxArr[2] = (uint8_t) (package.reg_address >> 8);
    usart.TxArr[3] = (uint8_t) (package.reg_address & 0x00ff);
    if (size_arr == 1) {
        pac_size = 6;
        usart.TxArr[4] = (uint8_t) (package.reg_value >> 8);
        usart.TxArr[5] = (uint8_t) (package.reg_value & 0x00ff);
    } else {
        pac_size = 4 + size_arr * 2;
        uint8_t j = 0;
        for (uint8_t i = 4; i < pac_size; i + 2) {
            usart.TxArr[i] = (uint8_t) (package.pData[j] >> 8);
            usart.TxArr[i++] = (uint8_t) (package.pData[j] & 0x00ff);
            j++;
        }
    }
    crc = crc_calculate(usart.TxArr, pac_size);
    /* ToDO check is it correct? */
            usart.TxArr[pac_size++] = (uint8_t) (crc & 0x00ff);
    usart.TxArr[pac_size++] = (uint8_t) (crc >> 8);
    usart_send(usart.TxArr, pac_size);
}

bool ProtocolModBus::modbus_receive_package(uint8_t& id, int16_t* data_arr, uint8_t& arr_size)
{
    uint16_t crc {};
    uint16_t crc_rx {};

    /* Option for error processing */
    uint16_t pack_size {4};

    id = usart.RxArr[0];
    uint8_t code = usart.RxArr[1];
    if (code == 0) {
        return false;
    } else if (code == READ_HOLDING_REG) {
        pack_size = usart.RxArr[2] + 3;

        crc_rx = crc_calculate(usart.RxArr, pack_size);
        crc |= (uint16_t) usart.RxArr[pack_size++];
        crc |= ((uint16_t) usart.RxArr[pack_size++]) << 8;
        if (crc == crc_rx) {
            uint8_t j = 2;
            arr_size = usart.RxArr[2] / 2;
            for (uint8_t i = 0; i < arr_size; ++i) {
                data_arr[i] = {0};
                data_arr[i] |= ((uint16_t) usart.RxArr[++j]) << 8;
                data_arr[i] |= usart.RxArr[++j];
            }
        } else {
            // ToDo error checksumm
        }
    }
    for (uint8_t i = 0; i < pack_size; i++)
        usart.RxArr[i] = 0;

    return true;
}

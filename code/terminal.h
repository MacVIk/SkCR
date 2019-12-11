/*
 * USBUserInterface.h
 *
 *  Created on: 29.10.2018
 *      Author: Taras.Melnik
 */

#ifndef CODE_TERMINAL_H
#define CODE_TERMINAL_H

#include "TaskWrapper.h"

class Terminal: public TaskWrapper {

    inline void add_error_byte(uint8_t& answerLength);
    inline void calculate_checksum(const uint8_t answerLength);

public:
    void run();
};

extern Terminal terminal;

#endif /* CODE_TERMINAL_H */

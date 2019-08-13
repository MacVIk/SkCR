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
public:
        Terminal();
	virtual ~Terminal();

	void init();
	void run();
};

extern Terminal terminal;

#endif /* CODE_TERMINAL_H */

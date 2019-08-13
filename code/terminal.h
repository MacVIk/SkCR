/*
 * USBUserInterface.h
 *
 *  Created on: 29.10.2018
 *      Author: Taras.Melnik
 */

#ifndef CODE_TERMINAL_H
#define CODE_TERMINAL_H

class Terminal {
public:
        Terminal();
	virtual ~Terminal();

	void init();
	static void run(void *parameters);
};

extern Terminal terminal;

#endif /* CODE_TERMINAL_H */

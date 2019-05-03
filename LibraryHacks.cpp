/*
 * LibraryHacks.cpp
 *
 *  Created on: 23 Jan 2011
 *      Author: Andy
 */

#include <stdlib.h>
#include <sys/types.h>
#include <FreeRTOS.h>

/*
 * The default pulls in 70K of garbage
 */

namespace __gnu_cxx {
void __verbose_terminate_handler() {
	for (;;)
		;
}
}

/*
 * The default pulls in about 12K of garbage
 */

extern "C" void __cxa_pure_virtual() {
	for (;;)
		;
}

/*
 * Implement C++ new/delete operators using the heap
 *
 *
 */

uint32_t new_count = 0;
uint32_t new_count_arr = 0;

void *operator new(size_t size) {
	new_count++;
	return pvPortMalloc(size);
}

void *operator new[](size_t size) {
	new_count_arr++;
	return pvPortMalloc(size);
}

void operator delete(void *p) {
	new_count--;
	vPortFree(p);
}

void operator delete[](void *p) {
	new_count_arr--;
	vPortFree(p);
}

/*
 * sbrk function for getting space for malloc and friends
 */

extern int end;

extern "C" {
caddr_t _sbrk(int incr) {
	static unsigned char *heap = NULL;
	unsigned char *prev_heap;

	if (heap == NULL) {
		heap = (unsigned char *) &end;
	}
	prev_heap = heap;
	/* check removed to show basic approach */

	heap += incr;

	return (caddr_t) prev_heap;
}
}

extern "C" {
static void HardFault_Handler(void) __attribute__( ( naked ) );
static void HardFault_Handler(void) {
	__asm volatile
	(
			" tst lr, #4                                                \n"
			" ite eq                                                    \n"
			" mrseq r0, msp                                             \n"
			" mrsne r0, psp                                             \n"
			" ldr r1, [r0, #24]                                         \n"
			" ldr r2, handler2_address_const                            \n"
			" bx r2                                                     \n"
			" handler2_address_const: .word prvGetRegistersFromStack    \n"
	);
}

void prvGetRegistersFromStack(uint32_t *pulFaultStackAddress) {
	/* These are volatile to try and prevent the compiler/linker optimising them
	 away as the variables never actually get used.  If the debugger won't show the
	 values of the variables, make them global my moving their declaration outside
	 of this function. */
	volatile uint32_t r0;
	volatile uint32_t r1;
	volatile uint32_t r2;
	volatile uint32_t r3;
	volatile uint32_t r12;
	volatile uint32_t lr; /* Link register. */
	volatile uint32_t pc; /* Program counter. */
	volatile uint32_t psr;/* Program status register. */

	r0 = pulFaultStackAddress[0];
	r1 = pulFaultStackAddress[1];
	r2 = pulFaultStackAddress[2];
	r3 = pulFaultStackAddress[3];

	r12 = pulFaultStackAddress[4];
	lr = pulFaultStackAddress[5];
	pc = pulFaultStackAddress[6];
	psr = pulFaultStackAddress[7];

	/* When the following line is hit, the variables contain the register values. */
	for (;;)
		;
}
}

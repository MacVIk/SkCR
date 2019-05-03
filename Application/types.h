/*******************************************************************************
* FILENAME: types.h
*
* DESCRIPTION: Глобальные типы
*
******************************************************************************/
#ifndef TYPES_H
#define TYPES_H
#include <stm32f4xx.h>        //Регистры STM32

typedef GPIO_TypeDef* tPort;

typedef unsigned char   tU8;       // 8 bits, unsigned 
typedef signed char     tS8;       // 8 bits, signed 
typedef unsigned int    tU16;      // Two-byte unsigned int 
typedef signed int      tS16;      // Two-byte signed int 
typedef unsigned long   tU32;      // Four-byte unsigned long 
typedef signed long     tS32;      // Four-byte signed long 
typedef float           tF32;      // 4-byte IEEE      
typedef bool            tBoolean;  // TRUE or FALSE


//Misc global application defines
#ifdef TRUE
#undef TRUE
#endif // TRUE
#define TRUE (tBoolean) true

#ifdef FALSE
#undef FALSE
#endif // FALSE
#define FALSE (tBoolean) false

#endif // TYPES_H

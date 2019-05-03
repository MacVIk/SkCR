/*******************************************************************************
*  FILENAME: susuassert.h
*
*  DESCRIPTION: 
*
*  Copyright (c) 2015 by SUSU.
*
******************************************************************************/
#ifndef SUSUASSERT_H
#define SUSUASSERT_H

#include <Application/types.h>   // for tBoolean

#ifdef   __cplusplus
extern "C" {
#endif

#ifdef DEBUG

#pragma inline=forced
inline void assertBreak(void)
{ //lint !e957   No prototype OK
  asm("BREAK");
}

#define ASSERT(e)    ((e) ? (void)0 : assertBreak())

#define ALLEGE(e, val)  ASSERT(e == val)
#define NALLEGE(e, val)  ASSERT(e != val)

#else /* !DEBUG */
/* If not debugging, ASSERT does nothing.  */
#define ASSERT(e)    ((void)0)

#define ALLEGE(e, val)  (void) e
#define NALLEGE(e, val) (void) e

#endif   /* DEBUG */

#ifdef   __cplusplus
}
#endif

#endif // SUSUASSERT_H

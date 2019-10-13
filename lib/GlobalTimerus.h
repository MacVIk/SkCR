/*
 * GlobalTimerus.h
 *
 *  Created on: 25 сент. 2019 г.
 *      Author: Taras.Melnik
 */

#ifndef LIB_GLOBALTIMERUS_H_
#define LIB_GLOBALTIMERUS_H_

#include "stm32f4xx.h"

class GlobalTimer_us {
public:
        GlobalTimer_us();
        virtual ~GlobalTimer_us();
        GlobalTimer_us(const GlobalTimer_us&);
        GlobalTimer_us& operator=(GlobalTimer_us&);

        static GlobalTimer_us* get_instance();

        /* Get time in us with a chance of overflow
         * (once per 5 * 10^9  hours) */
        uint64_t get_time();
private:
        void timer_init();
        void interrupt_init();
        uint64_t tickValue;
        static GlobalTimer_us* mClassPointer;
};

#endif /* LIB_GLOBALTIMERUS_H_ */

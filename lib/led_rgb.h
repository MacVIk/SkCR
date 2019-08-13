/*
 * Led_rgb.h
 *
 *  Created on: 8 рту. 2019 у.
 *      Author: Taras.Melnik
 */

#ifndef LIB_LED_RGB_H_
#define LIB_LED_RGB_H_

/* Type for led color */
typedef enum {
        RED             = 1,
        GREEN           = 2,
        BLUE            = 3,
        WHITE           = 4,
        RED_FLASH       = 5,
        BLACK           = 6
} cl;

class Led_rgb {
public:
        Led_rgb();
        virtual ~Led_rgb();

        /* Turn on the led */
        void init_led();

        /* Control function */
        void mutex_take();
        void mutex_give();

        /* User function */
        void set_color(cl color);
private:
        void init_gpio();
        bool buttonMutex;
};

#endif /* LIB_LED_RGB_H_ */

/*
 * Led_rgb.h
 *
 *  Created on: 8 рту. 2019 у.
 *      Author: Taras.Melnik
 */

#ifndef LIB_LEDRGB_H_
#define LIB_LEDRGB_H_

/* Type for led color */
typedef enum LedColor : char {
        RED             = 1,
        GREEN           = 2,
        BLUE            = 3,
        WHITE           = 4,
        RED_FLASH       = 5,
        BLACK           = 6
} cl;

class LedRgb {
public:
    LedRgb();
    virtual ~LedRgb();

    /* Turn on the led */
    void init_led();

    /* Control function */
    void mutex_take(const cl color);
    void mutex_take();
    void mutex_give();

    /* User function */
    void set_color(const cl color);
private:
    void init_gpio();
    bool buttonMutex;
};

extern LedRgb ledRgb;

#endif /* LIB_LEDRGB_H_ */

/*
 * ReadEncoders.h
 *
 *  Created on: 02.07.2018
 *      Author: Taras.Melnik
 */

#ifndef READENCODERS_H_
#define READENCODERS_H_

#include "iActiveObject.h"
#include "stm32f4xx_gpio.h"
//#include "InitialisationList.h"
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "semphr.h"
#include "queue.h"
//#include "MovementControl.h"

#define PRIMARY_COUNTER_VALUE 		((uint16_t) 32767)
//#define ENCODER_RESOLUTION 		((float) 0.00142857f)  	// 1 / (encoder resolution - 700 tpr)
//#define ENCODER_RESOLUTION 		((float) 0.00667f)  	// 1 / (encoder resolution - 150 tpr)
#define ENCODER_RESOLUTION 			((float) 0.002857f)  	// 1 / (encoder resolution - 350 tpr)
#define TIMER_RESOLUTION 			((float) 10000)

#define ENC_HISTORY_ARR				((uint8_t)  10)  			//size of array for previous velocity data
#define OPPOSIT_SMOOTHING_FACTOR	((float)  	0.5f)  			//(1 - alfa)


class ReadEncoders: public iActiveObject {
public:

	ReadEncoders();
	virtual ~ReadEncoders();

	void writeToQueue(float* inputArr);
	void readFromQueue(float &outputData);
	float bubleSortMid(float velEnc[][10], uint8_t pos);

	void run();
};

#endif /* READENCODERS_H_ */

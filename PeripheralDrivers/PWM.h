#ifndef PWM_H_
#define PWM_H_

#include "TimerManager.h"
#include "PortManager.h"
#include "arm_math.h"

class PWM {
public:
	PWM(GPIO_TypeDef *portx, uint8_t pinx, TIM_TypeDef *timerx,
			uint8_t channel);
	void init(uint32_t prescaler, uint32_t length, uint32_t pulse);
	void init(float32_t period_usec, float32_t width_usec);
//	void init_ns(float32_t period_nsec, float32_t width_nsec);
	void stop();
	void start();
	void setPeriod(float32_t period_usec);
	void setWidth(float32_t width);
	void setWidthPercents(float32_t width);
	void preload(float32_t preload_usec);
//	void setWidthStepByStep(uint32_t width, float step);
	float32_t getPeriodUsec();
	float32_t getWidthUsec();
	void update();

private:
	GPIO_TypeDef *portx;
	uint8_t pinx;
public:
	TIM_TypeDef *timerx;
private:
	uint8_t channel;

private:
	float32_t period_usec;
	float32_t width_usec;
};

#endif /* PWM_H_ */

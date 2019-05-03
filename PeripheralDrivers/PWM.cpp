#include "PWM.h"

PWM::PWM(GPIO_TypeDef *portx, uint8_t pinx, TIM_TypeDef *timerx,
		uint8_t channel) {
	this->portx = portx;
	this->pinx = pinx;
	this->timerx = timerx;
	this->channel = channel;
}
//mks
void PWM::init(float32_t period_usec, float32_t width_usec) {
	//TODO:optimize
	this->period_usec = period_usec;
	this->width_usec = width_usec;

	uint32_t prescaler = (uint32_t) (1 + (period_usec * 168) / (65535 * 2));
	uint32_t length = (uint32_t) ((period_usec * 168) / (2 * prescaler));
	uint32_t pulse = (uint32_t) ((width_usec * length) / period_usec);

	this->init(prescaler, length, pulse);
}

void PWM::init(uint32_t prescaler, uint32_t length, uint32_t pulse) {

	PortManager::initPin(this->portx, this->pinx, GPIO_Mode_AF,
			GPIO_Speed_100MHz, GPIO_PuPd_NOPULL, GPIO_OType_PP);

	if ((timerx == TIM1) || (timerx == TIM2)) {
		PortManager::pinAFConfig(this->portx, this->pinx, GPIO_AF_TIM1);
	}
	if ((timerx == TIM3) || (timerx == TIM4) || (timerx == TIM5)) {
		PortManager::pinAFConfig(this->portx, this->pinx, GPIO_AF_TIM3);
	}
	if (timerx == TIM8 || timerx == TIM9 || timerx == TIM10 || timerx == TIM11) {
		PortManager::pinAFConfig(this->portx, this->pinx, GPIO_AF_TIM8);
	}
	if (timerx == TIM12 || timerx == TIM13 || timerx == TIM14) {
		PortManager::pinAFConfig(this->portx, this->pinx, GPIO_AF_TIM12);
	}

	TimerManager::init(timerx, prescaler - 1, length);
	TimerManager::OCinit(timerx, channel, TIM_OCMode_PWM2,
			TIM_OutputState_Enable, TIM_OutputNState_Disable, pulse,
			TIM_OCPolarity_High, TIM_OCPolarity_Low, TIM_OCIdleState_Reset,
			TIM_OCNIdleState_Reset);
//	TimerManager::enable(timerx);
}

void PWM::setPeriod(float32_t period_us) {
	this->period_usec = period_us;
	this->update();
}

void PWM::setWidth(float32_t width) {
	this->width_usec = width;
	this->update();
}

void PWM::setWidthPercents(float32_t width) {
	this->width_usec = this->period_usec * width / 100.0;
	this->update();
}

void PWM::preload(float32_t preload_usec) {
	if (preload_usec < width_usec) {
		uint32_t prescaler = (uint32_t) (1
				+ (period_usec * 168.0) / (65535.0 * 2.0));
		uint32_t length = (uint32_t) ((period_usec * 168.0) / (2.0 * prescaler));
		uint32_t preload = (uint32_t) ((preload_usec * length) / period_usec);
		TimerManager::setCounter(timerx, preload);
	} else {
		uint32_t prescaler = (uint32_t) (1
				+ (period_usec * 168.0) / (65535.0 * 2.0));
		uint32_t length = (uint32_t) ((period_usec * 168.0) / (2.0 * prescaler));
		uint32_t preload = (uint32_t) ((width_usec * length) / period_usec);
		TimerManager::setCounter(timerx, preload - 1);
	}

}

void PWM::stop() {
	TimerManager::disable(this->timerx);
}

void PWM::start() {
	TimerManager::enable(this->timerx);
}

float32_t PWM::getPeriodUsec() {
	return period_usec;
}

float32_t PWM::getWidthUsec() {
	return width_usec;
}

void PWM::update() {
	uint32_t prescaler =
			(uint32_t) (1 + (period_usec * 168.0) / (65535.0 * 2.0));
	uint32_t length = (uint32_t) ((period_usec * 168.0) / (2.0 * prescaler));
	uint32_t pulse = (uint32_t) ((width_usec * length) / period_usec);

	TimerManager::setPrescaler(timerx, prescaler - 1);
	TimerManager::setAutoreload(timerx, length);
	TimerManager::setCompare(timerx, channel, pulse);
}

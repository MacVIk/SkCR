/*
 * UARTuserInit.h
 *
 *  Created on: 03.04.2019
 *      Author: Taras.Melnik
 */

#ifndef LIB_DRIVERUSART_H_
#define LIB_DRIVERUSART_H_

#include "stm32f4xx.h"

#define MAX_MESSEGE_LENGTH_RX   32
#define MAX_MESSEGE_LENGTH_TX   32

class DriverUsart {
public:
        DriverUsart();
	virtual ~DriverUsart();

	/* Features initialization */
        void init_usart(GPIO_TypeDef* GPIOx, USART_TypeDef* USARTx, bool rsStatus);
        void gpioSwitchInit(GPIO_TypeDef* GPIOx, uint16_t swPin);
//	void UARTx_IRQnFunc();
	/* Interface */
	void usart_send(uint8_t* sendArr, uint8_t length);
	uint8_t usart_receive_byte();
	uint8_t usart_get_rx_arr_size();
	void usart_stop_receiving();

//protected:
	/* ToDO fix this */
	uint8_t usartRxArr[MAX_MESSEGE_LENGTH_RX];
	uint8_t usartTxArr[MAX_MESSEGE_LENGTH_TX];
private:
        /* Peripheral initialization */
        void init_gpio(GPIO_TypeDef* GPIOx, uint16_t rxPin, uint16_t txPin, uint8_t af);
        void init_dma(USART_TypeDef* USARTx);

//	USART_TypeDef* nvicUart;

	/* Peripheral for user functions */
	GPIO_TypeDef* nvicPort;
	uint16_t nvicPin;
	DMA_Stream_TypeDef* txStream;
	uint8_t pointer;

};

#endif /* LIB_DRIVERUSART_H_ */

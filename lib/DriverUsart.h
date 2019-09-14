/*
 * UARTuserInit.h
 *
 *  Created on: 03.04.2019
 *      Author: Taras.Melnik
 */

#ifndef LIB_DRIVERUSART_H_
#define LIB_DRIVERUSART_H_

#include "stm32f4xx.h"

#define MAX_MESSEGE_LENGTH_RX   24
#define MAX_MESSEGE_LENGTH_TX   24

struct UsartOptions {
        GPIO_TypeDef* nvicPort;
        DMA_Stream_TypeDef* txStream;
        uint16_t nvicPin;
        uint8_t pointer;

        /* Should refer to the specific usart arr */
        uint8_t* RxArr;
        uint8_t* TxArr;
};

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
	uint8_t usart_get_recieved_size();
	void usart_stop_receiving();

protected:
	/* ToDO fix this */
	UsartOptions usart;

private:
        /* Peripheral initialization */
        void init_gpio(GPIO_TypeDef* GPIOx, uint16_t rxPin, uint16_t txPin, uint8_t af);
        void init_dma(USART_TypeDef* USARTx);

	/* Protection from multiple USART initialization */
        static uint8_t usart_3_Rx_Arr[MAX_MESSEGE_LENGTH_RX];
        static uint8_t usart_3_Tx_Arr[MAX_MESSEGE_LENGTH_RX];
        static uint8_t uart_4_Rx_Arr[MAX_MESSEGE_LENGTH_RX];
        static uint8_t uart_4_Tx_Arr[MAX_MESSEGE_LENGTH_RX];
        static uint8_t usart_6_Rx_Arr[MAX_MESSEGE_LENGTH_RX];
        static uint8_t usart_6_Tx_Arr[MAX_MESSEGE_LENGTH_RX];

};

#endif /* LIB_DRIVERUSART_H_ */

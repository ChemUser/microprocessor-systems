/*
 * uart.h
 *
 *  Created on: Jan 7, 2025
 *      Author: Arseni Skrabneu
 */

#ifndef SRC_UART_H_
#define SRC_UART_H_

/*	Configuration and initialisation
 * 	of the LPUART interface
 */
int LPUART_init(void);

//	Send single character
int LPUART_SendChar(char data);

//	Receive single character
int LPUART_ReceiveChar(char* data);

//	Send string
int LPUART_SendString(char* data);

#endif /* SRC_UART_H_ */

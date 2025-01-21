/*
 * uart.c
 *
 *  Created on: Jan 7, 2025
 *      Author: Arseni Skrabneu
 */

#include "uart.h"
#include "main.h"

typedef struct
{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t CR3;
	volatile uint32_t BRR;
	volatile uint32_t RESERVED0;
	volatile uint32_t RESERVED1;
	volatile uint8_t RQR;
	volatile uint8_t RESERVED2;
	volatile uint16_t RESERVED3;
	volatile uint32_t ISR;
	volatile uint32_t ICR;
	volatile uint8_t RDR;
	volatile uint8_t RESERVED4;
	volatile uint16_t RESERVED5;
	volatile uint8_t TDR;
	volatile uint8_t RESERVED6;
	volatile uint16_t RESERVED7;

} LPUART;

#define APB1_BASE 0x40000000UL
#define LPUART_BASE (APB1_BASE + 0x8000UL)
#define LPUART1 ((LPUART*) LPUART_BASE)

#define WORD_7B	((0x1UL << 28)|(0x0UL << 12))
#define WORD_8B	((0x0UL << 28)|(0x0UL << 12))
#define WORD_9B	((0x0UL << 28)|(0x1UL << 12))

#define STOP_1B	(0x0UL << 12)
#define STOP_2B	(0x2UL << 12)

#define TXE 	((LPUART1->ISR & (0x1UL << 7)) >> 7)
#define TC		((LPUART1->ISR & (0x1UL << 6)) >> 6)
#define RXNE 	((LPUART1->ISR & (0x1UL << 5)) >> 5)


/*	Configuration and initialisation
 * 	of the LPUART interface
 */
int LPUART_init(void)
{
	LPUART1->CR1 |= WORD_8B;
	LPUART1->CR2 |= STOP_1B;
	LPUART1->BRR = (256 * 4000000)/115200;
	LPUART1->CR1 |= 0x1UL;
	LPUART1->CR1 |= (0x1UL << 3);
	LPUART1->CR1 |= (0x1UL << 2);
	return 0;
}

//	Send single character
int LPUART_SendChar(char data)
{
	while(TXE == 0){continue;}
	LPUART1->TDR = data;
	while(TC == 0){continue;}
	return 0;
}

//	Receive single character
int LPUART_ReceiveChar(char* data)
{
	while(RXNE != 1){continue;}
	*data = LPUART1->RDR;
	return 0;
}

//	Send string
int LPUART_SendString(char* data) //Every string should be null-terminated
{
	char pos = 0;
	while(*(data + pos) != '\0')
	{
		LPUART_SendChar(*(data + pos));
		pos++;
	}
	return 0;
}

void lpuart_status()
{
	uint32_t temp;
	uint32_t baudrate;
	temp = ((LPUART1->CR1 & (0x1UL << 28) >> 27))|((LPUART1->CR1 & (0x1UL << 12)) >> 12);
	LPUART_SendString("LPUART status:\n\r\t\0");
	LPUART_SendString("Baudrate: 115200\n\r\t\0");
	switch(temp)
	{
		case 0x10UL:
			LPUART_SendString("7 data bits, \0");
			break;
		case 0x00UL:
			LPUART_SendString("8 data bits, \0");
			break;
		case 0x01UL:
			LPUART_SendString("9 data bits, \0");
			break;
	}
	temp = (LPUART1->CR1 & (0x3UL << 9)) >> 9;
	switch(temp)
	{
		case 0x10UL:
			LPUART_SendString("even parity.\n\r\t\0");
			break;
		case 0x11UL:
			LPUART_SendString("odd parity.\n\r\t\0");
			break;
		default:
			LPUART_SendString("no parity.\n\r\t\0");
			break;
	}
	LPUART_SendString("Transmit register: \"\0");
	LPUART_SendChar(LPUART1->TDR);
	LPUART_SendString("\"\n\r\tReceive register: \"\0");
	LPUART_SendChar(LPUART1->RDR);
	LPUART_SendString("\"\n\r\tCalculated baudrate: \0");


	baudrate = (uint32_t) (256*4000000/LPUART1->BRR);
	char buff[10];
	unsigned i;
	for(i = 0; i < 10; i++){ buff[i] = '\0'; }
	i = 0;
	while(baudrate != 0)
	{
		buff[i] = baudrate%10 + 48;
		baudrate = (baudrate - baudrate%10)/10;
		i++;
	}
	i--;
	for(; i >= 0; i--){ LPUART_SendChar(buff[i]); }
	LPUART_SendString("\n\r\0");
}

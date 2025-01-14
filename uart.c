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

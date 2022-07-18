/*
 * Console.c
 *
 *  Created on: Nov 17, 2021
 *      Author: Kunal
 */

#include "Console.h"
 void Console_Init(int baudrate)
{

		RCC -> APB2ENR |= RCC_APB2ENR_USART1EN;
		RCC -> AHB1ENR   |= RCC_AHB1ENR_GPIOAEN;
		GPIOA -> MODER   |= ((0xC0  & 0x8C)>>6) << (2 * 9);
		GPIOA -> OTYPER  |= ((0x30  & 0x8C)>>4) << (1 * 9);
		GPIOA -> OSPEEDR |= ((0x0C  & 0x8C)>>2) << (2 * 9);
		GPIOA -> PUPDR   |= ((0x03  & 0x8C)>>0) << (2 * 9);
		GPIOA -> AFR[1] |= ( 7 << (4 * (9 - 8)));
		USART1 ->CR1 |= USART_CR1_UE;
		USART1->BRR |= (int)(84000000 / (16 * baudrate)) << 4;
		USART1 ->CR1 |= USART_CR1_TE ;
		USART1 ->CR1 |= USART_CR1_RE ;

}


 void printConsole(char *msg, ...)
{

char buff[10000];
//	#ifdef DEBUG_UART

	va_list args;
	va_start(args, msg);
	vsprintf(buff, msg, args);

	for(int i = 0; i<= strlen(buff)-1; i++)
	{
		USART1 -> DR = buff[i];
		while (!(USART1->SR & USART_SR_TXE));
	}

//	#endif
}

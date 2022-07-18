/*
 * GPIO.c
 *
 *  Created on: Nov 17, 2021
 *      Author: Kunal
 */


#include "GPIO.h"


void GPIO_Pin_Setup(GPIO_TypeDef *Port, uint8_t pin, uint8_t function, uint8_t alternate_function)
{
	if(Port == GPIOA)
	{
		RCC -> AHB1ENR   |= RCC_AHB1ENR_GPIOAEN;
	}else if(Port == GPIOB)
	{
		RCC -> AHB1ENR   |= RCC_AHB1ENR_GPIOBEN;
	}else if(Port == GPIOC)
	{
		RCC -> AHB1ENR   |= RCC_AHB1ENR_GPIOCEN;
	}else if(Port == GPIOD)
	{
		RCC -> AHB1ENR   |= RCC_AHB1ENR_GPIODEN;
	}else if(Port == GPIOE)
	{
		RCC -> AHB1ENR   |= RCC_AHB1ENR_GPIOEEN;
	}

	Port -> MODER   |= ((0xC0  & function)>>6) << (2 * pin);
	Port -> OTYPER  |= ((0x30  & function)>>4) << (1 * pin);
	Port -> OSPEEDR |= ((0x0C  & function)>>2) << (2 * pin);
	Port -> PUPDR   |= ((0x03  & function)>>0) << (2 * pin);

	if(pin < 8)Port -> AFR[0] |= ( alternate_function << (4 * (pin)));
	else	   Port -> AFR[1] |= ( alternate_function << (4 * (pin - 8)));

	if (alternate_function == NONE) {}
}

void GPIO_Pin_High(GPIO_TypeDef *Port, int pin)
{
	Port -> BSRR |= 1 << pin;
}


void GPIO_Pin_Low(GPIO_TypeDef *Port, int pin)
{
	Port -> BSRR |= GPIO_BSRR_BR0 << pin;
}


void GPIO_Interrupt_Setup(int pin, int edge_select)
{
	EXTI ->IMR |= 1 << pin;
	switch (edge_select) {
		case 0:
			EXTI ->RTSR |= 1 << pin;
			break;
		case 1:
			EXTI ->FTSR |= 1 << pin;
			break;
		case 2:
			EXTI ->RTSR |= 1 << pin;
			EXTI ->FTSR |= 1 << pin;
			break;
	}


}

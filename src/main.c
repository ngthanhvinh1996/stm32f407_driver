/*
 * main.c
 *
 *  Created on: Jun 5, 2021
 *      Author: vinhkuto
 */


 #include "stm32f407xx.h"

void EXTI0_IRQHandler(void);

int main(void)
{

	while(1)
	{

	}

	return 0;
}


void EXTI0_IRQHandler(void)
{
	//Handle the interrupt
	GPIO_IRQHandling(0);
}

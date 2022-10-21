/*
 * 005_button_interrupt.c
 *
 *  Created on: Jun 6, 2021
 *      Author: vinhkuto
 */


#include "stm32f407xx_gpio_driver.h"
#include "string.h"

void delay(void)
{
	for( uint32_t i = 0; i < 500000/2; i++ );
}

int main(void)
{

	GPIO_Handle_t GPIOLed, GPIOBtn;

	memset(&GPIOLed, 0, sizeof(GPIOLed));
	memset(&GPIOBtn, 0, sizeof(GPIOBtn));

	GPIOBtn.pGPIOx = GPIOD;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl( GPIOD, ENABLE );

	GPIO_Init( &GPIOBtn );

	GPIOLed.pGPIOx = GPIOD;
	GPIOLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	GPIOLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIOLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIOLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;

	GPIO_PeriClockControl( GPIOD, ENABLE );

	GPIO_Init( &GPIOLed );

	//IRQ configurations
	GPIO_IRQPriorityConfig( IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15 );
	GPIO_IRQInterruptConfig( IRQ_NO_EXTI9_5, ENABLE );

	while(1)
	{

	}
	return 0;
}


void EXTI9_5_IRQHandler(void)
{
	delay();
	GPIO_IRQHandling( GPIO_PIN_NO_5 );
	GPIO_ToggleOutputPin( GPIOD ,GPIO_PIN_NO_12 );
}

/*
 * 001_ToggleLed.c
 *
 *  Created on: Jun 2, 2021
 *      Author: Thanh Vinh
 */


#include "stm32f407xx_gpio_driver.h"

void delay(void)
{
	for( uint32_t i = 0; i < 500000; i++ );
}

int main(void)
{

	GPIO_Handle_t GPIOLed;

	GPIOLed.pGPIOx = GPIOD;
	GPIOLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	GPIOLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIOLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIOLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;

	GPIO_PeriClockControl( GPIOD, ENABLE );

	GPIO_Init( &GPIOLed );

	GPIOLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init( &GPIOLed );

	GPIOLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init( &GPIOLed );

	GPIOLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init( &GPIOLed );

	while(1)
	{
		GPIO_ToggleOutputPin( GPIOD ,GPIO_PIN_NO_12 );
		delay();
		GPIO_ToggleOutputPin( GPIOD ,GPIO_PIN_NO_13 );
		delay();
		GPIO_ToggleOutputPin( GPIOD ,GPIO_PIN_NO_14 );
		delay();
		GPIO_ToggleOutputPin( GPIOD ,GPIO_PIN_NO_15 );
		delay();
	}
	return 0;
}



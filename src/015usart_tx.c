/*
 * 015usart_tx.c
 *
 *  Created on: Jul 6, 2021
 *      Author: vinhkuto
 */

#include "stm32f407xx.h"

USART_Handle_t usart2_handle;
char msg[1024] = "UASRT Tx Testing...\n";

void delay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++)
	{

	}
}

void GPIO_ButtonInit(void)
{
    GPIO_Handle_t GPIOBtn;

    memset(&GPIOBtn, 0, sizeof(GPIOBtn));

	GPIOBtn.pGPIOx = GPIOD;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init( &GPIOBtn );
}

void USART2_GPIOInit(void)
{
	GPIO_Handle_t usart_gpio;

	usart_gpio.pGPIOx = GPIOA;
	usart_gpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	usart_gpio.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP; //Can't not use Open-drain
	usart_gpio.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	usart_gpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	usart_gpio.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_PIN_AF7;

	//USART2 TX
	usart_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
	GPIO_Init(&usart_gpio);

	//USART2 RX
	usart_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	GPIO_Init(&usart_gpio);
}

void USART2_Init(void)
{
	usart2_handle.pUSARTx = USART2;
	usart2_handle.USART_Config.USART_BaudRate = USART_STD_BAUD_115200;
	usart2_handle.USART_Config.USART_Mode = USART_MODE_ONLY_TX;
	usart2_handle.USART_Config.USART_WordLength = USART_WORDLENGTH_8BITS;
	usart2_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	usart2_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	usart2_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;

	USART_Init(&usart2_handle);
}

int main(void)
{
	GPIO_ButtonInit();

	USART2_GPIOInit();

	USART2_Init();

	USART_PeripheralControl(USART2, ENABLE);

	while(1)
	{
		//Wait till button is pressed
		while( ! GPIO_ReadFromInputPin(GPIOD, GPIO_PIN_NO_5));

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		USART_SendData(&usart2_handle, (uint8_t *)msg, strlen(msg));
	}

	return 0;
}

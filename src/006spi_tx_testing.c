/*
 * 006spi_tx_testing.c
 *
 *  Created on: Jun 16, 2021
 *      Author: vinhkuto
 */

#include "stm32f407xx.h"
#include <string.h>

/*
 * PB12 -> NSS
 * PB13 -> SCLK
 * PB14 -> MISO
 * PB15 -> MOSI
 * ALT F5
 */
void SPI_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_PIN_AF5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//MISO
	// SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	// GPIO_Init(&SPIPins);

	//NSS
	// SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	// GPIO_Init(&SPIPins);
}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2Handle;
	
	SPI2Handle.SPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_EN; // Software slave management enabled for NSS pin

	SPI_Init(&SPI2Handle);

}

int main(void)
{
	char user_data[] = "Hello World";

	//This function is used to initialize the GPIO pins to behave as SPI2 pins 
	SPI_GPIOInits();
	
	//This function is used to initialize the SPI2 peripheral parameters
	SPI2_Inits();

	//This make NSS signal internally high and avoids MODF error
	SPI_SSIConfig(SPI2, ENABLE);

	//Enable the SPI2 peripheral
	SPI_PeripheralControl(SPI2, ENABLE);

	//To send data
	SPI_SendData( SPI2, (uint8_t *)user_data, strlen(user_data) );

	//Disable the SPI2 peripheral
	SPI_PeripheralControl(SPI2, DISABLE);

	while(1)
	{

	}
	return 0;
}

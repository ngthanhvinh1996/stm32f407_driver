/*
 * 006spi_tx_testing.c
 *
 *  Created on: Jun 16, 2021
 *      Author: vinhkuto
 */

#include "stm32f407xx.h"
#include <string.h>


void delay(void)
{
	for( uint32_t i = 0; i < 500000/2; i++ );
}


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
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);
}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2Handle;
	
	SPI2Handle.SPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DIS; // Hardware slave management enabled for NSS pin

	SPI_Init(&SPI2Handle);

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


int main(void)
{
	char user_data[] = "Hello World";

    GPIO_ButtonInit();

	//This function is used to initialize the GPIO pins to behave as SPI2 pins 
	SPI_GPIOInits();
	
	//This function is used to initialize the SPI2 peripheral parameters
	SPI2_Inits();

	//This make NSS signal internally high and avoids MODF error
	// SPI_SSIConfig(SPI2, ENABLE);

    /*
    * Making SSOE 1 does NSS output enable
    * The NSS pin is automatically managed by the hardware
    * i.e when SPE = 1, NSS will be pulled to low
    * and NSS pin will be high when SPE = 0
    */
    SPI_SSOEConfig(SPI2, ENABLE);
    while(1)
    {
        while( ! GPIO_ReadFromInputPin(GPIOD, GPIO_PIN_NO_5) );

        delay();

        //Enable the SPI2 peripheral
        SPI_PeripheralControl(SPI2, ENABLE);
        //First send length data informantion
        uint8_t dataLen = strlen(user_data);
        SPI_SendData( SPI2, &dataLen, 1);
        //To send data
        SPI_SendData( SPI2, (uint8_t *)user_data, strlen(user_data) );

        while( SPI_GetFlagStatus(SPI2, SPI_FLAG_SR_BSY));

        //Disable the SPI2 peripheral
        SPI_PeripheralControl(SPI2, DISABLE);        
    }

	return 0;
}

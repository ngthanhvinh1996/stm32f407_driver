/*
 * 006spi_tx_testing.c
 *
 *  Created on: Jun 16, 2021
 *      Author: vinhkuto
 */

#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#include <string.h>

//Command code
#define COMMAND_LED_CTRL        0x50
#define COMMAND_SENSOR_READ     0x51
#define COMMAND_LED_READ        0x52
#define COMMNAD_PRINT           0x53
#define COMMNAD_ID_READ         0x54

#define LED_ON                  1
#define LED_OFF                 0

//Arduino analoh pins
#define ANALOG_PIN0             0
#define ANALOG_PIN1             1
#define ANALOG_PIN2             2
#define ANALOG_PIN3             3
#define ANALOG_PIN4             4

//Arduino led
#define LED_PIN                 9




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

uint8_t SPI_VerifyResponse(uint8_t ackbyte)
{
    if(ackbyte == 0xF5)
    {
        //ACK
        return 1;
    }

    return 0;
}

int main(void)
{
	//char user_data[] = "Hello World";

    uint8_t dummy_write = 0xFF;
    uint8_t dummy_read;

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

        //1. CMD_LED_CTRL   <pin no(1)>     <value(1)>
        uint8_t cmdcode = COMMAND_LED_CTRL;
        uint8_t ackbyte = 0x00;
        uint8_t agrs[2];

        SPI_SendData(SPI2, &cmdcode, 1);

        //Do dummy read to clear of the RXNE
        SPI_ReceiveData(SPI2, &dummy_read, 1);

        //Send some dummy bits (1byte) to fetch the response from the slave
        SPI_SendData(SPI2, &dummy_write, 1);

        //Read the ACK byte receive
        SPI_ReceiveData(SPI2, &ackbyte, 1);

        if(SPI_VerifyResponse(ackbyte) == TRUE)
        {
            //Send arguments
            agrs[0] = LED_PIN;
            agrs[1] = LED_ON;
            SPI_SendData(SPI2, (uint8_t *)agrs, 2);
        }
        //end of COMMAND_LED_CTRL

        //2. CMD_SENDOR_READ    <analog pin number(1)>

        while( ! GPIO_ReadFromInputPin(GPIOD, GPIO_PIN_NO_5) );

        delay();

        cmdcode = COMMAND_LED_READ;

        //Send command
        SPI_SendData(SPI2, &cmdcode, 1);

        //Do dummy read to clear of the RXNE
        SPI_ReceiveData(SPI2, &dummy_read, 1);

        //Send some dummy bits (1bute) to fetch the response from the slave
        SPI_SendData(SPI2, &dummy_write, 1);

        //Read the ACK byte receive 
        SPI_ReceiveData(SPI2, &ackbyte, 1);

        if(SPI_VerifyResponse(ackbyte) == TRUE)
        {
            //send arguments
            agrs[0] = ANALOG_PIN0;
            SPI_SendData(SPI2, (uint8_t *)agrs, 1);

            delay();
            
            //Do dummy read to clear of the RXNE
            SPI_ReceiveData(SPI2, &dummy_read, 1);

            //Send some dummy bits (1 byte) fetch the response from the slave
            SPI_SendData(SPI2, &dummy_write, 1);

        }



        uint8_t analog_read;
        SPI_ReceiveData(SPI2, &analog_read, 1);

        while( SPI_GetFlagStatus(SPI2, SPI_SR_BSY_FLAG));

        //Disable the SPI2 peripheral
        SPI_PeripheralControl(SPI2, DISABLE);        
    }

	return 0;
}

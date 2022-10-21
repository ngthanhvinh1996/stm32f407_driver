/*
 * 009spi_message_rcv_it.c
 *
 *  Created on: Jun 19, 2021
 *      Author: vinhkuto
 */

/*
 * This applications receives the prints the user message received from the Arduino peripheral in SPI interrupt mode
 * User sends the message through Arduino IDE's serial monitor tool
 * Monitor the message received in the SSW ITM data console
 */

/*
 * Note: Follow the instructions to test the this code
 * 1. Download this code on to STM32 board, acts as Master
 * 2. Download Slave code (003SPISlaveUARTReadOverSPI.ino) on to Arduino board (Slave)
 * 3. Reset both the boards
 * 4. Enable SSW ITM data console to see the message
 * 5. Open Arduino IDE serial monitor tool
 * 6. Type anything and send the message ( Make sure that in the serial monitor tool line ending set to carriage return
*/

#include <string.h>
#include <stdio.h>
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"

extern void initialise_monitor_handles();
SPI_Handle_t SPI2handle;

#define MAX_LEN 500

char RcvBuff[MAX_LEN];

volatile char ReadByte;

volatile uint8_t RcvStop = 0;

/* This flag will be set in the interrupt handler of the Arduino interrupt GPIO*/
volatile uint8_t dataAvailable = 0;

void delay()
{
	for(uint32_t i = 0; i < 500000/2; i++);
}

/*
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * PB13 --> SPI_SCLK
 * PB12 --> SPI2_NSS
 * ALT functions mode: 5
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
	 SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	 GPIO_Init(&SPIPins);

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
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV32;
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DIS; // Hardware slave management enabled for NSS pin

	SPI_Init(&SPI2Handle);

}

void GPIO_ButtonInit(void)
{
    GPIO_Handle_t GPIOBtn, GPIOLed;

    memset(&GPIOBtn, 0, sizeof(GPIOBtn));
    memset(&GPIOLed, 0, sizeof(GPIOLed));

    GPIOLed.pGPIOx = GPIOA;
    GPIOLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
    GPIOLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
    GPIOLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init( &GPIOLed );

	GPIOBtn.pGPIOx = GPIOD;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIOBtn.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;

	GPIO_Init( &GPIOBtn );
}

/* This function configures the GPIO Pin over which SPI peripheral issues data available interrupt */
void Slave_GPIO_InterruptPinInit(void)
{
	GPIO_Handle_t spiInitPin;
	memset(&spiInitPin, 0, sizeof(spiInitPin));

	//This is led GPIO configuration
	spiInitPin.pGPIOx = GPIOD;
	spiInitPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	spiInitPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	spiInitPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	spiInitPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&spiInitPin);

	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);

}

int main(void)
{

	uint8_t dummy = 0xFF;

	initialise_monitor_handles();

	Slave_GPIO_InterruptPinInit();

	//This function is used to initialize the GPIO Pins to behave as SPI pins
	SPI_GPIOInits();

	//This function is used to initialize the SPI2 peripheral parameters
	SPI2_Inits();

	/*
	 * making SSOE 1 does NSS output enable
	 * The NSS pin is automatically managed by hardware
	 * i.e when SPE = 1, NSS will be pulled to low
	 * and NSS pin will be high when SPE = 0
	 */
	SPI_SSOEConfig(SPI2, ENABLE);

	SPI_IRQInterruptConfig(IRQ_NO_SPI2, ENABLE);

	while(1)
	{
		RcvStop = 0;

		while(!dataAvailable); //wait till data available interrupt from transmitter device (slave)

		GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, DISABLE);

		//enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		while(!RcvStop)
		{
			/*fetch the data from the SPI peripheral byte by byte in interrupt mode */
			while( SPI_SendDataIT(&SPI2handle, &dummy, 1) == SPI_BUSY_IN_TX);
			while( SPI_ReceiveDataIT(&SPI2handle, &ReadByte, 1) == SPI_BUSY_IN_RX);
		}

		//confirm SPI is not busy
		while( SPI_GetFlagStatus(SPI2, SPI_FLAG_SR_BSY));

		//Disable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, DISABLE);

		printf("Rcvd data = %s\n", RcvBuff);

		dataAvailable = 0;

		GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);

	}

	return 0;
}


//Runs when a data byte is received from the peripheral over SPI
void SPI2_IRQHandler(void)
{
	SPI_IRQHandling(&SPI2handle);
}


void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvt)
{
	static uint32_t i = 0;
	/* In the RX complete event, copy data in to rcv buffer. '\0' indicates end of message(rcvStop = 1) */
	if(AppEvt == SPI_EVENT_RX_CMPLT)
	{
		RcvBuff[i++] = ReadByte;
		if(ReadByte == '\0' || ( i == MAX_LEN ))
		{
			RcvStop = 1;
			RcvBuff[i - 1] = '\0';
			i = 0;
		}
	}
}


/* Slave data available interrupt handler*/
void EXTI9_5_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_NO_6);
	dataAvailable = 1;
}







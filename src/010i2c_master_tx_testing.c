
#include "stm32f407xx.h"
#include <string.h>
#include <stdio.h>

#define MY_ADDR     0x61
#define SLAVE_ADDR  0x68

I2C_Handle_t I2C1Handle;

//some data
uint8_t some_data[] = "We are testing I2C master Tx\n";

void delay(void)
{
    for(uint32_t i = 0; i < 500000/2; i++);
}

/*
 * PB6 -> SCL
 * PB9 -> SDA
 * ALT F4
 */

void I2C1_GPIOInits(void)
{
	GPIO_Handle_t I2CPins;

    I2CPins.pGPIOx = GPIOB;
    I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
    I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
    I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_PIN_AF4;

    //SCL
    I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
    GPIO_Init(&I2CPins);

    //SDA
    I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
    GPIO_Init(&I2CPins);


}

void I2C1_Inits(void)
{
    I2C1Handle.pI2Cx = I2C1;

    I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
    I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
    I2C1Handle.I2C_Config.I2C_AddreesMode = I2C_ADD_MODE_7BIT;
    I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
    I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

    I2C_Init(&I2C1Handle);
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

    GPIO_ButtonInit();

    //Configure GPIO I2C
    I2C1_GPIOInits();

    //Configure I2C peripheral
    I2C1_Inits();

    //Enable I2C peripheral
    I2C_PeripheralControl(I2C1Handle.pI2Cx, ENABLE);

    while(1)
    {
        //Wait till button is press
        while( !GPIO_ReadFromInputPin(GPIOD, GPIO_PIN_NO_5));

        //to avoid button de-boucing related issues 200ms of delay
        delay();

        //Send some data to the slave
        I2C_MasterSendData(&I2C1Handle, (uint8_t *)some_data, strlen((char *)some_data), SLAVE_ADDR, I2C_DISABLE_SR);
    }

    return 0;
}

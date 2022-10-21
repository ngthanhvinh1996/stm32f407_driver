
#include "stm32f407xx.h"
#include <string.h>
#include <stdio.h>

extern void initialise_monitor_handles();

#define MY_ADDR     0x61
#define SLAVE_ADDR  0x68

I2C_Handle_t I2C1Handle;

//rcv buffer
uint8_t rcv_buf[32];

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
	uint8_t command_code;
	uint8_t length_data;

	initialise_monitor_handles();

	printf("\nApplication is running\n");

    GPIO_ButtonInit();

    //Configure GPIO I2C
    I2C1_GPIOInits();

    //Configure I2C peripheral
    I2C1_Inits();

    //Enable I2C peripheral
    I2C_PeripheralControl(I2C1Handle.pI2Cx, ENABLE);

    //ACK bit is made 1 after PE=1
    I2C_ManageAcking(I2C1Handle.pI2Cx, I2C_ACK_ENABLE);

    while(1)
    {
        //Wait till button is press
        while( !GPIO_ReadFromInputPin(GPIOD, GPIO_PIN_NO_5));

        //to avoid button de-boucing related issues 200ms of delay
        delay();

        command_code = 0x51;

        I2C_MasterSendData(&I2C1Handle, &command_code, 1, SLAVE_ADDR, I2C_ENABLE_SR);

        I2C_MasterReceiveData(&I2C1Handle, &length_data, 1, SLAVE_ADDR, I2C_ENABLE_SR);

        command_code = 0x52;

        I2C_MasterSendData(&I2C1Handle, &command_code, 1, SLAVE_ADDR, I2C_ENABLE_SR);

        I2C_MasterReceiveData(&I2C1Handle, rcv_buf, length_data, SLAVE_ADDR, I2C_DISABLE_SR);
    }

    rcv_buf[length_data+1] = '\0';

    printf("Data: %s", rcv_buf);
    return 0;
}

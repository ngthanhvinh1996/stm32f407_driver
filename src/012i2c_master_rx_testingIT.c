
#include "stm32f407xx.h"
#include <string.h>
#include <stdio.h>

extern void initialise_monitor_handles();

#define MY_ADDR     0x61
#define SLAVE_ADDR  0x68

I2C_Handle_t I2C1Handle;

//Flag variable
uint8_t rxComplt = RESET;

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

    //I2C IRQ configurations
    I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
    I2C_IRQInterruptConfig(IRQ_NO_I2C2_ER, ENABLE);

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

        while(I2C_MasterSendDataIT(&I2C1Handle, &command_code, 1, SLAVE_ADDR, I2C_ENABLE_SR) != I2C_READY );

        while(I2C_MasterReceiveDataIT(&I2C1Handle, &length_data, 1, SLAVE_ADDR, I2C_ENABLE_SR) != I2C_READY );

        command_code = 0x52;

        while(I2C_MasterSendDataIT(&I2C1Handle, &command_code, 1, SLAVE_ADDR, I2C_ENABLE_SR) != I2C_READY );

        while(I2C_MasterReceiveDataIT(&I2C1Handle, rcv_buf, length_data, SLAVE_ADDR, I2C_DISABLE_SR) != I2C_READY );
    }

    rxComplt = RESET;

    //Wait till rx cpmpletes
    while( rxComplt != SET )
    {
        
    }

    rcv_buf[length_data+1] = '\0';

    printf("Data: %s", rcv_buf);

    rxComplt = RESET; 
    return 0;
}

void I2C1_ER_IRQHandler(void)
{
    I2C_ER_IRQHandling(&I2C1Handle);
}

void I2C1_EV_IRQHandler(void)
{
    I2C_EV_IRQHandling(&I2C1Handle);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEvt)
{
    if(AppEvt == I2C_EV_TX_CMPLT)
    {
        printf("Tx is completed\n");
    }
    else if(AppEvt == I2C_EV_RX_CMPLT)
    {
        printf("RX is completed\n");
        rxComplt = SET;
    }
    else if(AppEvt == I2C_ERROR_AF)
    {
        printf("Error: ACK failure\n");
        //In master ACK failure happens when slave fails to send ACK for the byte
        //Sent from the master.
        I2C_CloseSendData(pI2CHandle); 

        //Generate the STOP Condition to release the bus
        I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

        //Hang in infinite loop
        while(1);
    }
}
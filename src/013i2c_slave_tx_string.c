
#include "stm32f407xx.h"
#include <string.h>
#include <stdio.h>

#define SLAVE_ADDR  0x68
#define MY_ADDR     SLAVE_ADDR

#define I2C_CMD_SEND_LENGTH 0x51
#define I2C_CMD_SEND_DT     0x52

I2C_Handle_t I2C1Handle;

//rcv buffer
uint8_t Tx_buf[] = "STM32 Slave mode testing..";

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

    //I2C IRQ configurations
    I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
    I2C_IRQInterruptConfig(IRQ_NO_I2C2_ER, ENABLE);

    I2C_SlaveEnableDisableCallbackEvents(I2C1Handle.pI2Cx, ENABLE);

    //Enable I2C peripheral
    I2C_PeripheralControl(I2C1Handle.pI2Cx, ENABLE);

    //ACK bit is made 1 after PE=1
    I2C_ManageAcking(I2C1Handle.pI2Cx, I2C_ACK_ENABLE);

    while(1)
    {
        
    }


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
    static uint8_t commandCode = 0;
    static uint8_t Cnt = 0;

    if(AppEvt == I2C_EV_DATA_REQ)
    {
        //Master wants some data, slave has to send it
        if(commandCode == I2C_CMD_SEND_LENGTH)
        {
            //Send the length information to the master
            I2C_SlaveSendData(pI2CHandle->pI2Cx, strlen((char *)Tx_buf));
        }
        else if(commandCode == I2C_CMD_SEND_DT)
        {
            //Send the contents of Tx_buf
            I2C_SlaveSendData(pI2CHandle->pI2Cx, Tx_buf[Cnt++]);
        }
    }
    else if(AppEvt == I2C_EV_DATA_RCV)
    {
        //Data is waiting for the slave to read, slave has to read it
        commandCode = I2C_SlaveReceiveData(pI2CHandle->pI2Cx);
    }
    else if(AppEvt == I2C_ERROR_AF)
    {
        //This happens onlu during slave Txing.
        //Master has sent the NACK, so slave should understand that master doesn't need more data
        commandCode = 0xFF;
        Cnt = 0;
    }
    else if(AppEvt == I2C_EV_STOP)
    {
        //This happens only during slave reception.
        //Master has ended the I2C communication with the slave.
    }
}

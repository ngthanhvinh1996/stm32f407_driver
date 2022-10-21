/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: Jun 21, 2021
 *      Author: vinhkuto
 */

#include "stm32f407xx.h"


static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);
/*****************************************************************************************
 * 						APIs supported by this driver
 * 			For more information about the APIs check the function definition
 *****************************************************************************************/

/*
 * Peripheral Clock Setup
 */

/****************************************************************
 *@fn				: I2C_PeriClockControl
 *
 *@brief			: This function enables or disables peripheral clock for the given I2C peripheral
 *
 *@param[in]		: Base address of the I2C peripheral
 *@param[in]		: ENABLE or DISABLE macros
 *@param[in]		:
 *@param[in]
 *
 *@return			: None
 *
 *@Note				: None
 ****************************************************************/
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDis)
{
	if( EnOrDis == ENABLE )
	{
		if( pI2Cx == I2C1 )
		{
			I2C1_PCLK_EN();
		}
		else if( pI2Cx == I2C2 )
		{
			I2C2_PCLK_EN();
		}
		else if( pI2Cx == I2C3 )
		{
			I2C3_PCLK_EN();
		}
	}
	else
	{
		if( pI2Cx == I2C1 )
		{
			I2C1_PCLK_DIS();
		}
		else if( pI2Cx == I2C2 )
		{
			I2C2_PCLK_DIS();
		}
		else if( pI2Cx == I2C3 )
		{
			I2C3_PCLK_DIS();
		}
	}
}

/****************************************************************
 *@fn				: I2C_PeripheralControl
 *
 *@brief			: This function enables or disables I2C peripheral
 *
 *@param[in]		: Base address of the I2C peripheral
 *@param[in]		: ENABLE or DISABLE macros
 *@param[in]		:
 *@param[in]
 *
 *@return			: None
 *
 *@Note				: None
 ****************************************************************/
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDis)
{
	if( EnOrDis == ENABLE )
	{
		pI2Cx->CR1 |= ( 1 << I2C_CR1_PE );
	}
	else
	{
		pI2Cx->CR1 &= ~( 1 << I2C_CR1_PE );
	}
}

/****************************************************************
 *@fn				: RCC_GetPLLOutClock
 *
 *@brief			: This function get PLL clock of RCC register
 *
 *@param[in]		: 
 *@param[in]		: 
 *@param[in]		:
 *@param[in]
 *
 *@return			: 0 (This is function not required in this exercise)
 *
 *@Note				: None
 ****************************************************************/
//uint32_t RCC_GetPLLOutClock(void)
//{
//	return 0;
//}

/****************************************************************
 *@fn				: RCC_GetPCLK1Value
 *
 *@brief			: This function get PCLK1 clock of RCC
 *
 *@param[in]		: 
 *@param[in]		: 
 *@param[in]		:
 *@param[in]
 *
 *@return			: PCLK1 clock of RCC
 *
 *@Note				: None
 ****************************************************************/
//uint32_t RCC_GetPCLK1Value(void)
//{
//	uint32_t pclk1;
//	uint32_t SystemClk;
//	uint8_t clksrc;
//	uint8_t temp;
//	uint8_t ahbp;
//	uint8_t apb1p;
//
//	clksrc = ((RCC->CFGR >> 2) & 0x03);
//
//	if(clksrc == 0)
//	{
//		SystemClk = 16000000;
//	}
//	else if(clksrc == 1)
//	{
//		SystemClk = 8000000;
//	}
//	else if(clksrc == 2)
//	{
//		SystemClk = RCC_GetPLLOutClock();
//	}
//
//	//For AHB Prescaler
//	temp = ((RCC->CFGR >> 4) & 0xF);
//
//	if(temp < 8 || temp > 16)
//	{
//		ahbp = 1;
//	}
//	else
//	{
//		ahbp = AHB_PreScaler[temp-8];
//	}
//
//	//For ABP1 Prescaler
//	temp = ((RCC->CFGR >> 10) & 0x07);
//
//	if(temp < 4){
//		apb1p = 1;
//	}
//	else
//	{
//		apb1p = APB1_PreScaler[temp-4];
//	}
//
//	//Calculate PCLK1
//	pclk1 = (SystemClk / ahbp) / apb1p;
//
//	return pclk1;
//
//}
/****************************************************************
 *@fn				: I2C_Init
 *
 *@brief			: This function initialization the I2C peripheral
 *
 *@param[in]		: pI2CHandle
 *@param[in]		:
 *@param[in]		:
 *@param[in]
 *
 *@return			: None
 *
 *@Note				: None
 ****************************************************************/
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0;

	//Enable the clock for the I2C peripheral
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	//ACK control bit
	tempreg |= pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK;
	pI2CHandle->pI2Cx->CR1 |= tempreg; 

	//Configure the FREQ field of CR2
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

	//Configure the Address mode (7 bit or 10 bit)
	tempreg = 0;
	tempreg |= pI2CHandle->I2C_Config.I2C_AddreesMode << I2C_OAR1_ADDMODE;
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//Program the device own address
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_AddreesMode == I2C_ADD_MODE_7BIT)
	{
		tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << I2C_OAR1_ADD1;
	}
	else if(pI2CHandle->I2C_Config.I2C_AddreesMode == I2C_ADD_MODE_10BIT)
	{
		tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << I2C_OAR1_ADD0;
	}
	tempreg |= ( 1 << 14 );
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//CCR calculation
	uint16_t ccr_value = 0;
	tempreg = 0;

	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		/* Mode is Standard mode */
 		//Configure I2C master mode selection => Standard mode
		tempreg &= ~(1 << I2C_CCR_FS);
		//Calculation CCR in Standard mode
		ccr_value = (RCC_GetPCLK1Value() / ( 2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		tempreg |= (ccr_value & 0xFFF);
	}
	else 
	{
		//Mode is Fast mode
		//Configure I2C master mode selection => Fast mode
		tempreg |= (1 << I2C_CCR_FS);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << I2C_CCR_DUTY);
		//Calculation CCR in Fast mode
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = (RCC_GetPCLK1Value() / ( 3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		else if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_16_9)
		{
			ccr_value = (RCC_GetPCLK1Value() / ( 25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		tempreg |= (ccr_value & 0xCFFF);
		pI2CHandle->pI2Cx->CCR = tempreg;
	}

	tempreg = 0;
	//Config the TRISE
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//Configure the TRISE in STD mode
		tempreg = (RCC_GetPCLK1Value() / 1000000U ) + 1;
	}
	else
	{
		//Configure the TRISE in Fast mode
		tempreg = (((RCC_GetPCLK1Value() * 300) / 1000000U) +1);
	}
	pI2CHandle->pI2Cx->TRISE |= (tempreg & 0x3F);

}

/****************************************************************
 *@fn				: I2C_DeInit
 *
 *@brief			: This function de-initialization the I2C peripheral
 *
 *@param[in]		: Base address of the I2C peripheral
 *@param[in]		:
 *@param[in]		:
 *@param[in]
 *
 *@return			: None
 *
 *@Note				: None
 ****************************************************************/
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if( pI2Cx == I2C1 )
	{
		I2C1_REG_RESET();
	}
	else if( pI2Cx == I2C2 )
	{
		I2C2_REG_RESET();
	}
	else if( pI2Cx == I2C3 )
	{
		I2C3_REG_RESET();
	}
}

/****************************************************************
 *@fn				: I2C_GenerateStartCondition
 *
 *@brief			: This function is generate START condition
 *
 *@param[in]		: 
 *@param[in]		:
 *@param[in]		:
 *@param[in]
 *
 *@return			: None
 *
 *@Note				: None
 ****************************************************************/
void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= ( 1 << I2C_CR1_START);

}

/****************************************************************
 *@fn				: I2C_GenerateStopCondition
 *
 *@brief			: This function is generate START condition
 *
 *@param[in]		: 
 *@param[in]		:
 *@param[in]		:
 *@param[in]
 *
 *@return			: None
 *
 *@Note				: None
 ****************************************************************/
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= ( 1 << I2C_CR1_STOP );
}

/****************************************************************
 *@fn				: I2C_GetFlagStatus
 *
 *@brief			: This function get flag for the I2C peripheral
 *
 *@param[in]		: I2C_RegDef_t
 *@param[in]		: FlagName
 *@param[in]		: 
 *@param[in]
 *
 *@return			: FLAG_SET or FLAG_RESET
 *
 *@Note				: None
 ****************************************************************/
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pSPIx, uint32_t FlagName)
{
	if( pSPIx->SR1 & FlagName )
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/****************************************************************
 *@fn				: I2C_ExecuteAddressPhaseWrite
 *
 *@brief			: This function execute Address phase write
 *
 *@param[in]		: I2C_RegDef_t
 *@param[in]		: SlaveAddr
 *@param[in]		: 
 *@param[in]
 *
 *@return			: None
 *
 *@Note				: None
 ****************************************************************/
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1); //SlaveAddr is slave address + r/nw bit=0
	pI2Cx->DR = SlaveAddr;
   
}

/****************************************************************
 *@fn				: I2C_ExecuteAddressPhaseRead
 *
 *@brief			: This function execute Address phase read
 *
 *@param[in]		: I2C_RegDef_t
 *@param[in]		: SlaveAddr
 *@param[in]		: 
 *@param[in]
 *
 *@return			: None
 *
 *@Note				: None
 ****************************************************************/
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= 1; //SlaveAddr is slave address + r/nw bit=1
	pI2Cx->DR = SlaveAddr;
   
}

/****************************************************************
 *@fn				: I2C_ClearADDRFlag
 *
 *@brief			: This function clear ADDR bit
 *
 *@param[in]		: SPI_RegDef_t
 *@param[in]		: 
 *@param[in]		: 
 *@param[in]
 *
 *@return			: None
 *
 *@Note				: None
 ****************************************************************/
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummy_read;
	//Check for device mode
	if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL ))
	{
		//Device is in master mode
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->RxSize == 1)
			{
				//First disable the ACK
				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

				//Clear the ADDR flag (read SR1, read SR2)
				dummy_read = pI2CHandle->pI2Cx->SR1;
				dummy_read = pI2CHandle->pI2Cx->SR2;
				(void)dummy_read;
			}
		}
		else
		{
			//Clear the ADDR flag (read SR1, read SR2)
			dummy_read = pI2CHandle->pI2Cx->SR1;
			dummy_read = pI2CHandle->pI2Cx->SR2;
			(void)dummy_read;
		}
	}
	else
	{
		//Device is in slave mode
		//Clear the ADDR flag (read SR1, read SR2)
		dummy_read = pI2CHandle->pI2Cx->SR1;
		dummy_read = pI2CHandle->pI2Cx->SR2;
		(void)dummy_read;
	}

}

/****************************************************************
 *@fn				: I2C_ManageAcking
 *
 *@brief			: This function manage ACK bit
 *
 *@param[in]		: SPI_RegDef_t
 *@param[in]		: I2C_ACK_ENABLE or I2C_ACK_DISABLE
 *@param[in]		: 
 *@param[in]
 *
 *@return			: None
 *
 *@Note				: None
 ****************************************************************/
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnOrDis)
{
	if(EnOrDis == I2C_ACK_ENABLE)
	{
		//Enable the ACK
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}
	else{
		//Disable the ACK
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}	
}
/****************************************************************
 *@fn				: I2C_MasterSendData
 *
 *@brief			: This function send data from master to slave for the I2C peripheral
 *
 *@param[in]		: 
 *@param[in]		:
 *@param[in]		:
 *@param[in]
 *
 *@return			: None
 *
 *@Note				: None
 ****************************************************************/
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	//1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. Confirm that start generation is completed by checking the SB flag in SR1
	//   Note: Untill SB is cleared SCL will be stretched (pulled to LOW)
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SR1_SB) );

	//3. Send the address of the slave with r/nw bit set to w(0) (total 8 bits)
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

	//4. Confirm that address phase completed by checking the ADDR flag in the SR1
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SR1_ADDR) );

	//5. Clear the ADDR flag according to its software sequence
	//   Note: Untill ADDR is cleared SCL will be stretched (pulled to LOW)
	I2C_ClearADDRFlag(pI2CHandle);

	//6. Send the data untill Len becomes 0
	while(Len > 0)
	{
		while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SR1_TXNE) ); //Wait till TXE is set
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}

	//7. When Len becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition
	//   Note: TXE=1, BTF=1, means that both SR and DR are empty and next transmission should begin
	//   when BTF=1 SCL will be stretched (pulled to LOW)
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SR1_TXNE) );

	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SR1_BTF) );

	//8. Generate STOP condition and master ceed not to wait for the completion of stop condition.
	//   Note: generating STOP, automatically clears the BTF
	if(Sr == I2C_DISABLE_SR)
	{
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	}

}

/****************************************************************
 *@fn				: I2C_MasterReceiveData
 *
 *@brief			: This function receive data from salve to master for the I2C peripheral
 *
 *@param[in]		: 
 *@param[in]		:
 *@param[in]		:
 *@param[in]
 *
 *@return			: None
 *
 *@Note				: None
 ****************************************************************/
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr)
{

	//1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. Confirm that START generation is completed by checking the SB flag in the SR1
	//	 Note: Untill SB is cleared SCL will be stretched (pulled to LOW)
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SR1_SB));

	//3. Send the address of the slave with r/nw bit set to R(1) (total 8 bits)
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

	//4. Wait until address phase is completed by checking the ADDR flag in the SR1
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SR1_ADDR));

	if(Len == 1)
	{
		//Disable Acking
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

		//Clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//Wait until RXNE becomes 1
		while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SR1_RXNE));

		//Generate STOP condition
		if(Sr == I2C_DISABLE_SR)
		{
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		//Read data in to buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;

	}

	//Produre to read data from slave when len > 1
	if(Len > 1)
	{
		//Clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//Read the data until Len becomes zero
		for(uint32_t i = Len; i > 0; i--)
		{
			//Wait until RXNE becomes 1
			while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SR1_RXNE));

			if(i == 2)	//If last 2 bytes are remaining
			{
				//Disable Acking
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

				//Generate STOP condition
				if(Sr == I2C_DISABLE_SR)
				{
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				}
			}

			//Read the data from data register in to buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;

			//Increment the buffer address
			pRxBuffer++;

		}
	}

	//Re-enable ACKing
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
	I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
	}
	
}

/****************************************************************
 *@fn				: I2C_MasterSendDataIT
 *
 *@brief			: This function send data from master to slave for the I2C peripheral interrupt
 *
 *@param[in]		: 
 *@param[in]		:
 *@param[in]		:
 *@param[in]
 *
 *@return			: State
 *
 *@Note				: None
 ****************************************************************/
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX) )
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to generate START condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);

	}

	return busystate;
}

/****************************************************************
 *@fn				: I2C_CloseReceiveData
 *
 *@brief			:
 *
 *@param[in]		:
 *@param[in]		:
 *@param[in]		:
 *@param[in]
 *
 *@return			: State
 *
 *@Note				: None
 ****************************************************************/
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN );

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN );

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
	}
}

/****************************************************************
 *@fn				: I2C_CloseSendData
 *
 *@brief			:
 *
 *@param[in]		:
 *@param[in]		:
 *@param[in]		:
 *@param[in]
 *
 *@return			: State
 *
 *@Note				: None
 ****************************************************************/
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN );

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN );

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;

}

/****************************************************************
 *@fn				: I2C_MasterReceiveDataIT
 *
 *@brief			: This function receive data from salve to master for the I2C peripheral interrupt
 *
 *@param[in]		: 
 *@param[in]		:
 *@param[in]		:
 *@param[in]
 *
 *@return			: State 
 *
 *@Note				: None
 ****************************************************************/
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busystate;

	busystate =  pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX) )
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len;	//RxSize is used in the ISR code to manage the data reception
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN );

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 <<  I2C_CR2_ITEVTEN );

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN );

	}
	return busystate;
}

/****************************************************************
 *@fn				: I2C_MasterHandleTXEInterrupt
 *
 *@brief			:
 *
 *@param[in]		:
 *@param[in]		:
 *@param[in]		:
 *@param[in]
 *
 *@return			: None
 *
 *@Note				: Interrupt handling for different I2C events (refer SR1)
 ****************************************************************/
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle)
{
	//We have to do the data transmission
	if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
	{
		if(pI2CHandle->TxLen > 0)
		{
			//1. Load the data in to DR
			pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

			//2. Decrement the TxLen
			pI2CHandle->TxLen--;

			//3. Increment the buffer address
			pI2CHandle->pTxBuffer++;
		}
	}
}

/****************************************************************
 *@fn				: I2C_MasterHandleRXNEInterrupt
 *
 *@brief			:
 *
 *@param[in]		:
 *@param[in]		:
 *@param[in]		:
 *@param[in]
 *
 *@return			: None
 *
 *@Note				: Interrupt handling for different I2C events (refer SR1)
 ****************************************************************/
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle)
{
	//We have to do the data reception
	if(pI2CHandle->RxSize == 1)
	{
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;

		pI2CHandle->RxLen--;
	}

	if(pI2CHandle->RxSize > 1)
	{
		if(pI2CHandle->RxLen == 2)
		{
			//Clear the ACK bit
			I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
		}

		//Read DR
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->pRxBuffer++;
		pI2CHandle->RxLen--;
	}

	if(pI2CHandle->RxLen == 0)
	{
		//Close the I2C data reception and notify the application

		//1. Generate the STOP condition
		if(pI2CHandle->Sr == I2C_DISABLE_SR)
		{
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		//2. Close the I2C Rx
		I2C_CloseReceiveData(pI2CHandle);

		//3. Notify the application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
	}
}

/****************************************************************
 *@fn				: I2C_EV_IRQHandling
 *
 *@brief			: 
 *
 *@param[in]		: 
 *@param[in]		:
 *@param[in]		:
 *@param[in]
 *
 *@return			: None 
 *
 *@Note				: Interrupt handling for different I2C events (refer SR1)
 ****************************************************************/
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	//Interrupt handling for both master and slave mode of a device
	uint8_t temp1;
	uint8_t temp2;
	uint8_t temp3;

	temp1 = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITEVTEN );
	temp2 = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITBUFEN );

	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_SB );
	
	//1. Handle For interrupt generated by SB event
	//	Note : SB flag is only applicable in Master mode
	if(temp1 && temp3)
	{
		//The interrupt is generated because of SB event
		//This block will not be executed in slave mode because for slave SB is always zero
		//In this block lets executed the address phase
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		} 
		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
	}
	
	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_ADDR );
	//2. Handle For interrupt generated by ADDR event 
	//Note : When master mode : Address is sent 
	//		 When Slave mode   : Address matched with own address
	if(temp1 && temp3)
	{
		//ADDR flag is set
		I2C_ClearADDRFlag(pI2CHandle);
	} 
	
	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_BTF );
	//3. Handle For interrupt generated by BTF(Byte Transfer Finished) event 
	if(temp1 && temp3)
	{
		//BTF flag is set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			//Make sure that TXE is also set
			if(pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_TXE ))
			{
				//BTF, TXE = 1
				if(pI2CHandle->TxLen == 0)
				{
					//1. Generate the STOP condition
					if(pI2CHandle->Sr == I2C_DISABLE_SR)
					{
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					}

					//2. Reset all the member elements of the handle structure.
					I2C_CloseSendData(pI2CHandle);

					//3. Notify the application about tranmission complete
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}
			}
		}
		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{

		}
	} 
	
	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_STOPF );
	//4. Handle For interrupt generated by STOPF event 
	// Note : Stop detection flag is applicable only slave mode . For master this flag will never be set
	if(temp1 && temp3)
	{
		//STOPF flag is set
		//Clear the STOPF (i.e 1) read SR1 2) write to CR1

		pI2CHandle->pI2Cx->CR1 |= 0x0000;

		//Notify the application that STOP is detected
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 <<I2C_SR1_TXE );	
	//5. Handle For interrupt generated by TXE event 
	if(temp1 && temp2 && temp3)
	{
		//Check for the device mode
		if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL ))
		{
			//TXE flag is set
			//We have to do the data transmission
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		}
		else
		{
			//Slave
			if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_TRA ))
			{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
			}
		}
	}
	
	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_RXNE );
	//6. Handle For interrupt generated by RXNE event 
	if(temp1 && temp2 && temp3)
	{
		//Check device mode
		if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL ))
		{
			//The device is master
			//RXNE flag is set
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);
			}
		}
		else
		{
			//Slave
			//Mkae sure that slave is really in receiver mode
			if(! (pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_TRA ) ))
			{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
			}
		}
	}

}

/****************************************************************
 *@fn				: I2C_ER_IRQHandling
 *
 *@brief			: 
 *
 *@param[in]		: 
 *@param[in]		:
 *@param[in]		:
 *@param[in]
 *
 *@return			: None 
 *
 *@Note				: Interrupt handling for different I2C events (refer SR1)
 					: Complete the code also define these macros in the driver header file
						#define I2C_ERROR_BERR  3
						#define I2C_ERROR_ARLO  4
						#define I2C_ERROR_AF    5
						#define I2C_ERROR_OVR   6
						#define I2C_ERROR_TIMEOUT 7
 ****************************************************************/
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR );

		//Implement the code to notify the application about the error
	   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_ARLO );

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO );

	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_AF );

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVR );

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_TIMEOUT );

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
	}
}

/****************************************************************
 *@fn				: I2C_IRQInterruptConfig
 *
 *@brief			: 
 *
 *@param[in]		: 
 *@param[in]		:
 *@param[in]		:
 *@param[in]
 *
 *@return			: None 
 *
 *@Note				: 
 ****************************************************************/
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t IRQEnDis)
{
	if( IRQEnDis == ENABLE )
	{
		if( IRQNumber <= 31 )
		{
			*NVIC_ISER0 |= ( 1 << IRQNumber );
		}
		else if( IRQNumber > 31 && IRQNumber < 64 )
		{
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if( IRQNumber >= 64 && IRQNumber < 96 )
		{
			*NVIC_ISER2 |= ( 1 << (IRQNumber % 64) );
		}
	}
	else
	{
		if( IRQNumber <= 31 )
		{
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}
		else if( IRQNumber > 31 && IRQNumber < 64 )
		{
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if( IRQNumber >= 64 && IRQNumber < 96 )
		{
			*NVIC_ICER2 |= ( 1 << (IRQNumber % 64) );
		}
	}
}

/****************************************************************
 *@fn				: I2C_IRQPriorityConfig
 *
 *@brief			: 
 *
 *@param[in]		: 
 *@param[in]		:
 *@param[in]		:
 *@param[in]
 *
 *@return			: None 
 *
 *@Note				: 
 ****************************************************************/
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. First lets find out the irq register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQPriority % 4;
	uint8_t shift_amount = ( 8 * iprx_section ) + ( 8 - NO_PR_BITS_IMPLEMENTD );

	*(NVIC_PR_BASE_ADDR + iprx ) |= (IRQPriority << shift_amount );
}

/****************************************************************
 *@fn				: I2C_SlaveSendData
 *
 *@brief			: 
 *
 *@param[in]		: 
 *@param[in]		:
 *@param[in]		:
 *@param[in]
 *
 *@return			: None 
 *
 *@Note				: 
 ****************************************************************/
void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data)
{
	pI2Cx->DR = data;
}

/****************************************************************
 *@fn				: I2C_SlaveReceiveData
 *
 *@brief			: 
 *
 *@param[in]		: 
 *@param[in]		:
 *@param[in]		:
 *@param[in]
 *
 *@return			: None 
 *
 *@Note				: 
 ****************************************************************/
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx)
{
	return (uint8_t)pI2Cx->DR;
}

/****************************************************************
 *@fn				: I2C_SlaveEnableDisableCallbackEvents
 *
 *@brief			: 
 *
 *@param[in]		: 
 *@param[in]		:
 *@param[in]		:
 *@param[in]
 *
 *@return			: None 
 *
 *@Note				: 
 ****************************************************************/
void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnOrDis)
{
	if(EnOrDis == ENABLE)
	{
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN );
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN );
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN );
	}
	else
	{
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN );
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN );
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITERREN );		
	}
}

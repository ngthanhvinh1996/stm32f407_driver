/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Jun 8, 2021
 *      Author: vinhkuto
 */

#include "stm32f407xx_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

/*****************************************************************************************
 * 						APIs supported by this driver
 * 			For more information about the APIs check the function definition
 *****************************************************************************************/

/*
 * Peripheral Clock Setup
 */

/****************************************************************
 *@fn				: SPI_PeriClockControl
 *
 *@brief			: This function enables or disables peripheral clock for the given SPI peripheral
 *
 *@param[in]		: Base address of the SPI peripheral
 *@param[in]		: ENABLE or DISABLE macros
 *@param[in]		:
 *@param[in]
 *
 *@return			: None
 *
 *@Note				: None
 ****************************************************************/

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDis)
{
	if( EnOrDis == ENABLE )
	{
		if( pSPIx == SPI1 )
		{
			SPI1_PCLK_EN();
		}
		else if( pSPIx == SPI2 )
		{
			SPI2_PCLK_EN();
		}
		else if( pSPIx == SPI3 )
		{
			SPI3_PCLK_EN();
		}
		else if( pSPIx == SPI4 )
		{
			SPI4_PCLK_EN();
		}
	}
	else
	{
		if( pSPIx == SPI1 )
		{
			SPI1_PCLK_DIS();
		}
		else if( pSPIx == SPI2 )
		{
			SPI2_PCLK_DIS();
		}
		else if( pSPIx == SPI3 )
		{
			SPI3_PCLK_DIS();
		}
		else if( pSPIx == SPI4 )
		{
			SPI4_PCLK_DIS();
		}
	}

}


/*
 * SPI Initialization
 */

/****************************************************************
 *@fn				: SPI_Init
 *
 *@brief			: This function initialization the SPI peripheral
 *
 *@param[in]		: pSPIHandle
 *@param[in]		:
 *@param[in]		:
 *@param[in]
 *
 *@return			: None
 *
 *@Note				: None
 ****************************************************************/

void SPI_Init(SPI_Handle_t *pSPIHandle)
{

	//Peripheral clock enable
	SPI_PeriClockControl(pSPIHandle->SPIx, ENABLE); 

	//First lets configure the SPI_CR1 register

	uint32_t temp = 0;

	//1. Configure the Device Mode
	temp |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//2. Configure the Bus Config
	if( pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD )
	{
		//BIDIMODE should be cleared
		temp &= ~( 1 << SPI_CR1_BIDIMODE );
	}
	else if( pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD )
	{
		//BIDIMODE should be set
		temp |= ( 1 << SPI_CR1_BIDIMODE );
	}
	else if( pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY )
	{
		//BIDIMODE should be cleared
		temp &= ~( 1 << SPI_CR1_BIDIMODE );
		//RXONLY must be set
		temp |= ( 1 << SPI_CR1_RXONLY );
	}

	//3. Configure the SCLK speed
	temp |= ( pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR );

	//4. Configure the DFF
	temp |= ( pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF );

	//5. Configure the CPOL
	temp |= ( pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL );

	//6. Configure the CPHA
	temp |= ( pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA );

	//7. Configure the SSM
	temp |= ( pSPIHandle->SPIConfig.SPI_SSM << 9 );

	//8. Set value for CR1 register
	pSPIHandle->SPIx->CR1 = temp;

}


/****************************************************************
 *@fn				: SPI_DeInit
 *
 *@brief			: This function de-initialization the SPI peripheral
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
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if( pSPIx == SPI1 )
	{
		SPI1_REG_RESET();
	}
	else if( pSPIx == SPI2 )
	{
		SPI2_REG_RESET();
	}
	else if( pSPIx == SPI3 )
	{
		SPI3_REG_RESET();
	}
}

/****************************************************************
 *@fn				: SPI_SendData
 *
 *@brief			: This function send data the SPI peripheral
 *
 *@param[in]		: SPI_RegDef_t
 *@param[in]		: pTxBuffer
 *@param[in]		: Size
 *@param[in]
 *
 *@return			: None
 *
 *@Note				: None
 ****************************************************************/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint8_t Size)
{
	while( Size > 0 )
	{
		//1. Wait until TXE is set
		while( SPI_GetFlagStatus(pSPIx, SPI_FLAG_SR_TXE) == FLAG_RESET );

		//2. Check the DFF bit in CR1
		if( pSPIx->CR1 & SPI_FLAG_CR1_DFF )
		{
			//16 BIT DFF
			//1. Load the data in to the DR
			pSPIx->DR = *( (uint16_t *)pTxBuffer );
			Size -= 2;
			(uint16_t *)pTxBuffer++;
		}
		else
		{
			//8 BIT DFF
			//1. Load the data in to the DR
			pSPIx->DR = *pTxBuffer;
			Size--;
			pTxBuffer++;
		}

	}
}

/****************************************************************
 *@fn				: SPI_ReceiveData
 *
 *@brief			: This function receive data the SPI peripheral
 *
 *@param[in]		: pSPIHandle
 *@param[in]		:
 *@param[in]		:
 *@param[in]
 *
 *@return			: None
 *
 *@Note				: None
 ****************************************************************/
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint8_t Size)
{
	while( Size > 0 )
	{
		//1. Wait until RXNE is set
		while( SPI_GetFlagStatus(pSPIx, SPI_FLAG_SR_RXNE) == FLAG_RESET );

		//2. Check the DFF bit in CR1
		if( pSPIx->CR1 & SPI_FLAG_CR1_DFF )
		{
			//16 BIT DFF
			//1. Load the data from the DR to RxBuffer address
			*( (uint16_t *)pRxBuffer ) = pSPIx->DR;
			Size -= 2;
			(uint16_t *)pRxBuffer++;
		}
		else
		{
			//8 BIT DFF
			//1. Load the data in to the DR
			*pRxBuffer = pSPIx->DR ;
			Size--;
			pRxBuffer++;
		}

	}
}

/****************************************************************
 *@fn				: SPI_SendDataIT
 *
 *@brief			: This function send data the SPI peripheral for Interrupt
 *
 *@param[in]		: SPI_RegDef_t
 *@param[in]		: pTxBuffer
 *@param[in]		: Size
 *@param[in]
 *
 *@return			: State
 *
 *@Note				: None
 ****************************************************************/
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint8_t Size)
{
	uint8_t state = pSPIHandle->TxState;

	if( state != SPI_BUSY_IN_TX )
	{
		//1. Save the TX buffer address and Len information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Size;

		//2. Mark the SPI state as busy in transmission so that
		//	 no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3. Enable the RXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->SPIx->CR2 |= ( 1 < SPI_CR2_TXEIE );

		//4. Data transmission will be handled by the ISR code (will implement later)		
	}

	return state;

}

/****************************************************************
 *@fn				: SPI_ReceiveDataIT
 *
 *@brief			: This function receive data the SPI peripheral for Interrupt
 *
 *@param[in]		: pSPIHandle
 *@param[in]		: pRxBuffer
 *@param[in]		: Size
 *@param[in]
 *
 *@return			: State
 *
 *@Note				: None
 ****************************************************************/
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint8_t Size)
{
	uint8_t state = pSPIHandle->RxState;

	if( state != SPI_BUSY_IN_RX )
	{
		//1. Save the RX buffer address and Len information in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Size;

		//2. Mark the SPI state as busy in transmission so that
		//	 no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//3. Enable the RXEIE control bit to get interrupt whenever RXNE flag is set in SR
		pSPIHandle->SPIx->CR2 |= ( 1 < SPI_CR2_RXNEIE );

		//4. Data transmission will be handled by the ISR code (will implement later)		
	}

	return state;
}


/****************************************************************
 *@fn				: SPI_PeripheralControl
 *
 *@brief			: This function enable the SPI peripheral
 *
 *@param[in]		: SPI_RegDef_t
 *@param[in]		: ENABLE or DISABLE
 *@param[in]		: 
 *@param[in]
 *
 *@return			: None
 *
 *@Note				: None
 ****************************************************************/
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDis)
{
	if(EnOrDis == ENABLE)
	{
		pSPIx->CR1 |= ( 1 << SPI_CR1_SPE );
	}
	else
	{
		pSPIx->CR1 &= ~( 1 << SPI_CR1_SPE );
	}

}

/****************************************************************
 *@fn				: SPI_GetFlagStatus
 *
 *@brief			: This function get flag for the SPI peripheral
 *
 *@param[in]		: SPI_RegDef_t
 *@param[in]		: FlagName
 *@param[in]		: 
 *@param[in]
 *
 *@return			: FLAG_SET or FLAG_RESET
 *
 *@Note				: None
 ****************************************************************/
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if( pSPIx->SR & FlagName )
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/****************************************************************
 *@fn				: SPI_SSIConfig
 *
 *@brief			: This function enable SSI for the SPI peripheral
 *
 *@param[in]		: SPI_RegDef_t
 *@param[in]		: ENABLE or DISABLE
 *@param[in]		: 
 *@param[in]
 *
 *@return			: None
 *
 *@Note				: None
 ****************************************************************/
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDis)
{
	if(EnOrDis == ENABLE)
	{
		pSPIx->CR1 |= ( 1 << SPI_CR1_SSI );
	}
	else
	{
		pSPIx->CR1 &= ~( 1 << SPI_CR1_SSI );
	}
}

/****************************************************************
 *@fn				: SPI_SSOEConfig
 *
 *@brief			: This function enable SSOE for the SPI peripheral
 *
 *@param[in]		: SPI_RegDef_t
 *@param[in]		: ENABLE or DISABLE
 *@param[in]		: 
 *@param[in]
 *
 *@return			: None
 *
 *@Note				: None
 ****************************************************************/
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDis)
{
	if(EnOrDis == ENABLE)
	{
		pSPIx->CR2 |= ( 1 << SPI_CR2_SSOE );
	}
	else
	{
		pSPIx->CR2 &= ~( 1 << SPI_CR2_SSOE );
	}
}

/****************************************************************
 *@fn				: SPI_IRQInterruptConfig
 *
 *@brief			: Configure IQR for SPI
 *
 *@param[in]		: Number of IRQ
 *@param[in]		:
 *@param[in]		: ENABLE or DISABLE
 *@param[in]
 *
 *@return			: None
 *
 *@Note				: None
 ****************************************************************/
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t IRQEnDis)
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
 *@fn				: SPI_IRQPriorityConfig
 *
 *@brief			: Configure Priority IRQ for SPI
 *
 *@param[in]		: Number of IRQ
 *@param[in]		:
 *@param[in]		: IRQPriority
 *@param[in]
 *
 *@return			: None
 *
 *@Note				: None
 ****************************************************************/
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. First lets find out the irq register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQPriority % 4;
	uint8_t shift_amount = ( 8 * iprx_section ) + ( 8 - NO_PR_BITS_IMPLEMENTD );

	*(NVIC_PR_BASE_ADDR + iprx ) |= (IRQPriority << shift_amount );
}

/****************************************************************
 *@fn				: SPI_IRQHandling
 *
 *@brief			: Handling IRQ for SPI
 *
 *@param[in]		: pHandle
 *@param[in]		:
 *@param[in]		: 
 *@param[in]
 *
 *@return			: None
 *
 *@Note				: None
 ****************************************************************/
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1, temp2;
	//First lets check for TXE
	temp1 = pHandle->SPIx->SR & ( 1 << SPI_SR_TXE );
	temp2 = pHandle->SPIx->CR2 & ( 1 << SPI_CR2_TXEIE );

	if( temp1 && temp2 )
	{
		//Handle TXE
		spi_txe_interrupt_handle(pHandle);
	}

	//Check for RXNE
	temp1 = pHandle->SPIx->SR & ( 1 << SPI_SR_RXNE );
	temp2 = pHandle->SPIx->CR2 & ( 1 << SPI_CR2_RXNEIE );

	if( temp1 &&  temp2 )
	{
		//Handle RXNE
		spi_rxne_interrupt_handle(pHandle);
	}

	//Check for over flag
	temp1 = pHandle->SPIx->SR & ( 1 << SPI_SR_OVR );
	temp2 = pHandle->SPIx->CR2 & ( 1 << SPI_CR2_ERRIE );

	if( temp1 &&  temp2 )
	{
		//Handle Error
		spi_ovr_err_interrupt_handle(pHandle);
	}
}

//Some helper function implementation

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//Check the DFF bit in CR1
	if( pSPIHandle->SPIx->CR1 & SPI_FLAG_CR1_DFF )
	{
		//16 BIT DFF
		//1. Load the data in to the DR
		pSPIHandle->SPIx->DR = *( (uint16_t *)pSPIHandle->pTxBuffer );
		pSPIHandle->TxLen -= 2;
		(uint16_t *)pSPIHandle->pTxBuffer++;
	}
	else
	{
		//8 BIT DFF
		//1. Load the data in to the DR
		pSPIHandle->SPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if( !pSPIHandle->TxLen )
	{
		//TxLen is zero, so close the spi communication and inform the application that
		//TX is over.
		//this prevents interrupts from setting up of TXE flag
		SPI_CloseTranmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//Check the DFF bit in CR1
	if( pSPIHandle->SPIx->CR1 & SPI_FLAG_CR1_DFF )
	{
		//16 BIT DFF
		//1. Load the data in to the DR
		*( (uint16_t *)pSPIHandle->pRxBuffer ) = (uint16_t)pSPIHandle->SPIx->DR;
		pSPIHandle->RxLen -= 2;
		pSPIHandle->pRxBuffer--;
		pSPIHandle->pRxBuffer--;
	}
	else
	{
		//8 BIT DFF
		//1. Load the data in to the DR
		*pSPIHandle->pRxBuffer = pSPIHandle->SPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer--;
	}

	if( !pSPIHandle->RxLen )
	{
		//reception is complete
		//lets turn off the rxneie interrupt
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	//1. clear the ovr flag
	if( pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->SPIx->DR;
		temp = pSPIHandle->SPIx->SR;
	}
	(void)temp;
	//2. inform the application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}


void SPI_CloseTranmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->SPIx->CR2 &= ~( 1 << SPI_CR2_TXEIE );
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}


void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->SPIx->CR2 &= ~( 1 << SPI_CR2_RXNEIE );
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}


void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;

	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvt)
{
	//This is a weak implementation. The application may override this function
}

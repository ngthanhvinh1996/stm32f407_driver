/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: May 30, 2021
 *      Author: Thanh Vinh
 */



#include "stm32f407xx_gpio_driver.h"


/*****************************************************************************************
 * 						APIs supported by this driver
 * 			For more information about the APIs check the function definition
 *****************************************************************************************/

/*
 * Peripheral Clock Setup
 */

/****************************************************************
 *@fn				: GPIO_PeriClockControl
 *
 *@brief			: This function enables or disables peripheral clock for the given GPIO port
 *
 *@param[in]		: Base address of the gpio peripheral
 *@param[in]		: ENABLE or DISABLE macros
 *@param[in]		:
 *@param[in]
 *
 *@return			: None
 *
 *@Note				: None
 ****************************************************************/

void GPIO_PeriClockControl( GPIO_RegDef_t *pGPIOx, uint8_t EnOrDis )
{
	if( EnOrDis == ENABLE )
	{
		if( pGPIOx == GPIOA )
		{
			GPIOA_PCLK_EN();
		}
		else if ( pGPIOx == GPIOB )
		{
			GPIOB_PCLK_EN();
		}
		else if ( pGPIOx == GPIOC )
		{
			GPIOC_PCLK_EN();
		}
		else if ( pGPIOx == GPIOD )
		{
			GPIOD_PCLK_EN();
		}
		else if ( pGPIOx == GPIOE )
		{
			GPIOE_PCLK_EN();
		}
		else if ( pGPIOx == GPIOF )
		{
			GPIOF_PCLK_EN();
		}
		else if ( pGPIOx == GPIOG )
		{
			GPIOG_PCLK_EN();
		}
		else if ( pGPIOx == GPIOH )
		{
			GPIOH_PCLK_EN();
		}
		else if ( pGPIOx == GPIOI )
		{
			GPIOI_PCLK_EN();
		}
	}
	else
	{
		if( pGPIOx == GPIOA )
		{
			GPIOA_PCLK_DIS();
		}
		else if ( pGPIOx == GPIOB )
		{
			GPIOB_PCLK_DIS();
		}
		else if ( pGPIOx == GPIOC )
		{
			GPIOC_PCLK_DIS();
		}
		else if ( pGPIOx == GPIOD )
		{
			GPIOD_PCLK_DIS();
		}
		else if ( pGPIOx == GPIOE )
		{
			GPIOE_PCLK_DIS();
		}
		else if ( pGPIOx == GPIOF )
		{
			GPIOF_PCLK_DIS();
		}
		else if ( pGPIOx == GPIOG )
		{
			GPIOG_PCLK_DIS();
		}
		else if ( pGPIOx == GPIOH )
		{
			GPIOH_PCLK_DIS();
		}
		else if ( pGPIOx == GPIOI )
		{
			GPIOI_PCLK_DIS();
		}
	}
}

/*
 * Initialization and DeInit
 */

/****************************************************************
 *@fn				: GPIO_Init
 *
 *@brief			: This function initialization the GPIO port
 *
 *@param[in]		: GPIO handle
 *@param[in]		:
 *@param[in]		:
 *@param[in]
 *
 *@return			: None
 *
 *@Note				: None
 ****************************************************************/

void GPIO_Init( GPIO_Handle_t *pGPIOHandle )
{
	uint32_t	temp = 0;		//temp. register

	//Preipheral clock enable
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);
	
	//1. Configure the mode of gpio pin
	if( pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG )
	{
		temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ));
		pGPIOHandle->pGPIOx->MODER &= ~( 0x03 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		pGPIOHandle->pGPIOx->MODER |= temp;
	}
	else
	{
		//This part will code later. (interrupt mode)
		if( pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT )
		{
			//1. Configure the FTSR
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			//Cleared the corresponding RTSR bit
			EXTI->RTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		}
		else if( pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT )
		{
			//1. Configure the RTSR
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			//Cleared the corresponding FTSR bit
			EXTI->FTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		}
		else if( pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT )
		{
			//1. Configure both FTSR and RTSR
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		}

		//2. Configure the GPIO port selection in SYSCFG_EXTICR
		uint32_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint32_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE( pGPIOHandle->pGPIOx );

		SYSCFG_PCKL_EN();

		SYSCFG->EXTICR[temp2] |= ( portcode << (temp1 * 4) );

		//3. Enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );

	}

	temp = 0;

	//2. Configure the speed
	temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x03 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp = 0;

	//3. Configure the pull-up or pull-down
	temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ));
	pGPIOHandle->pGPIOx->PUPDR &= ~( 0x03 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;

	//4. Configure the output type
	temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x01 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp = 0;

	//5. Configure the alternate function
	if( pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN )
	{
		if( pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber <= GPIO_PIN_NO_7 )
		{
			temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ));
			pGPIOHandle->pGPIOx->AFRL &= ~( 0x0F << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			pGPIOHandle->pGPIOx->AFRL |= temp;
		}
		else
		{
			uint32_t value = 0;

			value = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8 ;
			temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4 * value ));
			pGPIOHandle->pGPIOx->AFRH &= ~( 0x0F << value );
			pGPIOHandle->pGPIOx->AFRH |= temp;
		}
	}

	temp = 0;

}

/****************************************************************
 *@fn				: GPIO_DeInit
 *
 *@brief			: This function de-initialization the GPIO port
 *
 *@param[in]		: GPIO Register
 *@param[in]		:
 *@param[in]		:
 *@param[in]
 *
 *@return			: None
 *
 *@Note				: None
 ****************************************************************/

void GPIO_DeInit( GPIO_RegDef_t *pGPIOx )
{
	if( pGPIOx == GPIOA )
	{
		GPIOA_REG_RESET();
	}
	else if ( pGPIOx == GPIOB )
	{
		GPIOB_REG_RESET();
	}
	else if ( pGPIOx == GPIOC )
	{
		GPIOC_REG_RESET();
	}
	else if ( pGPIOx == GPIOD )
	{
		GPIOD_REG_RESET();
	}
	else if ( pGPIOx == GPIOE )
	{
		GPIOE_REG_RESET();
	}
	else if ( pGPIOx == GPIOF )
	{
		GPIOF_REG_RESET();
	}
	else if ( pGPIOx == GPIOG )
	{
		GPIOG_REG_RESET();
	}
	else if ( pGPIOx == GPIOH )
	{
		GPIOH_REG_RESET();
	}
	else if ( pGPIOx == GPIOI )
	{
		GPIOI_REG_RESET();
	}
}

/*
 * Data read and write
 */

/****************************************************************
 *@fn				: GPIO_ReadFromInputPin
 *
 *@brief			: Read data from input pin number
 *
 *@param[in]		: GPIO Register
 *@param[in]		: Pin Number
 *@param[in]		:
 *@param[in]
 *
 *@return			: Value data from input pin (0 or 1)
 *
 *@Note				: None
 ****************************************************************/

uint8_t GPIO_ReadFromInputPin( GPIO_RegDef_t *pGPIOx, uint8_t PinNumber )
{
	uint8_t	temp;

	temp = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);

	return temp;
}

/****************************************************************
 *@fn				: GPIO_ReadFromInputPort
 *
 *@brief			: Read data from input port
 *
 *@param[in]		: GPIO Register
 *@param[in]		:
 *@param[in]		:
 *@param[in]
 *
 *@return			: Value data from input port
 *
 *@Note				: None
 ****************************************************************/

uint16_t GPIO_ReadFromInputPort( GPIO_RegDef_t *pGPIOx )
{

	uint16_t	temp;

	temp = (uint16_t)pGPIOx->IDR;

	return temp;
}

/****************************************************************
 *@fn				: GPIO_WriteToOutputPin
 *
 *@brief			: Write data from output pin
 *
 *@param[in]		: GPIO Register
 *@param[in]		: Pin Number
 *@param[in]		: GPIO_PIN_SET or GPIO_PIN_RESET
 *@param[in]
 *
 *@return			: Value data from input port
 *
 *@Note				: None
 ****************************************************************/

void GPIO_WriteToOutputPin( GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value )
{
	if( Value == GPIO_PIN_SET )
	{
		// Write 1 to the output data register at the bit field corresponding to the pin number
		pGPIOx->ODR |= ( 1 << PinNumber);
	}
	else
	{
		// Write 0
		pGPIOx->ODR &= ~( 1 << PinNumber);
	}

}

/****************************************************************
 *@fn				: GPIO_WriteToOutputPort
 *
 *@brief			: Write data from output port
 *
 *@param[in]		: GPIO Register
 *@param[in]		:
 *@param[in]		: Value data
 *@param[in]
 *
 *@return			: Value data from input port
 *
 *@Note				: None
 ****************************************************************/

void GPIO_WriteToOutputPort( GPIO_RegDef_t *pGPIOx, uint16_t Value )
{
	pGPIOx->ODR |= Value;
}

/****************************************************************
 *@fn				: GPIO_ToggleOutputPin
 *
 *@brief			: Change data of output pin
 *
 *@param[in]		: GPIO Register
 *@param[in]		:
 *@param[in]		: Value data
 *@param[in]
 *
 *@return			: Value data from input port
 *
 *@Note				: None
 ****************************************************************/

void GPIO_ToggleOutputPin( GPIO_RegDef_t *pGPIOx, uint8_t PinNumber )
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

/*
 * IQR Configuration and ISR handling
 */

/****************************************************************
 *@fn				: GPIO_IQRInterruptConfig
 *
 *@brief			: Configure IQR for GPIO
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

void GPIO_IRQInterruptConfig( uint8_t IRQNumber, uint8_t EnOrDis )
{
	if( EnOrDis == ENABLE )
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
 *@fn				: GPIO_IRQPriorityConfig
 *
 *@brief			: Configure IQR Priority for GPIO
 *
 *@param[in]		: Number priority of IRQ
 *@param[in]		:
 *@param[in]		:
 *@param[in]
 *
 *@return			: None
 *
 *@Note				: None
 ****************************************************************/

void GPIO_IRQPriorityConfig( uint8_t IRQNumber, uint32_t IRQPriority )
{
	//1. First lets find out the irq register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQPriority % 4;
	uint8_t shift_amount = ( 8 * iprx_section ) + ( 8 - NO_PR_BITS_IMPLEMENTD );

	*(NVIC_PR_BASE_ADDR + iprx ) |= (IRQPriority << shift_amount );
}

/****************************************************************
 *@fn				: GPIO_IRQHandling
 *
 *@brief			: Handle IRQ for GPIO
 *
 *@param[in]		: Number pin of IRQ GPIO
 *@param[in]		:
 *@param[in]		:
 *@param[in]
 *
 *@return			: None
 *
 *@Note				: None
 ****************************************************************/

void GPIO_IRQHandling( uint8_t PinNumber )
{
	//Clear the EXTI PR register corresponding to the pin number
	if( EXTI->PR & ( 1 << PinNumber ) )
	{
		//Clear
		EXTI->PR |= ( 1 << PinNumber );
	}

}



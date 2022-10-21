/*
 * stm32f407xx_rcc_driver.c
 *
 *  Created on: Jul 6, 2021
 *      Author: vinhkuto
 */

#include "stm32f407xx_rcc_driver.h"

uint16_t AHB_PreScaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint16_t APBx_PreScaler[4] = {2, 4, 8, 16};

uint8_t RCC_GetFlagStatus(uint32_t FlagName);

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
uint32_t RCC_GetPLLOutClock(void)
{
	return 0;
}

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
  *@Note			: None
  ****************************************************************/
 uint32_t RCC_GetPCLK1Value(void)
 {
 	uint32_t pclk1;
 	uint32_t SystemClk;
 	uint8_t clksrc;
 	uint8_t temp;
 	uint8_t ahbp;
 	uint8_t apb1p;

 	clksrc = ((RCC->CFGR >> 2) & 0x03);

 	if(clksrc == 0)
 	{
 		SystemClk = 16000000;
 	}
 	else if(clksrc == 1)
 	{
 		SystemClk = 8000000;
 	}
 	else if(clksrc == 2)
 	{
 		SystemClk = RCC_GetPLLOutClock();
 	}

 	//For AHB Prescaler
 	temp = ((RCC->CFGR >> 4) & 0xF);

 	if(temp < 8 || temp > 16)
 	{
 		ahbp = 1;
 	}
 	else
 	{
 		ahbp = AHB_PreScaler[temp-8];
 	}

 	//For ABP1 Prescaler
 	temp = ((RCC->CFGR >> 10) & 0x07);

 	if(temp < 4){
 		apb1p = 1;
 	}
 	else
 	{
 		apb1p = APBx_PreScaler[temp-4];
 	}

 	//Calculate PCLK1
 	pclk1 = (SystemClk / ahbp) / apb1p;

 	return pclk1;

 }

 /****************************************************************
  *@fn				: RCC_GetPCLK2Value
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
  *@Note			: None
  ****************************************************************/
 uint32_t RCC_GetPCLK2Value(void)
 {
 	uint32_t pclk2;
 	uint32_t SystemClk;
 	uint8_t clksrc;
 	uint8_t temp;
 	uint8_t ahbp;
 	uint8_t apb1p;

 	clksrc = ((RCC->CFGR >> 2) & 0x03);

 	if(clksrc == 0)
 	{
 		SystemClk = 16000000;
 	}
 	else if(clksrc == 1)
 	{
 		SystemClk = 8000000;
 	}
 	else if(clksrc == 2)
 	{
 		SystemClk = RCC_GetPLLOutClock();
 	}

 	//For AHB Prescaler
 	temp = ((RCC->CFGR >> 4) & 0xF);

 	if(temp < 8 || temp > 16)
 	{
 		ahbp = 1;
 	}
 	else
 	{
 		ahbp = AHB_PreScaler[temp-8];
 	}

 	//For ABP2 Prescaler
 	temp = ((RCC->CFGR >> 13) & 0x07);

 	if(temp < 4){
 		apb1p = 1;
 	}
 	else
 	{
 		apb1p = APBx_PreScaler[temp-4];
 	}

 	//Calculate PCLK1
 	pclk2 = (SystemClk / ahbp) / apb1p;

 	return pclk2;

 }

 /****************************************************************
  *@fn				: RCC_OSCInit
  *
  *@brief			: This function initilization for RCC Oscillator
  *
  *@param[in]		:
  *@param[in]		:
  *@param[in]		:
  *@param[in]
  *
  *@return			: None
  *
  *@Note			: None
  ****************************************************************/
 void RCC_OSCInit(RCC_OSCConfig_t *pOSC_Handle)
 {
	 //Configuration for HSE Clock
	 if( pOSC_Handle->OscillatorType == RCC_OSCILLATORTYPE_HSE )
	 {
		 if( pOSC_Handle->HSEState != RCC_HSE_OFF )
		 {
			 if( pOSC_Handle->HSEState == RCC_HSE_ON )
			 {
				 RCC->CR |= ( 1 << RCC_CR_HSEON );
			 }
			 else if( pOSC_Handle->HSEState == RCC_HSE_BYPASS )
			 {
				 RCC->CR |= ( 1 << RCC_CR_HSEBYP );
				 RCC->CR |= ( 1 << RCC_CR_HSEON );
			 }

			 while( RCC_GetFlagStatus(RCC_FLAG_CR_HSERDY) == FLAG_SET );
		 }
		 else
		 {
			 RCC->CR &= ~( 1 << RCC_CR_HSEON );

			 while( RCC_GetFlagStatus(RCC_FLAG_CR_HSERDY) == FLAG_RESET );
		 }
	 }

	 //Configuration for HSI Clock
	 else if( pOSC_Handle->OscillatorType == RCC_OSCILLATORTYPE_HSI )
	 {
		 if( pOSC_Handle->HSIState == RCC_HSI_ON )
		 {
			 RCC->CR |= ( 1 << RCC_CR_HSION );
			 RCC->CR |= ( pOSC_Handle->HSICalibrationValue << RCC_CR_HSITRIM );

			 while( RCC_GetFlagStatus(RCC_FLAG_CR_HSIRDY) == FLAG_SET );
		 }
		 else
		 {
			 RCC->CR &= ~( 1 << RCC_CR_HSION );

			 while( RCC_GetFlagStatus(RCC_FLAG_CR_HSIRDY) == FLAG_RESET );
		 }
	 }

	 //Configuration for LSE Clock
	 else if( pOSC_Handle->OscillatorType == RCC_OSCILLATORTYPE_LSE )
	 {
		 /* Update LSE configuration in Backup Domain control register    */
		 /* Requires to enable write access to Backup Domain of necessary */
		 if( (RCC->APB1ENR & RCC_APB1ENR_PWREN) == RESET )
		 {
			 RCC->APB1ENR |= ( 1 << RCC_APB1ENR_PWREN );
		 }

		 /* Enable write access to Backup domain */
		 if( (PWR->CR & PWR_CR_DBP) == RESET )
		 {
			 PWR->CR |= ( 1 << PWR_CR_DBP );
		 }

		 /* Set the new LSE configuration -----------------------------------------*/
		 if( pOSC_Handle->LSEState != RCC_LSE_OFF )
		 {
			 if( pOSC_Handle->LSEState == RCC_LSE_ON )
			 {

			 }
		 }
	 }
 }

 uint8_t RCC_GetFlagStatus(uint32_t FlagName)
 {
	 if(RCC->CR & FlagName)
	 {
		 return FLAG_SET;
	 }
	 return FLAG_RESET;
 }

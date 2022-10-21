/*
 * stm32f407xx_rcc_driver.h
 *
 *  Created on: Jul 6, 2021
 *      Author: vinhkuto
 */

#ifndef INC_STM32F407XX_RCC_DRIVER_H_
#define INC_STM32F407XX_RCC_DRIVER_H_

#include "stm32f407xx.h"


/*
 * Configuration structure for RCC PLL peripheral
 */
typedef struct
{
    uint32_t     PLLSource;			/* RCC_PLLSource: PLL entry clock source. @PLLSource */
    uint32_t	 PLLState;			/* The new state of the PLL. @PLLState */
    uint32_t     PLLN;				/* PLLN: Multiplication factor for PLL VCO output clock. @PLLN */
    uint32_t     PLLQ;				/* PLLQ: Division factor for OTG FS, SDIO and RNG clocks. @PLLQ */
    uint32_t     PLLM;				/* PLLM: Division factor for PLL VCO input clock. @PLLM*/
    uint32_t     PLLP;				/* PLLP: Division factor for main system clock (SYSCLK). @PLLP */
}RCC_PLLConfig_t;


/*
 * Configuration structure for RCC peripheral
 */
typedef struct
{
	uint32_t			OscillatorType;			/* The oscillators to be configured. @OscillatorType */
    uint32_t     		HSEState;				/* The new state of the HSE. @HSEState */
    uint32_t     		HSIState;				/* The new state of the HSI. @HSIState */
    uint32_t     		LSIState;				/* The new state of the LSI. @LSIState */
    uint32_t     		LSEState;				/* The new state of the LSE. @LSEState */
    uint32_t     		HSICalibrationValue;	/* The HSI calibration trimming value (default is RCC_HSICALIBRATION_DEFAULT). @HSICalibrationValue*/
    RCC_PLLConfig_t 	PLL;					/* PLL structure parameters */
}RCC_OSCConfig_t;



/*
 * Configuration structure for RCC CLK peripheral
 */
typedef struct
{
	uint32_t	ClockType;				/* The clock to be configured. @ClockType */
	uint32_t	SYSCLKSource;			/* The clock source (SYSCLKS) used as system clock. @SYSCLKSource */
	uint32_t 	AHBCLKDivider;			/* The AHB clock (HCLK) divider. This clock is derived from the system clock (SYSCLK). @AHBCLKDivider */
	uint32_t 	APB1CLKDivider;			/* The APB1 clock (PCLK1) divider. This clock is derived from the AHB clock (HCLK). @APB1CLKDivider */
	uint32_t 	APB2CLKDivider;			/* The APB2 clock (PCLK2) divider. This clock is derived from the AHB clock (HCLK). @APB2CLKDivider */
}RCC_CLKConfig_t;

/*
 * @OscillatorType
 */
#define RCC_OSCILLATORTYPE_NONE		0
#define RCC_OSCILLATORTYPE_HSE		1
#define RCC_OSCILLATORTYPE_HSI		2
#define RCC_OSCILLATORTYPE_LSE		3
#define RCC_OSCILLATORTYPE_LSI		4

/*
 * @HSEState
 */
#define RCC_HSE_OFF					0
#define RCC_HSE_ON					1
#define RCC_HSE_BYPASS				2

/*
 * @HSIState
 */
#define RCC_HSI_OFF					0
#define RCC_HSI_ON					1

/*
 * @LSIState
 */
#define RCC_LSI_OFF					0
#define RCC_LSI_ON					1

/*
 * @LSEState
 */
#define RCC_LSE_OFF					0
#define RCC_LSE_ON					1
#define RCC_LSE_BYPASS				2

/*
 * @PLLSource
 */
#define RCC_PLLSOURCE_HSI			0
#define RCC_PLLSOURCE_HSE			1

/*
 * @PLLState
 */
#define RCC_PLL_NONE				0
#define RCC_PLL_OFF					1
#define RCC_PLL_ON					2

/*
 * @PLLP
 */
#define RCC_PLLP_DIV2				0
#define RCC_PLLP_DIV4				1
#define RCC_PLLP_DIV6				2
#define RCC_PLLP_DIV8				3

/*
 * @ClockType
 */
#define RCC_CLOCKTYPE_SYSCLK		0
#define RCC_CLOCKTYPE_HCLK			1
#define RCC_CLOCKTYPE_PCLK1			2
#define RCC_CLOCKTYPE_PCLK2			3

/*
 * @SYSCLKSource
 */
#define RCC_SYSCLKSOURCE_HSI		0
#define RCC_SYSCLKSOURCE_HSE		1
#define RCC_SYSCLKSOURCE_PLLCLK		2
#define RCC_SYSCLKSOURCE_PLLRCLK	3

/*
 * @AHBCLKDivider
 */
#define RCC_SYSCLK_DIV0				0x00
#define RCC_SYSCLK_DIV2				0x08
#define RCC_SYSCLK_DIV4				0x09
#define RCC_SYSCLK_DIV8				0x0A
#define RCC_SYSCLK_DIV16			0x0B
#define RCC_SYSCLK_DIV64			0x0C
#define RCC_SYSCLK_DIV128			0x0D
#define RCC_SYSCLK_DIV256			0x0E
#define RCC_SYSCLK_DIV512			0x0F

/*
 * @APB1CLKDivider and @APB2CLKDivider
 */
#define RCC_HCLK_DIV0				0x00
#define RCC_HCLK_DIV2				0x04
#define RCC_HCLK_DIV4				0x05
#define RCC_HCLK_DIV8				0x06
#define RCC_HCLK_DIV16				0x07

/*
 * RCC related control flag definitions (RCC_CR)
 */
#define RCC_FLAG_CR_HSIRDY			( 1 << RCC_CR_HSIRDY )
#define RCC_FLAG_CR_PLLRDY			( 1 << RCC_CR_PLLRDY )
#define RCC_FLAG_CR_PLLI2SRDY		( 1 << RCC_CR_PLLI2SRDY )
#define RCC_FLAG_CR_HSERDY			( 1 << RCC_CR_HSERDY )


uint32_t RCC_GetPCLK1Value(void);
uint32_t RCC_GetPCLK2Value(void);
uint32_t RCC_GetPLLOutClock(void);

void RCC_OSCInit(RCC_OSCConfig_t *pOSC_Handle);












#endif /* INC_STM32F407XX_RCC_DRIVER_H_ */

/*
- * stm32f407xx.h
 *
 *  Created on: May 28, 2021
 *      Author: Thanh Vinh
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

#define __vo	volatile
#define __weak __attribute__((weak))

/**********************************START: Processor Specific Details *******************************/
/*
 * ARM Cortex Mx Processor NVIC ISER register address
 */

#define NVIC_ISER0		(( __vo uint32_t *)0xE000E100)
#define NVIC_ISER1		(( __vo uint32_t *)0xE000E104)
#define NVIC_ISER2		(( __vo uint32_t *)0xE000E108)
#define NVIC_ISER3		(( __vo uint32_t *)0xE000E10C)
#define NVIC_ISER4		(( __vo uint32_t *)0xE000E110)
#define NVIC_ISER5		(( __vo uint32_t *)0xE000E114)
#define NVIC_ISER6		(( __vo uint32_t *)0xE000E118)
#define NVIC_ISER7		(( __vo uint32_t *)0xE000E11C)

/*
 * ARM Cortex Mx Processor NVIC ICER register address
 */

#define NVIC_ICER0		(( __vo uint32_t *)0xE000E180)
#define NVIC_ICER1		(( __vo uint32_t *)0xE000E184)
#define NVIC_ICER2		(( __vo uint32_t *)0xE000E188)
#define NVIC_ICER3		(( __vo uint32_t *)0xE000E18C)
#define NVIC_ICER4		(( __vo uint32_t *)0xE000E190)
#define NVIC_ICER5		(( __vo uint32_t *)0xE000E194)
#define NVIC_ICER6		(( __vo uint32_t *)0xE000E198)
#define NVIC_ICER7		(( __vo uint32_t *)0xE000E19C)

/*
 * ARM Cortex Mx Processor NVIC ISPR register address
 */

#define NVIC_ISPR0		(( __vo uint32_t *)0xE000E200)
#define NVIC_ISPR1		(( __vo uint32_t *)0xE000E204)
#define NVIC_ISPR2		(( __vo uint32_t *)0xE000E208)
#define NVIC_ISPR3		(( __vo uint32_t *)0xE000E20C)
#define NVIC_ISPR4		(( __vo uint32_t *)0xE000E210)
#define NVIC_ISPR5		(( __vo uint32_t *)0xE000E214)
#define NVIC_ISPR6		(( __vo uint32_t *)0xE000E218)
#define NVIC_ISPR7		(( __vo uint32_t *)0xE000E21C)

/*
 * ARM Cortex Mx Processor NVIC ICPR register address
 */

#define NVIC_ICPR0		(( __vo uint32_t *)0xE000E280)
#define NVIC_ICPR1		(( __vo uint32_t *)0xE000E284)
#define NVIC_ICPR2		(( __vo uint32_t *)0xE000E288)
#define NVIC_ICPR3		(( __vo uint32_t *)0xE000E28C)
#define NVIC_ICPR4		(( __vo uint32_t *)0xE000E290)
#define NVIC_ICPR5		(( __vo uint32_t *)0xE000E294)
#define NVIC_ICPR6		(( __vo uint32_t *)0xE000E298)
#define NVIC_ICPR7		(( __vo uint32_t *)0xE000E29C)

/*
 * ARM Cortex Mx Processor NVIC IABR register address
 */

#define NVIC_IABR0		(( __vo uint32_t *)0xE000E300)
#define NVIC_IABR1		(( __vo uint32_t *)0xE000E304)
#define NVIC_IABR2		(( __vo uint32_t *)0xE000E308)
#define NVIC_IABR3		(( __vo uint32_t *)0xE000E30C)
#define NVIC_IABR4		(( __vo uint32_t *)0xE000E310)
#define NVIC_IABR5		(( __vo uint32_t *)0xE000E314)
#define NVIC_IABR6		(( __vo uint32_t *)0xE000E318)
#define NVIC_IABR7		(( __vo uint32_t *)0xE000E31C)

/*
 * ARM Cortex Mx Processor NVIC IPR register address
 */

#define NVIC_PR_BASE_ADDR	(( __vo uint32_t *)0xE000E400)

#define NO_PR_BITS_IMPLEMENTD	4
//#define NVIC_IPR0		(( __vo uint32_t *)0xE000E400)
//#define NVIC_IPR1		(( __vo uint32_t *)0xE000E404)
//#define NVIC_IPR2		(( __vo uint32_t *)0xE000E408)
//#define NVIC_IPR3		(( __vo uint32_t *)0xE000E40C)
//#define NVIC_IPR4		(( __vo uint32_t *)0xE000E410)
//#define NVIC_IPR5		(( __vo uint32_t *)0xE000E414)
//#define NVIC_IPR6		(( __vo uint32_t *)0xE000E418)
//#define NVIC_IPR7		(( __vo uint32_t *)0xE000E41C)
//#define	NVIC_IPR8		(( __vo uint32_t *)0xE000E420)
//#define	NVIC_IPR9		(( __vo uint32_t *)0xE000E424)
//#define	NVIC_IPR10		(( __vo uint32_t *)0xE000E428)
//#define	NVIC_IPR11		(( __vo uint32_t *)0xE000E42C)
//#define	NVIC_IPR12		(( __vo uint32_t *)0xE000E430)
//#define	NVIC_IPR13		(( __vo uint32_t *)0xE000E434)
//#define	NVIC_IPR14		(( __vo uint32_t *)0xE000E438)
//#define	NVIC_IPR15		(( __vo uint32_t *)0xE000E43C)
//#define	NVIC_IPR16		(( __vo uint32_t *)0xE000E440)
//#define	NVIC_IPR17		(( __vo uint32_t *)0xE000E444)
//#define	NVIC_IPR18		(( __vo uint32_t *)0xE000E448)
//#define	NVIC_IPR19		(( __vo uint32_t *)0xE000E44C)
//#define	NVIC_IPR20		(( __vo uint32_t *)0xE000E450)
//#define	NVIC_IPR21		(( __vo uint32_t *)0xE000E454)
//#define	NVIC_IPR22		(( __vo uint32_t *)0xE000E458)
//#define	NVIC_IPR23		(( __vo uint32_t *)0xE000E45C)
//#define	NVIC_IPR24		(( __vo uint32_t *)0xE000E460)
//#define	NVIC_IPR25		(( __vo uint32_t *)0xE000E464)
//#define	NVIC_IPR26		(( __vo uint32_t *)0xE000E468)
//#define	NVIC_IPR27		(( __vo uint32_t *)0xE000E46C)
//#define	NVIC_IPR28		(( __vo uint32_t *)0xE000E470)
//#define	NVIC_IPR29		(( __vo uint32_t *)0xE000E474)
//#define	NVIC_IPR30		(( __vo uint32_t *)0xE000E478)
//#define	NVIC_IPR31		(( __vo uint32_t *)0xE000E47C)
//#define	NVIC_IPR32		(( __vo uint32_t *)0xE000E480)
//#define	NVIC_IPR33		(( __vo uint32_t *)0xE000E484)
//#define	NVIC_IPR34		(( __vo uint32_t *)0xE000E488)
//#define	NVIC_IPR35		(( __vo uint32_t *)0xE000E48C)
//#define	NVIC_IPR36		(( __vo uint32_t *)0xE000E490)
//#define	NVIC_IPR37		(( __vo uint32_t *)0xE000E494)
//#define	NVIC_IPR38		(( __vo uint32_t *)0xE000E498)
//#define	NVIC_IPR39		(( __vo uint32_t *)0xE000E49C)
//#define	NVIC_IPR40		(( __vo uint32_t *)0xE000E4A0)
//#define	NVIC_IPR41		(( __vo uint32_t *)0xE000E4A4)
//#define	NVIC_IPR42		(( __vo uint32_t *)0xE000E4A8)
//#define	NVIC_IPR43		(( __vo uint32_t *)0xE000E4AC)
//#define	NVIC_IPR44		(( __vo uint32_t *)0xE000E4B0)
//#define	NVIC_IPR45		(( __vo uint32_t *)0xE000E4B4)
//#define	NVIC_IPR46		(( __vo uint32_t *)0xE000E4B8)
//#define	NVIC_IPR47		(( __vo uint32_t *)0xE000E4BC)
//#define	NVIC_IPR48		(( __vo uint32_t *)0xE000E4C0)
//#define	NVIC_IPR49		(( __vo uint32_t *)0xE000E4C4)
//#define	NVIC_IPR50		(( __vo uint32_t *)0xE000E4C8)
//#define	NVIC_IPR51		(( __vo uint32_t *)0xE000E4CC)
//#define	NVIC_IPR52		(( __vo uint32_t *)0xE000E4D0)
//#define	NVIC_IPR53		(( __vo uint32_t *)0xE000E4D4)
//#define	NVIC_IPR54		(( __vo uint32_t *)0xE000E4D8)
//#define	NVIC_IPR55		(( __vo uint32_t *)0xE000E4DC)
//#define	NVIC_IPR56		(( __vo uint32_t *)0xE000E4E0)
//#define	NVIC_IPR57		(( __vo uint32_t *)0xE000E4E4)
//#define	NVIC_IPR58		(( __vo uint32_t *)0xE000E4E8)
//#define	NVIC_IPR59		(( __vo uint32_t *)0xE000E4EC)

/*
 * ARM Cortex Mx Processor STIR register address
 */

#define STIR			(( __vo uint32_t *)0xE000EF00)

/*
 * Base address of Flash and SRAM1 memories
 */

#define FLASH_BASEADDR			0x08000000U		/* !<explain this macro briefly here */
#define SRAM1_BASEADDR			0x20000000U		/* !<explain this macro briefly here */
#define SRAM2_BASEADDR			0x20001C00U		/* !<explain this macro briefly here */
#define ROM_BASEADDR			0x1FFF0000U		/* !<explain this macro briefly here */
#define SRAM					SRAM1_BASEADDR

/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASE				0x40000000U
#define APB1PERIPH_BASE			PERIPH_BASE
#define APB2PERIPH_BASE			0x40010000U
#define AHB1PERIPH_BASE			0x40020000U
#define AHB2PERIPH_BASE			0x50000000U

/*
 * Base addresses of peripherals which are hanging on AHB1
 * TODO: Complete for all peripherals
 */

#define GPIOA_BASEADDR			(AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR			(AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR			(AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR			(AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR			(AHB1PERIPH_BASE + 0x1000)
#define GPIOF_BASEADDR			(AHB1PERIPH_BASE + 0x1400)
#define GPIOG_BASEADDR			(AHB1PERIPH_BASE + 0x1800)
#define GPIOH_BASEADDR			(AHB1PERIPH_BASE + 0x1C00)
#define GPIOI_BASEADDR			(AHB1PERIPH_BASE + 0x2000)
#define GPIOJ_BASEADDR			(AHB1PERIPH_BASE + 0x2400)
#define GPIOK_BASEADDR			(AHB1PERIPH_BASE + 0x2800)
#define CRC_BASEADDR			(AHB1PERIPH_BASE + 0x3000)
#define RCC_BASEADDR			(AHB1PERIPH_BASE + 0x3800)
#define FLASHINT_BASEADDR			(AHB1PERIPH_BASE + 0x3C00)
#define BKPSRAM_BASEADDR		(AHB1PERIPH_BASE + 0x4000)
#define DMA1_BASEADDR			(AHB1PERIPH_BASE + 0x6000)
#define DMA2_BASEADDR			(AHB1PERIPH_BASE + 0x6400)
#define ETNMAC_BASEADDR			(AHB1PERIPH_BASE + 0x8000)
#define DMA2D_BASEADDR			(AHB1PERIPH_BASE + 0xB000)
#define USBOTGHG_BASEADDR		(AHB1PERIPH_BASE + 0x40000)

/*
 * Base addresses of peripherals which are hanging on AHB2 bus
 * TODO: Complete for all other peripherals
 */

#define USBOTGFS_BASEADDR		(AHB2PERIPH_BASE + 0x0000)
#define DCMI_BASEADDR			(AHB2PERIPH_BASE + 0x50000)
#define CRYP_BASEADDR			(AHB2PERIPH_BASE + 0x60000)
#define HASH_BASEADDR			(AHB2PERIPH_BASE + 0x60400)
#define RNG_BASEADDR			(AHB2PERIPH_BASE + 0x60800)

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 * TODO: Complete for all other peripherals
 */

#define TIM2_BASEADDR			(APB1PERIPH_BASE + 0x0000)
#define TIM3_BASEADDR			(APB1PERIPH_BASE + 0x0400)
#define TIM4_BASEADDR			(APB1PERIPH_BASE + 0x0800)
#define TIM5_BASEADDR			(APB1PERIPH_BASE + 0x0C00)
#define TIM6_BASEADDR			(APB1PERIPH_BASE + 0x1000)
#define TIM7_BASEADDR			(APB1PERIPH_BASE + 0x1400)
#define TIM12_BASEADDR			(APB1PERIPH_BASE + 0x1800)
#define TIM13_BASEADDR			(APB1PERIPH_BASE + 0x1C00)
#define TIM14_BASEADDR			(APB1PERIPH_BASE + 0x2000)
#define RCCBKP_BASEADDR			(APB1PERIPH_BASE + 0x2800)
#define WWDG_BASEADDR			(APB1PERIPH_BASE + 0x2C00)
#define IWDG_BASEADDR			(APB1PERIPH_BASE + 0x3000)
#define I2S2_BASEADDR			(APB1PERIPH_BASE + 0x3400)
#define SPI2I2S2_BASEADDR		(APB1PERIPH_BASE + 0x3800)
#define SPI3I2S3_BASEADDR		(APB1PERIPH_BASE + 0x3C00)
#define I2S3_BASEADDR			(APB1PERIPH_BASE + 0x4000)
#define USART2_BASEADDR			(APB1PERIPH_BASE + 0x4400)
#define USART3_BASEADDR			(APB1PERIPH_BASE + 0x4800)
#define UART4_BASEADDR			(APB1PERIPH_BASE + 0x4C00)
#define UART5_BASEADDR			(APB1PERIPH_BASE + 0x5000)
#define I2C1_BASEADDR			(APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR			(APB1PERIPH_BASE + 0x5800)
#define I2C3_BASEADDR			(APB1PERIPH_BASE + 0x5C00)
#define CAN1_BASEADDR			(APB1PERIPH_BASE + 0x6400)
#define CAN2_BASEADDR			(APB1PERIPH_BASE + 0x6800)
#define PWR_BASEADDR			(APB1PERIPH_BASE + 0x7000)
#define DAC_BASEADDR			(APB1PERIPH_BASE + 0x7400)
#define UART7_BASEADDR			(APB1PERIPH_BASE + 0x7800)
#define UART8_BASEADDR			(APB1PERIPH_BASE + 0x7C00)

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 * TODO: Complete for all other peripherals
 */

#define TIM1_BASEADDR			(APB2PERIPH_BASE + 0x0000)
#define TIM8_BASEADDR			(APB2PERIPH_BASE + 0x0400)
#define USART1_BASEADDR			(APB2PERIPH_BASE + 0x1000)
#define USART6_BASEADDR			(APB2PERIPH_BASE + 0x1400)
#define ADC1_BASEADDR			(APB2PERIPH_BASE + 0x2000)
#define SDIO_BASEADDR			(APB2PERIPH_BASE + 0x2C00)
#define SPI1_BASEADDR			(APB2PERIPH_BASE + 0x3000)
#define SPI4_BASEADDR			(APB2PERIPH_BASE + 0x3400)
#define SYSCFG_BASEADDR			(APB2PERIPH_BASE + 0x3800)
#define EXTI_BASEADDR			(APB2PERIPH_BASE + 0x3C00)
#define TIM9_BASEADDR			(APB2PERIPH_BASE + 0x4000)
#define TIM10_BASEADDR			(APB2PERIPH_BASE + 0x4400)
#define TIM11_BASEADDR			(APB2PERIPH_BASE + 0x4800)
#define SPI5_BASEADDR			(APB2PERIPH_BASE + 0x5000)
#define SPI6_BASEADDR			(APB2PERIPH_BASE + 0x5400)
#define SAI1_BASEADDR			(APB2PERIPH_BASE + 0x5800)
#define LCDTFT_BASEADDR			(APB2PERIPH_BASE + 0x6800)


/**************************Peripheral register definition structures***********************/

/*
 * Note: Register of a peripheral are specific to MCU
 * e.g: Number of Register of SPI peripheral of STM32F4XX family of MCUs may be different(more or less)
 * Compared to number of registers of SPI peripheral of STM32Lx or STM32F0x family of MCUs
 * Please check your Device RM
 */

/*
 * Peripheral register definition structure for GPIO
 */

typedef struct
{
	__vo uint32_t	MODER;		/* GPIO port mode register, 				Address offset: 0x00 */
	__vo uint32_t	OTYPER;		/* GPIO port output type register, 			Address offset: 0x04 */
	__vo uint32_t	OSPEEDR;	/* GPIO port output speed register, 		Address offset: 0x08 */
	__vo uint32_t	PUPDR;		/* GPIO port pull-up/pull-down register,	Address offset: 0x0C */
	__vo uint32_t	IDR;		/* GPIO port input data register, 			Address offset: 0x10 */
	__vo uint32_t	ODR;		/* GPIO port output data register, 			Address offset: 0x14 */
	__vo uint32_t	BSRR;		/* GPIO port bit set/reset register, 		Address offset: 0x18 */
	__vo uint32_t	LCKR;		/* GPIO port configuration lock register,	Address offset: 0x1C */
	__vo uint32_t	AFRL;		/* GPIO alternate function low register, 	Address offset: 0x20 */
	__vo uint32_t	AFRH;		/* GPIO alternate function high register, 	Address offset: 0x24 */
}GPIO_RegDef_t;


/*
 * Peripheral register definition structure for RCC
 */

typedef struct
{
	__vo uint32_t	CR;				/* RCC clock control register, 									Address offset: 0x00 */
	__vo uint32_t	PLLCFGR;		/* RCC PLL configuration register, 								Address offset: 0x04 */
	__vo uint32_t	CFGR;			/* RCC clock configuration register, 							Address offset: 0x08 */
	__vo uint32_t	CIR;			/* RCC clock interrupt register, 								Address offset: 0x0C */
	__vo uint32_t	AHB1RSTR;		/* RCC AHB1 peripheral reset register, 							Address offset: 0x10 */
	__vo uint32_t	AHB2RSTR;		/* RCC AHB2 peripheral reset register, 							Address offset: 0x14 */
	__vo uint32_t	AHB3RSTR;		/* RCC AHB3 peripheral reset register, 							Address offset: 0x18 */
	__vo uint32_t	Reserved;		/* Reserved														 	 				 */
	__vo uint32_t	APB1RSTR;		/* RCC APB1 peripheral reset register, 							Address offset: 0x20 */
	__vo uint32_t	APB2RSTR;		/* RCC APB2 peripheral reset register, 							Address offset: 0x24 */
	__vo uint32_t	Reserved1[2];	/* Reserved														 	 				 */
	__vo uint32_t	AHB1ENR;		/* RCC AHB1 peripheral clock enable register,					Address offset: 0x30 */
	__vo uint32_t	AHB2ENR;		/* RCC AHB2 peripheral clock enable register, 					Address offset: 0x34 */
	__vo uint32_t	AHB3ENR;		/* RCC AHB3 peripheral clock enable register, 					Address offset: 0x38 */
	__vo uint32_t	Reserved2;		/* Reserved														 	 				 */
	__vo uint32_t	APB1ENR;		/* RCC APB1 peripheral clock enable register, 					Address offset: 0x40 */
	__vo uint32_t	APB2ENR;		/* RCC APB2 peripheral clock enable register, 					Address offset: 0x44 */
	__vo uint32_t	Reserved3[2];	/* Reserved														 	 				 */
	__vo uint32_t	AHB1LPENR;		/* RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
	__vo uint32_t	AHB2LPENR;		/* RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54 */
	__vo uint32_t	AHB3LPENR;		/* RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58 */
	__vo uint32_t	Reserved4;		/* Reserved														 	 				 */
	__vo uint32_t	APB1LPENR;		/* RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
	__vo uint32_t	APB2LPENR;		/* RCC APB2 peripheral clock enabled in low power mode register,Address offset: 0x64 */
	__vo uint32_t	Reserved5[2];	/* Reserved														 	 				 */
	__vo uint32_t	BDCR;			/* RCC Backup domain control register, 							Address offset: 0x70 */
	__vo uint32_t	CSR;			/* RCC clock control & status register, 						Address offset: 0x74 */
	__vo uint32_t	Reserved6[2];	/* Reserved														 	 				 */
	__vo uint32_t	SSCGR;			/* RCC spread spectrum clock generation register, 				Address offset: 0x80 */
	__vo uint32_t	PLLI2SCFGR;		/* RCC PLLI2S configuration register, 							Address offset: 0x84 */
}RCC_RegDef_t;

/*
 * Peripheral register definition structure for PWR
 */
typedef struct
{
	__vo uint32_t	CR;				/* PWR power control register (PWR_CR),			Address offset: 0x00 */
	__vo uint32_t	CSR;			/* PWR power control/status register (PWR_CSR), Address offset: 0x04 */
}PWR_RegDef_t;

/*
 * Peripheral register definition structure for EXTI
 */

typedef struct
{
	__vo uint32_t	IMR;			/* Interrupt mask register,				Address offset: 0x00 */
	__vo uint32_t	EMR;			/* Event mask register,					Address offset: 0x04 */
	__vo uint32_t	RTSR;			/* Rising trigger selection register,	Address offset: 0x08 */
	__vo uint32_t	FTSR;			/* Falling trigger selection register,	Address offset: 0x0C */
	__vo uint32_t	SWIER;			/* Software interrupt event register,	Address offset: 0x10 */
	__vo uint32_t	PR;				/* Pending register,					Address offset: 0x14 */
}EXTI_RegDef_t;


/*
 * Peripheral register definition structure for GPIO
 */

typedef struct
{
	__vo uint32_t	MEMRMP;			/* SYSCFG memory remap register,						Address offset: 0x00 */
	__vo uint32_t	PMC;			/* SYSCFG peripheral mode configuration register,		Address offset: 0x04 */
	__vo uint32_t	EXTICR[4];		/* SYSCFG external interrupt configuration register,	Address offset: 0x08 */
	__vo uint32_t	RESERVED[2];
	__vo uint32_t	CMPCR;			/* Compensation cell control register,					Address offset: 0x20 */
	__vo uint32_t	RESERVED1[2];
	__vo uint32_t 	CFGR;
}SYSCFG_RegDef_t;


/*
 * Peripheral register definition structure for SPI
 */

typedef struct
{
	__vo uint32_t	CR1;		/* SPI control register 1,						Address offset: 0x00 */
	__vo uint32_t	CR2;		/* SPI control register 2,						Address offset: 0x04 */
	__vo uint32_t	SR;			/* SPI status register,							Address offset: 0x08 */
	__vo uint32_t	DR;			/* SPI data register,							Address offset: 0x0C */
	__vo uint32_t	CRCPR;		/* SPI CRC polynomial register,					Address offset: 0x10 */
	__vo uint32_t	RXCRCR;		/* SPI RX CRC register,							Address offset: 0x14 */
	__vo uint32_t	TXCRCR;		/* SPI TX CRC register,							Address offset: 0x18 */
	__vo uint32_t	I2SCFGR;	/* SPI_I2S configuration register,				Address offset: 0x1C */
	__vo uint32_t	I2SPR;		/* SPI_I2S prescaler register,					Address offset: 0x20 */
}SPI_RegDef_t;

/*
 * Peripheral register definition structure for I2C
 */
typedef struct
{
	__vo uint32_t	CR1;		/* I2C Control register 1,			Address offset: 0x00 */
	__vo uint32_t	CR2;		/* I2C Control register 2,			Address offset: 0x04 */
	__vo uint32_t	OAR1;		/* I2C Own address register 1,		Address offset: 0x08 */
	__vo uint32_t	OAR2;		/* I2C Own address register 1,		Address offset: 0x0C */
	__vo uint32_t	DR;			/* I2C Data register,				Address offset: 0x10 */
	__vo uint32_t	SR1;		/* I2C Status register 1, 			Address offset: 0x14 */
	__vo uint32_t	SR2;		/* I2C Status register 1, 			Address offset: 0x18 */
	__vo uint32_t	CCR;		/* I2C Clock control register,		Address offset: 0x1C */
	__vo uint32_t	TRISE;		/* I2C TRISE register,				Address offset: 0x20 */
	__vo uint32_t	FLTR;		/* I2C FLTR register,				Address offset: 0x24 */
}I2C_RegDef_t;

/*
 * Peripheral register definition structure for USART
 */
typedef struct
{
	__vo uint32_t	SR;			/* Status register,						Address offset: 0x00 */
	__vo uint32_t	DR;			/* Data register,						Address offset: 0x04 */
	__vo uint32_t	BRR;		/* Baud rate register,					Address offset: 0x08 */
	__vo uint32_t	CR1;		/* Control register 1,					Address offset: 0x0C */
	__vo uint32_t	CR2;		/* Control register 2,					Address offset: 0x10 */
	__vo uint32_t	CR3;		/* Control register 3,					Address offset: 0x14 */
	__vo uint32_t	GTPR;		/* Guard time and prescaler register,	Address offset: 0x18 */
}USART_RegDef_t;

/*
 * Peripheral definition (Peripheral base addresses type casted to xxx_RegDef_t
 */
#define GPIOA			((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB			((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC			((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define GPIOD			((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define GPIOE			((GPIO_RegDef_t *)GPIOE_BASEADDR)
#define GPIOF			((GPIO_RegDef_t *)GPIOF_BASEADDR)
#define GPIOG			((GPIO_RegDef_t *)GPIOG_BASEADDR)
#define GPIOH			((GPIO_RegDef_t *)GPIOH_BASEADDR)
#define GPIOI			((GPIO_RegDef_t *)GPIOI_BASEADDR)
#define GPIOJ			((GPIO_RegDef_t *)GPIOJ_BASEADDR)
#define GPIOK			((GPIO_RegDef_t *)GPIOK_BASEADDR)

#define RCC				((RCC_RegDef_t *)RCC_BASEADDR)

#define PWR				((PWR_RegDef_t *)PWR_BASEADDR)

#define EXTI			((EXTI_RegDef_t *)EXTI_BASEADDR)

#define SYSCFG			((SYSCFG_RegDef_t *) SYSCFG_BASEADDR)

#define SPI1			((SPI_RegDef_t *) SPI1_BASEADDR)
#define SPI2			((SPI_RegDef_t *) SPI2I2S2_BASEADDR)
#define SPI3			((SPI_RegDef_t *) SPI3I2S3_BASEADDR)
#define SPI4			((SPI_RegDef_t *) SPI4_BASEADDR)

#define I2C1			((I2C_RegDef_t *) I2C1_BASEADDR)
#define I2C2			((I2C_RegDef_t *) I2C2_BASEADDR)
#define I2C3			((I2C_RegDef_t *) I2C3_BASEADDR)

#define USART1			((USART_RegDef_t *) USART1_BASEADDR)
#define USART2			((USART_RegDef_t *) USART2_BASEADDR)
#define USART3			((USART_RegDef_t *) USART3_BASEADDR)
#define UART4			((USART_RegDef_t *) UART4_BASEADDR)
#define UART5			((USART_RegDef_t *) UART5_BASEADDR)
#define USART6			((USART_RegDef_t *) USART6_BASEADDR)


/*
 * Clock Enables Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN()	( RCC->AHB1ENR |= ( 1 << RCC_AHB1ENR_GPIOAEN ) )
#define GPIOB_PCLK_EN()	( RCC->AHB1ENR |= ( 1 << RCC_AHB1ENR_GPIOBEN ) )
#define GPIOC_PCLK_EN()	( RCC->AHB1ENR |= ( 1 << RCC_AHB1ENR_GPIOCEN ) )
#define GPIOD_PCLK_EN()	( RCC->AHB1ENR |= ( 1 << RCC_AHB1ENR_GPIODEN ) )
#define GPIOE_PCLK_EN()	( RCC->AHB1ENR |= ( 1 << RCC_AHB1ENR_GPIOEEN ) )
#define GPIOF_PCLK_EN()	( RCC->AHB1ENR |= ( 1 << RCC_AHB1ENR_GPIOFEN ) )
#define GPIOG_PCLK_EN()	( RCC->AHB1ENR |= ( 1 << RCC_AHB1ENR_GPIOGEN ) )
#define GPIOH_PCLK_EN()	( RCC->AHB1ENR |= ( 1 << RCC_AHB1ENR_GPIOHEN ) )
#define GPIOI_PCLK_EN()	( RCC->AHB1ENR |= ( 1 << RCC_AHB1ENR_GPIOIEN ) )


/*
 * Clock Enables Macros for I2Cx peripheral
 */

#define I2C1_PCLK_EN()	( RCC->APB1ENR |= ( 1 << RCC_APB1ENR_I2C1EN ) )
#define I2C2_PCLK_EN()	( RCC->APB1ENR |= ( 1 << RCC_APB1ENR_I2C2EN ) )
#define I2C3_PCLK_EN()	( RCC->APB1ENR |= ( 1 << RCC_APB1ENR_I2C3EN ) )




/*
 * Clock Enables Macros for SPIx peripheral
 */

#define SPI1_PCLK_EN()	( RCC->APB2ENR |= ( 1 << RCC_APB2ENR_SPI1EN ) )
#define SPI2_PCLK_EN()	( RCC->APB1ENR |= ( 1 << RCC_APB1ENR_SPI2EN ) )
#define SPI3_PCLK_EN()	( RCC->APB1ENR |= ( 1 << RCC_APB1ENR_SPI3EN ) )
#define SPI4_PCLK_EN()	( RCC->APB2ENR |= ( 1 << 13 ) )




/*
 * Clock Enables Macros for UARTx peripheral
 */

#define USART1_PCLK_EN()	( RCC->APB2ENR |= ( 1 << RCC_APB2ENR_USART1EN ) )
#define USART2_PCLK_EN()	( RCC->APB1ENR |= ( 1 << RCC_APB1ENR_UART2EN ) )
#define USART3_PCLK_EN()	( RCC->APB1ENR |= ( 1 << RCC_APB1ENR_UART3EN ) )
#define UART4_PCLK_EN()		( RCC->APB1ENR |= ( 1 << RCC_APB1ENR_UART4EN ) )
#define UART5_PCLK_EN()		( RCC->APB1ENR |= ( 1 << RCC_APB1ENR_UART5EN ) )
#define USART6_PCLK_EN()	( RCC->APB2ENR |= ( 1 << RCC_APB2ENR_USART6EN ) )



/*
 * Clock Enables Macros for SYSCFG peripheral
 */

#define SYSCFG_PCKL_EN()	( RCC->APB2ENR |= ( 1 << RCC_APB2ENR_SYSCFGEN ) )




/*
 * Clock Disable Macros for GPIOx peripheral
 */

#define GPIOA_PCLK_DIS()	( RCC->AHB1ENR &= ~( 1 << RCC_AHB1ENR_GPIOAEN) )
#define GPIOB_PCLK_DIS()	( RCC->AHB1ENR &= ~( 1 << RCC_AHB1ENR_GPIOBEN ) )
#define GPIOC_PCLK_DIS()	( RCC->AHB1ENR &= ~( 1 << RCC_AHB1ENR_GPIOCEN ) )
#define GPIOD_PCLK_DIS()	( RCC->AHB1ENR &= ~( 1 << RCC_AHB1ENR_GPIODEN ) )
#define GPIOE_PCLK_DIS()	( RCC->AHB1ENR &= ~( 1 << RCC_AHB1ENR_GPIOEEN ) )
#define GPIOF_PCLK_DIS()	( RCC->AHB1ENR &= ~( 1 << RCC_AHB1ENR_GPIOFEN ) )
#define GPIOG_PCLK_DIS()	( RCC->AHB1ENR &= ~( 1 << RCC_AHB1ENR_GPIOGEN ) )
#define GPIOH_PCLK_DIS()	( RCC->AHB1ENR &= ~( 1 << RCC_AHB1ENR_GPIOHEN ) )
#define GPIOI_PCLK_DIS()	( RCC->AHB1ENR &= ~( 1 << RCC_AHB1ENR_GPIOIEN ) )


/*
 * Clock Disables Macros for I2Cx peripheral
 */

#define I2C1_PCLK_DIS()	( RCC->APB1ENR &= ~( 1 << RCC_APB1ENR_I2C1EN ) )
#define I2C2_PCLK_DIS()	( RCC->APB1ENR &= ~( 1 << RCC_APB1ENR_I2C2EN ) )
#define I2C3_PCLK_DIS()	( RCC->APB1ENR &= ~( 1 << RCC_APB1ENR_I2C3EN ) )


/*
 * Clock Disables Macros for SPIx peripheral
 */

#define SPI1_PCLK_DIS()	( RCC->APB2ENR &= ~( 1 << RCC_APB2ENR_SPI1EN ) )
#define SPI2_PCLK_DIS()	( RCC->APB1ENR &= ~( 1 << RCC_APB1ENR_SPI2EN ) )
#define SPI3_PCLK_DIS()	( RCC->APB1ENR &= ~( 1 << RCC_APB1ENR_SPI3EN ) )
#define SPI4_PCLK_DIS()	( RCC->APB2ENR &= ~( 1 << 13 ) )



/*
 * Clock Disables Macros for UARTx peripheral
 */

#define USART1_PCLK_DIS()	( RCC->APB2ENR &= ~( 1 << RCC_APB2ENR_USART1EN ) )
#define USART2_PCLK_DIS()	( RCC->APB1ENR &= ~( 1 << RCC_APB1ENR_UART2EN ) )
#define USART3_PCLK_DIS()	( RCC->APB1ENR &= ~( 1 << RCC_APB1ENR_UART3EN ) )
#define UART4_PCLK_DIS()	( RCC->APB1ENR &= ~( 1 << RCC_APB1ENR_UART4EN ) )
#define UART5_PCLK_DIS()	( RCC->APB1ENR &= ~( 1 << RCC_APB1ENR_UART5EN ) )
#define USART6_PCLK_DIS()	( RCC->APB2ENR &= ~( 1 << RCC_APB2ENR_USART6EN ) )


/*
 * Clock Disables Macros for SYSCFG peripheral
 */

#define SYSCFG_PCKL_DIS()	( RCC->APB2ENR &= ~( 1 << 14 ) )


/*
 * GPIO reset Macros for GPIOx peripherals
 */
#define GPIOA_REG_RESET()	do { ( RCC->AHB1RSTR |= ( 1 << RCC_AHB1RSTR_GPIOARST ) );  ( RCC->AHB1RSTR &= ~( 1 << RCC_AHB1RSTR_GPIOARST ) ); } while(0);
#define GPIOB_REG_RESET()	do { ( RCC->AHB1RSTR |= ( 1 << RCC_AHB1RSTR_GPIOBRST ) );  ( RCC->AHB1RSTR &= ~( 1 << RCC_AHB1RSTR_GPIOBRST ) ); } while(0);
#define GPIOC_REG_RESET()	do { ( RCC->AHB1RSTR |= ( 1 << RCC_AHB1RSTR_GPIOCRST ) );  ( RCC->AHB1RSTR &= ~( 1 << RCC_AHB1RSTR_GPIOCRST ) ); } while(0);
#define GPIOD_REG_RESET()	do { ( RCC->AHB1RSTR |= ( 1 << RCC_AHB1RSTR_GPIODRST ) );  ( RCC->AHB1RSTR &= ~( 1 << RCC_AHB1RSTR_GPIODRST ) ); } while(0);
#define GPIOE_REG_RESET()	do { ( RCC->AHB1RSTR |= ( 1 << RCC_AHB1RSTR_GPIOERST ) );  ( RCC->AHB1RSTR &= ~( 1 << RCC_AHB1RSTR_GPIOERST ) ); } while(0);
#define GPIOF_REG_RESET()	do { ( RCC->AHB1RSTR |= ( 1 << RCC_AHB1RSTR_GPIOFRST ) );  ( RCC->AHB1RSTR &= ~( 1 << RCC_AHB1RSTR_GPIOFRST ) ); } while(0);
#define GPIOG_REG_RESET()	do { ( RCC->AHB1RSTR |= ( 1 << RCC_AHB1RSTR_GPIOGRST ) );  ( RCC->AHB1RSTR &= ~( 1 << RCC_AHB1RSTR_GPIOGRST ) ); } while(0);
#define GPIOH_REG_RESET()	do { ( RCC->AHB1RSTR |= ( 1 << RCC_AHB1RSTR_GPIOHRST ) );  ( RCC->AHB1RSTR &= ~( 1 << RCC_AHB1RSTR_GPIOHRST ) ); } while(0);
#define GPIOI_REG_RESET()	do { ( RCC->AHB1RSTR |= ( 1 << RCC_AHB1RSTR_GPIOIRST ) );  ( RCC->AHB1RSTR &= ~( 1 << RCC_AHB1RSTR_GPIOIRST ) ); } while(0);

/*
 * SPI reset Macros for SPIx peripherals
 */
#define SPI1_REG_RESET()	do { ( RCC->APB2RSTR |= ( 1 << RCC_APB2RSTR_SPI1RST ) ); ( RCC->APB2RSTR &= ~( 1 << RCC_APB2RSTR_SPI1RST ) ); } while(0);
#define SPI2_REG_RESET()	do { ( RCC->APB1RSTR |= ( 1 << RCC_APB1RSTR_SPI2RST ) ); ( RCC->APB2RSTR &= ~( 1 << RCC_APB1RSTR_SPI2RST ) ); } while(0);
#define SPI3_REG_RESET()	do { ( RCC->APB1RSTR |= ( 1 << RCC_APB1RSTR_SPI3RST ) ); ( RCC->APB2RSTR &= ~( 1 << RCC_APB1RSTR_SPI3RST ) ); } while(0);

/*
 * I2C reset Macros for SPIx peripherals
 */
#define I2C1_REG_RESET()	do { ( RCC->APB1RSTR |= ( 1 << RCC_APB1RSTR_I2C1RST ) ); ( RCC->APB1RSTR &= ~( 1 << RCC_APB1RSTR_I2C1RST ) ); } while(0);
#define I2C2_REG_RESET()	do { ( RCC->APB1RSTR |= ( 1 << RCC_APB1RSTR_I2C2RST ) ); ( RCC->APB1RSTR &= ~( 1 << RCC_APB1RSTR_I2C1RST ) ); } while(0);
#define I2C3_REG_RESET()	do { ( RCC->APB1RSTR |= ( 1 << RCC_APB1RSTR_I2C3RST ) ); ( RCC->APB1RSTR &= ~( 1 << RCC_APB1RSTR_I2C3RST ) ); } while(0);

/*
 * USART reset Macros for SPIx peripherals
 */
#define USART1_REG_RESET() 	do { ( RCC->APB2RSTR |= ( 1 << RCC_APB2RSTR_USART1RST ) ); ( RCC->APB2RSTR &= ~( 1 << RCC_APB2RSTR_USART1RST) ); } while(0);
#define USART2_REG_RESET()	do { ( RCC->APB1RSTR |= ( 1 << RCC_APB1RSTR_UART2RST ) ); ( RCC->APB1RSTR &= ~( 1 << RCC_APB1RSTR_UART2RST ) ); } while(0);
#define USART3_REG_RESET()	do { ( RCC->APB1RSTR |= ( 1 << RCC_APB1RSTR_UART3RST ) ); ( RCC->APB1RSTR &= ~( 1 << RCC_APB1RSTR_UART3RST ) ); } while(0);
#define UART4_REG_RESET()	do { ( RCC->APB1RSTR |= ( 1 << RCC_APB1RSTR_UART4RST ) ); ( RCC->APB1RSTR &= ~( 1 << RCC_APB1RSTR_UART4RST ) ); } while(0);
#define UART5_REG_RESET()	do { ( RCC->APB1RSTR |= ( 1 << RCC_APB1RSTR_UART5RST ) ); ( RCC->APB1RSTR &= ~( 1 << RCC_APB1RSTR_UART5RST ) ); } while(0);
#define USART6_REG_RESET()	do { ( RCC->APB2RSTR |= ( 1 << RCC_APB2RSTR_USART6RST ) ); ( RCC->APB2RSTR &= ~( 1 << RCC_APB2RSTR_USART6RST ) ); } while(0);
/*
 * returns port code for given GPIO base address
 */

#define GPIO_BASEADDR_TO_CODE( x )	  (	( x == GPIOA ) ? 0 : \
										( x == GPIOB ) ? 1 : \
										( x == GPIOC ) ? 2 : \
										( x == GPIOD ) ? 3 : \
										( x == GPIOE ) ? 4 : \
										( x == GPIOF ) ? 5 : \
										( x == GPIOG ) ? 6 : \
										( x == GPIOH ) ? 7 : \
										( x == GPIOI ) ? 8 : 0 )

/*
 * IRQ(Interrupt request) Number of STM32F407XX MCU
 * NOTE: update these macros with valid values according to your MCU
 * TODO: You may complete this list for other peripherals
 */

#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40

/*
 * IRQ number for SPI
 */

#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2			36
#define IRQ_NO_SPI3			51

/*
* IRQ number for I2C
*/
#define IRQ_NO_I2C1_EV		31
#define IRQ_NO_I2C1_ER		32
#define IRQ_NO_I2C2_EV		33
#define IRQ_NO_I2C2_ER		34
#define IRQ_NO_I2C3_EV		72
#define IRQ_NO_I2C3_ER		73

/*
* IRQ number for USARTx
*/
#define IRQ_NO_USART1		37
#define IRQ_NO_USART2		38
#define IRQ_NO_USART3		39
#define IRQ_NO_UART4		52
#define IRQ_NO_UART5		53
#define IRQ_NO_USART6		71

/*
 * macros for all the possible priority levels
 */

#define NVIC_IRQ_PRI0		0
#define NVIC_IRQ_PRI1		1
#define NVIC_IRQ_PRI2		2
#define NVIC_IRQ_PRI3		3
#define NVIC_IRQ_PRI4		4
#define NVIC_IRQ_PRI5		5
#define NVIC_IRQ_PRI6		6
#define NVIC_IRQ_PRI7		7
#define NVIC_IRQ_PRI8		8
#define NVIC_IRQ_PRI9		9
#define NVIC_IRQ_PRI10		10
#define NVIC_IRQ_PRI11		11
#define NVIC_IRQ_PRI12		12
#define NVIC_IRQ_PRI13		13
#define NVIC_IRQ_PRI14		14
#define NVIC_IRQ_PRI15		15



/*
 * Some Generic macros
 */

#define ENABLE			1
#define DISABLE			0
#define SET				ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET
#define FLAG_RESET		RESET
#define FLAG_SET		SET
#define TRUE			SET
#define FALSE			RESET


/*****************************************************************************
 * Bit position definitions of SPI peripheral
 *****************************************************************************/

/*
 * SPI control register 1 (SPI_CR1)
 */
#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSBFIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_CRCEN		13
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE	15

/*
 * SPI control register 2 (SPI_CR2)
 */
#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7

/*
 * SPI status register (SPI_SR)
 */
#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8


/*****************************************************************************
 * Bit position definitions of I2C peripheral
 *****************************************************************************/
/*
 * Bit position definitions I2C_CR1
 */
#define	I2C_CR1_PE			0
#define I2C_CR1_SMBUS		1
#define I2C_CR1_SMBTYPE		3
#define I2C_CR1_ENARB		4
#define I2C_CR1_ENPEC		5
#define I2C_CR1_ENGC		6
#define I2C_CR1_NOSTRETCH	7
#define I2C_CR1_START		8
#define I2C_CR1_STOP		9
#define I2C_CR1_ACK			10
#define I2C_CR1_POS			11
#define I2C_CR1_PEC			12
#define I2C_CR1_ALERT		13
#define I2C_CR1_SWRST		15


/*
 * Bit position definitions I2C_CR2
 */
#define I2C_CR2_FREQ		0
#define I2C_CR2_ITERREN		8
#define I2C_CR2_ITEVTEN		9
#define I2C_CR2_ITBUFEN		10
#define I2C_CR2_DMAEN		11
#define I2C_CR2_LAST		12

/*
 * Bit position definitions I2C_OAR1
 */
#define I2C_OAR1_ADD0		0
#define I2C_OAR1_ADD1		1
#define I2C_OAR1_ADD2		8
#define I2C_OAR1_ADDMODE	15

/*
 * Bit position definitions I2C_OAR2
 */
#define I2C_OAR2_ENDUAL		0
#define I2C_OAR2_ADD2		1

/*
 * Bit position definitions I2C_SR1
 */
#define I2C_SR1_SB			0
#define I2C_SR1_ADDR		1
#define I2C_SR1_BTF			2
#define I2C_SR1_ADD10		3
#define I2C_SR1_STOPF		4
#define I2C_SR1_RXNE		6
#define I2C_SR1_TXE			7
#define I2C_SR1_BERR		8
#define I2C_SR1_ARLO		9
#define I2C_SR1_AF			10
#define I2C_SR1_OVR			11
#define I2C_SR1_PECERR		12
#define I2C_SR1_TIMEOUT		14
#define I2C_SR1_SMBALERT	15

/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL			0
#define I2C_SR2_BUSY		1
#define I2C_SR2_TRA			2
#define I2C_SR2_GENCALL		4
#define I2C_SR2_SMBDEFAULT	5
#define I2C_SR2_SMBHOST		6
#define I2C_SR2_DUALF		7
#define I2C_SR2_PEC			8

/*
 * Bit position definitions I2C_CCR
 */
#define I2C_CCR_CCR			0
#define I2C_CCR_DUTY		14
#define I2C_CCR_FS			15

/*****************************************************************************
 * Bit position definitions of USART peripheral
 *****************************************************************************/

/*
 * Bit position definitions USART_SR
 */
#define USART_SR_PE			0
#define USART_SR_FE			1
#define USART_SR_NF			2
#define USART_SR_ORE		3
#define USART_SR_IDLE		4
#define USART_SR_RXNE		5
#define USART_SR_TC			6
#define USART_SR_TXE		7
#define USART_SR_LBD		8
#define USART_SR_CTS		9


/*
 * Bit position definitions USART_BRR
 */
#define USART_BRR_DIV_FRAC	0
#define USART_BRR_DIV_MANT	4

/*
 * Bit position definitions USART_CR1
 */
#define USART_CR1_SBK		0
#define USART_CR1_RWU		1
#define USART_CR1_RE		2
#define USART_CR1_TE		3
#define USART_CR1_IDLEIE	4
#define USART_CR1_RXNEIE	5
#define USART_CR1_TCIE		6
#define USART_CR1_TXEIE		7
#define USART_CR1_PEIE		8
#define USART_CR1_PS		9
#define USART_CR1_PCE		10
#define USART_CR1_WAKE		11
#define USART_CR1_M			12
#define USART_CR1_UE		13
#define USART_CR1_OVER8		15

/*
 * Bit position definitions USART_CR2
 */
#define USART_CR2_ADD		0
#define USART_CR2_LBDL		5
#define USART_CR2_LBDIE		6
#define USART_CR2_LBCL		8
#define USART_CR2_CPHA		9
#define USART_CR2_CPOL		10
#define USART_CR2_CLKEN		11
#define USART_CR2_STOP		12
#define USART_CR2_LINEN		14

/*
 * Bit position definitions USART_CR3
 */
#define USART_CR3_EIE		0
#define USART_CR3_IREN		1
#define USART_CR3_IRLP		2
#define USART_CR3_HDSEL		3
#define USART_CR3_NACK		4
#define USART_CR3_SCEN		5
#define USART_CR3_DMAR		6
#define USART_CR3_DMAT		7
#define USART_CR3_RTSE		8
#define USART_CR3_CTSE		9
#define USART_CR3_CTSIE		10
#define USART_CR3_ONEBIT	11

/*
 * Bit position definitions USART_GTPR
 */
#define USART_GTPR_PSC		0
#define USART_GTPR_GT		8

/*****************************************************************************
 * Bit position definitions of RCC peripheral
 *****************************************************************************/

/*
 * Bit position definitions RCC_CR
 */
#define RCC_CR_HSION		0
#define RCC_CR_HSIRDY		1
#define RCC_CR_HSITRIM		3
#define RCC_CR_HSICAL		8
#define RCC_CR_HSEON		16
#define RCC_CR_HSERDY		17
#define RCC_CR_HSEBYP		18
#define RCC_CR_CSSON		19
#define RCC_CR_PLLON		24
#define RCC_CR_PLLRDY		25
#define RCC_CR_PLLI2SON		26
#define RCC_CR_PLLI2SRDY	27

/*
 * Bit position definitions RCC_PLLCFGR
 */
#define RCC_PLLCFGR_PLLM	0
#define RCC_PLLCFGR_PLLN	6
#define RCC_PLLCFGR_PLLP	16
#define RCC_PLLCFGR_PLLSRC	22
#define RCC_PLLCFGR_PLLQ	24

/*
 * Bit position definitions RCC_CFGR
 */
#define RCC_CFGR_SW			0
#define RCC_CFGR_SWS		2
#define RCC_CFGR_HPRE		4
#define RCC_CFGR_PRE1		10
#define RCC_CFGR_PRE2		13
#define RCC_CFGR_RTCPRE		16
#define RCC_CFGR_MCO1		21
#define RCC_CFGR_I2SSCR		23
#define RCC_CFGR_MCO1PRE	24
#define RCC_CFGR_MCO2PRE	27
#define RCC_CFGR_MCO2		30

/*
 * Bit position definitions RCC_CIR
 */
#define RCC_CIR_LSIRDYF		0
#define RCC_CIR_LSERDYF		1
#define RCC_CIR_HSIRDYF		2
#define RCC_CIR_HSERDYF		3
#define RCC_CIR_PLLRDYF		4
#define RCC_CIR_PLLI2SRDYF	5
#define RCC_CIR_CSSF		7
#define RCC_CIR_LSIRDYIE	8
#define RCC_CIR_LSERDYIE	9
#define RCC_CIR_HSIRDYIE	10
#define RCC_CIR_HSERDYIE	11
#define RCC_CIR_PLLRDYIE	12
#define RCC_CIR_PLLI2SRDYIE	13
#define RCC_CIR_LSIRDYC		16
#define RCC_CIR_LSERDYC		17
#define RCC_CIR_HSIRDYC		18
#define RCC_CIR_HSERDYC		19
#define RCC_CIR_PLLRDYC		20
#define RCC_CIR_PLLI2SRDYC	21
#define RCC_CIR_CSSC		23


/*
 * Bit position definitions RCC_AHB1RSTR
 */
#define RCC_AHB1RSTR_GPIOARST	0
#define RCC_AHB1RSTR_GPIOBRST	1
#define RCC_AHB1RSTR_GPIOCRST	2
#define RCC_AHB1RSTR_GPIODRST	3
#define RCC_AHB1RSTR_GPIOERST	4
#define RCC_AHB1RSTR_GPIOFRST	5
#define RCC_AHB1RSTR_GPIOGRST	6
#define RCC_AHB1RSTR_GPIOHRST	7
#define RCC_AHB1RSTR_GPIOIRST	8
#define RCC_AHB1RSTR_CRCRST		12
#define RCC_AHB1RSTR_DMA1RST	21
#define RCC_AHB1RSTR_DMA2RST	22
#define RCC_AHB1RSTR_ETHMACRST	25
#define RCC_AHB1RSTR_OTGHSRST	29


/*
 * Bit position definitions RCC_APB1RSTR
 */
#define RCC_APB1RSTR_TIM2RST	0
#define RCC_APB1RSTR_TIM3RST	1
#define RCC_APB1RSTR_TIM4RST	2
#define RCC_APB1RSTR_TIM5RST	3
#define RCC_APB1RSTR_TIM6RST	4
#define RCC_APB1RSTR_TIM7RST	5
#define RCC_APB1RSTR_TIM12RST	6
#define RCC_APB1RSTR_TIM13RST	7
#define RCC_APB1RSTR_TIM14RST	8
#define RCC_APB1RSTR_WWDGRST	11
#define RCC_APB1RSTR_SPI2RST	14
#define RCC_APB1RSTR_SPI3RST	15
#define RCC_APB1RSTR_UART2RST	17
#define RCC_APB1RSTR_UART3RST	18
#define RCC_APB1RSTR_UART4RST	19
#define RCC_APB1RSTR_UART5RST	20
#define RCC_APB1RSTR_I2C1RST	21
#define RCC_APB1RSTR_I2C2RST	22
#define RCC_APB1RSTR_I2C3RST	23
#define RCC_APB1RSTR_CAN1RST	25
#define RCC_APB1RSTR_CAN2RST	26
#define RCC_APB1RSTR_PWRRST		28
#define RCC_APB1RSTR_DACRST		29

/*
 * Bit position definitions RCC_APB2RSTR
 */
#define RCC_APB2RSTR_TIM1RST	0
#define RCC_APB2RSTR_TIM8RST	1
#define RCC_APB2RSTR_USART1RST	4
#define RCC_APB2RSTR_USART6RST	5
#define RCC_APB2RSTR_ADCRST		8
#define RCC_APB2RSTR_SDIORST	11
#define RCC_APB2RSTR_SPI1RST	12
#define RCC_APB2RSTR_SYSCFGRST	14
#define RCC_APB2RSTR_TIM9RST	16
#define RCC_APB2RSTR_TIM10RST	17
#define RCC_APB2RSTR_TIM11RST	18


/*
 * Bit position definitions RCC_AHB1ENR
 */
#define RCC_AHB1ENR_GPIOAEN			0
#define RCC_AHB1ENR_GPIOBEN			1
#define RCC_AHB1ENR_GPIOCEN			2
#define RCC_AHB1ENR_GPIODEN			3
#define RCC_AHB1ENR_GPIOEEN			4
#define RCC_AHB1ENR_GPIOFEN			5
#define RCC_AHB1ENR_GPIOGEN			6
#define RCC_AHB1ENR_GPIOHEN			7
#define RCC_AHB1ENR_GPIOIEN			8
#define RCC_AHB1ENR_CRCEN			12
#define RCC_AHB1ENR_BKPSRAMEN		18
#define RCC_AHB1ENR_CCMDATARAMEN	20
#define RCC_AHB1ENR_DMA1EN			21
#define RCC_AHB1ENR_DMA2EN			22
#define RCC_AHB1ENR_ETHMACEN		25
#define RCC_AHB1ENR_ETHMACTXEN		26
#define RCC_AHB1ENR_ETHMACRXEN		27
#define RCC_AHB1ENR_ETHMACPTPEN		28
#define RCC_AHB1ENR_OTGHSEN			29
#define RCC_AHB1ENR_OTGHSULPIEN		30

/*
 * Bit position definitions RCC_AHB2ENR
 */
#define RCC_AHB2ENR_DCMIEN			0
#define RCC_AHB2ENR_CRYPEN			4
#define RCC_AHB2ENR_HASHEN			5
#define RCC_AHB2ENR_RNGEN			6
#define RCC_AHB2ENR_OTGFSEN			7

/*
 * Bit position definitions RCC_APB1ENR
 */
#define RCC_APB1ENR_TIM2EN		0
#define RCC_APB1ENR_TIM3EN		1
#define RCC_APB1ENR_TIM4EN		2
#define RCC_APB1ENR_TIM5EN		3
#define RCC_APB1ENR_TIM6EN		4
#define RCC_APB1ENR_TIM7EN		5
#define RCC_APB1ENR_TIM12EN		6
#define RCC_APB1ENR_TIM13EN		7
#define RCC_APB1ENR_TIM14EN		8
#define RCC_APB1ENR_WWDGEN		11
#define RCC_APB1ENR_SPI2EN		14
#define RCC_APB1ENR_SPI3EN		15
#define RCC_APB1ENR_UART2EN		17
#define RCC_APB1ENR_UART3EN		18
#define RCC_APB1ENR_UART4EN		19
#define RCC_APB1ENR_UART5EN		20
#define RCC_APB1ENR_I2C1EN		21
#define RCC_APB1ENR_I2C2EN		22
#define RCC_APB1ENR_I2C3EN		23
#define RCC_APB1ENR_CAN1EN		25
#define RCC_APB1ENR_CAN2EN		26
#define RCC_APB1ENR_PWREN		28
#define RCC_APB1ENR_DACEN		29

/*
 * Bit position definitions RCC_APB2ENR
 */
#define RCC_APB2ENR_TIM1EN		0
#define RCC_APB2ENR_TIM8EN		1
#define RCC_APB2ENR_USART1EN	4
#define RCC_APB2ENR_USART6EN	5
#define RCC_APB2ENR_ADCEN		8
#define RCC_APB2ENR_SDIOEN		11
#define RCC_APB2ENR_SPI1EN		12
#define RCC_APB2ENR_SYSCFGEN	14
#define RCC_APB2ENR_TIM9EN		16
#define RCC_APB2ENR_TIM10EN		17
#define RCC_APB2ENR_TIM11EN		18

/*
 * Bit position definitions RCC_BDCR
 */
#define RCC_BDCR_LSEON			0
#define RCC_BDCR_LSERDY			1
#define RCC_BDCR_LSEBYP			2


/*****************************************************************************
 * Bit position definitions of PWR peripheral
 *****************************************************************************/

/*
 * Bit position definitions PWR_CR
 */
#define PWR_CR_LPDS				0
#define PWR_CR_PDDS				1
#define PWR_CR_CWUF				2
#define PWR_CR_CSBF				3
#define PWR_CR_PVDE				4
#define PWR_CR_PLS				5
#define PWR_CR_DBP				8
#define PWR_CR_FPDS				9
#define PWR_CR_VOS				14






#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_i2c_driver.h"
#include "stm32f407xx_usart_driver.h"
#include "stm32f407xx_rcc_driver.h"

#endif /* INC_STM32F407XX_H_ */


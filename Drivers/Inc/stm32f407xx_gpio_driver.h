/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: May 30, 2021
 *      Author: Thanh Vinh
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"


/*
 * This is a Configuration structure for a GPIO pin
 */

typedef struct
{
	uint8_t	GPIO_PinNumber;			/*!< Possible values from @GPIO_PIN_NUMBER >*/
	uint8_t	GPIO_PinMode;			/*!< Possible values from @GPIO_PIN_MODE >*/
	uint8_t	GPIO_PinSpeed;			/*!< Possible values from @GPIO_PIN_SPEED>*/
	uint8_t	GPIO_PinOPType;			/*!< Possible values from @GPIO_PIN_TYPE >*/
	uint8_t	GPIO_PinPuPdControl;	/*!< Possible values from @GPIO_PIN_PUPD >*/
	uint8_t	GPIO_PinAltFunMode;		/*!< Possible values from @GPIO_PIN_ALT >*/
}GPIO_PinConfig_t;



/*
 * This is a handle structure for a GPIO pin
 */

typedef struct
{
	GPIO_RegDef_t		*pGPIOx;		/*!< This hold the base address of the GPIO port to which the pin belong >*/
	GPIO_PinConfig_t	GPIO_PinConfig;	/*!< This hold GPIO pin configuration settings >*/
}GPIO_Handle_t;



/*
 * @GPIO_PIN_NUMBER
 * GPIO pin number
 */

#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15



/*
 * @GPIO_PIN_MODE
 * GPIO pin possible modes
 */

#define GPIO_MODE_INPUT		0
#define GPIO_MODE_OUTPUT	1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4
#define GPIO_MODE_IT_RT		5
#define	GPIO_MODE_IT_RFT	6



/*
 * @GPIO_PIN_TYPE
 * GPIO pin possible output types
 */

#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1



/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible output speeds
 */

#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MED		1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3


/*
 * @GPIO_PIN_PUPD
 * GPIO pin pull-up or pull-down configure macros
 */
#define GPIO_NO_PUPD		0
#define	GPIO_PIN_PU			1
#define GPIO_PIN_PD			2
#define GPIO_PIN_REV		3


/*
 * @GPIO_PIN_ALT
 * GPIO pin alternate function configure macros
 */
#define	GPIO_PIN_AF0		0
#define	GPIO_PIN_AF1		1
#define	GPIO_PIN_AF2		2
#define	GPIO_PIN_AF3		3
#define	GPIO_PIN_AF4		4
#define	GPIO_PIN_AF5		5
#define	GPIO_PIN_AF6		6
#define	GPIO_PIN_AF7		7
#define	GPIO_PIN_AF8		8
#define	GPIO_PIN_AF9		9
#define	GPIO_PIN_AF10		10
#define	GPIO_PIN_AF11		11
#define	GPIO_PIN_AF12		12
#define	GPIO_PIN_AF13		13
#define	GPIO_PIN_AF14		14
#define	GPIO_PIN_AF15		15













/*****************************************************************************************
 * 						APIs supported by this driver
 * 			For more information about the APIs check the function definition
 *****************************************************************************************/

/*
 * Peripheral Clock Setup
 */
void GPIO_PeriClockControl( GPIO_RegDef_t *pGPIOx, uint8_t EnOrDis );

/*
 * Initialization and DeInit
 */
void GPIO_Init( GPIO_Handle_t *pGPIOHandle );
void GPIO_DeInit( GPIO_RegDef_t *pGPIOx );

/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin( GPIO_RegDef_t *pGPIOx, uint8_t PinNumber );
uint16_t GPIO_ReadFromInputPort( GPIO_RegDef_t *pGPIOx );
void GPIO_WriteToOutputPin( GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value );
void GPIO_WriteToOutputPort( GPIO_RegDef_t *pGPIOx, uint16_t Value );
void GPIO_ToggleOutputPin( GPIO_RegDef_t *pGPIOx, uint8_t PinNumber );

/*
 * IQR Configuration and ISR handling
 */
void GPIO_IRQInterruptConfig( uint8_t IRQRNumber, uint8_t EnOrDis );
void GPIO_IRQPriorityConfig( uint8_t IRQNumber, uint32_t IRQPriority );
void GPIO_IRQHandling( uint8_t PinNumber );












#endif /*INC_STM32F407XX_GPIO_DRIVER_H_*/


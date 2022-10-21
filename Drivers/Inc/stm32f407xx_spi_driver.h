/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Jun 8, 2021
 *      Author: vinhkuto
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"
/*
 * Configuration structure for SPI peripheral
 */

typedef struct
{
	uint8_t	SPI_DeviceMode;
	uint8_t	SPI_BusConfig;
	uint8_t	SPI_SclkSpeed;
	uint8_t	SPI_DFF;
	uint8_t	SPI_CPOL;
	uint8_t	SPI_CPHA;
	uint8_t	SPI_SSM;
}SPI_Config_t;

/*
 * Handle structure for SPI peripheral
 */

typedef struct
{
	SPI_RegDef_t	*SPIx;			/*!< This holds the base address of SPIx(x: 0,1,2) peripheral*/
	SPI_Config_t	SPIConfig;
	uint8_t			*pTxBuffer;		/*!<To store the app. Tx buffer address > */
	uint8_t			*pRxBuffer;		/*!<To store the app. RX buffer address > */
	uint32_t		TxLen;			/*!<To store TX Len > */
	uint32_t		RxLen;			/*!<To store RX Len > */
	uint8_t			TxState;		/*!<To store Tx State > */
	uint8_t			RxState;		/*!<To store Rx State > */
}SPI_Handle_t;


/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER				1
#define SPI_DEVICE_MODE_SLAVE				0


/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD					1
#define SPI_BUS_CONFIG_HD					2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY		3

/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2					1
#define SPI_SCLK_SPEED_DIV4					2
#define SPI_SCLK_SPEED_DIV8					3
#define SPI_SCLK_SPEED_DIV16				4
#define SPI_SCLK_SPEED_DIV32				5
#define SPI_SCLK_SPEED_DIV64				6
#define SPI_SCLK_SPEED_DIV128				7
#define SPI_SCLK_SPEED_DIV256				8

/*
 * @SPI_DFF
 */
#define SPI_DFF_8BITS						0
#define SPI_DFF_16BITS						1

/*
 * @SPI_CPOL
 */
#define SPI_CPOL_HIGH						1
#define SPI_CPOL_LOW						0

/*
 * @SPI_CPHA
 */
#define SPI_CPHA_HIGH						1
#define SPI_CPHA_LOW						0

/*
 * @SPI_SSM
 */
#define SPI_SSM_DIS							0
#define SPI_SSM_EN							1


/*
 * SPI related status flag definitions
 */
#define SPI_FLAG_SR_RXNE				( 1 << SPI_SR_RXNE )
#define SPI_FLAG_SR_TXE					( 1 << SPI_SR_TXE )
#define SPI_FLAG_SR_CHSIDE				( 1 << SPI_SR_CHSIDE )
#define SPI_FLAG_SR_UDR					( 1 << SPI_SR_UDR )
#define SPI_FLAG_SR_CRCERR				( 1 << SPI_SR_CRCERR )
#define SPI_FLAG_SR_MODF				( 1 << SPI_SR_MODF )
#define SPI_FLAG_SR_OVR					( 1 << SPI_SR_OVR )
#define SPI_FLAG_SR_BSY					( 1 << SPI_SR_BSY )
#define SPI_FLAG_SR_FRE					( 1 << SPI_SR_FRE )

/*
 * SPI related control register 1 definitions
 */

#define SPI_FLAG_CR1_DFF				( 1 << SPI_CR1_DFF )

/*
* Possible SPI Application States
*/
#define SPI_READY						0
#define SPI_BUSY_IN_RX					1
#define SPI_BUSY_IN_TX					2

/*
* Possible SPI Application events
*/
#define SPI_EVENT_TX_CMPLT				1
#define SPI_EVENT_RX_CMPLT				2
#define SPI_EVENT_OVR_ERR				3
#define SPI_EVENT_CRC_ERR				4

/**********************************************************************************************
 * 							APIs supported by this driver
 * 			For more information about the APIs check the function definitions
 *********************************************************************************************/

/*
 * Peripheral Clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDis);

/*
 * Init and De-Init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data Send and Receive
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint8_t Size);
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint8_t Size);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint8_t Size);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint8_t Size);

/*
 * IRQ Configuration and ISR Handling
 */

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t IRQEnDis);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);


/*
 * Other Peripheral Control APIs
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDis);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDis);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDis);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTranmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);


/*
 * Application callback
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvt);

#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */

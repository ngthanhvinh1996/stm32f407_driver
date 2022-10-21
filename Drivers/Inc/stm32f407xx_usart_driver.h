/*
 * stm32f407xx_usart_driver.h
 *
 *  Created on: Jul 3, 2021
 *      Author: vinhkuto
 */

#ifndef INC_STM32F407XX_USART_DRIVER_H_
#define INC_STM32F407XX_USART_DRIVER_H_

#include "stm32f407xx.h"

/*
 * Configuration structure for USARTx peripheral
 */
typedef struct
{
	uint8_t		USART_Mode;
	uint32_t	USART_BaudRate;
	uint8_t		USART_NoOfStopBits;
	uint8_t		USART_WordLength;
	uint8_t		USART_ParityControl;
	uint8_t		USART_HWFlowControl;
}USART_Config_t;

/*
 * Handle structure for I2Cx peripheral
 */
typedef struct
{
	USART_RegDef_t	*pUSARTx;
	USART_Config_t	USART_Config;
	uint32_t		TxLen;
	uint32_t		RxLen;
	uint8_t			*pTxBuffer;
	uint8_t			*pRxBuffer;
	uint32_t		TxBusyState;
	uint32_t		RxBusyState;
}USART_Handle_t;

/*
 * @USART_Mode
 */
#define USART_MODE_ONLY_TX			0
#define USART_MODE_ONLY_RX			1
#define USART_MODE_TXRX				2


/*
 * @USART_BaudRate
 */
#define USART_STD_BAUD_1200			1200
#define USART_STD_BAUD_2400			2400
#define USART_STD_BAUD_9600			9600
#define USART_STD_BAUD_19200		19200
#define USART_STD_BAUD_38400		38400
#define USART_STD_BAUD_57600		57600
#define USART_STD_BAUD_115200		115200
#define USART_STD_BAUD_230400		230400
#define USART_STD_BAUD_460800		460800
#define USART_STD_BAUD_921600		921600
#define USART_STD_BAUD_2M			2000000
#define USART_STD_BAUD_3M			3000000

/*
 * @USART_WordLength
 */
#define USART_WORDLENGTH_8BITS		0
#define USART_WORDLENGTH_9BITS		1

/*
 * @USART_ParityControl
 */
#define USART_PARITY_DISABLE		0
#define USART_PARITY_ENABLE_EVEN	1
#define USART_PARITY_ENABLE_ODD		2

/*
 * @USART_NoOfStopBits
 */
#define USART_STOPBITS_1			0
#define USART_STOPBITS_0_5			1
#define USART_STOPBITS_2			2
#define USART_STOPBITS_1_5			3

/*
 * @USART_HWFlowControl
 */
#define USART_HW_FLOW_CTRL_NONE		0
#define USART_HW_FLOW_CTRL_CTS		1
#define USART_HW_FLOW_CTRL_RTS		2
#define USART_HW_FLOW_CTRL_CTS_RTS	3

/*
* @USART application states
*/
#define USART_READY					0
#define USART_BUSY_IN_RX			1
#define USART_BUSY_IN_TX			2

/*
 * USART application events marcos
 */
#define USART_EVENT_TX_CMPLT		0
#define USART_EVENT_RX_CMPLT		1
#define USART_EVENT_CTS				2
#define USART_EVENT_IDLE			3
#define USART_EVENT_ORE				4
#define USART_ERREVENT_FE			5
#define USART_ERREVENT_NF			6
#define USART_ERREVENT_ORE			7

/*
 * USART related status flag definitions
 */
/* SR Register -> USART Status register */
#define USART_FLAG_SR_PE			( 1 << USART_SR_PE )
#define USART_FLAG_SR_FE			( 1 << USART_SR_FE )
#define USART_FLAG_SR_NF			( 1 << USART_SR_NF )
#define USART_FLAG_SR_ORE			( 1 << USART_SR_ORE )
#define USART_FLAG_SR_IDLE			( 1 << USART_SR_IDLE )
#define USART_FLAG_SR_RXNE			( 1 << USART_SR_RXNE )
#define USART_FLAG_SR_TC			( 1 << USART_SR_TC )
#define USART_FLAG_SR_TXE			( 1 << USART_SR_TXE )
#define USART_FLAG_SR_LBD			( 1 << USART_SR_LBD )
#define USART_FLAG_SR_CTS			( 1 << USART_SR_CTS )


/**********************************************************************************************
 * 							APIs supported by this driver
 * 			For more information about the APIs check the function definitions
 *********************************************************************************************/

/*
 * Peripheral clock setup
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDis);

/*
 * Init and De-init
 */
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_RegDef_t *pUSARTx);

/*
 * Data send and Receive
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t	USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISR handling
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_IRQHandling(USART_Handle_t *pUSARTHandle);

/*
 * Other Peripheral Control APIs
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName);
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName);
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);

/*
 * Application callback
 */
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t AppEv);


#endif /* INC_STM32F407XX_USART_DRIVER_H_ */

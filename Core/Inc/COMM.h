// ============================================================================
// COMM.h
// Author : AK
// Date   : 01 Dec 2021
// ============================================================================

#ifndef __COMM_H
#define __COMM_H

//*****************************************************************************
//   INCLUDE
//*****************************************************************************
//#include "GSM_Quectel2G.h"
#include "GSM.h"

//*****************************************************************************
//   DEFINITION
//*****************************************************************************
#define USART1TX_DEBUGRX_Pin 		GPIO_PIN_7
#define USART1TX_DEBUGRX_GPIO_Port  GPIOB

#define USART1RX_DEBUGTX_Pin 		GPIO_PIN_6
#define USART1RX_DEBUGTX_GPIO_Port  GPIOB

// #define GSMTX_USART2RX_Pin GPIO_PIN_9
// #define GSMTX_USART2RX_GPIO_Port GPIOB

// #define GSMRX_USART2TX_Pin GPIO_PIN_8
// #define GSMRX_USART2TX_GPIO_Port GPIOB


#define UART1RxCntr		UARTData.uiUART1RxCntr
#define UART1TimeOut		UARTData.ulUART1TimeOut
#define UART1RxOneByte	UARTData.cUART1RxData
#define UART1Buffer		UARTData.cUART1Buff
#define UART1TxComplete	UARTData.bUART1TxComplete
#define UART1RxComplete	UARTData.ucUART1RxComplete

//----------------------------------------------------------
/*
#define UART2RxCntr		UARTData.uiUART2RxCntr
#define UART2TimeOut		UARTData.ulUART2TimeOut
#define UART2RxOneByte	UARTData.cUART2RxData
#define UART2Buffer		UARTData.cUART2Buff
#define UART2TxComplete	UARTData.bUART2TxComplete
#define UART2RxComplete	UARTData.ucUART2RxComplete
#define UART2RxCompleteStatus	UARTData.ucUART2RxCompleteStatus
#define UART2TempBuffer	UARTData.cUART2TempBuff
#define UART2RxStart1		UARTData.ucUART2RxStart1
#define UART2RxCntrStart1	UARTData.ucUART2RxCntrStart1
#define UART2RxStart2		UARTData.ucUART2RxStart2
#define UART2RxCntrStart2	UARTData.ucUART2RxCntrStart2
*/
//----------------------------------------------------------

// #define UART3RxCntr		UARTData.uiUART3RxCntr
// #define UART3TimeOut		UARTData.ulUART3TimeOut
// #define UART3RxOneByte	UARTData.cUART3RxData
// #define UART3Buffer		UARTData.cUART3Buff
// #define UART3TxComplete	UARTData.bUART3TxComplete
// #define UART3RxComplete	UARTData.bUART3RxComplete

//----------------------------------------------------------
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

//*****************************************************************************
//   VARIABLES
//*****************************************************************************

typedef enum
{
	CHAR_RECEIVED_TIMEOUT = 0,
	CHAR_NOT_RECEIVED_TIMEOUT,
}eNUM_UARTDATASTATUS;

#pragma pack(push,1)
typedef struct
{
//	uint16_t	uiUART2RxCntr;
//	uint32_t	ulUART2TimeOut;
//	bool		bUART2TxComplete;
//	uint8_t	ucUART2RxComplete;
//	uint8_t	ucUART2RxCompleteStatus;
//	uint8_t	ucUART2RxStart1;
//	uint8_t	ucUART2RxCntrStart1;
//	uint8_t	ucUART2RxStart2;
//	uint8_t	ucUART2RxCntrStart2;
//	char		cUART2RxData[2];
//	char		cUART2TempBuff[10];
//	char		cUART2Buff[RS485_BUFF_SIZE];

	uint16_t	uiUART3RxCntr;
	uint32_t	ulUART3TimeOut;
	bool		bUART3TxComplete;
	bool		bUART3RxComplete;
	char		cUART3RxData[2];
	//char		cUART3Buff[GSM_BUFF_SIZE];

#if 0
	uint16_t	uiUART5RxCntr;
	uint32_t	ulUART5TimeOut;
	char		cUART5RxData[2];
	char		cUART5Buff[DG_BUFF_SIZE];
	bool		bUART5TxComplete;
	bool		bUART5RxComplete;
#endif
	uint16_t	uiUARTTxCntr;
} TsUARTData;
#pragma pack(pop)

//*****************************************************************************
//   PROTO
//*****************************************************************************
void 	vCOMM_eUSART1Init(void);
void 	vCOMM_eUSART2Init(void);
void 	vCOMM_eUSART3Init(void);
void	vCOMM_eTxString_Exe(UART_HandleTypeDef* USARTx, char *pcMessage, uint16_t uiTimeOut);
#endif

/***** END OF FILE ****/

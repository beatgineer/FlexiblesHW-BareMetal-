/*
 * APP.h
 *
 *  Created on: Mar 1, 2025
 *      Author: Brij.Z
 */

#ifndef __APP_H
#define __APP_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include "stm32g0xx_hal.h"
#include "CONFIG_EEPROM.h"

#define TRUE 1
#define FALSE 0
	/* Exported functions prototypes ---------------------------------------------*/
	void Error_Handler(void);
	void vAPP_eWriteMsgCntrInEEPROM_Exe(void);
	void vAPP_eFeedTheWDT_Exe(void);

/*Pin definition*/
//#define MCU_LED_Pin GPIO_PIN_15
//#define MCU_LED_GPIO_Port GPIOA
//
//#define GSM_SUPPLY_CONTROL_Pin GPIO_PIN_3
//#define GSM_SUPPLY_CONTROL_GPIO_Port GPIOA
//
//#define IGN_IN_Pin GPIO_PIN_4
//#define IGN_IN_GPIO_Port GPIOB
//
//#define GSM_RI_IN_Pin GPIO_PIN_2
//#define GSM_RI_IN_GPIO_Port GPIOB
//#define GSM_RI_EXTI1_EXTI_IRQn EXTI2_3_IRQn

#define IMAGE1_MCU_FLASH_ADDR ((uint32_t)0x0800B000) // page: 22
#define IMAGE2_MCU_FLASH_ADDR ((uint32_t)0x08016000) // Page: 42

/********************************************************************************************/
#define MAX_IGN_ALERT 5

	extern char cAPP_eGlobalBuffer[];
	extern char cAPP_eGPSDataBuffer[];

	// typedef enum
	// {
	//     // FALSE,
	//     // TRUE,

	//     IGN_OFF = 0,
	//     IGN_ON,

	// } Boolean;

	// *****************************************************************************/
	typedef enum
	{
		VEHICLE_STOP = 0,
		VEHICLE_RUN,
		MOTOR_POWER_OFF,
		CAN_DATA_NOT_RECD,
		VEH_MOVING,
		ALREADY_DISABLE,

		IGN_OFF = 0,
		IGN_ON,

		ANTI_THEFT_LINE_LO = 0,
		ANTI_THEFT_LINE_HI,

		IGN_TYPE_HW_LINE = 1,
		IGN_TYPE_RS485,
		IGN_TYPE_NOT_DECIDED = 0x55,
	} Enum_VehState_Mode;

	// *****************************************************************************/

#pragma pack(push, 1)
	typedef struct
	{
		char cIMEI[LEN_IMEINUM];
		char cAPN[LEN_APN];
		uint8_t ucIMEICCIDStatus; // Bit 0 : IMEI Status, Bit 1 : CCID Status
		char cDataServerIP[LEN_SERVERIP];
		char cDataServerPort[LEN_PORTNUM];
		char cCCID[LEN_CCID];
		uint8_t ucProtocol;
	} TsAPP_eConfig;
#pragma pack(pop)
	// *****************************************************************************/

#pragma pack(push, 1)
	typedef struct
	{
		uint16_t uiLEDCntr;

		uint32_t ul30SecCntr;
		uint32_t ul18SecCntr;
		uint32_t ul15SecCntr;
		uint32_t uiTimer360mS;
		uint32_t ul300SecCntr;
		uint32_t ul1SecCntr;

		uint16_t uiTimerCntr;

		volatile uint32_t ulTxRate;
		uint32_t ulPresentTxRate; // Current Tx Rate applicable as per Ign On/Off & Pwr Off Status

		char cRTC[21];
		bool bRTCStatus; // 0 - Not Valid, 1 - Valid
	} TsAPP_eTimer;
#pragma pack(pop)
	/********************************************************************/

#pragma pack(push, 1)
	typedef struct
	{
		bool bIgnStatus;
		bool bIgnStatusPrev;

		uint16_t uiMsgCntr;

		uint8_t ucIgnReadTimer;
		uint8_t ucIgnAlertCntr;
		uint8_t ucIgnAlertStatus[MAX_IGN_ALERT];
		bool bVehRunStatus : 1; // 0 - Run, 1 - Stop
		bool bValChanged : 1;	// 0 - No Value Changed, 1 - Value Changed, Store in EEPROM

		uint16_t uiBATTDataCntr;
	} TsAPP;
#pragma pack(pop)

	// *****************************************************************************/

#pragma pack(push, 1)
	typedef struct
	{
		uint8_t ucResetStatus; // Bit 0 : LSB : Option Byte Loader Reset RCC_FLAG_OBLRST
							   // Bit 1 : Pin Reset RCC_FLAG_PINRST
							   // Bit 2 : POR/PDR Reset RCC_FLAG_PORRST
							   // Bit 3 : Software Reset RCC_FLAG_SFTRST
							   // Bit 4 : Independent Watch Dog Reset RCC_FLAG_IWDGRST
							   // Bit 5 : Window Watch Dog Reset RCC_FLAG_WWDGRST
							   // Bit 6 : POR/Low Pwr Reset RCC_FLAG_LPWRRST

		uint16_t uiBlankPage;
		uint8_t ucBlankOffset;

		uint16_t uiMsgCntr; // Message Counter
		uint8_t ucHWVer;	// 'D' for Ver D (New HW), 'C' for ver C (Old HW)

		uint8_t ucFWCheckedDate; // Date when FW Status Checked

		uint32_t ulFTPFileSize;
		uint32_t ulFileHandle;
		char cVersion[5];
		char cFilePathAtFTP[35];
		char cBINFileName[15];
		char cCRCFileName[15];
		//	uint8_t		ucValidAPPNum;
		uint8_t ucCurrentAPPNum;
		uint8_t ucNewAPPNum;

		uint8_t ucImmobCmd;		  // 0 - No Cmd, 1 to 6 is Command Type
		uint8_t ucImmobPulseCntr; // 0 - No Cmd, 1 to 6 is Command Type
		uint16_t uiImmobCmdCntr;

		uint16_t uiWDTTimer;

		uint8_t ucIgnType; // 1 - Read Ign Line Status, 2 - Read from RS485
	} TsAPPData;
#pragma pack(pop)
	// *****************************************************************************/

#ifdef __cplusplus
}
#endif
#endif /* INC_APP_H_ */

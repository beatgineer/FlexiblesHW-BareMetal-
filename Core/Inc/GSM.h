// ============================================================================
// Module	: Quectel GSM / GPS Module EC20
// Version	: 00
// Author	: AK
// Date		: 24 Jul 2018
// ============================================================================

#ifndef __GSM_Quectel_H
#define __GSM_Quectel_H

//*****************************************************************************
//   INCLUDE
//*****************************************************************************
#include "stm32g0xx_hal.h"
#include "stm32g0xx.h"
#include "stdbool.h"

//*****************************************************************************
//   DEFINITION
//*****************************************************************************

#define GSMTX_USART2RX_Pin GPIO_PIN_3
#define GSMTX_USART2RX_GPIO_Port GPIOA

#define GSMRX_USART2TX_Pin GPIO_PIN_2
#define GSMRX_USART2TX_GPIO_Port GPIOA

#define GSM_RESET_Pin GPIO_PIN_14
#define GSM_RESET_GPIO_Port GPIOC

#define GSM_PWRKEY_Pin GPIO_PIN_15
#define GSM_PWRKEY_GPIO_Port GPIOC

// #define GSM_SUPPLY_CONTROL_Pin GPIO_PIN_3
// #define GSM_SUPPLY_CONTROL_GPIO_Port GPIOA

// #define GSM_RI_IN_Pin GPIO_PIN_0
// #define GSM_RI_IN_GPIO_Port GPIOA

#define GSMComPortHandle &huart2

#define GSM_BuffSize 2100
#define GSM_BaudRate 115200
#define GSM_DataBuffer GSMData.cUART2Buff
#define GSM_RxCntr GSMData.uiUART2RxCntr
#define GSM_TimeOut GSMData.ulUART2TimeOut
#define GSM_RxOneByte GSMData.cUART2RxData

#define CONTROL_Z 0x1A

#define GSM_NO_SOCKET_COUNTER 50
#define GSM_ERROR_COUNTER 10
#define GSM_NO_RESPONSE_COUNTER 5

#define GSM_DATA_SENT_TIMEOUT 200

#define TOTAL_SOCKET 1

#define PDP_CONTEXT_ID 1
#define LOCATION_SOCKET_ID 0

#define GSM_SIGNAL_THRESHOLD_LOW 7
#define GSM_SIGNAL_THRESHOLD_HIGH 12

extern const char *cGSM_QUECTEL_eATCommandTbl[];
extern const char *cGSM_QUECTEL_eATResponseTbl[];

//*****************************************************************************//
typedef enum
{
	GSM_NOT_READY = 0,
	GSM_READY,
	GSM_WAS_READY,

	RESPONSE_MATCHING = 0,
	RESPONSE_NOT_MATCHING,
	RESPONSE_ERROR,
	RESPONSE_CME_ERROR,
	RESPONSE_CMS_ERROR,
	NO_RESPONSE,
	BOTH_RESPONSES_MATCHING,
	RESPONSE1_MATCHING,
	RESPONSE2_MATCHING,

	NO_GSM_SIGNAL = 0,
	GSM_SIGNAL_ERROR,
	GSM_SIGNAL_LEVEL_LOW,
	GSM_SIGNAL_LEVEL_OK,

	NO_NETWORK = 0,
	GSM = 2,
	WCDMA = 3,
	LTE = 4,

	SIM_NOT_INSERTED = 0,
	SIM_INSERTED,

	NO_SIM = 0,
	SAME_SIM = 0x55,
	DIFFERENT_SIM = 0xAA,

	NETWORK_REGISTRATION_FAILED = 0,
	NETWORK_REGISTRATION_SUCCESS,

	GSM_NETWORK_REGISTRATION = 1,
	GPRS_NETWORK_REGISTRATION,
	EPS_NETWORK_REGISTRATION,

	SOCKET_NOT_ACTIVE = 0,
	SOCKET_ACTIVE,

	ALREADY_CONNECT = 0,
	CONNECT_OK,
	CONNECT_FAIL,

	DATA_SENT_ERROR = 0,
	DATA_SENT,
	DATA_SENT_FAIL,

	GPRS_NOT_ACTIVE = 0,
	GPRS_ACTIVE,

	SOCKET_ALREADY_ACTIVE = 0,
	NEW_SOCKET_MADE,
	SOCKET_NOT_READY,

	GSM_ACTIVE = 0,
	GSM_POWER_DOWN,
	GSM_POWER_DOWN_ALERT_SENT,

	NETWORK_SEARCHING = 0,
	NETWORK_REGISTERED_HOME,
	NETWORK_NOT_REGISTERED,
	NETWORK_REGISTRATION_DENIED,
	NETWORK_UNKNOWN,
	NETWORK_REGISTERED_ROAMING,

	NO_SMS_PRESENT = 0,
	SMS_PRESENT,
	SMS_FROM_INVALID_SENDER,
	SPAM_SMS,
	INVALID_SMS_PRESENT,

	NO_OPERATOR = 0,
	DEFAULT_VAL,
	AIRTEL,
	VODAFONE,

	UNKNOWN_ERROR = 0,
	NO_FILE_FOUND,
	SAME_FILE_FOUND_MATCHING_SIZE,
	SAME_FILE_FOUND_NOT_MATCHING_SIZE,

	SM_SMS_STORAGE = 1,
	ME_SMS_STORAGE,

	INITIAL_STATE = 0,
	READ_SOCKET_AT_CMD_SENT,
	READ_SOCKET_RESPONSE_RECEIVED,
	READ_SOCKET_TIMEOUT,

	NO_CHAR_RECD_TIMEOUT = 0,
	BOTH_RESPONSE_RECD,
	SOME_CHAR_RECD_TIMEOUT,
	BOTH_RESPONSES_OR_ERROR_RECD, // V3.02@20112023
} Enum_GSMStatus;

typedef enum
{
	LOCATION_DATA = 0,
	EVENT_DATA,
	HISTORY_DATA,
	BATTERY_DATA,
	SOCKET_COMMAND,

	REMOTE_VEHICLE_MOVEMENT_DISABLE = 0,
	REMOTE_VEHICLE_MOVEMENT_ENABLE,

	NO_SOCKET_STATE = 0,
	SOCKET_COMMAND_ISSUED,
	WAIT_FOR_SOCKET_RESPONSE,
	SOCKET_RESPONSE1_RECEIVED,
	SOCKET_RESPONSE2_RECEIVED,
	SOCKET_RESPONSE_UPDATED,
	SOCKET_TIME_OUT,
} Enum_DataStringType;

typedef enum
{
	ATCOMMAND_AT = 0,
	ATCOMMAND_ATE0,
	ATCOMMAND_CMEE,
	ATCOMMAND_ATV1,
	ATCOMMAND_CSCS,
	ATCOMMAND_CPIN, // 5
	ATCOMMAND_GETCREG,
	ATCOMMAND_GETCGREG,
	ATCOMMAND_CGSN,
	ATCOMMAND_CSQ,
	ATCOMMAND_QICSGP, // 10
	ATCOMMAND_QIACT,
	ATCOMMAND_QIDEACT,
	ATCOMMAND_QIOPEN,
	ATCOMMAND_QISTATE,
	ATCOMMAND_QISEND, // 15
	ATCOMMAND_QICLOSE,
	ATCOMMAND_CMGF,
	ATCOMMAND_CSMP,
	ATCOMMAND_CMGS,
	ATCOMMAND_GETCSCA, // 20
	ATCOMMAND_CPMS_SM,
	ATCOMMAND_CSDH,
	ATCOMMAND_CMGR,
	ATCOMMAND_CMGD,
	ATCOMMAND_QURCCFG, // 25
	ATCOMMAND_GETQIACT,
	ATCOMMAND_QCMGS,
	ATCOMMAND_QFTPCFGCONID,
	ATCOMMAND_QFTPCFGACCOUNT,
	ATCOMMAND_QFTPCFGFILETYPE, // 30
	ATCOMMAND_QFTPCFGTRANSMODE,
	ATCOMMAND_QFTPCFGRSPTOUT,
	ATCOMMAND_QFTPOPEN,
	ATCOMMAND_QFTPCWD,
	ATCOMMAND_QFTPSIZE, // 35
	ATCOMMAND_QFTPGET,
	ATCOMMAND_QFLST,
	ATCOMMAND_QFDEL,
	ATCOMMAND_CCLK,
	ATCOMMAND_CTZU, // 40
	ATCOMMAND_QFTCLOSE,
	ATCOMMAND_QCCID,
	ATCOMMAND_QNWINFO,
	ATCOMMAND_QPOWD,
	ATCOMMAND_CPMS_ME, // 45
	ATCOMMAND_GETCEREG,
	ATCOMMAND_QFOPEN,
	ATCOMMAND_QFOPENQUERY,
	ATCOMMAND_QFSEEK,
	ATCOMMAND_QFREAD, // 50
	ATCOMMAND_QFCLOSE,
	GPSATCOMMAND_GETGPSPWR,
	GPSATCOMMAND_QGPSPWR1,
	GPSATCOMMAND_QGPSLOC,
	GPSATCOMMAND_QGPSPWR0, // 55
} Enum_GSM_ATCommand;

typedef enum
{
	ATRESPONSE_OK = 0,
	ATRESPONSE_CPINREADY,
	ATRESPONSE_COPS,
	ATRESPONSE_CREG,
	ATRESPONSE_CGREG,
	ATRESPONSE_CSQ, // 5,
	ATRESPONSE_ERROR,
	ATRESPONSE_CMEERROR,
	ATRESPONSE_QISTATE,
	ATRESPONSE_SEND,
	ATRESPONSE_CLOSEOK, // 10
	ATRESPONSE_CMGS,
	ATRESPONSE_CSCA,
	ATRESPONSE_CPMS,
	ATRESPONSE_CMGR,
	ATRESPONSE_CEREG, // 15
	ATRESPONSE_QIACT,
	ATRESPONSE_QFTPCWD,
	ATRESPONSE_QFTPSIZE,
	ATRESPONSE_QFTPGET,
	ATRESPONSE_QFLST, // 20
	ATRESPONSE_CCLK,
	ATRESPONSE_SENDOK,
	ATRESPONSE_QIOPEN,
	ATRESPONSE_CCLK80,
	ATRESPONSE_QNWINFO, // 25
	ATRESPONSE_GSM,
	ATRESPONSE_WCDMA,
	ATRESPONSE_LTE,
	ATRESPONSE_QFTPOPEN,
	ATRESPONSE_POWERDOWN, // 30
	ATRESPONSE_CMSERROR,
	ATRESPONSE_GETQICSGP,
	ATRESPONSE_QIOPEN561,
	ATRESPONSE_QFOPEN,
	ATRESPONSE_CONNECT, // 35
	GPSATRESPONSE_QGNSSC,
	GPSATRESPONSE_QGPSLOC, // 37
} Enum_GSM_ATResponse;

//----------------------------------------------------------------

typedef union
{
	struct
	{
		uint8_t ucGSMReadyStatus : 2; // 0 - Not Ready, 1 - Ready, 2 - Was Ready
		bool bGPRSStatus : 1;		  // 0 - Not Active, 1 - Active
		bool bGSMRebootRequired : 1;  // 0 - no, 1 - Required
		bool bSocketStatus : 1;		  // 0 - Not Active, 1 - Active
		bool bSIMInsertStatus : 1;	  // 0 - Not Inserted, 1 - Inserted
		bool bCREGStatus : 1;		  // 0 - Not Reg., 1 - Reg
		bool bCGREGStatus : 1;		  // 0 - Not Reg., 1 - Reg

		bool bCEREGStatus : 1;		   // 0 - Not Reg., 1 - Reg
		uint8_t ucNetworkInfo : 3;	   // 0 - No Network, 1 -, 2 - GSM, 3 - WCDMA, 4 - LTE
		uint8_t ucGSMSignalStatus : 2; // 0 - No Signal, 1 - Error, 2 - Low, 3 - Ok
		bool bAPNStatus : 1;		   // 0 - Not Set, 1 - APN Set
		bool bRTCStatus : 1;		   // 0 - Not Valid, 1 - Valid

		bool bPwrDownStatus : 1; // 0 - GSM Active, 1 - Power Down,
		bool bFWSMSPending : 1;	 // 0 - No, 1 - Pending
		uint8_t ucSpare : 6;
	};
	uint8_t ucData[3];
} TsGSMStatus;

#pragma pack(push, 1)
typedef struct
{
	uint16_t uiUART2RxCntr;
	uint32_t ulUART2TimeOut;
	char cUART2RxData[2];
	char cUART2Buff[GSM_BuffSize];

	uint8_t ucNoResponseCntr;
	uint8_t ucErrorCntr;
	uint8_t ucNoSocketCntr;
	char cErrCode[2];

	char cRTC[21];

	uint8_t ucGSMSignalVal;

	uint8_t ucPendingMESMS; // No. of Pending SMS
	uint8_t ucTotalMESMS;
	//	uint8_t		ucCurrentMESMSIndex;

	uint8_t ucPendingSMSMS; // No. of Pending SMS
	uint8_t ucTotalSMSMS;
	//	uint8_t		ucCurrentSMSMSIndex;
	char cSenderMobileNum[25];
	char cSMSData[27];
} TsGSMData;
#pragma pack(pop)

typedef struct
{
	volatile bool IoT_BLE_symbol;
	volatile bool IoT_DataSession_symbol;
	volatile bool IoT_GPS_symbol;
	volatile bool IoT_Speaker_symbol;
}IoT_ClusterMsg_t;

//*****************************************************************************
void vGSM_eInit(void);
void vGSM_ePowerONGSM_Exe(void);
bool bGSM_eGetIMEINum_Exe(char *cOutBuff);
bool bGSM_eGetICCIDNum_Exe(char *cOutBuff);
uint8_t ucGSM_eProcessATCmdWithCmdNum_Exe(uint8_t ucATCmdNum, uint8_t ucRespNum, uint32_t ulTimeOutCharRecd, uint32_t ulTimeOutNoCharRecd);
bool bGSM_eWait4RespOrTimeOut_Exe(uint32_t ulTimeOutCharRecd, uint32_t ulTimeOutNoCharRecd);
uint8_t ucGSM_eCompareStringInGSMBuff_Exe(char *cString);
void vGSM_eStartGSM_Exe(void);
uint8_t ucGSM_eCheckATResponse_Exe(void);
void vGSM_ePWRKeyStartGSM_Exe(void);
void vGSM_eResetGSM_Exe(void);
void vGSM_eDisableGPRS_Exe(uint8_t ucPDPContextID);
void vGSM_eCloseSocket_Exe(uint8_t ucSocketID);
uint8_t ucGSM_eProcessATCmdDualRespWithCmdNum_Exe(uint8_t ucATCmdNum, uint8_t ucResp1Num, uint8_t ucLenResp1, uint8_t ucResp2Num, uint8_t ucLenResp2, uint32_t ulTimeOutCharRecd, uint32_t ulTimeOutNoCharRecd);
uint8_t ucGSM_eWait4RespOrTimeOutDualResp_Exe(uint8_t ucLenResp1, uint8_t ucLenResp2, uint32_t ulTimeOutCharRecd, uint32_t ulTimeOutNoCharRecd);
uint8_t ucGSM_eProcessATCmd_Exe(char *cATCmd, uint8_t ucRespNum, uint32_t ulTimeOutCharRecd, uint32_t ulTimeOutNoCharRecd);
uint8_t ucGSM_eProcessATCmdDualResp_Exe(char *cATCmd, uint8_t ucResp1Num, uint8_t ucLenResp1, uint8_t ucResp2Num, uint8_t ucLenResp2, uint32_t ulTimeOutCharRecd, uint32_t ulTimeOutNoCharRecd);
uint8_t ucGSM_eProcessATCmdWithCmdPtr_Exe(char *cATCmd, uint8_t ucRespNum, uint32_t ulTimeOutCharRecd, uint32_t ulTimeOutNoCharRecd);
void vGSM_eRegisterAndSetModule_Exe(void);
void vGSM_eInitaliseGSM_Exe(void);
void vGSM_eInitialSetup_Exe(void);
void vGSM_eReadSIMInsertStatus_Exe(void);
uint8_t ucGSM_eReadSignalStrength_Exe(void);
bool bGSM_eNetworkSetup_Exe(void);
bool bGSM_eReadNetworkRegStatus_Exe(uint8_t ucNetworkStatusType);
void vGSM_eReadNetworkInfo_Exe(void);
bool bGSM_eCheckAPNSetting_Exe(void);
uint8_t ucGSM_eProcessATCmdDualRespWithPtr_Exe(char *cATCmd, uint8_t ucResp1Num, uint8_t ucLenResp1, uint8_t ucResp2Num, uint8_t ucLenResp2, uint32_t ulTimeOutCharRecd, uint32_t ulTimeOutNoCharRecd);
void vGSM_eSetAPN_Exe(void);
void vGSM_eUpdateTimeZoneAndRTC_Exe(void);
uint8_t ucGSM_eReadRTC_Exe(void);
void vGSM_eEnableGPRS_Exe(uint8_t ucPDPContextID);
void vGSM_eReadGPRSStatus_Exe(uint8_t ucPDPContextID);
void vGSM_eReStartGSM_Exe(void);
void vGSM_eTurnOFFGSM_Exe(void);
void vGSM_eSoftOFFGSM_Exe(void);
bool bGSM_eCheckPendingSMSFromSIM_Exe(void);
bool bGSM_eCheckPendingSMSFromME_Exe(void);
bool bGSM_eGetPendingSMS_Exe(uint8_t ucSMSStorage);
uint8_t ucGSM_eReadSMS_Exe(uint8_t ucSMSIndex);
bool bGSM_eDeleteSMS_Exe(uint8_t ucSMSIndex);
bool bGSM_eSendSMS_Exe(char *cReceiverMobile, char *cMessage);
bool bGSM_eCheckAndMakeSocket_Exe(void);
bool bGSM_eSendDataOnSocket_Exe(void);
bool bGSM_eReadSocketStatus_Exe(void);
bool bGSM_eMakeSocket_Exe(void);
void vGSM_eWakeGSMModule_Exe(void);
void vGSM_StatusSymbol_To_Cluster(void);

#endif

/***** END OF FILE ****/

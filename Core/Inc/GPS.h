// ============================================================================
// Module	: RS485
// Version	: 00
// Author	: AK
// Date		: 24 Jul 2018
// ============================================================================

#ifndef __GPS_Quectel_H
#define __GPS_Quecte_H

//*****************************************************************************
//   INCLUDE
//*****************************************************************************
#include "stm32g0xx_hal.h"
#include "stm32g0xx.h"
#include <stdbool.h>

//*****************************************************************************
//   DEFINITION
//*****************************************************************************

#define GPS_FIXED_SYMBOL 'F'
#define GPS_NOT_FIXED_SYMBOL 'N'

#define GPS_STATUS_LED_Pin GPIO_PIN_5
#define GPS_STATUS_LED_GPIO_Port GPIOA

//*****************************************************************************
typedef enum
{
	GPS_NOT_FIXED,
	GPS_FIXED,

	GPS_READING_ERROR = 0,
	GPS_READING_OK,
} Enum_GPSResponse;
/*
typedef enum
{
//	GPSATCOMMAND_GETGPSPWR		= 0,
	GPSATCOMMAND_QGPSPWR0,
//	GPSATCOMMAND_QGPSPWR1,
	GPSATCOMMAND_QGNSSRDGGA,
	GPSATCOMMAND_QGNSSRDRMC,
	GPSATCOMMAND_QGPSXTRA1		= 5,
	GPSATCOMMAND_GETQGPSXTRA,
//	GPSATCOMMAND_QGPSLOC,
	GPSATCOMMAND_QGPSXTRATIME,
	GPSATCOMMAND_QGNSSRDGSA,
	GPSATCOMMAND_QAGPS1,			// 10
//	GPSATCOMMAND_QGNSSTS
//	GPSATCOMMAND_QGNSSEPO1,
//	GPSATCOMMAND_QGEPOAID,
//	GPSATCOMMAND_QIFGCNT,
} Enum_GPS_ATCommand;

typedef enum
{
	GPSATRESPONSE_OK 			= 0,
	GPSATRESPONSE_ERROR,
	GPSATRESPONSE_CMEERROR,
//	GPSATRESPONSE_QGNSSC,
	GPSATRESPONSE_QGNSSRD,
	GPSATRESPONSE_MINUS99,		//5
	GPSATRESPONSE_QGPSLOC,
} Enum_GPS_ATResponse;
*/
typedef enum
{
	OFF = 0,
	ON
} Enum_GPS_Boolean;

//----------------------------------------------------------------
#pragma pack(push, 1)
typedef struct
{
	bool bGPSPower;
	uint8_t ucGPSLEDTimer;

	uint8_t ucGPSNotFixedCntr;
	uint8_t ucGPSLEDStatus;

	char cDegLAT[13];
	char cDegLONG[14];
        
        bool	bGPSStatus;			// 0 - Not Fixed, 1 - Fixed

	char cSat[5];
	char cAltitude[10];
	char cHeading[8];
	char cSpeed[7];
	char cDate[8];
	char cTime[8];
	char cHDOP[6];
	char cVDOP[6];
	char cFix[2];
} TsGPSData;
#pragma pack(pop)

//*****************************************************************************
void vGPS_eInit(void);
uint8_t ucGPS_eGetGPSPowerStatus_Exe(void);
void vGPS_eSetGPSPower_Exe(uint8_t ucGPSPwrStatus);
void vGPS_eStartGPS_Exe(void);
bool bGPS_eGetGPSLoc_Exe(void);
void vGPS_eReadGPSData_Exe(void);
void vGPS_eOperateGPSLED_Exe(void);
#endif

/***** END OF FILE ****/

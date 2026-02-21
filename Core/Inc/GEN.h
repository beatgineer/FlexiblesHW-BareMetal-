// ============================================================================
// GEN.h
// Author : AK
// Date   : 15-12-2021
// ============================================================================

#ifndef __GEN_H
#define __GEN_H

#include <stdbool.h>

#define MSG_SEPERATOR		';'
#define END_OF_DATA			'#'
#define TERMINATOR1			'\r'
#define TERMINATOR2			'\n'

#define GPS_DATA_BUFFER_LEN	800U

typedef enum
{
	NO_ALERT, // 0

	IGN_ON_ALERT, // 1
	IGN_OFF_ALERT,

	BATTERY_CONNECTED_ALERT, // 3
	BATTERY_REMOVED_ALERT,	 // 4
} Enum_AlertType;

typedef enum
{
	IMEI_NO_MSG = 1,
	HWVER_MSG,
	SWVER_MSG,
	ISTDATE_MSG,
	GPS_FIXED_MSG,	// 5
	LAT_MSG,
	LONG_MSG,
	ALTITUDE_MSG,
	DIR_MSG,
	VEHSPEED_MSG,	// 10
	HDOP_MSG,
	VDOP_MSG,
	SAT_MSG,
	GSM_SIGNAL_MSG,
	IGN_MSG,	// 15
	BATTVOL_MSG,
	BATTSOC_MSG,
	VEHTYPE_MSG,
	EVENTTYPE_MSG,
	MSG_COUNTER_MSG	= 99	//20
} Enum_GENDataType;


void vGEN_eLoadTxRate_Exe(void);
void vGEN_eLoadAndParseData_Exe(uint8_t ucDataType, uint8_t ucEventType);
void vGEN_eMakeIntBattChargingOnOff_Exe(bool bStatus);
void vGEN_eMakeGSMPowerDownIfBattNotPresent_Exe(void);
void vGEN_eONBattChg_Exe(void);

#endif

	/***** END OF FILE ****/

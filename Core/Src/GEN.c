/*
 * APP.c
 *
 *  Created on: Mar 1, 2025
 *      Author: Brij.Z
 */

#include "APP.h"
#include "GEN.h"
#include "GSM.h"
#include "GPS.h"
#include "COMM.h"


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>


extern TsAPP APPStatus;
extern TsAPP_eConfig APPCONFIG;
extern TsAPP_eTimer TIMERData;
extern TsGPSData GPSData;
extern TsGSMData GSMData;
extern TsGSMStatus GSMStatus;


extern IoT_ClusterMsg_t IoT_ClusterMsg;

const char *cGEN_eDataType[] = {"LD", "ED", "HD"}; // LD: Location Data, ED: Event Data, HD: History Data

void vGEN_eLoadAndParseData_Exe(uint8_t ucDataType, uint8_t ucEventType)
{
	vAPP_eFeedTheWDT_Exe();

	uint8_t ucGPSStatusSymbol;
	char cType[4], cMotorCurrent[8], cBattCurrent[8];
	char cODO[10], cTripA[10], cTripB[10];
	float fMotorCurrent;

	/*RPM to KMPH*/
	// CANData.ucSpeedInkmph = CANData.uiRPM / CFAC_RPM_SPEED_RV400;

	/*Range*/
	// vODO_eUpdateRangeBuff_Exe();

	/*Motor Current conversion*/
//	fMotorCurrent = (float)CANData.uiMotorCurrent / 10;
//	sprintf(cMotorCurrent, "%0.1f", fMotorCurrent);

	strcpy(cType, cGEN_eDataType[ucDataType]);

	if (GPSData.bGPSStatus == GPS_FIXED)
	{
		ucGPSStatusSymbol = 'F';
	}
	else
	{
		ucGPSStatusSymbol = 'N';
	}

	// if (CANData.bCurrPol == POLARITY_MINUS)
	// {
	// 	sprintf(cBattCurrent, "-%d", CANData.uiBattCurrent);
	// }
	// else
	// {
	// 	sprintf(cBattCurrent, "%d", CANData.uiBattCurrent);
	// }

	// sprintf(cODO, "%d", (ODOData.ulOdoValInKm / 10));
	// sprintf(cTripA, "%0.1f", ((float)ODOData.uiTripAValInKm / 10));
	// sprintf(cTripB, "%0.1f", ((float)ODOData.uiTripBValInKm / 10));

	/*Test Sample String*/
	strcpy(cAPP_eGlobalBuffer, "msg|1,01|49,02|867409072584985,03|0300,04|2019010400108,05|2024-02-24 16:06:23,06|N,07|,08|N,09|,10|E,11|,12|,13|,14|0.0,15|,16|31,17|0,18|1,19|1,20|1,22|0,23|0,24|0,25|0,26|0,27|LD,28|0,29|,30|0,31|0.0,32|0,33|0,34|0,35|0,36|0,37|0,38|0,39|72V,40|4,41|0,43|4,44|89911190185517582798,99|1;");

	// sprintf(cAPP_eGlobalBuffer, "msg|1,01|49,02|%s,03|%s,04|%s,05|%s,06|%c,07|%s,08|N,09|%s,10|E,11|%s,12|%s,13|%s,14|0.0,15|%s,16|%d,18|%d,20|%d,22|%d,23|%d,24|%d,25|%d,26|%d,27|%s,28|%d,29|%s,30|%d,31|%s,32|%d,33|%d,34|%s,35|%d,36|%d,37|%d,38|%d,39|72V,40|%d,42|%d,43|4,44|%s,45|%d,46|%s,47|%s,48|%d,49|%d,50|%d,51|%d,52|%d,53|%d,54|%d,55|%d,56|%d,57|%d,58|%d,59|%d,60|%d,61|%d,62|%d,63|%d,93|%d,96|%d,99|%d;",
	// 		APPCONFIG.cIMEI,		// 02    ->	ECU / IMEI No.  | 861107038771078
	// 		CONFIG.cSWRev,			// 03    ->	Firmware Version Number  | 0010
	// 		CANData.cVehBattID,		// 04	->	Battery ID  //2019010400108
	// 		GSMData.cRTC,			// 05	->	IST Date and Time (YYYY-MM-DD HH:MM:SS:MS)   | 2019-03-15 17:15:47
	// 		ucGPSStatusSymbol,		// 06	->	GPS Status (N - Not Fixed, F - Fixed)  | F
	// 		GPSData.cDegLAT,		// 07    -> Latitude (Lattitude in dd.ddddd) | 28.4017137
	// 		GPSData.cDegLONG,		// 09    -> Longitude (Longitude in ddd.ddddd) | 77.0797587
	// 		GPSData.cAltitude,		// 11	->	Altitude | 217.9
	// 		GPSData.cHeading,		// 12	->	Direction (Course over Ground) | 290.7
	// 		GPSData.cHDOP,			// 13	->  HDOP | 0.6 , 14 - VDOP | 0.6
	// 		GPSData.cSat,			// 15	->	Satellite | 10
	// 		GSMData.ucGSMSignalVal, // 16	->	GSM Signal Strength | 23
	// 		// ODOData.uiCalRangeInKm, // 17 	-> Remaining Mileage in km  | 77
	// 		APPStatus.bIgnStatus, // 18	->	Ignition Status (0 - Off, 1 - ON) | 1
	// 		// 1,							   // 19	->	MCB Status (0 - Off, 1 - ON)	| 1
	// 		BATTStatus.bBATTStatus,	   // 20	-> Battery Status (0 - No Battery, 1 - Battery Present)	| 1,	21: Helmet Status (0 - No Helmet, 1 - Helmet Present)	| 1
	// 		CANData.ucControllerTemp,  // 22	->	Controller Temperature	| 31
	// 		CANData.ucSpeedInkmph,	   // 23	->	Vehicle Speed	| 0
	// 		CANData.uiBattVolt,		   // 24	->	Battery Voltage	| 70
	// 		CANData.ucBattSOC,		   // 25	->	Battery SOC		| 97
	// 		CANData.ucBattCellMaxTemp, // 26	->	Battery Max Cell Temperature	| 22
	// 		cType,					   // 27	->	Data Type (LD-Location Data, ED-Event Data, HD-History Data)	| LD
	// 		ucEventType,			   /* 28	->	Event Type | 0 (0 - No Event, 1 - Ignition On, 2 - Ignition Off, 3 - Battery Charger Connected, 4 - Battery Charger Disconnected, 5 - Battery Inserted,
	// 														   6 - Battery Removed, 7 - MCB ON, 8 - MCB OFF, 9 - Vehicle Mode Changed to Eco, 10 - Vehicle Mode Changed to City, 11 - Vehicle Mode Changed to Sports,
	// 														   12 - Battery Error, 13 - Controller Error, 14 - Immobilizer Error, 15 - Immobilization Error, 16 - Vehicle Fall Down, 17 - Vehicle Crashed)*/
	// 		GPSData.cSpeed,			   // 29	->	GPS Speed in kms	| 0
	// 		CANData.ucVehMode,		   // 30	->	Vehicle Mode |1 (1 - Eco Mode, 2 - City Mode, 3 - Sports Mode, 4 - Reverse Mode)
	// 		cMotorCurrent,			   // 31	->	Motor Current	| 5
	// 		CANData.uiRPM,			   // 32	->	Wheel RPM		| 45
	// 		CANData.ucBattSOH,		   // 33	->	Battery Health	| 100
	// 		cBattCurrent,			   // 34	->	Battery Charge/Discharge Current	| 2
	// 		CANData.uiNumOfCycles,	   // 35	->	Charging cycle count	| 4
	// 		CANData.uiNumOfCycles,	   // 36	->	Discharging cycle count	| 1
	// 		CANData.ucBattErr1,		   /* 37	->	Battery Error Code	|0
	// 												Bit 0 & 1 - Cell Over Voltage, Bit 2 & 3 - Cell Under Voltage, Bit 4 & 5 - Pack Over Voltage, Bit 6 & 7 - Pack Under Voltage*/
	// 		CANStatus.bChargerStatus,  /* 38	->	Battery State Flag,	| 0
	// 											   0 - Normal, 1 - Charging, 2 - Low Voltage*/
	// 		GSMStatus.ucNetworkInfo,   /* 40	->	GSM Network Type,	| 4
	// 											   2 - 2G, 3 - 3G, 4 - 4G*/
	// 								   // 41 	-> History Data sync
	// 		BATTData.uiIntBATTVolt,	   // 42 	-> Internal Battery Voltage
	// 								   /* 43 	-> Vehicle Type
	// 											  3 - RV300, 4 - RV400*/
	// 		APPCONFIG.cCCID,		   // 44 	-> SIM Card Number | 12345678912345678912
	// 		CANData.ODO,			   // 45 	-> ODO Range | 999999
	// 		cTripA,					   // 46 	-> Trip A Distance Travelled	| 999.9
	// 		cTripB,					   // 47 	-> Trip B Distance Travelled	| 999.9
	// 		CANData.ucBattErr2,		   /* 48	->	Battery Error 2	| 0
	// 											   Bit 0 & 1 - Discharge Over Temperature, Bit 2 & 3 - Discharge Low Temperature, Bit 4 & 5 - Charging Over Temperature, Bit 6 & 7 - Charging Low Temperature*/
	// 		CANData.ucBattErr3,		   /* 49	->	Battery Error 3 | 0
	// 											   Bit 0 & 1 - Differential Cell Temperature Protection, Bit 2 & 3 - Over Current Charging, Bit 4 & 5 - Regen Over Current, Bit 6 & 7 - Discharge Over Plus Current*/
	// 		CANData.ucBattErr4,		   /* 50	->	Battery Error 4 | 0
	// 											   Bit 0 & 1 - DISCHARGE_OVER_PLUS_CURRENT_ERR, Bit 2 & 3 - Low Insulation, Bit 4 & 5 - Low SOC, Bit 6 & 7 - BATT_ID_CONFLICT_ERR*/
	// 		CANData.ucBattErr5,		   /* 51	->	Battery Error 5 | 0
	// 											   Bit 0 - AFE Comm Error, Bit 1 - CELL_VOLTAGE_ACQ_ERR, Bit 2 - TEMP_ACQ_ERR, Bit 3 - BALANCE_RESISTOR_OVER_TEMP_ERR, Bit 4 - MOS Over Temperature, Bit 5 - PRE_DISCHARGE_MOSFET_OVER_TEMP_ERR, Bit 6 - Pre Discharge TimeOut, Bit 7 - Pre DisCharge Over Current*/
	// 		CANData.ucBattErr6,		   /* 52	->	Battery Error 6 | 0
	// 											   Bit 0 - AFE Comm Error, Bit 1 - AFE Under Voltage, Bit 2 - AFE Discharge Over Current, Bit 3 - AFE Charge Over Current, Bit 4 - AFE Short Circuit, Bit 5 - AFE Charging Over Temperature, Bit 6 - AFE Charging Low Temperature, Bit 7 - AFE Discharge Over Temperature*/
	// 		CANData.ucBattErr7,		   /* 53	->	Battery Error 7 | 0
	// 											   Bit 0 - AFE Discharge Low Temp, Bit 1 - FULLY_CHARGE_ERR, Bit 2 - Zero Volt Charge Inhibition, Bit 3 - Pre Placed MOS Adhesion, Bit 4 - Discharge MOS Adhesion, Bit 5 - Charge MOS Fail, Bit 6 - Pre Placed MOS Adhesion, Bit 7 - Discharge MOS Driver Fail*/
	// 		CANData.ucBattErr8,		   /* 54	->	Battery Error 8 | 0
	// 											   Bit 0 - Charge MOSFET Driver Fail, Bit 1 - EOL, Bit 2 - Severe Over Voltage, Bit 6 - Charge Protection Error, Bit 7 - AFE Discharge Over Temperature*/
	// 		CANData.ucBattErr9,		   /* 55	->	Battery Error 9	| 0
	// 											   Bit 0 - Charger Connected but Not Charging, Bit 1 - Over Temperature > 60°C, Bit 2 - Thermal Runaway > 65°C*/
	// 		CANData.ucControllerErr1,  // 56	->	Motor Controller Error Code 1	| 0
	// 		CANData.ucControllerErr2,  // 57	->	Motor Controller Error Code 2	| 0
	// 		CANStatus.ucImmobErr,	   // 58	->	Immobilizer Error	| 0
	// 		CANData.ucClusterErr1,	   // 59	->	Cluster Error Code	| 0
	// 		CANData.ucChargerErr1,	   // 60	->	Charger Error Code	| 0
	// 		0,						   // 61	->	Braking Counter		| 0
	// 		0,						   // 62	->	Acceleration Counter	| 0
	// 		0,						   // 63	->	Tilt Angle		| 1
	// 		ODOData.uiCalRangeInKm,	   // 93	->	Remaining Mileage in km
	// 		APPStatus.bVehRunStatus,   // 96	->	Vehicle Run Status (0 - Stop, 1 - Run)
	// 		APPStatus.uiMsgCntr);	   // 99	->	Message Counter	| 1234

	// HAL_UART_Transmit(&huart1, (uint8_t *)cAPP_eGlobalBuffer, strlen(cAPP_eGlobalBuffer), 500);
	// char sizeMsg[32];
	// uint16_t msgLength = strlen(cAPP_eGlobalBuffer);

	// sprintf(sizeMsg, "Size: %u\r\n", strlen(cAPP_eGlobalBuffer));
	// HAL_UART_Transmit(&huart1, (uint8_t *)sizeMsg, strlen(sizeMsg), 500);
	IoT_ClusterMsg.IoT_DataSession_symbol = true;
	vAPP_eFeedTheWDT_Exe();
}

void vGEN_eLoadTxRate_Exe(void)
{
	// uint8_t ucHigh, ucLow;
	uint16_t uiTemp;

	if (APPStatus.bIgnStatus == IGN_ON)
	{
		APPStatus.bIgnStatus = IGN_ON; // Set Ignition Status to ON
		// uiTemp = EEPROMCONFIG.uiTxRateIgnON;

		// vGEN_eMakeIntBattChargingOnOff_Exe(ON);
	}
	else if (APPStatus.bIgnStatus == IGN_OFF)
	{
		APPStatus.bIgnStatus = IGN_OFF; // Set Ignition Status to OFF
		// uiTemp = EEPROMCONFIG.uiTxRateIgnOFF;

		// if (BATTStatus.bMainBATTVoltStatus == BATTERY_NOT_PRESENT && BATTStatus.bIntBATTVoltStatus == BATTERY_PRESENT)
		// {
		// 	APPStatus.bIgnStatus = IGN_OFF; // Set Ignition Status to OFF
		// 	uiTemp = EEPROMCONFIG.uiTxRateBattRemove;
		// }
	}

	TIMERData.ulTxRate = (uiTemp) * 1000;
}

// void vGEN_eMakeIntBattChargingOnOff_Exe(bool bStatus)
// {
// 	if (bStatus == OFF)
// 	{
// 		if (BATTStatus.bIntBattMOSFETChgStatus == ON)
// 		{
// 			HAL_GPIO_WritePin(INTBATT_CHG_DRV_GPIO_Port, INTBATT_CHG_DRV_Pin, GPIO_PIN_RESET);
// 			BATTStatus.bIntBattMOSFETChgStatus = OFF;
// 		}
// 		else
// 		{
// 		}
// 	}
// 	else
// 	{
// 		if (BATTStatus.bBATTVoltStatus == BATTERY_PRESENT)
// 		{
// 			if (BATTStatus.bIntBattMOSFETChgStatus == OFF)
// 			{
// 				vGEN_eONBattChg_Exe();
// 				BATTStatus.bIntBattMOSFETChgStatus = ON;
// 			}
// 			else
// 			{
// 			}
// 		}
// 		else
// 		{
// 			HAL_GPIO_WritePin(INTBATT_CHG_DRV_GPIO_Port, INTBATT_CHG_DRV_Pin, GPIO_PIN_RESET);
// 			BATTStatus.bIntBattMOSFETChgStatus = OFF;
// 		}
// 	}
// }

//void vGEN_eONBattChg_Exe(void)
//{
//	uint8_t ucTemp;
//
//	for (ucTemp = 0; ucTemp < 10; ucTemp++)
//	{
//		HAL_GPIO_WritePin(INTBATT_CHG_DRV_GPIO_Port, INTBATT_CHG_DRV_Pin, GPIO_PIN_RESET);
//		HAL_Delay(1);
//		HAL_GPIO_WritePin(INTBATT_CHG_DRV_GPIO_Port, INTBATT_CHG_DRV_Pin, GPIO_PIN_SET);
//		HAL_Delay(1);
//	}
//}

//void vGEN_eMakeGSMPowerDownIfBattNotPresent_Exe(void)
//{
//	vBATT_eReadMainBATTVolt_Exe();
//	if ((BATTStatus.bBATTVoltStatus == NO_BATTERY_VOLTAGE) && (GSMStatus.bPwrDownStatus == GSM_ACTIVE))
//	{
//		GSMStatus.bPwrDownStatus = GSM_POWER_DOWN;
//		GSMStatus.ucGSMReadyStatus = GSM_NOT_READY;
//		GPSData.bGPSStatus = GPS_NOT_FIXED;
//		vGSM_eSoftOFFGSM_Exe();
//		// if (APPStatus.ucHWVer == 'D')
//		// {
//		vGSM_eTurnOFFGSM_Exe();
//	}
//	else
//	{
//	}
//}

void vGEN_eSendBattRemovedEventData_Exe(void)
{
	// vBATT_eReadMainBATTVolt_Exe();

	// if (BATTStatus.bBATTVoltStatusPrev != BATTStatus.bBATTStatus)
	// {
	// 	if (BATTStatus.bBATTStatus == NO_BATTERY_VOLTAGE)
	// 	{
	// 		bGEN_eSendLiveModeData_Exe(EVENT_DATA, BATTERY_REMOVED_ALERT);
	// 	}
	// 	else
	// 	{
	// 		bGEN_eSendLiveModeData_Exe(EVENT_DATA, BATTERY_CONNECTED_ALERT);
	// 	}
	// }
	// else
	// {
	// }
	// BATTStatus.bBATTVoltStatusPrev = BATTStatus.bBATTStatus;
}
/***** END OF FILE ****/

/*
 * APP.c
 *
 *  Created on: April 15, 2025
 *      Author: Brij.Z
 */

//*****************************************************************************
//   INCLUDE
//*****************************************************************************
#include "stm32g0xx_hal.h"
#include "stm32g0xx.h"
#include "GPS.H"
#include "COMM.H"
#include "APP.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

//*****************************************************************************
// EXTERNAL VARIABLES
//*****************************************************************************

TsGPSData GPSData;
extern TsGSMData GSMData;
extern TsGSMStatus GSMStatus;
extern TsUARTData UARTData;
extern TsAPPData APPData;

extern IoT_ClusterMsg_t IoT_ClusterMsg;

//*****************************************************************************
// INTERNAL VARIABLES
//*****************************************************************************

//*****************************************************************************
// STRUCTURE VARIABLES
//*****************************************************************************
static void vGPS_iResetGPS_Exe(void);

// ============================================================================
// Name			: vGPS_eInit
// Objective	: Initialise Quectel GSM/GPS Module by
//		  		  1. GPS Not Fixed
// Input  		: none
// Output 		: none
// Return		: none
// Version		: -
// ============================================================================
void vGPS_eInit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	// GPIO Ports Clock Enable
	__HAL_RCC_GPIOA_CLK_ENABLE();

	// Configure GPIO pin Output Level
	HAL_GPIO_WritePin(GPS_STATUS_LED_GPIO_Port, GPS_STATUS_LED_Pin, GPIO_PIN_RESET);

	// Configure GPIO pins
	GPIO_InitStruct.Pin = GPS_STATUS_LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPS_STATUS_LED_GPIO_Port, &GPIO_InitStruct);

	GPSData.ucGPSLEDStatus = OFF;

	GPSData.bGPSPower = OFF;
	GPSData.ucGPSNotFixedCntr = 0;
	GPSData.cDegLAT[0] = 0;
	GPSData.cDegLONG[0] = 0;
	GPSData.cSat[0] = 0;
	GPSData.cAltitude[0] = 0;
	GPSData.cHeading[0] = 0;
	GPSData.cSpeed[0] = 0;
	GPSData.cHDOP[0] = 0;
	GPSData.cVDOP[0] = 0;
}

// ============================================================================
// Name		: ucGPS_eGetGPSPowerStatus_Exe
// Objective	: Read GPS Power Status
// Input  		: none
// Output 	: none
// Return		: GPS Power Status
//		  		  0 - OFF
//		  		  1 - ON
// Version	: -
// ============================================================================
uint8_t ucGPS_eGetGPSPowerStatus_Exe(void)
{
	uint8_t ucResp, ucStatus;
	char *cBuff;

	ucResp = ucGSM_eProcessATCmdDualRespWithCmdNum_Exe(GPSATCOMMAND_GETGPSPWR, GPSATRESPONSE_QGNSSC, 12, ATRESPONSE_OK, 6, 300, 1000);
	if (ucResp == BOTH_RESPONSES_MATCHING)
	{
		cBuff = strchr(GSM_DataBuffer, ':');
		if (cBuff != NULL)
		{
			if (*(cBuff + 2) == '1')
			{
				ucStatus = ON;
			}
			else
			{
				ucStatus = OFF;
			}
		}
		else
		{
			ucStatus = OFF;
		}
	}
	else
	{
		ucStatus = OFF;
	}

	GPSData.bGPSPower = ucStatus;
	return ucStatus;
}

// ============================================================================
// Name			: vGPS_eSetGPSPower_Exe
// Objective	: Switch ON / OFF GPS Power
// Input  		: Power Status
//				  OFF - Power OFF
//				  ON - Power ON
// Output 		: GPSData.ucGPSPower
//		  		  0 - OFF
//		  		  1 - ON
// Return		: none
// Version		: -
// ============================================================================
void vGPS_eSetGPSPower_Exe(uint8_t ucGPSPwrStatus)
{
	uint8_t ucResp;

	if (ucGPSPwrStatus == ON)
	{
		ucResp = ucGSM_eProcessATCmdWithCmdNum_Exe(GPSATCOMMAND_QGPSPWR1, ATRESPONSE_OK, 200, 1000);
		if (ucResp == RESPONSE_MATCHING)
		{
			GPSData.bGPSPower = ON;
		}
	}
	else if (ucGPSPwrStatus == OFF)
	{
		ucResp = ucGSM_eProcessATCmdWithCmdNum_Exe(GPSATCOMMAND_QGPSPWR0, ATRESPONSE_OK, 200, 1000);
		if (ucResp == RESPONSE_MATCHING)
		{
			GPSData.bGPSPower = OFF;
		}
	}
	else
	{
	}
}

// ============================================================================
// Name			: vGPS_eStartGPS_Exe
// Objective	: Start GPS Module
// Input  		: none
// Output 		: none
// Return		: none
// Version		: -
// ============================================================================
void vGPS_eStartGPS_Exe(void)
{
	uint8_t ucStatus;

	ucStatus = ucGPS_eGetGPSPowerStatus_Exe();
	if (ucStatus == OFF)
	{
		vGPS_eSetGPSPower_Exe(ON);
	}
	else
	{
	}
}

// ============================================================================
// Name			: bGPS_eGetGPSLoc_Exe
// Objective	: Read GPS Location
//
//  When GPS Not Fixed
//	----------------
// 	Command : AT+QGPSLOC=0<CR>
// 	Response : <CR><LF>
// 			+CMS ERROR: invalid length<CR><LF>
//
//  When GPS is Fixed
//	----------------
// 	Command : AT+QGPSLOC=0<CR>
// 	Response : <CR><LF>
//		+QGPSLOC: 132020.000,28.67403,77.14005,1.4,172.2,3,000.00,1.3,0.7,071221,20<CR><LF>
//
// UTC Time: 132020
// LAT: 28.67403
// LONG: 77.14005
// HDOP: 1.4
// Altitude: 172.2
// fix: 3
// COG: 000.00
// speed kmph: 1.3
// speed knot: 0.7
// Date:071221
// Sat: 20
//
// Input  		: none
// Output 		: Lat, Long, N/S, E/W, Dir, Speed, Sat etc at GPSData
// Return		: 	bFlag : FALSE, When CMS Error
//			  		bFlag : TRUE, Data is Ok
// Version		: -
// ============================================================================
bool bGPS_eGetGPSLoc_Exe(void)
{
	uint8_t ucResp;
	bool bFlag;
	char *pcStartAdr, *pcBuffAdr, *pcBuffAdr1;

	bFlag = GPS_NOT_FIXED;
	ucResp = ucGSM_eProcessATCmdWithCmdNum_Exe(GPSATCOMMAND_QGPSLOC, ATRESPONSE_OK, 200, 1000);
	if (ucResp == RESPONSE_MATCHING)
	{
		pcStartAdr = pucUTL_eSearchChar_Exe(',', 1, GSM_DataBuffer);
		if (pcStartAdr != 0)
		{
			pcStartAdr++;

			// LAT
			pcBuffAdr = pucUTL_eSearchChar_Exe(',', 1, pcStartAdr);
			if (pcBuffAdr != 0)
			{
				*pcBuffAdr = '\0';
				strcpy(GPSData.cDegLAT, pcStartAdr);
				*pcBuffAdr = ',';

				// LONG
				pcBuffAdr++;
				pcBuffAdr1 = pucUTL_eSearchChar_Exe(',', 1, pcBuffAdr);
				if (pcBuffAdr1 != 0)
				{
					*pcBuffAdr1 = '\0';
					strcpy(GPSData.cDegLONG, pcBuffAdr);
					*pcBuffAdr1 = ',';

					// HDOP
					pcBuffAdr1++;
					pcBuffAdr = pucUTL_eSearchChar_Exe(',', 1, pcBuffAdr1);
					if (pcBuffAdr != 0)
					{
						*pcBuffAdr = '\0';
						strcpy(GPSData.cHDOP, pcBuffAdr1);
						*pcBuffAdr = ',';

						// Altitude
						pcBuffAdr++;
						pcBuffAdr1 = pucUTL_eSearchChar_Exe(',', 1, pcBuffAdr);
						if (pcBuffAdr1 != 0)
						{
							*pcBuffAdr1 = '\0';
							strcpy(GPSData.cAltitude, pcBuffAdr);
							*pcBuffAdr1 = ',';

							// Fix: 2 for 2D, 3 for 3D
							pcBuffAdr1++;
							pcBuffAdr = pucUTL_eSearchChar_Exe(',', 1, pcBuffAdr1);
							if (pcBuffAdr != 0)
							{
								*pcBuffAdr = '\0';
								strcpy(GPSData.cFix, pcBuffAdr1);
								*pcBuffAdr = ',';

								// COG, Course over ground
								pcBuffAdr++;
								pcBuffAdr1 = pucUTL_eSearchChar_Exe(',', 1, pcBuffAdr);
								if (pcBuffAdr1 != 0)
								{
									*pcBuffAdr1 = '\0';
									strcpy(GPSData.cHeading, pcBuffAdr);
									*pcBuffAdr1 = ',';

									// Speed in kmph
									pcBuffAdr1++;
									pcBuffAdr = pucUTL_eSearchChar_Exe(',', 1, pcBuffAdr1);
									if (pcBuffAdr != 0)
									{
										*pcBuffAdr = '\0';
										strcpy(GPSData.cSpeed, pcBuffAdr1);
										*pcBuffAdr = ',';

										// SAT
										pcBuffAdr++;
										pcBuffAdr1 = pucUTL_eSearchChar_Exe(',', 2, pcBuffAdr);
										if (pcBuffAdr1 != 0)
										{
											pcBuffAdr1++;
											pcBuffAdr = pucUTL_eSearchChar_Exe('\r', 1, pcBuffAdr1);
											if (pcBuffAdr != 0)
											{
												*pcBuffAdr = '\0';
												strcpy(GPSData.cSat, pcBuffAdr1);
												*pcBuffAdr = '\r';
												bFlag = GPS_FIXED;
												GPSData.bGPSStatus = GPS_FIXED;
												GPSData.ucGPSNotFixedCntr = 0;
											}
											else
											{
											}
										}
										else
										{
										}
									}
									else
									{
									}
								}
								else
								{
								}
							}
							else
							{
							}
						}
						else
						{
						}
					}
					else
					{
					}
				}
				else
				{
				}
			}
			else
			{
			}
		}
		else
		{
		}
	}
	else
	{
	}

	//	GPSData.ucGPSStatusSymbol = ucFlag1;

	if (bFlag == GPS_NOT_FIXED)
	{
		GPSData.ucGPSNotFixedCntr++;

		//		strcpy(GPSData.cDegLAT,   	cGPS_QUECTEL_eATResponseTbl[GPSATRESPONSE_MINUS99]);
		//		strcpy(GPSData.cDegLONG,	cGPS_QUECTEL_eATResponseTbl[GPSATRESPONSE_MINUS99]);
		//		strcpy(GPSData.cSat,	  	cGPS_QUECTEL_eATResponseTbl[GPSATRESPONSE_MINUS99]);
		//		strcpy(GPSData.cSpeed,    	cGPS_QUECTEL_eATResponseTbl[GPSATRESPONSE_MINUS99]);
	}
	else
	{
	}

	GPSData.bGPSStatus = bFlag; // V3.02@18112023

	return bFlag;
}

// ============================================================================
// Name			: vGPS_eReadGPSData_Exe
// Objective	: Read GPS Data and store in Array
// Input  		: Array Member 0 to 9
// Output 		: Lat, Long, N/S, E/W, Dir, Speed, Sat etc at GPSData Struct
// Return		: none
// Version		: -
// ============================================================================
void vGPS_eReadGPSData_Exe(void)
{
	bGPS_eGetGPSLoc_Exe();

	if (GPSData.ucGPSNotFixedCntr > 20)
	{
		vGPS_iResetGPS_Exe();
	}
	else
	{
	}
}

static void vGPS_iResetGPS_Exe(void)
{
	bool bResp;

	bResp = ucGPS_eGetGPSPowerStatus_Exe();
	if (bResp == ON)
	{
		vGPS_eSetGPSPower_Exe(OFF);
	}
	else
	{
	}
	vGPS_eStartGPS_Exe();

	GPSData.ucGPSNotFixedCntr = 0;
}

void vGPS_eOperateGPSLED_Exe(void)
{
	if (GPSData.ucGPSLEDTimer >= 2)
	{
		GPSData.ucGPSLEDTimer = 0;

		if (GPSData.bGPSStatus == GPS_FIXED)
		{
			HAL_GPIO_TogglePin(GPS_STATUS_LED_GPIO_Port, GPS_STATUS_LED_Pin);
			IoT_ClusterMsg.IoT_GPS_symbol = true;
		}
		else
		{
			HAL_GPIO_WritePin(GPS_STATUS_LED_GPIO_Port, GPS_STATUS_LED_Pin, GPIO_PIN_RESET);
			IoT_ClusterMsg.IoT_GPS_symbol = false;
		}
	}
	else
	{
	}
}
/***** END OF FILE ****/

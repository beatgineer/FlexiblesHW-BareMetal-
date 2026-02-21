// ============================================================================
// Module	: Quectel GSM/GPS Module EC20
// Version	: 00
// ============================================================================

//*****************************************************************************
//   INCLUDE
//*****************************************************************************
#include "stm32g0xx.h"
#include "stm32g070xx.h"
#include "APP.h"
#include "GSM.H"
#include "GPS.H"
#include "COMM.H"
#include "main.H"
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>

//*****************************************************************************
// EXTERNAL VARIABLES
//*****************************************************************************

TsGSMData GSMData;
TsGSMStatus GSMStatus;
IoT_ClusterMsg_t IoT_ClusterMsg;
extern TsAPP_eTimer TIMERData;
extern TsGPSData GPSData;
extern TsAPP_eConfig APPCONFIG;

//*****************************************************************************
const char *cGSM_QUECTEL_eATCommandTbl[] =
    {
        "AT\r", // 0
        "ATE0\r",
        "AT+CMEE=2\r",
        "ATV1\r",
        "AT+CSCS=\"IRA\"\r",
        "AT+CPIN?\r", // 5
        "AT+CREG?\r",
        "AT+CGREG?\r",
        "AT+CGSN\r",
        "AT+CSQ\r",
        "AT+QICSGP=", // 10
        "AT+QIACT=",
        "AT+QIDEACT=",
        "AT+QIOPEN=1,",
        "AT+QISTATE=1,0\r",
        "AT+QISEND=", // 15
        "AT+QICLOSE=",
        "AT+CMGF=1\r",
        "AT+CSMP=17,167,0,0\r",
        "AT+CMGS=",
        "AT+CSCA?\r", // 20
        "AT+CPMS=\"SM\",\"SM\",\"SM\"\r",
        "AT+CSDH=1\r",
        "AT+CMGR=",
        "AT+CMGD=",
        "AT+QURCCFG=\"URCPORT\",\"uart1\"\r", // 25
        "AT+QIACT?\r",
        "AT+QCMGS=",
        "AT+QFTPCFG=\"contextid\",",
        "AT+QFTPCFG=\"account\",",
        "AT+QFTPCFG=\"filetype\",1\r", // 30
        "AT+QFTPCFG=\"transmode\",1\r",
        "AT+QFTPCFG=\"rsptimeout\",90\r",
        "AT+QFTPOPEN=",
        "AT+QFTPCWD=",
        "AT+QFTPSIZE=", // 35
        "AT+QFTPGET=",
        "AT+QFLST",
        "AT+QFDEL=\"UFS:",
        "AT+CCLK?\r",
        "AT+CTZU=3\r", // 40
        "AT+QFTPCLOSE\r",
        "AT+QCCID\r",
        "AT+QNWINFO\r",
        "AT+QPOWD\r",
        "AT+CPMS=\"ME\",\"ME\",\"ME\"\r", // 45
        "AT+CEREG?\r",
        "AT+QFOPEN=",
        "AT+QFOPEN?\r",
        "AT+QFSEEK=",
        "AT+QFREAD=", // 50
        "AT+QFCLOSE=",
        "AT+QGPS?\r",
        "AT+QGPS=1\r",
        "AT+QGPSLOC=2\r",
        "AT+QGPS=0\r", // 55
};

const char *cGSM_QUECTEL_eATResponseTbl[] =
    {
        "OK\r", // 0
        "+CPIN: READY",
        "+COPS:",
        "+CREG:",
        "+CGREG:",
        "+CSQ:", // 5
        "ERROR",
        "+CME ERROR:",
        "+QISTATE:",
        ">",
        "CLOSE OK", // 10
        "+CMGS:",
        "+CSCA:",
        "+CPMS:",
        "+CMGR:",
        "+CEREG:", // 15
        "+QIACT:",
        "+QFTPCWD: 0,0",
        "+QFTPSIZE: 0,",
        "+QFTPGET:",
        "+QFLST:", // 20
        "+CCLK:",
        "SEND OK",
        "+QIOPEN:",
        "+CCLK: \"80",
        "+QNWINFO:", // 25
        "GSM",
        "WCDMA",
        "LTE",
        "+QFTPOPEN: 0,0",
        "POWERED DOWN", // 30
        "+CMS ERROR:",
        "+QICSGP:",
        "+QIOPEN: 0,561",
        "+QFOPEN:",
        "CONNECT", // 35
        "+QGPS:",
        "+QGPSLOC:", // 37
};

//*****************************************************************************
// INTERNAL VARIABLES
//*****************************************************************************

//*****************************************************************************
// STRUCTURE VARIABLES
//*****************************************************************************
// ============================================================================
// Name		: vGSM_eInit
// Objective	: Initialise Quectel GSM Module by
//		  		  1. Make H/W Reset by giving a pulse on PWRKey Line
//		  		  2. Set Baud rate to 115.2 kbps
// Input  		: none
// Output 	: none
// Return		: none
// Version	: -
// ============================================================================
void vGSM_eInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    // GPIO Ports Clock Enable
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // Configure GPIO pin Output Level
    HAL_GPIO_WritePin(GPIOC, GSM_RESET_Pin | GSM_PWRKEY_Pin_Pin, GPIO_PIN_RESET);

    // Configure GPIO pins : PAPin PAPin PAPin PAPin PAPin PAPin PAPin
    GPIO_InitStruct.Pin = GSM_RESET_Pin | GSM_PWRKEY_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pin : GSM_RI_IN_Pin */
    // GPIO_InitStruct.Pin = GSM_RI_IN_Pin;
    // GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    // GPIO_InitStruct.Pull = GPIO_NOPULL;
    // HAL_GPIO_Init(GSM_RI_IN_GPIO_Port, &GPIO_InitStruct);

    GSMData.ucNoResponseCntr = 0;
    GSMData.ucErrorCntr = 0;
    GSMData.ucNoSocketCntr = 0;

    GSMStatus.bGSMRebootRequired = FALSE;
    GSMStatus.bPwrDownStatus = GSM_ACTIVE;

    GSMData.ucPendingMESMS = 0;
    GSMData.ucTotalMESMS = 0;

    GSMData.ucPendingSMSMS = 0;
    GSMData.ucTotalSMSMS = 0;

    GSMStatus.bFWSMSPending = FALSE;

    // if (APPData.ucHWVer == 'D')
    // {
    // 	vGSM_eTurnONGSM_Exe();
    // }
    // else
    // {
    // }

    vCOMM_eUSART3Init();
    HAL_UART_Receive_IT(GSMComPortHandle, (uint8_t *)GSM_RxOneByte, 1);

    GSM_RxCntr = 0;
    GSM_TimeOut = 0;
}

// ============================================================================
// Name		: vGSM_eTurnONGSM_Exe
// Objective	: Power ON GSM Module by Switching ON the P Ch MOSFET
// Input  		: none
// Output 	: none
// Return		: none
// Version	: -
// ============================================================================
// void vGSM_ePowerONGSM_Exe(void)
//{
//    uint8_t ucTemp;
//
//    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
//
//    ucTemp = 0;
//    while (ucTemp < 200)
//    {
//        TIM3->CCR1 = ucTemp;
//        ucTemp += 10;
//        HAL_Delay(1);
//    }
//    TIM3->CCR1 = 1000;
//
//    vAPP_eFeedTheWDT_Exe();
//    HAL_Delay(2000);
//    vAPP_eFeedTheWDT_Exe();
//}

// ============================================================================
// Name		: vGSM_eGetIMEINum_Exe
// Objective	: Read GSM Module IMEI No.
//				 AT+GSN<CR> to read
//				 Response
//				 <CR><LF>
//				 861359035557188<CR><LF>
//	                		 <CR><LF>
//			   	 OK<CR><LF>
//
// Input  		: Pointer for IMEI Number
// Output 	: IMEI No. at Pointer
// Return		: False : Not Ok, True : Ok
// Version	: -
// ============================================================================
bool bGSM_eGetIMEINum_Exe(char *cOutBuff)
{
    uint8_t ucResp;
    char *cBuffAdr, *cBuffAdr1;
    bool bResp;

    bResp = FALSE;
    ucResp = ucGSM_eProcessATCmdWithCmdNum_Exe(ATCOMMAND_CGSN, ATRESPONSE_OK, 200, 2000);

    if (ucResp == RESPONSE_MATCHING)
    {
        cBuffAdr = strchr((char *)GSM_DataBuffer, '\n');
        if (cBuffAdr != NULL)
        {
            cBuffAdr++;
            cBuffAdr1 = strchr((char *)cBuffAdr, '\r');
            if (cBuffAdr != NULL)
            {
                *cBuffAdr1 = '\0';
                if (strlen(cBuffAdr) == 15)
                {
                    strcpy(cOutBuff, cBuffAdr);
                    bResp = TRUE;
                    *cBuffAdr1 = 0x0D;
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

    if (bResp == FALSE)
    {
        *cOutBuff = 0;
    }
    else
    {
    }
    return bResp;
}

// ============================================================================
// Name		: vGSM_eGetICCIDNum_Exe
// Objective	: Read SIM ICCID
//				AT+QCCID<CR> to read
//				Response
//				 <CR><LF>
//				 89910271001095669812<CR><LF>
//	                 		<CR><LF>
//			   	 OK<CR><LF>
//
// Input  		: none
// Output 	: ICCID No. at OutBuffer
// Return		: False : Not Ok, True : Ok
// Version	: -
// ============================================================================
bool bGSM_eGetICCIDNum_Exe(char *cOutBuff)
{
    uint8_t ucResp;
    char *cBuffAdr, *cBuffAdr1;
    bool bResp;

    bResp = FALSE;
    ucResp = ucGSM_eProcessATCmdWithCmdNum_Exe(ATCOMMAND_QCCID, ATRESPONSE_OK, 200, 2000);

    if (ucResp == RESPONSE_MATCHING)
    {
        cBuffAdr = strchr((char *)GSM_DataBuffer, ':');
        if (cBuffAdr != NULL)
        {
            cBuffAdr++;
            cBuffAdr++;
            cBuffAdr1 = strchr((char *)cBuffAdr, 0x0D);
            if (cBuffAdr != NULL)
            {
                *cBuffAdr1 = '\0';

                if (strlen(cBuffAdr) == 20)
                {
                    strcpy(cOutBuff, cBuffAdr);
                    bResp = TRUE;
                    *cBuffAdr1 = 0x0D;
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

    if (bResp == FALSE)
    {
        *cOutBuff = 0;
    }
    else
    {
    }
    return bResp;
}

// ============================================================================
// Name			: ucGSM_eProcessATCommand_Exe
// Objective	:	1. Forward AT Commands to GSM Module whose pointer is inputed.
//					2. Wait for the Response up to Given Time out.
//					3. Compare Received Response with the Given Response
// Input		:	1. AT Command pointer
//					2. Desired Response Pointer
//					3. Time Out, if Character Received
//					4. Time Out, if No Character Received
// Output		: GSMHealth.ucCMEErrorCntr
// Return		: 	RESPONSE_MATCHING		- 0
//			  		RESPONSE_NOT_MATCHING	- 1
//			  		RESPONSE_ERROR			- 2
//			  		RESPONSE_CME_ERROR		- 3
//			  		RESPONSE_CMS_ERROR		- 4
//			  		NO_RESPONSE				- 5
// Version	: -
// ============================================================================
uint8_t ucGSM_eProcessATCmdWithCmdNum_Exe(uint8_t ucATCmdNum, uint8_t ucRespNum, uint32_t ulTimeOutCharRecd, uint32_t ulTimeOutNoCharRecd)
{
    char *cATCmd;
    uint8_t ucResp;

    cATCmd = (char *)cGSM_QUECTEL_eATCommandTbl[ucATCmdNum];
    ucResp = ucGSM_eProcessATCmd_Exe((char *)cATCmd, ucRespNum, ulTimeOutCharRecd, ulTimeOutNoCharRecd);
    return ucResp;
}

uint8_t ucGSM_eProcessATCmdWithCmdPtr_Exe(char *cATCmd, uint8_t ucRespNum, uint32_t ulTimeOutCharRecd, uint32_t ulTimeOutNoCharRecd)
{
    uint8_t ucResp;

    ucResp = ucGSM_eProcessATCmd_Exe((char *)cATCmd, ucRespNum, ulTimeOutCharRecd, ulTimeOutNoCharRecd);
    return ucResp;
}

uint8_t ucGSM_eProcessATCmd_Exe(char *cATCmd, uint8_t ucRespNum, uint32_t ulTimeOutCharRecd, uint32_t ulTimeOutNoCharRecd)
{
    uint8_t ucTemp;
    bool bResp;

    GSM_RxCntr = 0;
    GSM_TimeOut = 0;

    ucTemp = NO_RESPONSE;
    vCOMM_eTxString_Exe(GSMComPortHandle, (char *)cATCmd, 200);
    HAL_UART_Receive_IT(GSMComPortHandle, (uint8_t *)GSM_RxOneByte, 1);

    bResp = bGSM_eWait4RespOrTimeOut_Exe(ulTimeOutCharRecd, ulTimeOutNoCharRecd);

    if (bResp == FALSE)
    {
        GSMData.ucNoResponseCntr = 0;
        ucTemp = ucGSM_eCompareStringInGSMBuff_Exe((char *)cGSM_QUECTEL_eATResponseTbl[ucRespNum]);
        if ((ucTemp == RESPONSE_ERROR) || (ucTemp == RESPONSE_CME_ERROR) || (ucTemp == RESPONSE_CMS_ERROR))
        {
            GSMData.ucErrorCntr++;
        }
        else
        {
        }
    }
    else
    {
        if (GSM_RxCntr == 0)
        {
            GSMData.ucNoResponseCntr++;
        }
        else
        {
        }
    }
    return ucTemp;
}

// ============================================================================
// Name		: vGSM_eWait4RespOrTimeOut_Exe
// Objective	:1. Wait for Response from GSM Module
//	    		 2. Wait for Character Received Time out, if any character is received or
//	     		 3. Wait for No Character Received Time Out
//
// Output 	: none
// Return		: 0 - Character recd and its Timeout over
//		 	 1 - No char recd, Full Timeout Over
// ============================================================================
bool bGSM_eWait4RespOrTimeOut_Exe(uint32_t ulTimeOutCharRecd, uint32_t ulTimeOutNoCharRecd)
{
    bool bTemp;

    bTemp = FALSE;
    do
    {
        if (GSM_TimeOut > ulTimeOutNoCharRecd)
        {
            bTemp = TRUE;
        }
        else
        {
        }

        vAPP_eFeedTheWDT_Exe();
    } while ((GSM_RxCntr <= 1) && (bTemp == FALSE));

    if (bTemp == FALSE)
    {
        do
        {
            vAPP_eFeedTheWDT_Exe();

        } while (GSM_TimeOut < ulTimeOutCharRecd);
    }
    else
    {
    }

    return bTemp;
}

// ============================================================================
// Name			: ucGSMData_eCompareStringInGSMBuff_Exe
// Objective	: Compare Received Response at GSM Buffer with the Given Response
// Input  		: Given Response Pointer
// Output 		: Error Code at GSMData.cErrCode[2], First Element - MSB
// Return		:	RESPONSE_MATCHING		- 0
//		  	  		RESPONSE_NOT_MATCHING	- 1
//		  	  		RESPONSE_ERROR			- 2
//		  	  		RESPONSE_CME_ERROR		- 3
//		  	  		RESPONSE_CMS_ERROR		- 4
// Version		: -
// Author		: AK
// Date			: 26-07-2018
// ============================================================================
uint8_t ucGSM_eCompareStringInGSMBuff_Exe(char *cString)
{
    uint8_t ucTemp;
    uint16_t uiVal;
    char *cBuffAdd, *cBuffAdd1;

    if (strstr((char *)GSM_DataBuffer, cString) != 0)
    {
        ucTemp = RESPONSE_MATCHING;

        GSMData.cErrCode[0] = 0;
        GSMData.cErrCode[1] = 0;
    }
    else if (strstr((char *)GSM_DataBuffer, (char *)cGSM_QUECTEL_eATResponseTbl[ATRESPONSE_CMEERROR]) != 0)
    {
        ucTemp = RESPONSE_CME_ERROR;

        cBuffAdd = strchr((char *)GSM_DataBuffer, ':');
        if (cBuffAdd == NULL)
        {
            GSMData.cErrCode[0] = 0;
            GSMData.cErrCode[1] = 0;
        }
        else
        {
            cBuffAdd++;
            cBuffAdd++;
            cBuffAdd1 = strchr(cBuffAdd, 0x0D);
            *cBuffAdd1 = 0;

            uiVal = atoi(cBuffAdd);
            GSMData.cErrCode[0] = uiVal >> 8;
            GSMData.cErrCode[1] = uiVal;
        }
    }
    else if (strstr((char *)GSM_DataBuffer, (char *)cGSM_QUECTEL_eATResponseTbl[ATRESPONSE_CMSERROR]) != 0)
    {
        ucTemp = RESPONSE_CMS_ERROR;

        cBuffAdd = strchr((char *)GSM_DataBuffer, ':');
        if (cBuffAdd == NULL)
        {
            GSMData.cErrCode[0] = 0;
            GSMData.cErrCode[1] = 0;
        }
        else
        {
            cBuffAdd++;
            cBuffAdd++;
            cBuffAdd1 = strchr(cBuffAdd, 0x0D);
            *cBuffAdd1 = 0;

            uiVal = atoi(cBuffAdd);
            GSMData.cErrCode[0] = uiVal >> 8;
            GSMData.cErrCode[1] = uiVal;
        }
    }
    else if (strstr((char *)GSM_DataBuffer, (char *)cGSM_QUECTEL_eATResponseTbl[ATRESPONSE_ERROR]) != 0)
    {
        ucTemp = RESPONSE_ERROR;

        GSMData.cErrCode[0] = 0;
        GSMData.cErrCode[1] = 0;
    }
    else
    {
        ucTemp = RESPONSE_NOT_MATCHING;

        GSMData.cErrCode[0] = 0;
        GSMData.cErrCode[1] = 0;
    }
    return ucTemp;
}

// ============================================================================
// Name		: vGSM_eStartGSM_Exe
// Objective	: Start GSM Module
//				Power ON / Hard Reset / Warm Start message in 3 sec :
//				 RDY
//				+CFUN: 1
//				+CPIN: NOT INSERTED
// Input  		: none
// Output 	: GSMHealth.ucGSMReady
//		  		  0 - GSM_NOT_READY
//		  		  1 - GSM_READY
// Return		: none
// Version	: -
// ============================================================================
void vGSM_eStartGSM_Exe(void)
{
    uint8_t ucResp;

    ucResp = ucGSM_eCheckATResponse_Exe();
    if (ucResp == RESPONSE_MATCHING)
    {
        GSMStatus.ucGSMReadyStatus = GSM_WAS_READY;
    }
    else
    {
        vGSM_ePWRKeyStartGSM_Exe();
        ucResp = ucGSM_eCheckATResponse_Exe();
        if (ucResp == RESPONSE_MATCHING)
        {
            GSMStatus.ucGSMReadyStatus = GSM_READY;
        }
        else
        {
            vGSM_eResetGSM_Exe();
            vGSM_ePWRKeyStartGSM_Exe();

            ucResp = ucGSM_eCheckATResponse_Exe();
            if (ucResp == RESPONSE_MATCHING)
            {
                GSMStatus.ucGSMReadyStatus = GSM_READY;
            }
            else
            {
                GSMStatus.ucGSMReadyStatus = GSM_NOT_READY;
            }
        }
    }

    vGSM_eDisableGPRS_Exe(PDP_CONTEXT_ID);
    vGSM_eCloseSocket_Exe(LOCATION_SOCKET_ID);
}

uint8_t ucGSM_eCheckATResponse_Exe(void)
{
    uint8_t ucResp;

    ucResp = ucGSM_eProcessATCmdWithCmdNum_Exe(ATCOMMAND_ATE0, ATRESPONSE_OK, 300, 1000);
    if (ucResp != RESPONSE_MATCHING)
    {
        ucResp = ucGSM_eProcessATCmdWithCmdNum_Exe(ATCOMMAND_ATE0, ATRESPONSE_OK, 300, 1000);
        if (ucResp != RESPONSE_MATCHING)
        {
            ucResp = ucGSM_eProcessATCmdWithCmdNum_Exe(ATCOMMAND_ATE0, ATRESPONSE_OK, 300, 1000);
        }
        else
        {
        }
    }
    else
    {
    }

    return ucResp;
}

// ============================================================================
// Name		: vGSM_ePWRKeyStartGSM_Exe
// Objective	: Turn ON GSM Module by giving a Low pulse to PWRKEY Pin
//		   	  for >= 2S
//			  UART will be inactive for 5 Sec from the start of PWRKEY DOWN
// Input  		: none
// Output 	: none
// Return		: none
// Version	: -
// ============================================================================
void vGSM_ePWRKeyStartGSM_Exe(void)
{
    vAPP_eFeedTheWDT_Exe();
    HAL_GPIO_WritePin(GSM_PWRKEY_GPIO_Port, GSM_PWRKEY_Pin, GPIO_PIN_SET);
    HAL_Delay(2100);

    vAPP_eFeedTheWDT_Exe();
    HAL_GPIO_WritePin(GSM_PWRKEY_GPIO_Port, GSM_PWRKEY_Pin, GPIO_PIN_RESET);

    GSM_RxCntr = 0;
    TIMERData.uiTimerCntr = 0;
    do
    {
        vAPP_eFeedTheWDT_Exe();
        HAL_Delay(1000);
    } while ((GSM_RxCntr < 60) && (TIMERData.uiTimerCntr < 40000));
}

// ============================================================================
// Name		: vGSM_eResetGSM_Exe
// Objective	: Hardware Reset GSM Module by giving a Low pulse to RESET_N Pin
//				  for 150mS to 460mS.
//				  RESETTING TO BE DONE WHEN TURNING OFF THE MODULE BY AT+QPOWD COMMAND
//				  AND PWRKEY PIN FAILED.
// Input  		: none
// Output 	: none
// Return		: none
// Version	: -
// ============================================================================
void vGSM_eResetGSM_Exe(void)
{
    HAL_GPIO_WritePin(GSM_RESET_GPIO_Port, GSM_RESET_Pin, GPIO_PIN_SET);
    HAL_Delay(460);
    HAL_GPIO_WritePin(GSM_RESET_GPIO_Port, GSM_RESET_Pin, GPIO_PIN_RESET);
    vAPP_eFeedTheWDT_Exe();
    HAL_Delay(2000);
}

// ============================================================================
// Name		: vGSM_eDisableGPRS_Exe
// Objective	: Disable PDP Context.
//			AT+QIDEACT=1<CR>
//			Response
//	                 <CR><LF>
//			OK<CR><LF>
//
// Input  		: none
// Output 	: GPRS Status at GSMHealth.bGPRSStatus
//		  		GPRS_NOT_ACTIVE		0
// Return		: none
// ============================================================================
void vGSM_eDisableGPRS_Exe(uint8_t ucPDPContextID)
{
    uint8_t ucResp;
    char cBuff[50];

    sprintf(cBuff, "%s%d\r", cGSM_QUECTEL_eATCommandTbl[ATCOMMAND_QIDEACT], ucPDPContextID);
    ucResp = ucGSM_eProcessATCmdWithCmdPtr_Exe(cBuff, ATRESPONSE_OK, 300, 40000);
    if (ucResp == RESPONSE_MATCHING)
    {
        GSMStatus.bGPRSStatus = GPRS_NOT_ACTIVE;
    }
    else
    {
        GSMStatus.bGSMRebootRequired = TRUE;
    }
}

// ============================================================================
// Name			: vGSM_eCloseSocket_Exe
// Objective	: Close Socket if Enable
//
// Input  		: Socket ID
// Output 		: Socket Status updated
// Return		: none
// Version		: -
// ============================================================================
void vGSM_eCloseSocket_Exe(uint8_t ucSocketID)
{
    uint8_t ucResp;
    char cBuff[50];

    sprintf(cBuff, "%s%d\r", cGSM_QUECTEL_eATCommandTbl[ATCOMMAND_QICLOSE], ucSocketID);

    if (ucSocketID == LOCATION_SOCKET_ID)
    {
        ucResp = ucGSM_eProcessATCmdWithCmdPtr_Exe(cBuff, ATRESPONSE_CLOSEOK, 300, 10000);
        if (ucResp == RESPONSE_MATCHING)
        {
            GSMStatus.bSocketStatus = SOCKET_NOT_ACTIVE;
        }
        else
        {
        }
    }
    else
    {
    }
}
// ============================================================================
// Name		: ucGSM_eProcessATCmdWithDualResponse_Exe
// Objective	:1. Forward AT Commands to GSM Module whose pointer is inputed.
//			 2. Wait for the 2 Responses up to Given Time out.
//			 3. Compare both Received Response with the Given Response
// Input		:1. AT Command pointer
//			 2. Desired Response Pointer
//			 3. Time Out, if Character Received
//			 4. Time Out, if No Character Received
// Output		: GSMHealth.ucCMEErrorCntr
// Return		: RESPONSE_MATCHING		- 0
//			  RESPONSE_NOT_MATCHING	- 1
//			  RESPONSE_ERROR			- 2
//			  RESPONSE_CME_ERROR		- 3
//			  RESPONSE_CMS_ERROR		- 4
//			  NO_RESPONSE				- 5
//			  BOTH_RESPONSES_MATCHING	- 6
//			  RESPONSE1_MATCHING		- 7
//			  RESPONSE2_MATCHING		- 8
// Version	: -
// ============================================================================
uint8_t ucGSM_eProcessATCmdDualRespWithPtr_Exe(char *cATCmd, uint8_t ucResp1Num, uint8_t ucLenResp1, uint8_t ucResp2Num, uint8_t ucLenResp2, uint32_t ulTimeOutCharRecd, uint32_t ulTimeOutNoCharRecd)
{
    uint8_t ucResp;

    ucResp = ucGSM_eProcessATCmdDualResp_Exe((char *)cATCmd, ucResp1Num, ucLenResp1, ucResp2Num, ucLenResp2, ulTimeOutCharRecd, ulTimeOutNoCharRecd);
    return ucResp;
}

uint8_t ucGSM_eProcessATCmdDualRespWithCmdNum_Exe(uint8_t ucATCmdNum, uint8_t ucResp1Num, uint8_t ucLenResp1, uint8_t ucResp2Num, uint8_t ucLenResp2, uint32_t ulTimeOutCharRecd, uint32_t ulTimeOutNoCharRecd)
{
    uint8_t ucResp;
    char *cATCmd;

    cATCmd = (char *)cGSM_QUECTEL_eATCommandTbl[ucATCmdNum];
    ucResp = ucGSM_eProcessATCmdDualResp_Exe((char *)cATCmd, ucResp1Num, ucLenResp1, ucResp2Num, ucLenResp2, ulTimeOutCharRecd, ulTimeOutNoCharRecd);
    return ucResp;
}

uint8_t ucGSM_eProcessATCmdDualResp_Exe(char *cATCmd, uint8_t ucResp1Num, uint8_t ucLenResp1, uint8_t ucResp2Num, uint8_t ucLenResp2, uint32_t ulTimeOutCharRecd, uint32_t ulTimeOutNoCharRecd)
{
    uint8_t ucTemp, ucResp;
    char *cBuff1, *cBuff2;

    GSM_RxCntr = 0;
    GSM_TimeOut = 0;

    ucTemp = NO_RESPONSE;
    vCOMM_eTxString_Exe(GSMComPortHandle, (char *)cATCmd, 200);
    HAL_UART_Receive_IT(GSMComPortHandle, (uint8_t *)GSM_RxOneByte, 1);

    ucResp = ucGSM_eWait4RespOrTimeOutDualResp_Exe(ucLenResp1, ucLenResp2, ulTimeOutCharRecd, ulTimeOutNoCharRecd);
    if (ucResp == NO_CHAR_RECD_TIMEOUT)
    {
        if (GSM_RxCntr == 0)
        {
            GSMData.ucNoResponseCntr++;
        }
        else
        {
        }
    }
    else
    {
        vAPP_eFeedTheWDT_Exe();
        GSMData.ucNoResponseCntr = 0;

        if ((ucResp == BOTH_RESPONSE_RECD) || (ucResp == SOME_CHAR_RECD_TIMEOUT) ||
            (ucResp == BOTH_RESPONSES_OR_ERROR_RECD)) // V3.02@20112023
        {
            cBuff1 = strstr(GSM_DataBuffer, (char *)cGSM_QUECTEL_eATResponseTbl[ucResp1Num]);
            cBuff2 = strstr(GSM_DataBuffer, (char *)cGSM_QUECTEL_eATResponseTbl[ucResp2Num]);

            if ((cBuff1 != 0) && (cBuff2 != 0))
            {
                ucTemp = BOTH_RESPONSES_MATCHING;
                GSMData.ucErrorCntr = 0;
            }
            else if (cBuff1 != 0)
            {
                ucTemp = RESPONSE1_MATCHING;
                GSMData.ucErrorCntr = 0;
            }
            else if (cBuff2 != 0)
            {
                ucTemp = RESPONSE2_MATCHING;
                GSMData.ucErrorCntr = 0;
            }
            else
            {
                if (strstr(GSM_DataBuffer, (char *)ATRESPONSE_CMEERROR) != 0)
                {
                    ucTemp = RESPONSE_CME_ERROR;
                    GSMData.ucErrorCntr++;
                }
                else if (strstr(GSM_DataBuffer, (char *)ATRESPONSE_CMSERROR) != 0)
                {
                    ucTemp = RESPONSE_CMS_ERROR; // V3.02@27102023
                    GSMData.ucErrorCntr++;
                }
                else if (strstr(GSM_DataBuffer, (char *)ATRESPONSE_ERROR) != 0)
                {
                    ucTemp = RESPONSE_ERROR;
                    GSMData.ucErrorCntr++;
                }
                else
                {
                    ucTemp = RESPONSE_NOT_MATCHING;
                    GSMData.ucErrorCntr = 0;
                }
            }
        }
        else
        {
            ucTemp = RESPONSE_NOT_MATCHING;
        }
    }
    return ucTemp;
}

// ============================================================================
// Name		: bGSM_eWait4RespOrTimeOutDualResp_Exe
// Objective	:Wait for Response
// Input		:1. Length of Response 1
//			 2. Length of Response 2
//			 3. Time Out, if Character Received
//			 4. Time Out, if No Character Received
// Output	: None
// Return		: NO_CHAR_RECD_TIMEOUT		- 0
//			  BOTH_RESPONSE_RECD		- 1
//			  SOME_CHAR_RECD_TIMEOUT	- 2
// Version	: -
// ============================================================================
uint8_t ucGSM_eWait4RespOrTimeOutDualResp_Exe(uint8_t ucLenResp1, uint8_t ucLenResp2, uint32_t ulTimeOutCharRecd, uint32_t ulTimeOutNoCharRecd)
{
    bool bTemp;
    uint8_t ucResp;

    bTemp = FALSE;
    ucResp = SOME_CHAR_RECD_TIMEOUT; // V3.02@19112023

    do
    {
        if (GSM_TimeOut >= ulTimeOutNoCharRecd)
        {
            bTemp = TRUE;
            ucResp = NO_CHAR_RECD_TIMEOUT;
        }
        else
        {
        }

        vAPP_eFeedTheWDT_Exe();
    } while ((GSM_RxCntr <= 1) && (bTemp == FALSE));

    if (bTemp == FALSE)
    {
        do
        {
            if (GSM_RxCntr >= (ucLenResp1 + ucLenResp2))
            {
                bTemp = TRUE;
                //				ucResp	= BOTH_RESPONSE_RECD;
                ucResp = BOTH_RESPONSES_OR_ERROR_RECD; // V3.02@19112023
                do
                {
                    vAPP_eFeedTheWDT_Exe();

                } while (GSM_TimeOut < ulTimeOutCharRecd);
            }
            else
            {
            }
            vAPP_eFeedTheWDT_Exe();
        } while ((GSM_TimeOut < ulTimeOutNoCharRecd) && (bTemp == FALSE));
    }
    else
    {
        if ((GSM_TimeOut > ulTimeOutNoCharRecd) && (GSM_RxCntr > 1))
        {
            ucResp = SOME_CHAR_RECD_TIMEOUT;
        }
        else
        {
        }
    }

    return ucResp;
}

void vGSM_eInitaliseGSM_Exe(void)
{
    if ((GSMStatus.ucGSMReadyStatus == GSM_READY) || (GSMStatus.ucGSMReadyStatus == GSM_WAS_READY))
    {
        vGSM_eInitialSetup_Exe();
        vGSM_eRegisterAndSetModule_Exe();
        vGSM_eUpdateTimeZoneAndRTC_Exe();
        vGSM_eEnableGPRS_Exe(PDP_CONTEXT_ID);
        vGSM_eReadGPRSStatus_Exe(PDP_CONTEXT_ID);
    }
    else
    {
    }
}

// ============================================================================
// Name		: vGSM_eIntitialSetup_Exe
// Objective	: Make GSM Module Ready by executing initial AT Commands.
// 		 		1. Set ECHO OFF 					: ATE0\r
// 		 		2. Set Result Code and use Verbose Values 	: AT+CMEE=2\r
//		 		3. Set Result code information response 	: ATV1\r
// 		 		4. Set GSM as current character set 		: AT+CSCS="GSM"\r
// Input  		: none
// Output 	: none
// Return		: none
// Version	: -
// ============================================================================
void vGSM_eInitialSetup_Exe(void)
{
    // Set SMS Storage : Fixed in Profile
    ucGSM_eProcessATCmdDualRespWithCmdNum_Exe(ATCOMMAND_CPMS_ME, ATRESPONSE_CPMS, 28, ATRESPONSE_OK, 6, 200, 1000);

    // Set SMS in Text Mode : Fixed in Profile
    ucGSM_eProcessATCmdWithCmdNum_Exe(ATCOMMAND_CMGF, ATRESPONSE_OK, 200, 1000);

    // Set Result Code and use Verbose Values : Set in Profile
    ucGSM_eProcessATCmdWithCmdNum_Exe(ATCOMMAND_CMEE, ATRESPONSE_OK, 200, 1000);

    // Set Result code information response
    ucGSM_eProcessATCmdWithCmdNum_Exe(ATCOMMAND_ATV1, ATRESPONSE_OK, 200, 1000);

    // Set URC Indication
    ucGSM_eProcessATCmdWithCmdNum_Exe(ATCOMMAND_QURCCFG, ATRESPONSE_OK, 200, 1000);

    // Set GSM as current character set : Set in Profile
    ucGSM_eProcessATCmdWithCmdNum_Exe(ATCOMMAND_CSCS, ATRESPONSE_OK, 200, 1000);

    // Check Message Service Centre No.
    ucGSM_eProcessATCmdWithCmdNum_Exe(ATCOMMAND_GETCSCA, ATRESPONSE_CSCA, 200, 1000);

    // Set SMS Parameters for Text Mode
    ucGSM_eProcessATCmdWithCmdNum_Exe(ATCOMMAND_CSMP, ATRESPONSE_OK, 200, 1000);

    // Show Header in Text Mode : Set in Profile
    ucGSM_eProcessATCmdWithCmdNum_Exe(ATCOMMAND_CSDH, ATRESPONSE_OK, 200, 1000);
}

void vGSM_eRegisterAndSetModule_Exe(void)
{
    uint8_t ucResp, ucTemp;
    bool bStatus;

    vGSM_eReadSIMInsertStatus_Exe();

    if (GSMStatus.bSIMInsertStatus == SIM_INSERTED)
    {
        ucTemp = 0;
        do
        {
            ucResp = ucGSM_eReadSignalStrength_Exe(); // Read Signal Strength
            ucTemp++;
            HAL_Delay(1000);
        } while (!(ucResp == GSM_SIGNAL_LEVEL_OK) && (ucTemp < 15));

        GSM_TimeOut = 0;
        do
        {
            bStatus = bGSM_eNetworkSetup_Exe();
            if (bStatus == FALSE)
            {
                HAL_Delay(1000);
            }
            else
            {
            }
        } while ((bStatus == FALSE) && (GSM_TimeOut < 20000));

        vGSM_eReadNetworkInfo_Exe();

        bStatus = bGSM_eCheckAPNSetting_Exe();
        if (bStatus == FALSE)
        {
            vGSM_eSetAPN_Exe();
        }
        else
        {
            ;
        }
    }
    else
    {
    }
}

// ============================================================================
// Name			: vGSM_eReadSIMInsertStatus_Exe
// Objective	: Read SIM Insert Status
// 		  		   Check SIM Inserted	: AT+CPIN?\r
// Input  		: none
// Output 		: SIM Status at GSMHealth.ucSIMInsertStatus
//				  SIM Not Present	0
//		  		  SIM Present		1
//
// Return		: // Version		: -
// ============================================================================
void vGSM_eReadSIMInsertStatus_Exe(void)
{
    uint8_t ucResp;

    // Read SIM PIN Status
    ucResp = ucGSM_eProcessATCmdDualRespWithCmdNum_Exe(ATCOMMAND_CPIN, ATRESPONSE_CPINREADY, 14, ATRESPONSE_OK, 6, 300, 5000);

    if (ucResp == BOTH_RESPONSES_MATCHING)
    {
        GSMStatus.bSIMInsertStatus = SIM_INSERTED;
    }
    else
    {
        GSMStatus.bSIMInsertStatus = SIM_NOT_INSERTED;
    }
}

// ============================================================================
// Name			: ucGSM_eReadSignalStrength_Exe
// Objective	: Read GSM Signal Quality
//					0 -113 dBm or less
//					1 -111 dBm
//					2...30 -109... -53 dBm
//					31 -51 dBm or greater
//					99 		Not known or not detectable
//
// Input  		: none
// Output 		: GSM Signal Level at GSMDHealth.cSignalLevel
//					-99 if Invalid
//
// Return		:	GSM_SIGNAL_LEVEL_OK		0
//	    	  		GSM_SIGNAL_NOT_IN_RANGE	1
//		  			NO_GSM_SIGNAL			2
//		  			JAMMING_IN_GSM			3
//		  			GSM_NO_RESPONSE			4
//		  			GSM_SIGNAL_ERROR		5
// Version		: -
// ============================================================================
uint8_t ucGSM_eReadSignalStrength_Exe(void)
{
    uint8_t ucTemp, ucFlag;
    char *cBuffAdd1, *cBuffAdd2;

    ucFlag = NO_GSM_SIGNAL;
    ucTemp = ucGSM_eProcessATCmdDualRespWithCmdNum_Exe(ATCOMMAND_CSQ, ATRESPONSE_CSQ, 10, ATRESPONSE_OK, 6, 300, 1000);

    if (ucTemp == BOTH_RESPONSES_MATCHING)
    {
        // Check for Strength Level
        cBuffAdd1 = strchr((char *)GSM_DataBuffer, ':');
        if (cBuffAdd1 != NULL)
        {
            cBuffAdd1++;
            cBuffAdd1++;
            cBuffAdd2 = strchr(cBuffAdd1, ',');
            if (cBuffAdd2 != NULL)
            {
                *cBuffAdd2 = '\0';
                GSMData.ucGSMSignalVal = atoi(cBuffAdd1);

                if (GSMData.ucGSMSignalVal == 99)
                {
                    ucFlag = NO_GSM_SIGNAL;
                }
                else if (GSMData.ucGSMSignalVal < GSM_SIGNAL_THRESHOLD_LOW)
                {
                    ucFlag = GSM_SIGNAL_LEVEL_LOW;
                }
                else
                {
                    ucFlag = GSM_SIGNAL_LEVEL_OK;
                }
                *cBuffAdd2 = ',';

                GSMStatus.ucGSMSignalStatus = ucFlag;
            }
        }
        else
        {
            GSMData.ucGSMSignalVal = 99;
            ucFlag = GSM_SIGNAL_ERROR;
        }
    }
    else if (ucTemp == NO_RESPONSE)
    {
        ucFlag = NO_RESPONSE;
    }
    else
    {
        ;
    }
    return ucFlag;
}

// ============================================================================
// Name		: ucGSM_eNetworkSetup_Exe
// Objective	: Make GSM Module registered on Network
// 		 	1. Check SIM Inserted				: AT+CPIN?\r
//		 	If SIM Present then Check followings :
//			2. Read Signal Quality				: AT+CSQ\r
//				If Signal Quality is Ok then Check followings :
// 			3. Get Network Registration Status	: AT+CREG?\r
// 			4. Get Network Registration Status	: AT+CGREG?\r
//			5. Read Network Registration 		: AT+COPS?\r
//			6. Read EPS Network Registration 	: AT+CEREG?\r
// Input  		: none
// Output 	:  GSMHealth.bGSMNetworkRegStatus and GSMHealth.bGPRSNetworkRegStatus
//				NETWORK_REGISTRATION_FAILED		0
//		  	 	NETWORK_REGISTRATION_SUCCESS	1
// Return		: NETWORK_REGISTRATION_FAILED	0
//		 	 NETWORK_REGISTRATION_SUCCESS	1
// ============================================================================
bool bGSM_eNetworkSetup_Exe(void)
{
    uint8_t ucRetry = 0;
    bool bFlag, bResp;

    bFlag = NETWORK_REGISTRATION_FAILED;

    do
    {
        bResp = bGSM_eReadNetworkRegStatus_Exe(GSM_NETWORK_REGISTRATION);
        if (bResp != NETWORK_REGISTRATION_SUCCESS)
        {
            HAL_Delay(1000);
        }
        else
        {
        }
        ucRetry++;
        vAPP_eFeedTheWDT_Exe();
    } while ((bResp != NETWORK_REGISTRATION_SUCCESS) && (ucRetry < 90));

    if (bResp == NETWORK_REGISTRATION_SUCCESS)
    {
        GSMStatus.bCREGStatus = NETWORK_REGISTRATION_SUCCESS;

        ucRetry = 0;
        do
        {
            bResp = bGSM_eReadNetworkRegStatus_Exe(GPRS_NETWORK_REGISTRATION);
            if (bResp != NETWORK_REGISTRATION_SUCCESS)
            {
                HAL_Delay(1000);
            }
            else
            {
            }
            ucRetry++;
#if WDT
            WDT_REFRESH();
#endif
        } while ((bResp != NETWORK_REGISTRATION_SUCCESS) && (ucRetry < 60));

        if (bResp == NETWORK_REGISTRATION_SUCCESS)
        {
            GSMStatus.bCGREGStatus = NETWORK_REGISTRATION_SUCCESS;

            bFlag = NETWORK_REGISTRATION_SUCCESS;
            bResp = bGSM_eReadNetworkRegStatus_Exe(EPS_NETWORK_REGISTRATION);

            if (bResp == NETWORK_REGISTRATION_SUCCESS)
            {
                bFlag = NETWORK_REGISTRATION_SUCCESS;
            }
            else
            {
                ;
            }
        }
        else
        {
            GSMStatus.bCGREGStatus = NETWORK_REGISTRATION_FAILED;
        }
    }
    else
    {
        GSMStatus.bCREGStatus = NETWORK_REGISTRATION_FAILED;
    }
    return bFlag;
}

// ============================================================================
// Name		: bGSM_eReadNetworkRegStatus_Exe
// Objective	: Read Network Registration Status by :
//				AT+CREG?\r
//			Response :
//				+CREG:<n>,<stat>
//				OK
//			NETWORK_SEARCHING			0
//		  	NETWORK_REGISTERED_HOME	1
//		  	NETWORK_NOT_REGISTERED		2
//		  	NETWORK_REGISTRATION_DENIED	3
//		  	NETWORK_UNKNOWN			4
//		  	NETWORK_REGISTERED_ROAMING	5
//
// Input  		: Network Type :
//			1 - GSM Network Registration
//			2 - GPRS Network Registration
//			3 - EPS Network Registration
// Output 	:  none
// Return		: NETWORK_REGISTRATION_FAILED	0
//		  	 NETWORK_REGISTRATION_SUCCESS	1
// Version	: -
// ============================================================================
bool bGSM_eReadNetworkRegStatus_Exe(uint8_t ucNetworkStatusType)
{
    uint8_t ucResp, ucVal;
    char *cBuffAdd;
    bool bFlag;

    bFlag = NETWORK_REGISTRATION_FAILED;
    if (ucNetworkStatusType == GSM_NETWORK_REGISTRATION)
    {
        ucResp = ucGSM_eProcessATCmdDualRespWithCmdNum_Exe(ATCOMMAND_GETCREG, ATRESPONSE_CREG, 10, ATRESPONSE_OK, 6, 300, 9000);
    }
    else if (ucNetworkStatusType == GPRS_NETWORK_REGISTRATION)
    {
        ucResp = ucGSM_eProcessATCmdDualRespWithCmdNum_Exe(ATCOMMAND_GETCGREG, ATRESPONSE_CGREG, 10, ATRESPONSE_OK, 6, 300, 9000);
    }
    else if (ucNetworkStatusType == EPS_NETWORK_REGISTRATION)
    {
        ucResp = ucGSM_eProcessATCmdDualRespWithCmdNum_Exe(ATCOMMAND_GETCEREG, ATRESPONSE_CEREG, 10, ATRESPONSE_OK, 6, 300, 9000);
    }
    else
    {
        ;
    }

    if (ucResp == BOTH_RESPONSES_MATCHING)
    {
        cBuffAdd = strchr((char *)GSM_DataBuffer, ',');
        if (cBuffAdd != NULL)
        {
            cBuffAdd++;
            ucVal = *cBuffAdd;

            if ((ucVal == '1') || (ucVal == '5'))
            {
                bFlag = NETWORK_REGISTRATION_SUCCESS;
            }
            else
            {
                ;
            }
        }
        else
        {
            ;
        }
    }
    else
    {
        ;
    }
    return bFlag;
}

// ============================================================================
// Name			: vGSM_eReadNetworkInfo_Exe
// Objective	: Read Network Info
// Input  		: None
// Output 		: GSMHealth.ucNetworkInfo updated
//					No Network		0
//		  			NETWORK_2G		2
//		  			NETWORK_3G		3
//		  			NETWORK_4G		4
//
// Return		: none
// Version		: -
// Author		: AK
// Date			: 26-08-2019
// ============================================================================
void vGSM_eReadNetworkInfo_Exe(void)
{
    uint8_t ucResp, ucVal;
    ucResp = ucGSM_eProcessATCmdDualRespWithCmdNum_Exe(ATCOMMAND_QNWINFO, ATRESPONSE_QNWINFO, 40, ATRESPONSE_OK, 6, 300, 5000);
    if (ucResp == BOTH_RESPONSES_MATCHING)
    {
        ucResp = ucGSM_eCompareStringInGSMBuff_Exe((char *)cGSM_QUECTEL_eATResponseTbl[ATRESPONSE_LTE]);
        if (ucResp == RESPONSE_MATCHING)
        {
            ucVal = LTE;
        }
        else
        {
            ucResp = ucGSM_eCompareStringInGSMBuff_Exe((char *)cGSM_QUECTEL_eATResponseTbl[ATRESPONSE_WCDMA]);
            if (ucResp == RESPONSE_MATCHING)
            {
                ucVal = WCDMA;
            }
            else
            {
                ucResp = ucGSM_eCompareStringInGSMBuff_Exe((char *)cGSM_QUECTEL_eATResponseTbl[ATRESPONSE_GSM]);
                if (ucResp == RESPONSE_MATCHING)
                {
                    ucVal = GSM;
                }
                else
                {
                    ucVal = NO_NETWORK;
                }
            }
        }
    }
    GSMStatus.ucNetworkInfo = ucVal;
}

// ============================================================================
// Name		: bGSM_eCheckAPNSetting_Exe
// Objective	: Get APN
//			 Command:
//			 AT+CGDCONT?<CR>
//			Response, if Set :
//				+CGDCONT: 1,"IP","www","",0,0,,,,<CR><LF>
//				+CGDCONT: 8,"IPV4V6","IMS","254.128.0.0.0.0.0.0.0.0.0.89.102.239.246.1",0,0,0,2,1,1<CR><LF>
//				OK<CR><LF>
//
//			Response, if Not Set :
//			+CGDCONT: 1,"IP","www.mnc011.mcc404.gprs","10.126.160.82",0,0,,,,<CR><LF>
//			+CGDCONT: 8,"IPV4V6","IMS","254.128.0.0.0.0.0.0.0.0.0.89.102.239.246.1",0,0,0,2,1,1<CR><LF>
//			OK<CR><LF>
//
// Input  		: none
// Output 	: none
// Return		: 0 - Not Set
//			  1 - APN Set
// Version	: -
// ============================================================================
bool bGSM_eCheckAPNSetting_Exe(void)
{
    uint8_t ucLen, ucResp;
    char *cBuff1;
    bool bStatus;
    char cBuff[50];

    // query
    sprintf(cBuff, "%s%d\r", cGSM_QUECTEL_eATCommandTbl[ATCOMMAND_QICSGP], PDP_CONTEXT_ID);
    ucResp = ucGSM_eProcessATCmdDualRespWithPtr_Exe(cBuff, ATRESPONSE_GETQICSGP, 21, ATRESPONSE_OK, 6, 300, 150000);
    if (ucResp == BOTH_RESPONSES_MATCHING)
    {
        sprintf(cBuff, "%s %d,\"%s\",", (char *)cGSM_QUECTEL_eATResponseTbl[ATRESPONSE_GETQICSGP], PDP_CONTEXT_ID, APPCONFIG.cAPN);
        cBuff1 = strchr(GSM_DataBuffer, '+');
        if (cBuff1 != 0)
        {
            ucLen = strlen(cBuff);
            bStatus = bUTL_eCompareData_Exe(cBuff, cBuff1, ucLen);
        }
        else
        {
            bStatus = FALSE;
        }
    }
    else
    {
        bStatus = FALSE;
    }
    GSMStatus.bAPNStatus = bStatus;
    return bStatus;
}

// ============================================================================
// Name		: vGSM_eSetAPN_Exe
// Objective	: Set APN
//			AT+CGDCONT=1,"IP","www"<CR>
//			OK<CR><LF>
// Input  		: none
// Output 	: none
// Return		: none
// Version	: -
// ============================================================================
void vGSM_eSetAPN_Exe(void)
{
    uint8_t ucStatus;
    char cBuff[50];

    sprintf(cBuff, "%s%d,1,\"%s\",\"\",\"\",0\r", (char *)cGSM_QUECTEL_eATCommandTbl[ATCOMMAND_QICSGP], PDP_CONTEXT_ID, APPCONFIG.cAPN);
    ucStatus = ucGSM_eProcessATCmdDualRespWithPtr_Exe(cBuff, ATRESPONSE_GETQICSGP, 21, ATRESPONSE_OK, 6, 300, 150000);
    if (ucStatus == BOTH_RESPONSES_MATCHING)
    {
        GSMStatus.bAPNStatus = TRUE;
    }
    else
    {
        GSMStatus.bAPNStatus = FALSE;
    }
}

// ============================================================================
// Name		: vGSM_eUpdateTimeZoneAndRTC_Exe
// Objective	: UPDATE TIME ZONE
// Input  		: none
// Output 	: none
// Return		: none
// Version	: -
// ============================================================================
void vGSM_eUpdateTimeZoneAndRTC_Exe(void)
{
    ucGSM_eProcessATCmdWithCmdNum_Exe(ATCOMMAND_CTZU, ATRESPONSE_OK, 300, 1000);
    ucGSM_eReadRTC_Exe();
}

// ============================================================================
// Name		: ucGSM_eReadRTC_Exe
// Objective	: Read RTC in YYYY-MM-DD HH:MM:SS Format, if Valid date
//				  Store "-99" if Invalid
//
// Input  		: none
// Output 	: RTC at GSMData.cRTC onwards
//				GSMData.cRTC[0] : 2019 	YYYY
//				GSMData.cRTC[5] : 11	MM
//				GSMData.cRTC[8] : 10	DD
//				GSMData.cRTC[11]: 12	HH
//				GSMData.cRTC[14]: 20	MM
//				GSMData.cRTC[17]: 10	SS

// Return		: FALSE : If not a Valid Date
//			  TRUE : If Valid Date
// Version	: -
// ============================================================================
uint8_t ucGSM_eReadRTC_Exe(void)
{
    uint8_t ucResp, ucStatus;
    char *pBuff1;

    ucStatus = FALSE;
    ucResp = ucGSM_eProcessATCmdDualRespWithCmdNum_Exe(ATCOMMAND_CCLK, ATRESPONSE_CCLK, 30, ATRESPONSE_OK, 6, 300, 3000);
    if (ucResp == BOTH_RESPONSES_MATCHING)
    {
        pBuff1 = strchr(GSM_DataBuffer, '"');
        if (pBuff1 != NULL)
        {
            pBuff1++;

            GSMData.cRTC[0] = '2';
            GSMData.cRTC[1] = '0';
            GSMData.cRTC[2] = *pBuff1;
            pBuff1++;
            GSMData.cRTC[3] = *pBuff1; // 20YY
            pBuff1++;
            pBuff1++;
            GSMData.cRTC[4] = '-';
            GSMData.cRTC[5] = *pBuff1;
            pBuff1++;
            GSMData.cRTC[6] = *pBuff1; // MM
            GSMData.cRTC[7] = 0;
            pBuff1++;
            pBuff1++;
            GSMData.cRTC[7] = '-';
            GSMData.cRTC[8] = *pBuff1;
            pBuff1++;
            GSMData.cRTC[9] = *pBuff1; // DD
            GSMData.cRTC[10] = 0;
            pBuff1++;
            pBuff1++;
            GSMData.cRTC[10] = ' ';
            GSMData.cRTC[11] = *pBuff1;
            pBuff1++;
            GSMData.cRTC[12] = *pBuff1; // HH
            GSMData.cRTC[13] = 0;
            pBuff1++;
            pBuff1++;
            GSMData.cRTC[13] = ':';
            GSMData.cRTC[14] = *pBuff1;
            pBuff1++;
            GSMData.cRTC[15] = *pBuff1; // MM
            GSMData.cRTC[16] = 0;
            pBuff1++;
            pBuff1++;
            GSMData.cRTC[16] = ':';
            GSMData.cRTC[17] = *pBuff1;
            pBuff1++;
            GSMData.cRTC[18] = *pBuff1;
            GSMData.cRTC[19] = 0;

            GSMStatus.bRTCStatus = TRUE;
            ucStatus = TRUE;
        }
        else
        {
        }
    }
    else
    {
    }

    if (FALSE == ucStatus)
    {
        GSMData.cRTC[0] = '-';
        GSMData.cRTC[1] = '9';
        GSMData.cRTC[2] = '9';
        GSMData.cRTC[3] = '\0';
    }
    else
    {
    }
    return ucStatus;
}

// ============================================================================
// Name		: vGSM_eEnablePDPContext_Exe
// Objective	: Enable PDP Context after checking by following Status :
//				1. SIM is Inserted
//				2. GSM Network Registration is OK
//				3. GPRS Network Registration is OK
//
//				- Select CSD or GPRS as the Bearer
//				 AT+QICSGP=1,"APN",UserName,Password
//		  		e.g. :AT+QICSGP=1,"CMNET"
//				Response
//	           		 <CR><LF>
//				OK<CR><LF>
//
//		 		- Activate GPRS/CSD Context
//				 AT+QIACT=1<CR>
//				Response
//	            		 <CR><LF>
//				OK<CR><LF>
//
// Input  		: none
// Output 	: GPRS Status at GSMHealth.ucGPRSStatus
//				GPRS_NOT_ACTIVE		0
//				GPRS_ACTIVE			1
// Return		: none
// Version	: -
// ============================================================================
void vGSM_eEnableGPRS_Exe(uint8_t ucPDPContextID)
{
    uint8_t ucResp;
    char cBuff[50];

    // Activate
    sprintf(cBuff, "%s%d\r", cGSM_QUECTEL_eATCommandTbl[ATCOMMAND_QIACT], ucPDPContextID);
    ucResp = ucGSM_eProcessATCmdWithCmdPtr_Exe(cBuff, ATRESPONSE_OK, 300, 150000);
    if (ucResp == RESPONSE_MATCHING)
    {
        GSMStatus.bGPRSStatus = GPRS_ACTIVE;
    }
    else
    {
        GSMStatus.bGPRSStatus = GPRS_NOT_ACTIVE;
        vGSM_eDisableGPRS_Exe(ucPDPContextID);
    }
}

// ============================================================================
// Name		: vGSM_eReadGPRSStatus_Exe
// Objective	: Read GPRS Status
//
// Input  		: none
// Output 	: none
// Return		: none
// Version	: -
// ============================================================================
void vGSM_eReadGPRSStatus_Exe(uint8_t ucPDPContextID)
{
    uint8_t ucResp;

    ucResp = ucGSM_eProcessATCmdDualRespWithCmdNum_Exe(ATCOMMAND_GETQIACT, ATRESPONSE_QIACT, 25, ATRESPONSE_OK, 6, 300, 150000);
    if (ucResp == BOTH_RESPONSES_MATCHING)
    {
        GSMStatus.bGPRSStatus = GPRS_ACTIVE;
    }
    else
    {
        GSMStatus.bGPRSStatus = GPRS_NOT_ACTIVE;
    }
}

// ============================================================================
// Name		: vGSM_eReStartGSM_Exe
// Objective	: ReStart GSM Module
// Input  		: none
// Output 	: none
// Return		: none
// Version	: -
// ============================================================================
void vGSM_eReStartGSM_Exe(void)
{
    uint8_t ucCntr;

    if ((GSMStatus.ucGSMReadyStatus == GSM_READY) || (GSMStatus.ucGSMReadyStatus == GSM_WAS_READY))
    {
        vGSM_eCloseSocket_Exe(LOCATION_SOCKET_ID);
        vGSM_eDisableGPRS_Exe(PDP_CONTEXT_ID);
        vGSM_eSoftOFFGSM_Exe();
        // vGSM_eTurnOFFGSM_Exe();
    }
    else
    {
    }

    // if (APPData.ucHWVer == 'D')
    // {
    //     vAPP_eFeedTheWDT_Exe();
    // 	HAL_Delay(2500);

    // 	vGSM_eTurnONGSM_Exe();
    // }
    // else if (APPData.ucHWVer == 'C')
    // {
    // }
    // else
    // {
    // }

    for (ucCntr = 0; ucCntr < 4; ucCntr++)
    {
        vAPP_eFeedTheWDT_Exe();
        HAL_Delay(2500);
    }

    vGSM_ePWRKeyStartGSM_Exe();
    vGSM_eStartGSM_Exe();
    if ((GSMStatus.ucGSMReadyStatus == GSM_READY) || (GSMStatus.ucGSMReadyStatus == GSM_WAS_READY))
    {
        vGSM_eInitaliseGSM_Exe();
        vGPS_eStartGPS_Exe();
        vGSM_eReadSIMInsertStatus_Exe();
        ucGSM_eReadSignalStrength_Exe();
        bGSM_eReadNetworkRegStatus_Exe(GSM_NETWORK_REGISTRATION);
        bGSM_eReadNetworkRegStatus_Exe(GPRS_NETWORK_REGISTRATION);
        vGSM_eDisableGPRS_Exe(PDP_CONTEXT_ID);
        vGSM_eEnableGPRS_Exe(PDP_CONTEXT_ID);
    }
    else
    {
        GSMStatus.bGSMRebootRequired = TRUE;
    }

    GSMData.ucNoResponseCntr = 0;
    GSMData.ucErrorCntr = 0;
    GSMData.ucNoSocketCntr = 0;
}

// ============================================================================
// Name		: vGSM_eTurnOFFGSM_Exe
// Objective	: Turn OFF GSM Module by Switching OFF the P Ch MOSFET
// Input  		: none
// Output 	: none
// Return		: none
// Version	: -
/* FLOW
Turns OFF the GSM module by cutting power.
*/
// void vGSM_eTurnOFFGSM_Exe(void)
//{
//     TIM2->CCR1 = 0;
// }

// ============================================================================
// Name		: vGSM_eTurnOFFGSM_Exe
// Objective	: - Turn OFF GSM Module by giving a Low pulse to PWRKEY Pin
//				    for >= 650mS.
// Input  		: none
// Output 	: none
// Return		: none
// ============================================================================
void vGSM_eSoftOFFGSM_Exe(void)
{
    vAPP_eFeedTheWDT_Exe();
    HAL_GPIO_WritePin(GSM_PWRKEY_GPIO_Port, GSM_PWRKEY_Pin, GPIO_PIN_SET);
    HAL_Delay(700);

    ucGSM_eProcessATCmdWithCmdNum_Exe(ATCOMMAND_QPOWD, ATRESPONSE_OK, 200, 1000);
    do
    {
        vAPP_eFeedTheWDT_Exe();
    } while ((GSM_RxCntr < 20) && (TIMERData.uiTimerCntr < 64000));

    if (GSM_RxCntr >= 12)
    {
        ucGSM_eCompareStringInGSMBuff_Exe((char *)cGSM_QUECTEL_eATResponseTbl[ATRESPONSE_POWERDOWN]);
    }
    else
    {
        ;
    }
}

void vGSM_eWakeGSMModule_Exe(void)
{
    uint8_t ucTemp;

    vAPP_eFeedTheWDT_Exe();
    // if (APPData.ucHWVer == 'D')
    // {
    // 	vGSM_eTurnONGSM_Exe();
    // }
    // else
    // {
    // }

    vGSM_eStartGSM_Exe();
    vGPS_eStartGPS_Exe();
    vGSM_eInitaliseGSM_Exe();

    for (ucTemp = 0; (ucTemp < 100) && (GPSData.bGPSStatus == GPS_NOT_FIXED); ucTemp++)
    {
        vAPP_eFeedTheWDT_Exe();
        vGPS_eReadGPSData_Exe();
        GPSData.ucGPSNotFixedCntr = 0;
        HAL_Delay(2000);
    }
    GSMStatus.bPwrDownStatus = GSM_ACTIVE;
}

// ============================================================================
// Name		: bGSM_eCheckAndMakeSocket_Exe
// Objective	: Check and Make Socket, if not active
// Input  		: Socket No.
// Output 	: Socket Status at GSMHealth.bSocketStatus[ucSocket]
// Return		: Socket Status
//				0 - Not Active
//				1 - Active
// Version	: -
// ============================================================================
bool bGSM_eCheckAndMakeSocket_Exe(void)
{
    bool bRespn;

    vAPP_eFeedTheWDT_Exe();
    bRespn = bGSM_eReadSocketStatus_Exe();
    if (bRespn == FALSE)
    {
        bRespn = bGSM_eMakeSocket_Exe();
        if (bRespn == FALSE)
        {
            vGSM_eReadGPRSStatus_Exe(PDP_CONTEXT_ID);
            if (GSMStatus.bGPRSStatus == FALSE)
            {
                vGSM_eDisableGPRS_Exe(PDP_CONTEXT_ID);
                vGSM_eEnableGPRS_Exe(PDP_CONTEXT_ID);
                if (GSMStatus.bGPRSStatus == FALSE)
                {
                    vGSM_eReStartGSM_Exe();
                }
            }
            else
            {
                ;
            }

            if (GSMStatus.bGPRSStatus == TRUE)
            {
                bRespn = bGSM_eReadSocketStatus_Exe();
                if (bRespn == FALSE)
                {
                    bRespn = bGSM_eMakeSocket_Exe();
                }
                else
                {
                }
            }
            else
            {
            }
        }
    }
    else
    {
    }
    return bRespn;
}

// ============================================================================
// Name		: bGSM_eSendDataOnSocket_Exe
// Objective	: Send Data on Soket
// Input  		: Socket Num, No. of Bytes
// Output 	: none
// Return		: 0 - Fail
//			  1 - Data Sent
// Version	: -
// ============================================================================
bool bGSM_eSendDataOnSocket_Exe(void)
{
    uint8_t ucResp;
    uint16_t uiLen;
    bool bStatus, bResp;
    char *cBuff1, cBuff2[30];

    uiLen = strlen(cAPP_eGlobalBuffer);
    sprintf(cBuff2, "%s0,%d\r", (char *)cGSM_QUECTEL_eATCommandTbl[ATCOMMAND_QISEND], uiLen);
    ucResp = ucGSM_eProcessATCmdWithCmdPtr_Exe(cBuff2, ATRESPONSE_SEND, 300, 120000);
    if (ucResp == RESPONSE_MATCHING)
    {
        GSM_RxCntr = 0;
        GSM_TimeOut = 0;

        vCOMM_eTxString_Exe(GSMComPortHandle, cAPP_eGlobalBuffer, 200);
        HAL_UART_Receive_IT(GSMComPortHandle, (uint8_t *)GSM_RxOneByte, 1);

        bStatus = bGSM_eWait4RespOrTimeOut_Exe(300, 120000);

        if (bStatus == TRUE)
        {
            GSMData.ucNoResponseCntr++;
            bResp = FALSE;
        }
        else
        {
            GSMData.ucNoResponseCntr = 0;
            cBuff1 = strstr(GSM_DataBuffer, (char *)cGSM_QUECTEL_eATResponseTbl[ATRESPONSE_SENDOK]);
            if (cBuff1 != 0)
            {
                bResp = TRUE;
            }
            else
            {
                bResp = FALSE;
            }
        }
    }
    else
    {
        bResp = FALSE;
    }
    return bResp;
}

// ============================================================================
// Name		: bGSM_eReadSocketStatus_Exe
// Objective	: Read Socket Status
// Input  		: none
// Output 	: none
// Return		: 0 - Socket not Active
//			  1 - Socket Active
// Version	: -
// ============================================================================
bool bGSM_eReadSocketStatus_Exe(void)
{
    uint8_t ucStatus, ucLen;
    bool bStatus;
    char *cBuff1, cBuffer[50];

    vAPP_eFeedTheWDT_Exe();
    ucStatus = ucGSM_eProcessATCmdDualRespWithCmdNum_Exe(ATCOMMAND_QISTATE, ATRESPONSE_QISTATE, 40, ATRESPONSE_OK, 6, 300, 9000);
    if (ucStatus == BOTH_RESPONSES_MATCHING)
    {
        cBuff1 = strchr(GSM_DataBuffer, '+');
        if (cBuff1 != 0)
        {
            sprintf(cBuffer, "%s 0,\"TCP\",\"%s\",%s", (char *)cGSM_QUECTEL_eATResponseTbl[ATRESPONSE_QISTATE], APPCONFIG.cDataServerIP, APPCONFIG.cDataServerPort);
            ucLen = strlen(cBuffer);
            bStatus = bUTL_eCompareData_Exe(cBuffer, cBuff1, ucLen);
        }
        else
        {
            bStatus = FALSE;
        }
    }
    else
    {
        bStatus = FALSE;
    }

    GSMStatus.bSocketStatus = bStatus;
    return bStatus;
}

// ============================================================================
// Name		: vGSM_eMakeSocket_Exe
// Objective	: Make Socket
//			  Command:
//			  	AT+QIOPEN=0,"TCP","59.144.168.11",4111<CR>
//
//			  Response, if Activated
//				OK<CR><LF>
//				+QIOPEN: 0,0<CR><LF>
//
//			  Response, if NotActivated
//				+QIOPEN: 0,2<CR><LF>
//				ERROR<CR><LF>
// Input  		: Socket No., Ip Address & Port No.
// Output 	: none
// Return		: 0 - Socket Not Active
//			  1 - Socket Active
// Version	: -
// ============================================================================
bool bGSM_eMakeSocket_Exe(void)
{
    uint8_t ucStatus;
    bool bStatus;
    char cBuff1[60], cBuff2[30], *cBuffer;

    sprintf(cBuff1, "%s0,\"TCP\",\"%s\",%s\r", (char *)cGSM_QUECTEL_eATCommandTbl[ATCOMMAND_QIOPEN], APPCONFIG.cDataServerIP, APPCONFIG.cDataServerPort); // APPCONFIG.cDataServerIP, APPCONFIG.cDataServerPort);
    sprintf(cBuff2, "%s 0,0", (char *)cGSM_QUECTEL_eATResponseTbl[ATRESPONSE_QIOPEN]);
    ucStatus = ucGSM_eProcessATCmdDualRespWithPtr_Exe(cBuff1, ATRESPONSE_OK, 6, ATRESPONSE_QIOPEN, 12, 300, 150000);
    if (ucStatus == BOTH_RESPONSES_MATCHING)
    {
        bStatus = SOCKET_ACTIVE;
    }
    else
    {
        bStatus = SOCKET_NOT_ACTIVE;
        GSMData.ucNoSocketCntr++;

        if (ucStatus == RESPONSE_NOT_MATCHING)
        {
            // Check for PDP Context Opening Failed
            cBuffer = strstr(GSM_DataBuffer, cGSM_QUECTEL_eATResponseTbl[ATRESPONSE_QIOPEN561]);
            if (cBuffer == 0)
            {
                //				GSMHealth.ucGSMRebootRequired = 1;
            }
            else
            {
            }
        }
        else
        {
        }
    }
    GSMStatus.bSocketStatus = bStatus;
    return bStatus;
}

// ============================================================================
// Name		: bGSM_eCheckPendingSMS_Exe
// Objective	: Read if any Pending SMS is present. Also SIM set as storage for SMS.
//				Command
//				 AT+CPMS="SM","SM","SM"\r
//				Response
//				 +CPMS: 0,20,0,20,0,20
//				 OK
// Input  		: none
// Output 	: Pending SMS at GSMData.ucPendingSMS
// Return		: Status
//	    			0 - No SMS Present
//	    			1 - SMS Pending
// Version	: -
// ============================================================================
bool bGSM_eCheckPendingSMSFromSIM_Exe(void)
{
    uint8_t ucStatus;
    bool bFlag;

    ucStatus = ucGSM_eProcessATCmdDualRespWithCmdNum_Exe(ATCOMMAND_CPMS_SM, ATRESPONSE_CPMS, 28, ATRESPONSE_OK, 6, 300, 1000);
    if (ucStatus == BOTH_RESPONSES_MATCHING)
    {
        bFlag = bGSM_eGetPendingSMS_Exe(SM_SMS_STORAGE);
    }
    else
    {
        bFlag = NO_SMS_PRESENT;
    }

    return bFlag;
}

bool bGSM_eCheckPendingSMSFromME_Exe(void)
{
    uint8_t ucStatus;
    bool bFlag;

    ucStatus = ucGSM_eProcessATCmdDualRespWithCmdNum_Exe(ATCOMMAND_CPMS_ME, ATRESPONSE_CPMS, 28, ATRESPONSE_OK, 6, 300, 1000);
    if (ucStatus == BOTH_RESPONSES_MATCHING)
    {
        bFlag = bGSM_eGetPendingSMS_Exe(ME_SMS_STORAGE);
    }
    else
    {
        bFlag = NO_SMS_PRESENT;
    }

    return bFlag;
}

#if 0
bool bGSM_eCheckPendingSMS_Exe(uint8_t ucSMSStorage)
{
	uint8_t ucStatus;
	bool bFlag;

	if (ucSMSStorage == ME_SMS_STORAGE)
	{
		ucStatus = ucGSM_eProcessATCmdDualRespWithCmdNum_Exe(ATCOMMAND_CPMS_ME, ATRESPONSE_CPMS, 28, ATRESPONSE_OK, 6, 300, 1000);
		if (ucStatus == BOTH_RESPONSES_MATCHING)
		{
			bFlag = bGSM_eGetPendingSMSInME_Exe();
		}
		else
		{
			bFlag = NO_SMS_PRESENT;
		}
	}
	else if (ucSMSStorage == SM_SMS_STORAGE)
	{
		ucStatus = ucGSM_eProcessATCmdDualRespWithCmdNum_Exe(ATCOMMAND_CPMS_SM, ATRESPONSE_CPMS, 28, ATRESPONSE_OK, 6, 300, 1000);
		if (ucStatus == BOTH_RESPONSES_MATCHING)
		{
			bFlag = bGSM_eGetPendingSMSInSM_Exe();
		}
		else
		{
			bFlag = NO_SMS_PRESENT;
		}
	}
    else
    {
		bFlag = NO_SMS_PRESENT;
	}

	return bFlag;
}
#endif

#if 0
// ============================================================================
// Name		: vGSM_eGetPendingSMSInME_Exe
// Objective	: Read Pending and Total SMS
//				 +CPMS: 0,20,0,20,0,20
//				 OK
// Input  		: data at GSM_Buffer
// Output 	: Pending SMS at GSMData.ucPendingSMS & vGSM_eGetPendingSMS_Exe
// Return		: 	0 - SMS Not Present
//				1 - SMS Present
// Author	: AK
// Date		: 24-12-2021
// ============================================================================
bool bGSM_eGetPendingSMSInME_Exe(void)
{
	char *pBuff1, *pBuff2;
	bool bFlag;

	pBuff1 = strchr (GSM_DataBuffer, ':');
	if (pBuff1)
	{
		pBuff1++;
		pBuff1++;
		pBuff2 = strchr (pBuff1, ',');
		if (pBuff2)
		{
			*pBuff2 = '\0';
			GSMData.ucPendingMESMS = atoi(pBuff1);
			*pBuff2 = ',';
			if (GSMData.ucPendingMESMS)
			{
				bFlag = SMS_PRESENT;
				pBuff2++;
				pBuff1 = strchr (pBuff2, ',');
				if (pBuff1)
				{
					*pBuff1 = '\0';
					GSMData.ucTotalMESMS = atoi(pBuff2);
					*pBuff1 = ',';
				}
				else
				{
					bFlag = NO_SMS_PRESENT;
				}
			}
			else
			{
				bFlag = NO_SMS_PRESENT;
			}
		}
		else
		{
			bFlag = NO_SMS_PRESENT;
		}
	}
	else
	{
		bFlag = NO_SMS_PRESENT;
	}
	return bFlag;
}
#endif

bool bGSM_eGetPendingSMS_Exe(uint8_t ucSMSStorage)
{
    char *pBuff1, *pBuff2;
    uint8_t ucTotalSMS, ucPendingSMS;
    bool bFlag;

    pBuff1 = strchr(GSM_DataBuffer, ':');
    if (pBuff1)
    {
        pBuff1++;
        pBuff1++;
        pBuff2 = strchr(pBuff1, ',');
        if (pBuff2)
        {
            *pBuff2 = '\0';
            ucPendingSMS = atoi(pBuff1);
            *pBuff2 = ',';

            if (ucPendingSMS)
            {
                bFlag = SMS_PRESENT;
                pBuff2++;
                pBuff1 = strchr(pBuff2, ',');
                if (pBuff1)
                {
                    *pBuff1 = '\0';
                    ucTotalSMS = atoi(pBuff2);
                    *pBuff1 = ',';
                }
                else
                {
                    bFlag = NO_SMS_PRESENT;
                }
            }
            else
            {
                bFlag = NO_SMS_PRESENT;
            }
        }
        else
        {
            bFlag = NO_SMS_PRESENT;
        }
    }
    else
    {
        bFlag = NO_SMS_PRESENT;
    }

    if (ucSMSStorage == SM_SMS_STORAGE)
    {
        GSMData.ucPendingSMSMS = ucPendingSMS;
        GSMData.ucTotalSMSMS = ucTotalSMS;
    }
    else if (ucSMSStorage == ME_SMS_STORAGE)
    {
        GSMData.ucPendingMESMS = ucPendingSMS;
        GSMData.ucTotalMESMS = ucTotalSMS;
    }
    else
    {
    }
    return bFlag;
}

// ============================================================================
// Name			: ucGSM_eReadSMS_Exe
// Objective	: Read SMS from SIM Memory
//					Command
//					 AT+CMGR=1\r
//					Response
//						AT+CMGR=1
//						+CMGR: "REC READ","+919811451380","","2018/08/13 15:10:00+22",145,4,0,0,"+919811009998",145,33
//						Hi my first SMS To quectel board3
//						OK
//						AT+CMGR=2
//						+CMGR: "REC READ","+919811451380","","2018/08/13 15:10:00+22",145,4,0,0,"+919811009998",145,33
//						Hi my first SMS To quectel board3
//						OK
//						AT+CMGR=3
//						+CMGR: "REC READ","+919811451380","","2018/08/13 15:10:00+22",145,4,0,0,"+919811009998",145,33
//						Hi my first SMS To quectel board3
//						OK
//						AT+CMGR=4
//						+CMGR: "REC READ","+919811451380","","2018/08/13 15:12:11+22",145,4,0,0,"+919811009998",145,34
//						Hi my first SMS To quectel board4
//						OK
//					if No SMS present
//						AT+CMGR=5
//						OK
// Input  		: SMS Index Number
// Output 		: SMS Data at GSM_DataBuffer & OTAData.cOTABuffer
//				  SMS Sender Mobile No. at OTAData.cSenderMobile
// Return		: Status
//	    			0 - NO_SMS_PRESENT
//	    			1 - SMS_PRESENT
//				2 - SMS from Invalid Sender
//				3 - SPAM SMS
//	    			4 - INVALID_SMS_PRESENT
// Version		: -
// ============================================================================
uint8_t ucGSM_eReadSMS_Exe(uint8_t ucSMSIndex)
{
    uint8_t ucStatus, ucLen, ucType;
    char cBuff[100];
    char *pBuff1, *pBuff2;

    sprintf(cBuff, "%s%d\r", cGSM_QUECTEL_eATCommandTbl[ATCOMMAND_CMGR], ucSMSIndex);
    ucStatus = ucGSM_eProcessATCmdDualRespWithPtr_Exe(cBuff, ATRESPONSE_CMGR, 90, ATRESPONSE_OK, 6, 300, 1000);
    if ((ucStatus == BOTH_RESPONSES_MATCHING) || (ucStatus == RESPONSE1_MATCHING))
    {
        ucType = FALSE;
        ucStatus = INVALID_SMS_PRESENT;

        pBuff1 = strchr(GSM_DataBuffer, ',');
        if (pBuff1)
        {
            pBuff1++;
            pBuff1++;
            pBuff2 = strchr(pBuff1, '"');
            if (pBuff2)
            {
                *pBuff2 = '\0';
                ucLen = strlen(pBuff1);

                if ((ucLen > 5) && (ucLen < 24))
                {
                    strcpy(GSMData.cSenderMobileNum, pBuff1);
                    *pBuff2 = '"';

                    pBuff1 = strchr(pBuff2, 0x0A);
                    if (pBuff1)
                    {
                        pBuff1++;
                        pBuff2 = strchr(pBuff1, 0x0D);
                        if (pBuff2)
                        {
                            *pBuff2 = '\0';
                            if ((strlen(pBuff1)) < (sizeof(GSMData.cSMSData)))
                            {
                                strcpy(GSMData.cSMSData, pBuff1);
                                *pBuff2 = '"';

                                ucStatus = SMS_PRESENT;
                                ucType = TRUE;
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
                    *pBuff2 = '"';
                }
            }
            else
            {
            }
        }
        else
        {
        }

        if (ucType == FALSE)
        {
            ucStatus = SPAM_SMS;
        }
        else
        {
        }
    }
    else if (ucStatus == RESPONSE2_MATCHING)
    {
        ucStatus = INVALID_SMS_PRESENT;
    }
    else
    {
        ucStatus = INVALID_SMS_PRESENT;
    }
    return ucStatus;
}

// ============================================================================
// Name			: bGSM_eDeleteSMS_Exe
// Objective	: Delete SMS from SIM Memory
//					Command
//					 AT+CMGD=1\r
//					Response
//					 OK
// Input  		: SMS Index Number
// Output 		: Pending SMS reduced by 1 if not 0
// Return		: Status
//	    			0 - SUCCESS
//	    			1 - FAIL
// Version		: -
// ============================================================================
bool bGSM_eDeleteSMS_Exe(uint8_t ucSMSIndex)
{
    uint8_t ucStatus;
    bool bResp;
    char cBuff[50];

    sprintf(cBuff, "%s%d\r", cGSM_QUECTEL_eATCommandTbl[ATCOMMAND_CMGD], ucSMSIndex);
    ucStatus = ucGSM_eProcessATCmdWithCmdPtr_Exe(cBuff, ATRESPONSE_OK, 300, 3000);
    if (ucStatus == RESPONSE_MATCHING)
    {
        bResp = TRUE;
    }
    else
    {
        bResp = FALSE;
    }
    return bResp;
}

// ============================================================================
// Name		: bGSM_eSendSMS_Exe
// Objective	: Send SMS, use AT+CMGS for upto 160 Char.
//					AT+QCMGS for more than 160 Char
//					Command
//					 AT+CMGS="+919811451380"<CR><LF>
//					Response
//					 >
//					 Write Data with Control-Z
//					 +CMGS: 245
//					 OK
// Input  		: 1. Receiver Phone No.
//			  	Data Stored at cAPP_eGlobalBuffer without
//			  	- Write Control Z in this function
// Output 	: none
// Return		: Status
//	    			0 - FAIL
//	    			1 - SUCCESS
// Version	: -
// ============================================================================
bool bGSM_eSendSMS_Exe(char *cReceiverMobile, char *cMessage)
{
    uint8_t ucStatus;
    char cBuff[180];
    bool bStatus;

    bStatus = FALSE;

    sprintf(cBuff, "%s\"%s\"\r", cGSM_QUECTEL_eATCommandTbl[ATCOMMAND_CMGS], cReceiverMobile);
    ucStatus = ucGSM_eProcessATCmdWithCmdPtr_Exe(cBuff, ATRESPONSE_SEND, 300, 1000);
    if (ucStatus == RESPONSE_MATCHING)
    {
        ucStatus = strlen(cMessage);
        *(cMessage + ucStatus) = CONTROL_Z;
        *(cMessage + ucStatus + 1) = '\0';

        ucStatus = ucGSM_eProcessATCmdDualRespWithPtr_Exe(cMessage, ATRESPONSE_CMGS, 8, ATRESPONSE_SENDOK, 6, 300, 150000);
        if (ucStatus == BOTH_RESPONSES_MATCHING)
        {
            bStatus = TRUE;
        }
        else
        {
            bStatus = FALSE;
        }
    }
    else
    {
        bStatus = FALSE;
    }
    return bStatus;
}

// void vGSM_StatusSymbol_To_Cluster(void)
// {
//     uint8_t ucTemp;

//     ucCAN_eTxData[0] = 0xF5;

//     uint8_t b1 = ((IoT_ClusterMsg.IoT_BLE_symbol         ? 1U : 0U) << 0) |
//                  ((IoT_ClusterMsg.IoT_DataSession_symbol ? 1U : 0U) << 1) |
//                  ((IoT_ClusterMsg.IoT_GPS_symbol         ? 1U : 0U) << 2) |
//                  ((IoT_ClusterMsg.IoT_Speaker_symbol     ? 1U : 0U) << 3);

//     ucCAN_eTxData[1] = b1;

//     uint8_t b2 = (GSMData.ucGSMSignalVal > GSM_SIGNAL_THRESHOLD_HIGH) ? GSM_SIGNAL_THRESHOLD_HIGH : 0;

//     ucCAN_eTxData[2] = b2;
//     ucCAN_eTxData[3] = 0x00;
//     ucCAN_eTxData[4] = 0x00;
//     ucCAN_eTxData[5] = 0x00;
//     ucCAN_eTxData[6] = 0x00;
//     ucCAN_eTxData[7] = 0xF6;

//     // for (ucTemp = 0; ucTemp < 3; ucTemp++)
//     // {
//         // Make Motor Controller Disable
//         vCAN_eSendCANCommands_Exe(CANID_IoT_TO_Cluster);
//         // HAL_Delay(100);
//     // }
// }

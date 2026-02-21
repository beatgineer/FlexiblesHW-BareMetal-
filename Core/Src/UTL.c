// #############################################################################
//           UTL : UTILITY MODULE
//  FILE   : UTL.C
//  DESC   : Manages the Utility Functions.
//
//  LIST   :
//
//  HIST   :
//  ----------------------------------------------------------------------------
//  |Version | Date  | Author |                 Description                    |
//  ----------------------------------------------------------------------------
//  | 01.00  |170316 | Arvind | Creation of Header                             |
//  ----------------------------------------------------------------------------
// #############################################################################

//*****************************************************************************
//   INCLUDE
//*****************************************************************************
#include "UTL.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

//*****************************************************************************
// DEFINE and MACRO
//*****************************************************************************

//*****************************************************************************
// EXTERNAL VARIABLES
//*****************************************************************************

//*****************************************************************************
// INTERNAL VARIABLES
//*****************************************************************************

const uint8_t ASCIITable[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
const uint8_t HexTable[6] = {0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};

#if 0
// ****************************************************************************
// NAME   	: vUTL_eDelayInms_Exe
// Objective	: Delay of given mili seconds.
// INPUT  	: Delay Counts in mS
// Output	: none
// Return	: none
// Author	: AK
// Date		: 17 Mar 2016
// ****************************************************************************
void vUTL_eDelayInms_Exe (uint16_t uiDelayCount)
{
#if 1
	uint16_t uiTemp;

	for (uiTemp = 0; uiTemp < uiDelayCount; uiTemp++)
	{
		vUTL_eDelay1ms_Exe ();
	        //IWDG_ReloadCounter();		// Reload IWDG counter
	}
#endif

#if 0
	APPTimer.uiDelayCount = uiDelayCount / SCH_TMR;
	while(APPTimer.uiDelayCount != 0);
#endif
}

// ****************************************************************************
// NAME   	: vUTL_eDelay1ms_Exe
// Objective	: Delay of One mili seconds at Clock Freq 72 MHz.
// INPUT  	: none
// Output	: none
// Return	: none
// Author	: AK
// Date		: 17 Mar 2016
// ****************************************************************************
void vUTL_eDelay1ms_Exe (void)
{
	uint16_t uiTemp;

	for (uiTemp = 0; uiTemp < 3000; uiTemp++)
	{
		asm ("nop");
		asm ("nop");
		asm ("nop");
	}
}
#endif

// ****************************************************************************
// NAME   	: pucUTL_eSearchCharInBytes_Exe
// Objective	: Find the Pointer address for a Character in given bytes
// INPUT  	: ucSearchChar	- Character to be searched
//		  		ucSearchChar	- Char to be searched
//		 		ucStartAdd	- Start Address of Buffer
//				ucBytes		- No. of Bytes
// Output	: none
// Return		: NULL POINTER - If not Found
//	    	  	  Pointer Add  - If Found
// ****************************************************************************
char *pucUTL_eSearchCharInBytes_Exe(char ucSearchChar, char *ucStartAdd, uint8_t ucBytes)
{
	uint8_t ucTemp, ucVal;
	char *ucResp;

	ucTemp = 0;
	ucResp = 0;
	do
	{
		ucVal = *(ucStartAdd + ucTemp);
		if (ucVal != ucSearchChar)
		{
			ucTemp++;
		}
		else
		{
			ucResp = ucStartAdd + ucTemp;
		}
	} while ((ucResp == 0) && (ucTemp < ucBytes));

	return ucResp;
}

// ****************************************************************************
// NAME   	: ucUTL_eSearchNumOfCharInBytes_Exe
// Objective	: Find the Character appeared in the given bytes
// INPUT  	: ucSearchChar	- Character to be searched
//		  		ucSearchChar	- Number of occurrence (Freq)
//		 		ucStartAdd	- Start Address of Buffer
//				ucBytes		- No. of Bytes
// Output	: none
// Return		: NULL POINTER - If not Found
//	    	  	  Pointer Add  - If Found
// ****************************************************************************
uint8_t ucUTL_eSearchNumOfCharInBytes_Exe(char ucSearchChar, char *ucStartAdd, uint8_t ucBytes)
{
	uint8_t ucTemp, ucVal, ucCntr;

	ucCntr = 0;

	for (ucTemp = 0; ucTemp < ucBytes; ucTemp++)
	{
		ucVal = *(ucStartAdd + ucTemp);
		if (ucVal == ucSearchChar)
		{
			ucCntr++;
		}
		else
		{
			;
		}
	}

	return ucCntr;
}

// ****************************************************************************
// NAME   	: pucUTL_eSearchChar_Exe
// Objective	: Find the Pointer address for a Character repeated in a Null terminated
//		  	String stored in a Buffer for given number of times
// INPUT  	: ucSearchChar	- Character to be searched
//		  	ucPos		- Number of occurrence (Freq)
//		  	ucStartAdd	- Start Address of Buffer
// Output		: none
// Return		: NULL POINTER - If not Found
//	    	 	Pointer Add  - If Found
// ****************************************************************************
char *pucUTL_eSearchChar_Exe(char ucSearchChar, uint8_t ucPos, char *ucStartAdd)
{
	uint8_t ucTemp, ucTemp1;
	char *ucResp;

	ucTemp = 0;
	ucTemp1 = 0;
	do
	{
		ucResp = strchr(ucStartAdd + ucTemp1, ucSearchChar);
		if (ucResp != NULL)
		{
			ucTemp++;
			ucTemp1 = ucResp - ucStartAdd + 1;
		}
	} while ((ucResp != NULL) && (ucTemp != ucPos));

	return (ucResp);
}

// ============================================================================
// Name			: vUTL_eClearBuffer_Exe
// Objective	: Fill Buffer with 0
// Input  		: Pointer to the Buffer
//				  No. of Bytes
// Output 		: none
// Return		: none
// Version		: -
// ============================================================================
void vUTL_eClearBuffer_Exe(uint8_t *cBuffer, uint16_t uiBytes)
{
	/*	uint16_t uiTemp;

		for (uiTemp = 0; uiTemp < uiBytes; uiTemp++)
		{
			cBuffer[uiTemp] = 0;
		}*/
}

// ============================================================================
// Name		: 	ucUTL_eCalculateCheckSum_Exe
// Objective	: 	Calculate Checksum by XORing
//
// Input  		: 	Pointer of Buffer Address in pBuffer
//				Total Bytes in uiBytes
// Output 	: 	none
// Return		: 	8 Bit CheckSum
// Version	: 	-
// ============================================================================
uint8_t ucUTL_eCalculateCheckSum_Exe(uint8_t *pBuffer, uint16_t uiBytes)
{
	uint16_t uiCntr;
	uint8_t ucCheckSum;

	ucCheckSum = *pBuffer;
	for (uiCntr = 1; uiCntr < uiBytes; uiCntr++)
	{
		ucCheckSum = ucCheckSum ^ (*(pBuffer + uiCntr));
	}

	return ucCheckSum;
}

// ============================================================================
// Name		: 	ucUTL_eCalculateCheckSumBy2sComplement_Exe
// Objective	: 	Calculate Checksum by 2's Complement
//
// Input  		: 	Pointer of Buffer Address in pBuffer
//				Total Bytes in uiBytes
// Output 	: 	none
// Return		: 	8 Bit CheckSum
// Version	: 	-
// ============================================================================
uint8_t ucUTL_eCalculateCheckSumBy2sComplement_Exe(uint8_t *pBuffer, uint16_t uiBytes)
{
	uint16_t uiCntr, uiVal;
	uint8_t ucCheckSum;

	uiVal = *pBuffer;
	for (uiCntr = 1; uiCntr < uiBytes; uiCntr++)
	{
		uiVal += (*(pBuffer + uiCntr));
	}

	ucCheckSum = (uint8_t)((0xFFFF - uiVal) + 1);
	return ucCheckSum;
}

// ============================================================================
// NAME   : vUTL_eCompareData_Exe()
// ROLE   : Compare Two Set of Data.
//
// INPUT  : Pointer of Data-1
//        : Pointer of Data-2
//	    	Length of Data
// OUTPUT : none
// RETURN : 0 - No Matching
//		    1 - Matching
// GLOBAL VARIABLE USED :
//	    	none
// FUNCTION USED :
//	    	none
// HIST   :
// ============================================================================
bool bUTL_eCompareData_Exe(char *cData1Adr, char *cData2Adr, uint16_t uiLen)
{
	uint8_t ucVal1, ucVal2;
	uint16_t uiTemp;
	bool bStatus;

	bStatus = TRUE;

	for (uiTemp = 0; (uiTemp < uiLen) && (bStatus == TRUE); uiTemp++)
	{
		ucVal1 = cData1Adr[uiTemp];
		ucVal2 = cData2Adr[uiTemp];

		if (ucVal1 != ucVal2)
		{
			bStatus = FALSE;
		}
		else
		{
			bStatus = TRUE;
		}
	}

	return bStatus;
}

// ****************************************************************************
// NAME   : ucUTL_eASCIIChar2HexChar_Exe
// ROLE   : Convert ASCII Char to Hex char.
//
// INPUT  : ASCII Char
// OUTPUT : None
//
// GLOBAL VARIABLE USED : None
//
// FUNCTION USED : None
//
// HIST   :
// ****************************************************************************
uint8_t ucUTL_eASCIIChar2HexChar_Exe(uint8_t ucVal)
{
	uint8_t ucTemp;

	if ((ucVal - '0') > 0x10)
	{
		if ((ucVal - 'A') > 0x10)
		{
			ucTemp = HexTable[(ucVal - 'a')];
		}
		else
		{
			ucTemp = HexTable[(ucVal - 'A')];
		}
	}
	else
	{
		ucTemp = ucVal - '0';
	}

	return (ucTemp);
}

// ****************************************************************************
// NAME		: 	ucUTL_eHexChar2ASCIIChar_Exe
// ROLE		: 	Convert Hex Char into ASCII Char
//
// INPUT		: 	Hex char
// OUTPUT	: 	None
//
// HIST   :
// ****************************************************************************
uint8_t ucUTL_eHexChar2ASCIIChar_Exe(uint8_t ucHexChar)
{
	return (ASCIITable[(ucHexChar & 0xDF)]);
}
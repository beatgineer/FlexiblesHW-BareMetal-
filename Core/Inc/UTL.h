// ============================================================================
// UTL.h
// Author : AK
// Date   : 10-04-2014
// ============================================================================
#ifndef __UTL_H
#define __UTL_H

// Includes ------------------------------------------------------------------
#include "stm32g0xx.h"
#include <stdbool.h>

//*****************************************************************************
//   DEFINITION
//*****************************************************************************
#define OUTPUT 1
#define INPUT 0

#define HIGH 1
#define LOW 0

#define OK 1
#define FAIL 0

#define TRUE 1
#define FALSE 0

#define READY 0
#define FULL 1
#define ERASED 2

//*****************************************************************************
//   VARIABLES
//*****************************************************************************

//*****************************************************************************
//   PROTO
//*****************************************************************************
char *pucUTL_eSearchChar_Exe(char ucSearchChar, uint8_t ucPos, char *ucStartAdd);
void vUTL_eClearBuffer_Exe(uint8_t *cBuffer, uint16_t uiBytes);
uint8_t ucUTL_eCalculateCheckSum_Exe(uint8_t *pBuffer, uint16_t uiBytes);
uint8_t ucUTL_eCalculateCheckSumBy2sComplement_Exe(uint8_t *pBuffer, uint16_t uiBytes);
uint8_t ucUTL_eHexChar2ASCIIChar_Exe(uint8_t ucHexChar);
bool bUTL_eCompareData_Exe(char *cData1Adr, char *cData2Adr, uint16_t uiLen);
char *pucUTL_eSearchCharInBytes_Exe(char ucSearchChar, char *ucStartAdd, uint8_t ucBytes);
uint8_t ucUTL_eSearchNumOfCharInBytes_Exe(char ucSearchChar, char *ucStartAdd, uint8_t ucBytes);
uint8_t ucUTL_eASCIIChar2HexChar_Exe(uint8_t ucVal);

#endif

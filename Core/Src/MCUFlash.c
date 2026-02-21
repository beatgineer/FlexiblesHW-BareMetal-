// // ============================================================================
// // Module	: MCUFlash STM32G0
// // Version	: 00
// // Author	: AK
// // Date		: 29 May 2019
// // ============================================================================
// #include "MCUFLASH.h"
// #include "APP.h"

// #include <stdio.h>
// #include <string.h>
// #include <stdlib.h>
// #include <stdbool.h>

// // Private Proto
// static uint16_t uiMCUFLASH_iFindLastBlankMemoryInCurrentAPPStorage_Exe(void);
// static uint64_t ullMCUFLASH_iRead64DataFromMCUFlash_Exe(uint32_t ulAddr);
// static uint8_t ucMCUFLASH_iEraseOnePageOfMCUFlash_Exe(uint32_t ulAddr);
// static uint8_t ucMCUFLASH_iWrite64bitDataInMCUFlash_Exe(uint32_t ulDestAdr, uint64_t ullData);
// static uint32_t ulMCUFLASH_iGetPage_Exe(uint32_t ulAddr);
// static uint32_t ulMCUFLASH_iGetBank_Exe(uint32_t ulAddr);

// extern UART_HandleTypeDef huart1; // Ensure this is defined

// // ============================================================================
// // Name			: uiCONFIG_eReadCONFIGStatusFromMCUFlash_Exe
// // Objective	: Read CONFIG Status from MCU Flash
// // Input  		: None
// // Output 		: None
// // Return		: 	Last Valid CONFIG Status
// // 					01 : Valid APP is APP1
// //					02 : Valid APP is APP2
// //					0xFFFFFFFFFFFFFFFF - If Error
// // Version		: -
// // Author		: AK
// // Date			: 17 Sep 2019
// // ============================================================================
// uint64_t ullMCUFLASH_eReadCurrentActiveAPP_Exe(void)
// {
// 	uint16_t uiLastBlankAddr;
// 	uint32_t ulAddr;
// 	uint64_t ullVal = 0;
// 	uint8_t ucResp;

// 	uiLastBlankAddr	= uiMCUFLASH_iFindLastBlankMemoryInCurrentAPPStorage_Exe();
// 	if (uiLastBlankAddr != 0xFFFFU)
// 	{
// 		if (uiLastBlankAddr != 0)
// 		{
// 			uiLastBlankAddr -= 8;
// 		}
// 		else
// 		{
// 			// Load Default Value
// 			ullVal = VALID_APP1;
// 			ucResp = ucMCUFLASH_iWrite64bitDataInMCUFlash_Exe(MCUFLASH_APP_STORAGE_START_ADDR, ullVal);
// 			if(ucResp == MCU_FLASH_WRITING_OK)
// 			{
// 				uiLastBlankAddr = 0;
// 			}
// 			else
// 			{
// 				ullVal = 0xFFFFFFFFFFFFFFFFU;
// 				return ullVal;
// 			}
// 		}

// 		ulAddr = MCUFLASH_APP_STORAGE_START_ADDR + uiLastBlankAddr;

// 		ullVal = ullMCUFLASH_iRead64DataFromMCUFlash_Exe(ulAddr);
// 	}
// 	else
// 	{
// 		ullVal = 0xFFFFFFFFFFFFFFFFU;
// 	}

// 	return ullVal;
// }


// // ============================================================================
// // Name			: ucMCUFLASH_eWriteCONFIGStatusInMCUFlash_Exe
// // Objective	: Write CONFIG Status in MCU Flash
// // Input  		: 16 bit Data
// // 					Lower 8 bit : 01 : Jump to Execute APP
// //								: 02 : Firmware Update
// // 					Upper 8 Bit : 01 : Jump to Execute APP1
// //								: 02 : Jump to Execute APP2
// // Output 		: MCU Flash updated
// // Return		: Data Written Status
// //				0 - Fail
// //				1 - Success
// // Version		: -
// // Author		: AK
// // Date			: 17 Sep 2019
// // ============================================================================
// uint8_t ucMCUFLASH_eWriteCONFIGStatusInMCUFlash_Exe(uint64_t ullStatus)
// {
// 	uint16_t uiLastBlankAddr = 0;
// 	uint8_t ucResp = 0, ucStatus = 0;

// 	// Find Last blank Memory Address
// 	uiLastBlankAddr = uiMCUFLASH_iFindLastBlankMemoryInCurrentAPPStorage_Exe();

// 	if (uiLastBlankAddr > (MCUCONFIG_MEM_END_ADDR - MCUCONFIG_MEM_START_ADDR - 4))
// 	{
// 		ucStatus= ucMCUFLASH_iEraseOnePageOfMCUFlash_Exe(MCUFLASH_APP_STORAGE_START_ADDR);
// 		if(ucStatus == MCU_FLASH_ERASE_OK)
// 		{
// 			uiLastBlankAddr = 0;
// 		}
// 	}

// 	ucResp = ucMCUFLASH_iWrite64bitDataInMCUFlash_Exe((MCUCONFIG_MEM_START_ADDR + uiLastBlankAddr), ullStatus);

// 	return ucResp;
// }

// // ============================================================================
// // Name			: uiCONFIG_iFindLastBlankMemoryAddrInOneSectorOf2kb_Exe
// // Objective	: Find Last Blank Location in a Sector 0f 2kb MCU to Store the Config Data
// // Input  		: none
// // Output 		: none
// // Return		: 16 bit Index Number pointing to Blank Location
// //					0xFFFF - Incase of any error
// // Version		: -
// // Author		: AK
// // Date			: 19 Sep 2023
// // ============================================================================
// static uint16_t uiMCUFLASH_iFindLastBlankMemoryInCurrentAPPStorage_Exe(void)
// {
// 	uint16_t uiCntr;
// 	bool bStatus;
// 	uint32_t ulAddr;
// 	uint64_t ullData;
// 	uint8_t ucStatus;

// 	bStatus = false;
// 	for (uiCntr = 0; (uiCntr < 0x800U) && (bStatus == false); uiCntr += 8)
// 	{
// 		ulAddr = MCUFLASH_APP_STORAGE_START_ADDR + uiCntr;

// 		ullData = ullMCUFLASH_iRead64DataFromMCUFlash_Exe(ulAddr);

// 		if (ullData == 0xFFFFFFFFFFFFFFFFU)
// 		{
// 			bStatus = true;
// 		}
// 		else
// 		{
// 		}
// 	}

// 	if(bStatus == false)
// 	{
// 		// Read Last Data, Erase the Page and Store in beginning
// 		ullData	= ullMCUFLASH_iRead64DataFromMCUFlash_Exe(MCUFLASH_APP_STORAGE_END_ADDR - 8 + 1);
// 		ucStatus= ucMCUFLASH_iEraseOnePageOfMCUFlash_Exe(MCUFLASH_APP_STORAGE_START_ADDR);
// 		if(ucStatus == MCU_FLASH_ERASE_OK)
// 		{
// 			ucStatus = ucMCUFLASH_iWrite64bitDataInMCUFlash_Exe(MCUFLASH_APP_STORAGE_START_ADDR, ullData);
// 			if(ucStatus == MCU_FLASH_WRITING_OK)
// 			{
// 				uiCntr = 8;
// 			}
// 			else
// 			{
// 				uiCntr = 0xFFFFU;
// 			}
// 		}
// 		else
// 		{
// 			uiCntr = 0xFFFFU;
// 		}
// 	}
// 	else
// 	{
// 		uiCntr -= 8U;
// 	}

// 	return uiCntr;
// }

// // ============================================================================
// // Name			: ucMCUFLASH_iWrite64bitDataInMCUFlash_Exe
// // Objective	: Write 64 bit Value into MCU Flash without Erasing Flash
// // Input  		: 1. Source Address of Buffer
// //		  		  2. Byte to Write
// // Output 		: Data in MCU Flash
// // Return		: Status of Flash Writing
// // 					1 = MCU_FLASH_UNLOCK_ERROR,
// //					2 = MCU_FLASH_LOCK_ERROR,
// //					3 = MCU_FLASH_WRITING_ERROR,
// //					4 = MCU_FLASH_WRITING_OK,
// // Author		: AK
// // Date			: 19 Sep 2023
// // ============================================================================
// // static uint8_t ucMCUFLASH_iWrite64bitDataInMCUFlash_Exe(uint32_t ulDestAdr, uint64_t ullData)
// // {
// // 	uint8_t ucResp;

// // 	ucResp = HAL_FLASH_Unlock();
// // 	if(ucResp == HAL_OK)
// // 	{
// // 		// Program Flash
// // 		ucResp = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, ulDestAdr, ullData);
// // 		if(ucResp == HAL_OK)
// // 		{
// // 			ucResp = HAL_FLASH_Lock();

// // 			if(ucResp == HAL_OK)
// // 			{
// // 				ucResp = MCU_FLASH_WRITING_OK;
// // 			}
// // 			else
// // 			{
// // 				ucResp = MCU_FLASH_LOCK_ERROR;
// // 			}
// // 		}
// // 		else
// // 		{
// // 			ucResp = MCU_FLASH_WRITING_ERROR;
// // 		}
// // 	}
// // 	else
// // 	{
// // 		ucResp = MCU_FLASH_UNLOCK_ERROR;
// // 	}
// // 	return ucResp;
// // }


// static uint8_t ucMCUFLASH_iWrite64bitDataInMCUFlash_Exe(uint32_t ulDestAdr, uint64_t ullData)
// {
// 	uint8_t ucResp;
// 	char debugStr[128];

// 	// Unlock Flash
// 	ucResp = HAL_FLASH_Unlock();
// 	if (ucResp != HAL_OK)
// 	{
// 		sprintf(debugStr, "[ERROR] FLASH Unlock failed! HAL_Status=%d\r\n", ucResp);
// 		HAL_UART_Transmit(&huart1, (uint8_t *)debugStr, strlen(debugStr), 100);
// 		return MCU_FLASH_UNLOCK_ERROR;
// 	}

// 	// Program 64-bit (Double Word)
// 	ucResp = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, ulDestAdr, ullData);
// 	if (ucResp == HAL_OK)
// 	{
// 		ucResp = HAL_FLASH_Lock();
// 		if (ucResp == HAL_OK)
// 		{
// 			sprintf(debugStr, "[DEBUG] Flash write OK at 0x%08lX\r\n", ulDestAdr);
// 			HAL_UART_Transmit(&huart1, (uint8_t *)debugStr, strlen(debugStr), 100);
// 			return MCU_FLASH_WRITING_OK;
// 		}
// 		else
// 		{
// 			sprintf(debugStr, "[ERROR] FLASH Lock failed! HAL_Status=%d\r\n", ucResp);
// 			HAL_UART_Transmit(&huart1, (uint8_t *)debugStr, strlen(debugStr), 100);
// 			return MCU_FLASH_LOCK_ERROR;
// 		}
// 	}
// 	else
// 	{
// 		uint32_t flashErr = HAL_FLASH_GetError();
// 		sprintf(debugStr,
// 				"[ERROR] Flash write FAILED at 0x%08lX | HAL_Status=%d | FLASH_ERR=0x%lX\r\n",
// 				ulDestAdr, ucResp, flashErr);
// 		HAL_UART_Transmit(&huart1, (uint8_t *)debugStr, strlen(debugStr), 100);
// 		HAL_FLASH_Lock(); // Always lock flash
// 		return MCU_FLASH_WRITING_ERROR;
// 	}
// }

// static uint64_t ullMCUFLASH_iRead64DataFromMCUFlash_Exe(uint32_t ulAddr)
// {
// 	uint64_t ullVal = 0;

// 	ullVal	|= *((char *)ulAddr + 7);
// 	ullVal	= ullVal << 8;
// 	ullVal	|= *((char *)ulAddr + 6);
// 	ullVal	= ullVal << 8;
// 	ullVal	|= *((char *)ulAddr + 5);
// 	ullVal	= ullVal << 8;
// 	ullVal	|= *((char *)ulAddr + 4);
// 	ullVal	= ullVal << 8;
// 	ullVal	|= *((char *)ulAddr + 3);
// 	ullVal	= ullVal << 8;
// 	ullVal	|= *((char *)ulAddr + 2);
// 	ullVal	= ullVal << 8;
// 	ullVal	|= *((char *)ulAddr + 1);
// 	ullVal	= ullVal << 8;
// 	ullVal	|= *((char *)ulAddr + 0);

// 	return ullVal;
// }

// // Return		: Status of Flash Erase
// // 					1 = MCU_FLASH_UNLOCK_ERROR,
// //					2 = MCU_FLASH_LOCK_ERROR,
// //					3 = MCU_FLASH_WRITING_ERROR,
// //					4 = MCU_FLASH_WRITING_OK,
// //					5 = MCU_FLASH_ERASE_OK,
// //					6 = MCU_FLASH_ERASE_ERROR,
// static uint8_t ucMCUFLASH_iEraseOnePageOfMCUFlash_Exe(uint32_t ulAddr)
// {
// 	uint32_t ulPageError = 0, ulFirstPage = 0, ulNbOfPages = 0, ulBankNumber = 0;
// 	uint8_t ucResp;

// 	static FLASH_EraseInitTypeDef EraseInitStruct;

// 	HAL_FLASH_Unlock();

// 	/* Get the 1st page to erase */
// 	ulFirstPage = ulMCUFLASH_iGetPage_Exe(ulAddr);

// 	/* Get the number of pages to erase from 1st page */
// 	ulNbOfPages = ulMCUFLASH_iGetPage_Exe(ulAddr) - ulFirstPage + 1;

// 	/* Get the bank */
// 	ulBankNumber = ulMCUFLASH_iGetBank_Exe(ulAddr);

// 	/* Fill EraseInit structure*/
// 	EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
// 	EraseInitStruct.Banks       = ulBankNumber;
// 	EraseInitStruct.Page        = ulFirstPage;
// 	EraseInitStruct.NbPages     = ulNbOfPages;

//   /* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
//      you have to make sure that these data are rewritten before they are accessed during code
//      execution. If this cannot be done safely, it is recommended to flush the caches by setting the
//      DCRST and ICRST bits in the FLASH_CR register. */
// 	if (HAL_FLASHEx_Erase(&EraseInitStruct, &ulPageError) == HAL_OK)
// 	{
// 		ucResp = HAL_FLASH_Lock();

// 		if(ucResp == HAL_OK)
// 		{
// 			ucResp = MCU_FLASH_ERASE_OK;
// 		}
// 		else
// 		{
// 			ucResp = MCU_FLASH_LOCK_ERROR;
// 		}
// 	}
// 	else
// 	{
// 		ucResp = MCU_FLASH_ERASE_ERROR;
// 	}

// 	return ucResp;
// }

// /**
//   * @brief  Gets the page of a given address
//   * @param  Addr: Address of the FLASH Memory
//   * @retval The page of a given address
//   */
// static uint32_t ulMCUFLASH_iGetPage_Exe(uint32_t Addr)
// {
//   uint32_t page = 0;

//   if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
//   {
//     /* Bank 1 */
//     page = (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;
//   }
//   else
//   {
//     /* Bank 2 */
//     page = (Addr - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
//   }

//   return page;
// }

// /**
//   * @brief  Gets the bank of a given address
//   * @param  Addr: Address of the FLASH Memory
//   * @retval The bank of a given address
//   */
// static uint32_t ulMCUFLASH_iGetBank_Exe(uint32_t Addr)
// {
//   return FLASH_BANK_1;
// }

// // ============================================================================
// // Name			: ucMCUFLASH_eWriteBuffer2MCUFlash_Exe
// // Objective	: Write Buffer Data into MCU Flash
// // Input  		: 1. Source Address of Buffer
// //		  		  2. Destination Address of MCU Flash
// //		  		  3. No. of Bytes to Write
// // Output 		: Data in MCU Flash
// // Return		: Status of Flash Writing
// //					HAL_OK		- 0 - Flash Writing Successful
// //					HAL_ERROR	- 1 - Error
// //					HAL_BUSY	- 2 - Error
// //					HAL_TIMEOUT - 3 - Error
// // Author		: AK
// // Date			: 3 Jan 2019
// // ============================================================================
// Enum_FLASHStatus ucMCUFLASH_eWriteMCUFlash_Exe(uint32_t ulSourceAdr, uint32_t ulDestAdr, uint16_t uiBytes)
// {
// 	uint64_t ullVal = 0;
// 	uint32_t ulPageError = 0, ulFirstPage = 0, ulNbOfPages = 0, ulBankNumber = 0;
// 	uint16_t uiTemp;
// 	uint8_t ucStatus;

// 	Enum_FLASHStatus ucResp = MCU_FLASH_NO_STATUS;

// 	static FLASH_EraseInitTypeDef EraseInitStruct;

// 	__disable_irq();

// 	ucStatus = HAL_FLASH_Unlock();
// #if BLE
// 	sprintf(cAPP_eGlobalBuffer, "Unlock, %d\n", ucStatus);
// 	vBLE_eSendBytes_Exe();
// #endif

// 	if(ucStatus == HAL_OK)
// 	{
// #if BLE
// 	sprintf(cAPP_eGlobalBuffer, "After Unlock, %d\n", ucStatus);
// 	vBLE_eSendBytes_Exe();
// #endif
// 		/* Get the 1st page to erase */
// 		ulFirstPage = ulMCUFLASH_iGetPage_Exe(ulDestAdr);
// #if BLE
// 		sprintf(cAPP_eGlobalBuffer, "First Page:%d\n", ulFirstPage);
// 	vBLE_eSendBytes_Exe();
// #endif

// 		/* Get the number of pages to erase from 1st page */
// 		ulNbOfPages = ulMCUFLASH_iGetPage_Exe(ulDestAdr) - ulFirstPage + 1;
// #if BLE
// 		sprintf(cAPP_eGlobalBuffer, "Total Page:%d\n", ulNbOfPages);
// 	vBLE_eSendBytes_Exe();
// #endif

// 		/* Get the bank */
// 		ulBankNumber = ulMCUFLASH_iGetBank_Exe(ulDestAdr);
// #if BLE
// 		sprintf(cAPP_eGlobalBuffer, "Bank:%d\n", ulBankNumber);
// 	vBLE_eSendBytes_Exe();
// #endif

// 		/* Fill EraseInit structure*/
// 		EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
// 		EraseInitStruct.Banks       = ulBankNumber;
// 		EraseInitStruct.Page        = ulFirstPage;
// 		EraseInitStruct.NbPages     = ulNbOfPages;

// 		if (HAL_FLASHEx_Erase(&EraseInitStruct, &ulPageError) == HAL_OK)
// 		{
// #if BLE
// 		sprintf(cAPP_eGlobalBuffer, "After Erase\n");
// 	vBLE_eSendBytes_Exe();
// #endif
// 			// Program Flash
// 			uiTemp	= 0;
// 			do
// 			{
// 				ullVal 	= 0;
// 				ullVal	|= *((char *)ulSourceAdr + uiTemp + 7);
// 				ullVal	= ullVal << 8;
// 				ullVal	|= *((char *)ulSourceAdr + uiTemp + 6);
// 				ullVal	= ullVal << 8;
// 				ullVal	|= *((char *)ulSourceAdr + uiTemp + 5);
// 				ullVal	= ullVal << 8;
// 				ullVal	|= *((char *)ulSourceAdr + uiTemp + 4);
// 				ullVal	= ullVal << 8;
// 				ullVal	|= *((char *)ulSourceAdr + uiTemp + 3);
// 				ullVal	= ullVal << 8;
// 				ullVal	|= *((char *)ulSourceAdr + uiTemp + 2);
// 				ullVal	= ullVal << 8;
// 				ullVal	|= *((char *)ulSourceAdr + uiTemp + 1);
// 				ullVal	= ullVal << 8;
// 				ullVal	|= *((char *)ulSourceAdr + uiTemp + 0);

// 				ucStatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, ulDestAdr, ullVal);

// 				ulDestAdr += 8;
// 				uiTemp	  += 8;
// 			} while((uiTemp < uiBytes) && (ucStatus == HAL_OK));

// 			if(ucStatus == HAL_OK)
// 			{
// 				ucResp = MCU_FLASH_WRITING_OK;
// 			}
// 			else
// 			{
// 				ucResp = MCU_FLASH_WRITING_ERROR;
// 			}

// 			HAL_FLASH_Lock();
// 		}
// 		else
// 		{
// 			ucResp = MCU_FLASH_ERASE_ERROR;
// 		}
// 	}
// 	else
// 	{
// 		ucResp = MCU_FLASH_UNLOCK_ERROR;
// #if BLE
// 		sprintf(cAPP_eGlobalBuffer, "Erase Err:%d\n", ucResp);
// 	vBLE_eSendBytes_Exe();
// #endif
// 	}

// 	__enable_irq();

// 	return ucResp;
// }


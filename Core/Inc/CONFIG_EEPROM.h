// ============================================================================
// Module	: CONFIG
// Version	: 00
// Author	: AK
// Date		: 4 Aug 2019
// ============================================================================

#ifndef __CONFIG_H
#define __CONFIG_H

//*****************************************************************************
//   INCLUDE
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>

/*
#define LEN_SIG								5U
#define LEN_IMEINUM							18U
#define LEN_APN								17U
#define LEN_USERNAME							6U
#define LEN_PASSWORD							6U
#define LEN_PROTOCOL							1U
#define LEN_SERVERIP							20U
#define LEN_PORTNUM							6U
#define LEN_TXRATE							2U
#define LEN_CCIDNUM                         21U

#define LEN_FTPHostName 30
#define LEN_FTPUserName 15
#define LEN_FTPPasword 15
#define LEN_SWVER                           5U
#define LEN_CCID                           21U
#define LEN_AUTHMOBILENUM 20
#define LEN_REV 8
#define LEN_REVDATE 9
#define LEN_DOMAIN 40

#define EEPROM_ADDR_SIG						0x1E00
#define EEPROM_ADDR_IMEI						0x1E05
#define EEPROM_ADDR_APN						0x1E17
#define EEPROM_ADDR_APNUSERNAME			0x1E27
#define EEPROM_ADDR_APNPASSWORD			0x1E2D
#define EEPROM_ADDR_PROTOCOL				0x1E33
#define EEPROM_ADDR_DATA_SERVER_IPADDR		0x1E34
#define EEPROM_ADDR_DATA_SERVER_PORTNUM	0x1E48
#define EEPROM_ADDR_TXRATE_IGNON			0x1E4E
#define EEPROM_ADDR_TXRATE_IGNOFF			0x1E50
#define EEPROM_ADDR_GSM_CCID            0x1E52

#define EEPROM_ADDR_MSGCOUNTER				0x1E66

#define TOTAL_CONFIG_SIZE						(EEPROM_ADDR_MSGCOUNTER - EEPROM_ADDR_SIG) + 20
*/

// Lengths
#define LEN_SIG 5U
#define LEN_IMEINUM 18U
#define LEN_APN 17U
#define LEN_USERNAME 6U
#define LEN_PASSWORD 6U
#define LEN_PROTOCOL 1U
#define LEN_SERVERIP 20U
#define LEN_PORTNUM 6U
#define LEN_TXRATE 2U
#define LEN_CCIDNUM 21U

#define LEN_FTPHostName 30U
#define LEN_FTPUserName 15U
#define LEN_FTPPasword 15U
#define LEN_SWVER 5U
#define LEN_CCID 21U
#define LEN_AUTHMOBILENUM 20U
#define LEN_REV 8U
#define LEN_REVDATE 9U
#define LEN_DOMAIN 40U
#define LEN_VEHICLE_RUN_STATUS 1U
#define LEN_BATTID 14U
#define LEN_ODO_DIST 8U

// EEPROM Address Map
#define EEPROM_ADDR_SIG 0x0000
#define EEPROM_ADDR_IMEI (EEPROM_ADDR_SIG + LEN_SIG)                                    // 0x0005
#define EEPROM_ADDR_APN (EEPROM_ADDR_IMEI + LEN_IMEINUM)                                // 0x0017
#define EEPROM_ADDR_APNUSERNAME (EEPROM_ADDR_APN + LEN_APN)                             // 0x0028
#define EEPROM_ADDR_APNPASSWORD (EEPROM_ADDR_APNUSERNAME + LEN_USERNAME)                // 0x002E
#define EEPROM_ADDR_PROTOCOL (EEPROM_ADDR_APNPASSWORD + LEN_PASSWORD)                   // 0x0034
#define EEPROM_ADDR_DATA_SERVER_IPADDR (EEPROM_ADDR_PROTOCOL + LEN_PROTOCOL)            // 0x0035
#define EEPROM_ADDR_DATA_SERVER_PORTNUM (EEPROM_ADDR_DATA_SERVER_IPADDR + LEN_SERVERIP) // 0x0049
#define EEPROM_ADDR_TXRATE_IGNON (EEPROM_ADDR_DATA_SERVER_PORTNUM + LEN_PORTNUM)        // 0x004F
#define EEPROM_ADDR_TXRATE_IGNOFF (EEPROM_ADDR_TXRATE_IGNON + LEN_TXRATE)               // 0x0051
#define EEPROM_ADDR_GSM_CCID (EEPROM_ADDR_TXRATE_IGNOFF + LEN_TXRATE)                   // 0x0053
#define EEPROM_ADDR_FTPHOSTNAME (EEPROM_ADDR_GSM_CCID + LEN_CCIDNUM)                    // 0x0068
#define EEPROM_ADDR_FTPUSERNAME (EEPROM_ADDR_FTPHOSTNAME + LEN_FTPHostName)             // 0x0086
#define EEPROM_ADDR_FTPPASSWORD (EEPROM_ADDR_FTPUSERNAME + LEN_FTPUserName)             // 0x0095
#define EEPROM_ADDR_SWVER (EEPROM_ADDR_FTPPASSWORD + LEN_FTPPasword)                    // 0x00A4
#define EEPROM_ADDR_CCID (EEPROM_ADDR_SWVER + LEN_SWVER)                                // 0x00A9
#define EEPROM_ADDR_AUTHMOBILENUM (EEPROM_ADDR_CCID + LEN_CCID)                         // 0x00BE
#define EEPROM_ADDR_REV (EEPROM_ADDR_AUTHMOBILENUM + LEN_AUTHMOBILENUM)                 // 0x00D2
#define EEPROM_ADDR_REVDATE (EEPROM_ADDR_REV + LEN_REV)                                 // 0x00DA
#define EEPROM_ADDR_DOMAIN (EEPROM_ADDR_REVDATE + LEN_REVDATE)                          // 0x00E3
#define EEPROM_ADDR_VEHICLE_RUN_STATUS (EEPROM_ADDR_DOMAIN + LEN_DOMAIN)                // 0x00E4
#define EEPROM_ADDR_BATTID (EEPROM_ADDR_VEHICLE_RUN_STATUS + LEN_VEHICLE_RUN_STATUS)
#define EEPROM_ADDR_ODO_DIST (EEPROM_ADDR_BATTID + LEN_ODO_DIST)

// Firmware‐update flags (keep these well apart from the config block)
#define EEPROM_ADDR_FW_UPDATE_STATUS 0x300  // 1 byte
#define EEPROM_ADDR_FW_CHECKED_DATE  0x314  // 1 byte
// #define EEPROM_ADDR_IMMOB_VER_NUM    0x0124  // 5 bytes [0x0124..0x0128]
#define EEPROM_ADDR_IOT_VER_NUM      0x302  // 5 bytes [0x0129..0x012D]
// #define EEPROM_ADDR_IOT_SMS_VER_NUM  0x012E  // 5 bytes [0x012E..0x0132]
// #define EEPROM_ADDR_IMMOB_SMS_VER_NUM 0x0133 // 5 bytes [0x0133..0x0137]
#define EEPROM_ADDR_SMS_SENDER_NUM   0x316  // 25 bytes [0x0138..0x0150]

// #define EEPROM_ADDR_FW_UPDATE_STATUS 0x0300  // 1 byte
// #define EEPROM_ADDR_FW_CHECKED_DATE 0x0301   // 1 byte
// #define EEPROM_ADDR_IMMOB_VER_NUM 0x0302     // 5 bytes [0x302..0x306]
// #define EEPROM_ADDR_IOT_VER_NUM 0x0307       // 5 bytes [0x307..0x30B]
// #define EEPROM_ADDR_IOT_SMS_VER_NUM 0x030C   // 5 bytes [0x30C..0x310]
// #define EEPROM_ADDR_IMMOB_SMS_VER_NUM 0x0311 // 5 bytes [0x311..0x315]
// #define EEPROM_ADDR_SMS_SENDER_NUM 0x0316    // 25 bytes[0x316..0x32E]

#define EEPROM_ADDR_MSGCOUNTER (EEPROM_ADDR_ODO_DIST + LEN_ODO_DIST) // 0x010B

// Optional: total config data size
#define TOTAL_CONFIG_SIZE ((EEPROM_ADDR_MSGCOUNTER - EEPROM_ADDR_SIG) + 20) // = 0x010B - 0x0000 + 20 = 283 bytes


typedef enum
{
    VEHICLE_RV300 = 3,
    VEHICLE_RV400 = 4,
    Testing_IoT = 5,
} Enum_VehModel;

typedef enum
{
	NO_VERSION_CHECK				= 0,
	CHK_VERSION_UPDATE				= 1,
	NO_CLUSTER_VERSION_CHANGE		= 2,
	CLUSTER_FW_UPDATED				= 3,
	NO_IMMOB_VERSION_CHANGE			= 4,
	IMMOB_FW_UPDATED				= 5,
	SMS_CLUSTER_FW_VERSION			= 6,
	SMS_IMMOB_FW_VERSION			= 7,
	MASTER_MODE_CLUSTER_FW_VERSION	= 8,	//Rev 1.021 19-09-2024
}Enum_FTPFWMsg;


#pragma pack(push, 1)
typedef struct
{
    uint8_t ucSig1;
    uint8_t ucSig2;
    uint8_t ucSig3;
    uint8_t ucSig4;
    uint8_t ucSig5; // <-- 5th signature byte needed

    char cIMEI[LEN_IMEINUM];
    char cAPN[LEN_APN];

    uint8_t		ucDomainStatus; 			// 0 - IP Address & Port Enable, Domain Disable; 1 - IP Address & Port Disable, Domain Enable
    char cLocationIPAdd[LEN_SERVERIP];
    char cLocationPortNum[LEN_PORTNUM];
    char cDomainName[LEN_DOMAIN];

    uint16_t uiTxRateIgnON;
    uint16_t uiTxRateIgnOFF;
    uint16_t uiTxRateBattRemove;

    char cCCIDNum[LEN_CCIDNUM];
    uint8_t ucIMEICCIDStatus;
} TsEEPROMConfig;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct
{
    uint8_t ucVehicleType;

    uint8_t ucProtocol;

    char cFTPIPAdd[LEN_FTPHostName];
    char cFTPPortNum[LEN_PORTNUM];
    char cFTPUsername[LEN_FTPUserName];
    char cFTPPassword[LEN_FTPPasword];

    bool bFlagSendOTAFrmValidSource;
    char cMobile1[LEN_AUTHMOBILENUM];

    char cSWRev[LEN_REV];
    char cSWRevDate[LEN_REVDATE];
} TsConfig;
#pragma pack(pop)


//*****************************************************************************
//   Proto Declaration
//*****************************************************************************
void vCONFIGEEPROM_eReadConfigData_Exe(void);
void vCONFIGEEPROM_eLoadDefaultPar_Exe(void);

#endif

/***** END OF FILE ****/

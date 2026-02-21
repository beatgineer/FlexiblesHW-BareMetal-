// // Define to prevent recursive inclusion -------------------------------------
// #ifndef __BATT_H
// #define __BATT_H
// #ifdef __cplusplus
//  extern "C" {
// #endif

// // Includes ------------------------------------------------------------------
// #include "stm32g0xx_hal.h"
// #include <stdbool.h>

// void vADC_eInit(void);
// void vBATT_eReadMainBATTVolt_Exe(void);
// void vBATT_eReadIntBATTVolt_Exe(void);


// #define TOTAL_ADC_CHANNEL	3

// #define ADC_CFAC_MAINBATTVOL				(float)0.03304//0.031252 //0.025795
// #define ADC_CFAC_INTBATTVOL					(float)1.150
// #define ADC_CFAC_INTBATTCHGSTATUS			(float)1.150

// #define MAINBATT_ADCIN6_Pin 				GPIO_PIN_6
// #define MAINBATT_ADCIN6_GPIO_Port 			GPIOA

// #define INTBATT_ADC1IN7_Pin 				GPIO_PIN_7
// #define INTBATT_ADC1IN7_GPIO_Port 			GPIOA

// #define INTBATT_CHG_DRV_Pin 				GPIO_PIN_3
// #define INTBATT_CHG_DRV_GPIO_Port 			GPIOB

// #define INTBATT_ADC1IN7_Pin 				GPIO_PIN_7
// #define INTBATT_ADC1IN7_GPIO_Port 			GPIOA

// #define INTBATT_CHGSTATUS_ADCIN8_Pin 		GPIO_PIN_0
// #define INTBATT_CHGSTATUS_ADCIN8_GPIO_Port  GPIOB
// typedef enum
// {
//  	RAW_INT_BATT_CHG_VAL,
//  	RAW_MAIN_BATT_ADC_VAL,
//  	RAW_INT_BATT_ADC_VAL,
// } Enum_ADCVAL_Mode;


// #pragma pack(push,1)
// typedef struct
//  {
//  	uint32_t	ulRawADCVal[TOTAL_ADC_CHANNEL];

//  	uint8_t		ucBATTVolt;
//  	uint16_t	uiIntBATTVolt;
//  } TsBATTData;
// #pragma pack(pop)

// typedef union
// {
//  	struct
//  	{
//  		bool		bBATTVoltStatus			: 1;
//  		bool		bBATTVoltStatusPrev		: 1;
//  		bool		bBATTStatus				: 1;
//  		bool		bMCBStatus				: 1;
//  		bool		bIntBattMOSFETChgStatus	: 1;		// 0 - Chg Off, 1 - Chg ON
//  		uint8_t		ucSpare					: 3;
//  	};
//  	uint8_t	ucData[1];
// }TsBATTStatus;

// #endif /* INC_BATT_H_ */

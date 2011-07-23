/**
  ******************************************************************************
  * @file STM32F10x_CEC_Lib/inc/stm32f10x_cec.h 
  * @author  MCD Application Team
  * @version  V2.0.0
  * @date  04/27/2009
  * @brief  This file contains the CEC functions prototypes.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */ 


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_CEC_H 
#define __STM32F10x_CEC_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* System Tick prescaler computation */
/* Start bit timings (based SysTick end of count event each 100탎)  */
#define Sbit_Nom_LD    37 /* Start Bit Nominal Low duration: 37 x 100탎 = 3.7ms */
#define Sbit_Nom_HD    8  /* Start Bit Nominal High Duration: 8 x 100탎 = 0.8ms */
#define Sbit_Min_LD    35 /* Start Bit Minimum Low duration: 35 x 100탎 = 3.5ms */
#define Sbit_Max_LD    39 /* Start Bit Maximum Low duration: 39 x 100탎 = 3.9ms */
#define Sbit_Min_TD    43 /* Start Bit Minimum Total duration: 43 x 100탎 = 4.3ms */                       
#define Sbit_Max_TD    47 /* Start Bit Maximum Total duration: 47 x 100탎 = 4.7ms */

/* Data bit logical "0" timings  */
#define Dbit0_Nom_LD   15 /* Data Bit "0" Nominal Low duration: 15 x 100탎 = 1.5ms */
#define Dbit0_Nom_HD   9  /* Data Bit "0" Nominal High Duration: 9 x 100탎 = 0.9ms */
#define Dbit0_Min_LD   13 /* Data Bit "0" Minimum Low duration: 13 x 100탎 = 1.3ms */
#define Dbit0_Max_LD   17 /* Data Bit "0" Maximum Low duration: 17 x 100탎 = 1.7ms */

/* Data bit logical "1" timings  */
#define Dbit1_Nom_LD   6  /* Data Bit (logical "1") Nominal Low duration: 0.6ms  */
#define Dbit1_Nom_HD   18 /* Data Bit (logical "1") Nominal High Duration: 1.8ms */
#define Dbit1_Min_LD   4  /* Data Bit "1" Minimum Low duration: 0.4ms */
#define Dbit1_Max_LD   8  /* Data Bit "1" Maximum Low duration: 0.8ms */

/* Data bit duration  */
#define DbitX_Min_TD   20 /* Data Bit Minimum Total duration: 2ms   */                       
#define DbitX_Max_TD   27 /* Data Bit Maximum Total duration: 2.7ms */

/* Header or Data block definition */
#define DataBlock      0
#define HeaderBlock    1

/* Masks */
#define ReceiveFrameStatusMask  0x00010000
#define FrameSendToMeMask       0x00020000
#define InitiatorAddressMask    0x000000FF

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/* CEC low layer functions */
void CEC_Init(void);
void CEC_NVIC_Configuration(void);
void CEC_Wait100us(__IO uint32_t nTime);
void CEC_TimingDelay_Decrement(void);

/* CEC meduim layer functions */
void CEC_SendStartBit(void);
uint8_t CEC_ReceiveStartBit(void);
void CEC_SendAckBit(void);
uint8_t CEC_ReceiveAckBit(void);
void CEC_SendDataBit(uint8_t bit);
uint8_t CEC_ReceiveDataBit(void);
ErrorStatus CEC_SendByte(uint8_t byte);
uint8_t CEC_ReceiveByte(uint8_t HeaderDataIndicator);

/* CEC high layer functions */
ErrorStatus CEC_SendFrame(uint8_t InitiatorAddress, uint8_t FollowerAddress, uint8_t MessageLength, uint8_t* Message);
uint32_t CEC_ReceiveFrame(uint8_t* Message, uint8_t FollLogAdd);
					 
#endif /* __STM32F10x_CEC_H */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/

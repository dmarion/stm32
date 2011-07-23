/**
  ******************************************************************************
  * @file STM32F10x_CEC_Lib/src/stm32f10x_cec.c 
  * @author  MCD Application Team
  * @version  V2.0.0
  * @date  04/27/2009
  * @brief  This file provides all the CEC firmware functions.
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


/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_cec.h"
#include "stm32f10x.h"


/** @addtogroup STM32F10x_CEC_Lib
  * @{
  */ 


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static __IO uint32_t TimingDelay=0;
uint8_t cec_byte;
uint8_t cec_bit;
uint8_t cec_eom;
__IO uint32_t cec_counter;
uint8_t cec_last_byte;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/**
  * @brief  Configures Vector Table base location. It configures the
  *   System tick and EXTI0 global interrupts.
  * @param  None
  * @retval : None
  */
void CEC_NVIC_Configuration(void)
{
NVIC_InitTypeDef NVIC_InitStructure;
    

  /* Enable the EXTI0 global Interrupt (with higher priority) */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);	
}



/**
  * @brief  Inserts a delay time = 100Ýs x nTime
  * @param ime: specifies the delay to be inserted.
  *   nTime = delay time/100Ýs 
  * @retval : None
  */
void CEC_Wait100us(uint32_t nTime)
{
  
  TimingDelay = nTime;

   while (TimingDelay )  {}	    
   TimingDelay=0;
}



/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval : None
  */
void CEC_TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}



/**
  * @brief  Initialize peripherals to be ready to the CEC communication.
  * @param  None
  * @retval : None
  */
void CEC_Init(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  
  /* Enable GPIOA and AFIO clocks */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO , ENABLE);

  /* Go to high impedance: bus state = VDD */
  GPIOA->BSRR = GPIO_Pin_0;
      
  /* Configure GPIOA Pin 0 as Output open drain */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
   
  /* NVIC configuration for CEC communication */
  CEC_NVIC_Configuration();
   
  /* Connect EXTI Line 0 to GPIOA Pin 0 */
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);

  /* Configure CEC pin to generate an EXTI interrupt on falling edge */  
  EXTI_InitStructure.EXTI_Line = EXTI_Line0;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

	
      /* Setup SysTick Timer for 100 Ýs interrupts */
  if (SysTick_Config(SystemFrequency /10000)) /* SystemFrequency is defined in
   “system_stm32f10x.h” and equal to HCLK frequency */
   {
    /* Capture error */
     while (1);

    }     

		

	 /* Enable the SysTick_IRQn  Interrupt */
     NVIC_SetPriority(SysTick_IRQn, 0x00);
    
  
}



/**
  * @brief  Send the CEC start bit.
  * @param  None
  * @retval : None
  */
void CEC_SendStartBit(void)
{
  /* Start Low Period */
  GPIOA->BRR = GPIO_Pin_0;
  
  /* Wait 3.7ms*/
  CEC_Wait100us(Sbit_Nom_LD);

  /* Start High Period */
  GPIOA->BSRR = GPIO_Pin_0; 

  /* Wait 0.8ms for a total of 3.7 + 0.8 = 4.5ms */
  CEC_Wait100us(Sbit_Nom_HD); 
}



/**
  * @brief  Send a CEC data bit.
  * @param bit: the bit to be send. It can be 0 or 1.
  *   0: for logical 0
  *   1: for logical 1
  * @retval : None
  */
void CEC_SendDataBit(uint8_t bit)
{
  /* Start Low Period:the duration depends on the Logical Level "0" or "1"*/
  GPIOA->BRR = GPIO_Pin_0;

  /* Wait 0.6 ms if Logical Level is "1" and 1.5ms  if Logical Level is "0" */
  CEC_Wait100us( bit ? Dbit1_Nom_LD : Dbit0_Nom_LD );

  /* Start High Period: the duration depends on the Logical Level "0" or "1"*/
  GPIOA->BSRR = GPIO_Pin_0;;

  /* Wait 1.8 ms if Logical Level is "1" and 0.9 ms  if Logical Level is "0" */
  CEC_Wait100us(bit ? Dbit1_Nom_HD : Dbit0_Nom_HD); 
}




/**
  * @brief  Send the CEC ACK bit (ACK = logical 0)
  * @param  None
  * @retval : None
  */
void CEC_SendAckBit(void)
{
  /* Wait for falling edge: end of EOM bit sent by the initiator */
  while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0 ))
  {
  }
  
  /* Send ACK bit */
  CEC_SendDataBit(0);  
  
  /* Force the bus to 0: for ACK bit delimiting */
  GPIOA->BRR = GPIO_Pin_0; 
  
  /* Wait 100Ýs to allow the initiator to detect the end of ACK bit */
  CEC_Wait100us(1); 
  
  /* Go to high impedance: CEC bus state = VDD */
  GPIOA->BSRR = GPIO_Pin_0;
}



/**
  * @brief  Receive the CEC ACK bit.
  * @param  None
  * @retval : Ack value: 0 for success and 0xFF for error.
  */
uint8_t CEC_ReceiveAckBit(void)
{
  uint8_t AckValue = 0xFF;
   
  /* Get the ACK bit */
  AckValue = CEC_ReceiveDataBit();  
  
  /* If the byte has been acknowledged by the Follower (ACK = 0) */
  if (AckValue != 0xFF) 
  {
    /* Wait for falling edge of ACK bit (the end of ACK bit)*/
    while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0))
    {
    }
  }

  return AckValue;
}



/**
  * @brief  Receive the CEC start bit.
  * @param  None
  * @retval : 1: if the CEC start bit has been received correctly
  *   0: if the CEC start bit has not been received correctly
  */
uint8_t CEC_ReceiveStartBit(void)
{
  /* Initialize cec_counter */
  cec_counter = 0;
  
  /* Go to high impedance: CEC bus state = VDD */
  GPIOA->BSRR = GPIO_Pin_0;
  
  /* Wait for rising edge of the start bit */
  while (!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0))
  {
    /* Wait 100Ýs */ 
    CEC_Wait100us(1);	
    
    /* Increment cec_counter that contains the duration of low level duration */
    cec_counter++;
    
    /* If too long low level for start bit */
    if (cec_counter > Sbit_Max_LD) 
    {
      /* Exit: it's an error: 0 */
      return 0;  
    }
  }
  
  /* If too short duration of low level for start bit */
  if (cec_counter < Sbit_Min_LD ) 
  {
    /* Exit: it's an error: 0 */
    return 0;		
  }

  /* Wait for falling edge of the start bit (end of start bit) */
  while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0 ))	
  {
    /* Wait 100Ýs */ 
    CEC_Wait100us(1);	
    
    /* Increment cec_counter that contains the duration of high level duration */
    cec_counter++;
    
    /* If too long total duration for start bit */
    if (cec_counter > Sbit_Max_TD ) 	
    {
      /* Exit: it's an error: 0 */
      return 0;	
    }
  }
  
  /* If too short total duration for start bit */
  if (cec_counter < Sbit_Min_TD )
  {
    /* Exit: it's an error: 0 */
    return 0;
  }
    
  /* Exit: start bit received correctly */
  return 1;	
}


/**
  * @brief  Receive a CEC data bit.
  * @param  None
  * @retval : 1: if the data bit received was logical 1.
  *   0: if the data bit received was logical 0.
  *   0xFF: if the data bit has not been received correctly.
  */
uint8_t CEC_ReceiveDataBit(void)
{
  uint8_t bit = 0xFF;
  
  /* Initialize cec_counter */
  cec_counter = 0;
  
  /* Wait for rising edge of the data bit */
  while (!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0 )) 
  {
    /* Wait 100Ýs */  
    CEC_Wait100us(1);	
    
    /* Increment cec_counter that contains the duration of low level duration */
    cec_counter++;
    
    if (cec_counter > Dbit0_Max_LD )
    {
      /* Exit: it's an error: 0xFF */
      return 0xFF;	
    }
  }

  /* If the measured duration of the low level is greater than the minimum low duration
    of "logical 0" i.e > 1.3 ms */
  if (cec_counter> Dbit0_Min_LD)		
  {  
    /* The received bit is "logical 0" */ 
    bit = 0;
  }				
  else						
  { 
    /* If the measured duration of the low level is greater than the maximum low duration
    of "logical 1" i.e > 0.8 ms */
    if (cec_counter> Dbit1_Max_LD)
    {
      /* Exit: it's an error: 0xFF */
      return 0xFF; 
    }
    
    /* If the measured duration of the low level is greater than the minimum low duration
    of "logical 1" i.e > 0.4 ms */
    if (cec_counter> Dbit1_Min_LD)
    {
      /* The received bit is "logical 1" */ 
      bit = 1;
    } 
    else 
    {
      /* Exit: it's an error: 0xFF */
      return 0xFF; 
    }
  } 
 	
  /* Wait for falling edge of the data bit */
  while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0 ))	
  {
    /* Wait 100Ýs */
    CEC_Wait100us(1);	
   
    /* Increment cec_counter that contains the duration of high level duration */
    cec_counter++;
    
    /* If too long total duration for the data bit is detected	*/
    if (cec_counter > DbitX_Max_TD )	
    {
      /* Exit: it's an error: 0xFF */
      return 0xFF;
    }
  }
  
  /* If too short total duration for the data bit is detected	*/
  if (cec_counter < DbitX_Min_TD )
  {
    /* Exit: it's an error: 0xFF */
    return 0xFF;	
  }
    
  /* The data bit is received correctly. Return its value */
  return bit;				
}



/**
  * @brief  Send a CEC Header or Data block.
  * @param byte: the byte to be send.
  * @retval : The status of the CEC byte transmission. It can be:
  *   SUCCESS: CEC byte was acknowledged by the follower.
  *   ERROR: CEC byte wasn’t acknowledged by the follower.
  */
ErrorStatus CEC_SendByte(uint8_t byte)
{
  /* Send the data byte: bit by bit (MSB first) */
  for (cec_bit=0; cec_bit<=7; cec_bit++)
  {
    CEC_SendDataBit(byte & 0x80);
    byte <<=1;
  }

  /* Send EOM bit after sending the data byte */
  CEC_SendDataBit(cec_last_byte); 

  /* Force the bus to 0: for EOM bit delimiting */
  GPIOA->BRR = GPIO_Pin_0;  
  
  /* Wait 100Ýs to allow the follower to detect the end of EOM bit */
  CEC_Wait100us(1); 
  
  /* Go to high impedance: CEC bus state = VDD */
  GPIOA->BSRR = GPIO_Pin_0; 
  
  /* If the byte is acknowledged by the receiver */
  if(CEC_ReceiveAckBit() == 0)  
  {
    /* Exit: the data byte has been received by the follower */
    return SUCCESS;
  }
  else  /* The data byte is not acknowledged by the follower */
  {
    /* Exit: the data byte has not been received by the follower*/
    return ERROR; 
  }
}



/**
  * @brief  Receieve a CEC byte.
  * @param  HeaderDataIndicator: to indicate if the byte to receive  
  *   is a Header or Data block. It can be:
  *   HeaderBlock: for the Header block reception.
  *   DataBlock: for the Data block reception.
  * @retval : The receive status. It can be:
  *   1: the receive of the byte was successful.
  *   0: the receive of the byte was not successful.
  */
uint8_t CEC_ReceiveByte(uint8_t HeaderDataIndicator)
{
  uint8_t TempReceiveBit = 0;
  ErrorStatus ReceiveByteStatus = SUCCESS;

  /* Initialize cec_counter */
  cec_byte = 0;
  
  for (cec_bit=0; cec_bit<=7; cec_bit++)
  {
    /* Shift the data byte */
    cec_byte <<= 1;  

    /* Receive the data bit */
    TempReceiveBit = CEC_ReceiveDataBit();
    
    /* If the received bit was received incorrectly  */
    if (TempReceiveBit == 0xFF) /* If the received bit is wrong */
    {
      /* Store the error status */
      ReceiveByteStatus = ERROR;
    }
    
    /* Build the data byte (MSB first) */
    cec_byte |= TempReceiveBit;  
  }
  
  /* Read EOM bit */
  cec_eom = CEC_ReceiveDataBit();  
  
  /* If the EOM bit was received incorrectly  */
  if (cec_eom == 0xFF) /* If the received bit is wrong */
  {
    /* Store the error status */
    ReceiveByteStatus = ERROR;
  }

  /* If the byte to send is a "Data" block */
  if (HeaderDataIndicator == DataBlock) 
  {
    /* If the bits has been received correctly, acknowledge the data byte */
    if(ReceiveByteStatus != ERROR) 
    {               
      /* Send the Ack bit */
      CEC_SendAckBit(); 
     
      /* Byte received correctly: return 1 */
      return 1;         
    }
    else /* Otherwise do not acknowledge the received byte */
    {
      /* Byte received incorrectly: return 0 */
      return 0; 
    }
   }
   else   /* If the byte is a "Header" block */
   {
    /* If the bits has been received correctly and do not acknowledge the byte */
    if(ReceiveByteStatus != ERROR) 
    {                
      /* Byte received correctly: return 1 */
      return 1; 
    }
    else /* If the bits has been received incorrectly and do not acknowledge the byte */
    {
      /* Byte received incorrectly: return 0 */
      return 0; 
    }
  }
}



/**
  * @brief  Send a CEC frame.
  * @param InitiatorAddress: the initiator address: from 0 to 15.
  * @param FollowerAddress: the follower address: from 0 to 15.
  * @param MessageLength: the number of data byte to send.
  * @param Message: a pointer to the transmit buffer.
  * @retval : The status of the transmission. It can be:
  *   SUCCESS: If the follower received the frame.
  *   ERROR: If the follower doesn't received the frame.
  */
ErrorStatus CEC_SendFrame(uint8_t InitiatorAddress, uint8_t FollowerAddress, uint8_t MessageLength, uint8_t* Message )
{
   uint8_t i=0;
   uint8_t HeaderBlockValueToSend = 0;
   NVIC_InitTypeDef NVIC_InitStructure;
    
   cec_last_byte=0;
  
   /* Build the Header block to send */
   HeaderBlockValueToSend = (((InitiatorAddress&0xF)<<4) | (FollowerAddress&0xF));

   /* Disable EXTI0 global interrupt to avoid the EXTI to enter EXTI0 interrupt
     while transmitting a frame */

   NVIC_DisableIRQ(EXTI0_IRQn);
   
   /* Send start bit */
   CEC_SendStartBit();
  
   /* Send initiator and follower addresses. If the Header block is not
     transmitted successfully then exit and return error */ 
   if (CEC_SendByte(HeaderBlockValueToSend)== ERROR)  
   {
     /* Clear EXTI line 0 pending bit */
     EXTI_ClearITPendingBit(EXTI_Line0);
     
     /* Clear EXTI line 0 global interrupt pending bit */
     NVIC_ClearPendingIRQ( EXTI0_IRQn );
    
   
     /* Enable EXTI Line 0 global interrupt */
     NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
     NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
     NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
     NVIC_Init(&NVIC_InitStructure);	
     /* Exit and return send failed */
     return ERROR; 
   }
  
   /* Send data bytes */
   for(i=0;i< MessageLength;i++)
   {
     if (i==(MessageLength-1))
     {
       cec_last_byte=1;
     }
    
     /* Send data byte and check if the follower sent the ACK bit = 0 */
     if (CEC_SendByte(Message[i]) == ERROR) 
     {
       /* Clear EXTI line 0 pending bit */
       EXTI_ClearITPendingBit(EXTI_Line0);
     
       /* Clear EXTI line 0 global interrupt pending bit */
       NVIC_ClearPendingIRQ( EXTI0_IRQn );
     
     /* Enable EXTI Line 0 global interrupt */
       NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
       NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
       NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
       NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
       NVIC_Init(&NVIC_InitStructure);	
       
       /* Exit and return send failed */
       return ERROR; 
     }
   }

   /* Clear EXTI line 0 pending bit */
   EXTI_ClearITPendingBit(EXTI_Line0);
     
   /* Clear EXTI line 0 global interrupt pending bit */
   NVIC_ClearPendingIRQ( EXTI0_IRQn );
    
  /* Enable EXTI Line 0 global interrupt */
   NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);	  

   /* Exit and return send succeeded */
   return SUCCESS;
}



/**
  * @brief  Receive a CEC frame.
  * @param Message: a pointer to the receive buffer.
  * @param FollLogAdd: the address of the follower (0 to 15).
  * @retval : An integer that contains diffrent informations of the receive
  *   action.
  */
uint32_t CEC_ReceiveFrame(uint8_t* Message, uint8_t FollLogAdd)
{
  uint32_t i=0;
  uint32_t InitiatorAddress = 0;
  uint32_t ReceiveStatus = 1;
  uint32_t FrameSendToMe = 0;
  cec_byte = 0;
  cec_eom = 0;

  /* If the start bit has been received successfully */
  if (CEC_ReceiveStartBit())	
  {
    /* Get the first byte and ckeck if the byte has been received correctly */
    if (CEC_ReceiveByte(HeaderBlock)) 
    {  
      /* Get the initiator address */
      InitiatorAddress = cec_byte>>4;	
      
      /* If the frame was sent to me */
      if ((cec_byte & 0x0F) == FollLogAdd)	
      {
        CEC_SendAckBit(); /* Send Acknowledge bit = 0 */
	FrameSendToMe = 1;
        
        /* Continue to read the data byte since the frame is not completed */
        while (!cec_eom)		
        {
          /* Ckeck if the byte has been received correctly */
	  if (CEC_ReceiveByte(DataBlock)) 
          {  
             /* Build the frame */
	     Message[i]=cec_byte;
	     i++;
          }
          else /* If the byte has not been received correctly */
          {
            /* Set receive status bit (Error) */
            ReceiveStatus = 0; 
          }
        }
      }
    }
  }
  else   /* If the start bit has not been received successfully */
  {
    /* Set receive status bit (Error) */
    ReceiveStatus = 0; 
  } 
  
  /* Return the different receive infos: Receive status, number of data byte received, 
     Frame send to me or not and the initiator address */ 
  return ((((InitiatorAddress) | (i<<8)) | (ReceiveStatus << 16)) | (FrameSendToMe << 17));
}
	 
/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/

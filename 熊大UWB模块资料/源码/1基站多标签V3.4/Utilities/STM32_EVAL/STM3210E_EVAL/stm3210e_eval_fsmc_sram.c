/**
  ******************************************************************************
  * @file    stm3210e_eval_fsmc_sram.c
  * @author  MCD Application Team
  * @version V4.5.0
  * @date    07-March-2011
  * @brief   This file provides a set of functions needed to drive the 
  *          IS61WV51216BLL SRAM memory mounted on STM3210E-EVAL board.
  ******************************************************************************
  * @attention
  *
  * modefied by www.armjishu.com  
  *  
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************  
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm3210e_eval_fsmc_sram.h"

/** @addtogroup Utilities
  * @{
  */
  
/** @addtogroup STM32_EVAL
  * @{
  */ 

/** @addtogroup STM3210E_EVAL
  * @{
  */
  
/** @addtogroup STM3210E_EVAL_FSMC_SRAM
  * @brief      This file provides a set of functions needed to drive the 
  *             IS61WV51216BLL SRAM memory mounted on STM3210E-EVAL board.
  * @{
  */ 

/** @defgroup STM3210E_EVAL_FSMC_SRAM_Private_Types
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup STM3210E_EVAL_FSMC_SRAM_Private_Defines
  * @{
  */ 
/** 
  * @brief  FSMC Bank 1 NOR/SRAM3  
  */
#define Bank1_SRAM3_ADDR    ((uint32_t)0x68000000)     
/**
  * @}
  */ 


/** @defgroup STM3210E_EVAL_FSMC_SRAM_Private_Macros
  * @{
  */
/**
  * @}
  */ 
  

/** @defgroup STM3210E_EVAL_FSMC_SRAM_Private_Variables
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup STM3210E_EVAL_FSMC_SRAM_Private_Function_Prototypes
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup STM3210E_EVAL_FSMC_SRAM_Private_Functions
  * @{
  */ 

/**
  * @brief  Configures the FSMC and GPIOs to interface with the SRAM memory.
  *         This function must be called before any write/read operation
  *         on the SRAM.
  * @param  None 
  * @retval None
  */
void SRAM_Init(void)
{
  FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
  FSMC_NORSRAMTimingInitTypeDef  p;
  GPIO_InitTypeDef GPIO_InitStructure; 
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOG | RCC_APB2Periph_GPIOE |
                         RCC_APB2Periph_GPIOF, ENABLE);
  
/*-- GPIO Configuration ------------------------------------------------------*/
  /*FSMC������FSMC_D[0:15]��ʼ�������츴�����*/
  /*!< SRAM Data lines configuration */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_8 | GPIO_Pin_9 |
                                GPIO_Pin_10 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 |
                                GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | 
                                GPIO_Pin_15;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  
  /*!< SRAM Address lines configuration */
  /*FSMC��ַ��FSMC_A[0:17]��ʼ�������츴�����*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | 
                                GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_12 | GPIO_Pin_13 | 
                                GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_Init(GPIOF, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | 
                                GPIO_Pin_4 | GPIO_Pin_5;
  GPIO_Init(GPIOG, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13; 
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  /*!< NOE and NWE configuration */  
  /*FSMC NOE��NWE���Ի������츴�����*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 |GPIO_Pin_5;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  
  /*!< NE3 configuration */
  /*FSMC NE3���Ի������츴�����*/  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
  GPIO_Init(GPIOG, &GPIO_InitStructure);
  
  /*!< NBL0, NBL1 configuration */
  /*FSMC NBL0��NBL1���Ի������츴�����*/ 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; 
  GPIO_Init(GPIOE, &GPIO_InitStructure); 
  
/*-- FSMC Configuration ------------------------------------------------------*/
/*--------------FSMC ���� �洢����������------------------------------*/
  p.FSMC_AddressSetupTime = 0;              //��ַ����ʱ��    
  p.FSMC_AddressHoldTime = 0;               //��ַ����ʱ��  
  p.FSMC_DataSetupTime = 1;                 //���ݽ���ʱ��
  p.FSMC_BusTurnAroundDuration = 0;         //���߻ָ�ʱ��
  p.FSMC_CLKDivision = 0;                   // ʱ�ӷ�Ƶ���� 
  p.FSMC_DataLatency = 0;                   //���ݲ���ʱ��
  p.FSMC_AccessMode = FSMC_AccessMode_A;    //FSMC NOR������ʱ��
  
/*--------------FSMC ���� ��������------------------------------*/
  FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM3;                   //ʹ����FSMC��BANK1���Ӱ��3 
  FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable; //��ֹ��ַ�����߸���
  FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_SRAM;            //�洢������ΪSRAM
  FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;   //�洢�����ݿ��Ϊ16λ
  FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable; //�ر�ͻ��ģʽ����
  FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait = FSMC_AsynchronousWait_Disable;  
  //�ȴ��ź����ȼ���ֻ����ʹ��ͻ������ģʽ����Ч
  FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
  FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;             //ʹ�����BANK��д����
  FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
  FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
  //ʹ��/�رյȴ���Ϣ���ã�ֻ��ʹ��ͻ������ģʽ����Ч
  FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
  FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;  //�ر�Extend Mode
  FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;      //�ر�Write Burst Mode   
  FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &p;                //������ʱ�����
  FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &p;                    //д����ʱ�����

  FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure); 

  /*!< Enable FSMC Bank1_SRAM Bank */
/*--------------ʹ��BANK1���Ӱ��3------------------------------*/
  FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM3, ENABLE);  
}

/**
  * @brief  Writes a Half-word buffer to the FSMC SRAM memory. 
  * @param  pBuffer : pointer to buffer. 
  * @param  WriteAddr : SRAM memory internal address from which the data will be 
  *         written.
  * @param  NumHalfwordToWrite : number of half-words to write. 
  * @retval None
  */
void SRAM_WriteBuffer(uint16_t* pBuffer, uint32_t WriteAddr, uint32_t NumHalfwordToWrite)
{
  for(; NumHalfwordToWrite != 0; NumHalfwordToWrite--) /*!< while there is data to write */
  {
    /*!< Transfer data to the memory */
    *(uint16_t *) (Bank1_SRAM3_ADDR + WriteAddr) = *pBuffer++;
    
    /*!< Increment the address*/  
    WriteAddr += 2;
  }   
}

/**
  * @brief  Reads a block of data from the FSMC SRAM memory.
  * @param  pBuffer : pointer to the buffer that receives the data read from the 
  *         SRAM memory.
  * @param  ReadAddr : SRAM memory internal address to read from.
  * @param  NumHalfwordToRead : number of half-words to read.
  * @retval None
  */
void SRAM_ReadBuffer(uint16_t* pBuffer, uint32_t ReadAddr, uint32_t NumHalfwordToRead)
{
  for(; NumHalfwordToRead != 0; NumHalfwordToRead--) /*!< while there is data to read */
  {
    /*!< Read a half-word from the memory */
    *pBuffer++ = *(__IO uint16_t*) (Bank1_SRAM3_ADDR + ReadAddr);

    /*!< Increment the address*/  
    ReadAddr += 2;
  }  
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */  

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

/**
  ******************************************************************************
  * @file    stm3210e_eval_lcd.c
  * @author  MCD Application Team
  * @version V4.5.0
  * @date    07-March-2011
  * @brief   This file includes the LCD driver for AM-240320L8TNQW00H 
  *          (LCD_ILI9320) and AM-240320LDTNQW00H (LCD_SPFD5408B) Liquid Crystal
  *          Display Module of STM3210E-EVAL board.
  ******************************************************************************
  * @attention
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
#include "stm3210e_eval_lcd.h"
#include "../Common/fonts.c"
#include <stdio.h>

/** @addtogroup Utilities
  * @{
  */ 

u16 DeviceCode;
/** @addtogroup STM32_EVAL
  * @{
  */ 

/** @addtogroup STM3210E_EVAL
  * @{
  */
    
/** @defgroup STM3210E_EVAL_LCD 
  * @brief This file includes the LCD driver for AM-240320L8TNQW00H 
  *        (LCD_ILI9320) and AM-240320LDTNQW00H (LCD_SPFD5408B) Liquid Crystal
  *        Display Module of STM3210E-EVAL board.
  * @{
  */ 

/** @defgroup STM3210E_EVAL_LCD_Private_TypesDefinitions
  * @{
  */ 
typedef struct
{
  __IO uint16_t LCD_REG;
  __IO uint16_t LCD_RAM;
} LCD_TypeDef;
/**
  * @}
  */ 


/** @defgroup STM3210E_EVAL_LCD_Private_Defines
  * @{
  */
/* Note: LCD /CS is CE4 - Bank 4 of NOR/SRAM Bank 1~4 */
#define LCD_BASE           ((uint32_t)(0x60000000 | 0x0C000000))
#define LCD                ((LCD_TypeDef *) LCD_BASE)
#define MAX_POLY_CORNERS   200
#define POLY_Y(Z)          ((int32_t)((Points + Z)->X))
#define POLY_X(Z)          ((int32_t)((Points + Z)->Y))                                
/**
  * @}
  */ 

/** @defgroup STM3210E_EVAL_LCD_Private_Macros
  * @{
  */
#define ABS(X)  ((X) > 0 ? (X) : -(X))    
/**
  * @}
  */ 
  
/** @defgroup STM3210E_EVAL_LCD_Private_Variables
  * @{
  */ 
static sFONT *LCD_Currentfonts;
/* Global variables to set the written text color */
static  __IO uint16_t TextColor = 0x0000, BackColor = 0xFFFF;
  
/**
  * @}
  */ 


/** @defgroup STM3210E_EVAL_LCD_Private_FunctionPrototypes
  * @{
  */ 
#ifndef USE_Delay
static void delay(vu32 nCount);
#endif /* USE_Delay*/
static void PutPixel(int16_t x, int16_t y);
static void LCD_PolyLineRelativeClosed(pPoint Points, uint16_t PointCount, uint16_t Closed);
/**
  * @}
  */ 


/** @defgroup STM3210E_EVAL_LCD_Private_Functions
  * @{
  */ 

const u8 WelcomeStr[13][72]={
    //�ﻶӭ��ʹ������ϵ�п�����
    //No:0	��   ʹ��Ƶ��=1
    {	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x10,0x00,0x00,0x18,0x00,0x00,0x18,0x00,0x00,
    0x18,0x00,0x00,0x3C,0x00,0x00,0x3C,0x00,0x00,0x7C,0x00,0x3F,0xFF,0xFC,0x1F,0xFF,0xF8,0x07,0xFF,
    0xE0,0x03,0xFF,0xC0,0x01,0xFF,0x80,0x01,0xFF,0x80,0x01,0xFF,0x80,0x01,0xFF,0x80,0x03,0xE7,0xC0,
    0x03,0x81,0xC0,0x03,0x00,0xC0,0x04,0x00,0x60,0x00,0x00,0x00,0x00,0x00,0x00},
    //No:1	��   ʹ��Ƶ��=1
    {	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x06,0x00,0x00,0x46,0x00,0x3F,0xC4,0x00,0x00,
    0xC4,0x04,0x00,0x8F,0xFE,0x20,0x88,0x08,0x11,0x89,0x90,0x09,0x11,0x80,0x05,0x21,0x80,0x02,0x21,
    0x80,0x03,0x03,0x80,0x05,0x82,0x80,0x05,0x82,0x40,0x08,0xC2,0x40,0x08,0xC4,0x60,0x10,0x4C,0x20,
    0x20,0x08,0x30,0x40,0x10,0x18,0x00,0x60,0x0E,0x00,0x80,0x00,0x00,0x00,0x00},
    //No:2	ӭ   ʹ��Ƶ��=1
    {	0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x02,0x00,0x0C,0x0E,0x00,0x06,0x72,0x08,0x04,0x41,0xFC,0x00,
    0x41,0x08,0x00,0x41,0x08,0x04,0x41,0x08,0x7E,0x41,0x08,0x04,0x41,0x08,0x04,0x41,0x08,0x04,0x41,
    0x08,0x04,0x45,0x08,0x04,0x59,0x08,0x04,0x61,0x78,0x04,0x41,0x18,0x04,0x01,0x00,0x1A,0x01,0x00,
    0x71,0x00,0x00,0x60,0xE0,0x02,0x00,0x3F,0xFC,0x00,0x00,0x00,0x00,0x00,0x00},
    //No:3	��   ʹ��Ƶ��=1
    {	0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x18,0x00,0x06,0x18,0x00,0x04,0x30,0x00,0x0C,0x3F,0xFC,0x0C,
    0x44,0x10,0x14,0x43,0x20,0x24,0x82,0x00,0x44,0x32,0x40,0x04,0x22,0x20,0x04,0x42,0x18,0x04,0x82,
    0x18,0x05,0x1E,0x08,0x04,0x06,0x00,0x00,0x20,0x00,0x01,0x98,0x20,0x09,0x8C,0x10,0x09,0x88,0x4C,
    0x19,0x80,0x4C,0x31,0x80,0x44,0x01,0x80,0xE0,0x00,0xFF,0xC0,0x00,0x00,0x00},
    //No:4	ʹ   ʹ��Ƶ��=1
    {	0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x81,0x00,0x03,0x01,0x00,0x03,0x01,0x04,0x02,0xFF,0xFE,0x06,
    0x01,0x00,0x04,0x01,0x00,0x0E,0x21,0x18,0x0E,0x3F,0xE8,0x16,0x21,0x08,0x26,0x21,0x08,0x26,0x21,
    0x08,0x46,0x3F,0xF8,0x06,0x21,0x00,0x06,0x13,0x00,0x06,0x12,0x00,0x06,0x0A,0x00,0x06,0x06,0x00,
    0x06,0x06,0x00,0x06,0x0F,0x00,0x06,0x10,0xE0,0x06,0x60,0x3C,0x01,0x80,0x00},
    //No:5	��   ʹ��Ƶ��=1
    {	0x00,0x00,0x00,0x00,0x00,0x00,0x08,0x00,0x10,0x0F,0xFF,0xF8,0x0C,0x08,0x10,0x0C,0x08,0x10,0x0C,
    0x08,0x10,0x0C,0x08,0x10,0x0C,0x08,0x10,0x0F,0xFF,0xF0,0x08,0x08,0x10,0x08,0x08,0x10,0x08,0x08,
    0x10,0x08,0x08,0x10,0x0F,0xFF,0xF0,0x08,0x08,0x10,0x08,0x08,0x10,0x08,0x08,0x10,0x10,0x18,0x10,
    0x10,0x18,0x10,0x20,0x18,0x10,0x20,0x18,0xF0,0x40,0x00,0x30,0x00,0x00,0x00},
    //No:6	��   ʹ��Ƶ��=1
    {	0x00,0x00,0x00,0x00,0x01,0x00,0x04,0x00,0x80,0x02,0x00,0x80,0x02,0x00,0x80,0x00,0x00,0x88,0x00,
    0x9F,0xFC,0x3F,0x90,0x88,0x01,0x10,0x88,0x03,0x10,0x88,0x02,0x1F,0xF8,0x06,0x10,0x88,0x0D,0x90,
    0x88,0x0C,0xD0,0x88,0x14,0x50,0x8C,0x24,0x1F,0xFC,0x44,0x10,0x88,0x04,0x00,0x80,0x04,0x00,0x80,
    0x04,0x00,0x80,0x04,0x00,0x80,0x04,0x00,0x80,0x04,0x01,0x80,0x00,0x01,0x00},
    //No:7	��   ʹ��Ƶ��=1
    {	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x00,0x00,0x10,0x00,0x00,0x20,0x00,0x03,0xFF,0xE0,0x02,
    0x00,0x40,0x02,0x20,0x40,0x02,0x10,0x40,0x02,0x18,0x40,0x02,0x08,0x40,0x02,0x00,0x40,0x02,0x00,
    0x4E,0x3F,0xFF,0xF0,0x02,0x00,0x40,0x02,0x20,0x40,0x02,0x10,0x40,0x06,0x18,0x40,0x06,0x18,0x40,
    0x04,0x00,0x40,0x08,0x00,0x40,0x10,0x07,0xC0,0x20,0x01,0x80,0x40,0x00,0x00},
    //No:8	ϵ   ʹ��Ƶ��=1
    {	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF8,0x00,0x7F,0x00,0x1F,0x98,0x00,0x00,0x30,0x00,0x00,
    0x40,0x40,0x00,0x80,0xC0,0x07,0x3F,0x00,0x07,0xC4,0x00,0x00,0x18,0x00,0x00,0x60,0x40,0x00,0x80,
    0x20,0x07,0x7F,0xF8,0x07,0x88,0x18,0x01,0x08,0x00,0x00,0xC8,0x80,0x01,0x88,0x60,0x02,0x08,0x38,
    0x04,0x08,0x1C,0x18,0x08,0x0C,0x20,0xF8,0x0C,0x00,0x18,0x00,0x00,0x00,0x00},
    //No:9	��   ʹ��Ƶ��=1
    {	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0C,0x00,0x0C,0x08,0x3F,0xF0,0x08,0x01,0x00,0x08,0x03,
    0x01,0x88,0x02,0x00,0x88,0x02,0x18,0x88,0x07,0xF8,0x88,0x04,0x10,0x88,0x0C,0x30,0x88,0x0A,0x20,
    0x88,0x13,0x20,0x88,0x21,0x60,0x88,0x01,0x40,0x88,0x00,0xC0,0x88,0x00,0x81,0x88,0x01,0x01,0x08,
    0x02,0x00,0x08,0x04,0x00,0x08,0x18,0x00,0x78,0x20,0x00,0x18,0x00,0x00,0x00},
    //No:10	��   ʹ��Ƶ��=1
    {	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x1F,0xFF,0xE0,0x00,0x81,0x80,0x00,
    0x81,0x80,0x00,0x81,0x80,0x00,0x81,0x80,0x00,0x81,0x80,0x00,0x81,0x84,0x00,0x81,0x8C,0x1F,0xFF,
    0xF0,0x00,0x81,0x80,0x00,0x81,0x80,0x00,0x81,0x80,0x01,0x81,0x80,0x01,0x01,0x80,0x01,0x01,0x80,
    0x02,0x01,0x80,0x04,0x01,0x80,0x08,0x01,0x80,0x30,0x01,0x80,0x00,0x00,0x00},
    //No:11	��   ʹ��Ƶ��=1
    {	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x31,0x00,0x04,0x60,0xC0,0x0C,0x60,0x60,0x0C,0x60,0x60,0x08,
    0x40,0x00,0x18,0x40,0x0C,0x17,0xFF,0xF0,0x00,0xC0,0x00,0x00,0xC0,0x00,0x00,0xFF,0xE0,0x00,0xA0,
    0x60,0x01,0xA0,0x40,0x01,0x10,0x80,0x03,0x09,0x80,0x02,0x09,0x00,0x06,0x06,0x00,0x04,0x06,0x00,
    0x08,0x0D,0x80,0x10,0x30,0xE0,0x20,0x40,0x3E,0x43,0x80,0x08,0x00,0x00,0x00},
    //No:12	��   ʹ��Ƶ��=1
    {	0x00,0x00,0x00,0x00,0x00,0x00,0x06,0x00,0x30,0x04,0x00,0xF8,0x04,0x1F,0x00,0x04,0x10,0x00,0x04,
    0x90,0x00,0x3F,0x10,0x00,0x0C,0x10,0x10,0x0C,0x1F,0xF0,0x0E,0x14,0x30,0x0D,0x14,0x20,0x1C,0x94,
    0x20,0x14,0xB2,0x60,0x14,0x32,0x40,0x24,0x23,0xC0,0x44,0x21,0x80,0x44,0x21,0x80,0x04,0x23,0xC0,
    0x04,0x46,0x60,0x04,0x88,0x38,0x04,0xB0,0x1C,0x05,0xC0,0x00,0x00,0x00,0x00},
};

/**
  * @brief  Draws a chinacharacter on LCD.
  * @param  Xpos: the Line where to display the character shape.
  * @param  Ypos: start column address.
  * @param  c: pointer to the character data.
  * @retval None
  */
void LCD_DrawChinaChar(u8 Xpos, u16 Ypos, const u8 *c)
{
  u32 index = 0, i = 0, j = 0;
  u8 Xaddress = 0;
   
  Xaddress = Xpos;
  
  LCD_SetCursor(Xaddress, Ypos);

  for(index = 0; index < 24; index++)
  {
    LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
    for(j = 0; j < 3; j++)
    {
        for(i = 0; i < 8; i++)
        {
          if((c[3*index + j] & (0x80 >> i)) == 0x00)
          {
              LCD_WriteRAM(0xF800);
          }
          else
          {
            LCD_WriteRAM(0xFFE0);
          }
        }   
     }

    Xaddress++;
    LCD_SetCursor(Xaddress, Ypos);

  }
}

void LCD_DisplayWelcomeStr(u8 Line)
{
  u16 num = 0;

  /* Send the string character by character on LCD */
  for(num=0; num<13; num++)
  {
    /* Display one China character on LCD */
    LCD_DrawChinaChar(Line, num*24+4, (u8 *)WelcomeStr[num]);
  }
}

/**
  * @brief  DeInitializes the LCD.
  * @param  None
  * @retval None
  */
void LCD_DeInit(void)
{ 
  GPIO_InitTypeDef GPIO_InitStructure;

  /*!< LCD Display Off */
  LCD_DisplayOff();

  /* BANK 4 (of NOR/SRAM Bank 1~4) is disabled */
  FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM4, ENABLE);
  
  /*!< LCD_SPI DeInit */
  FSMC_NORSRAMDeInit(FSMC_Bank1_NORSRAM4);
   
  /* Set PD.00(D2), PD.01(D3), PD.04(NOE), PD.05(NWE), PD.08(D13), PD.09(D14),
     PD.10(D15), PD.14(D0), PD.15(D1) as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 |
                                GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_14 | 
                                GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  /* Set PE.07(D4), PE.08(D5), PE.09(D6), PE.10(D7), PE.11(D8), PE.12(D9), PE.13(D10),
     PE.14(D11), PE.15(D12) as alternate function push pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | 
                                GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | 
                                GPIO_Pin_15;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  /* Set PF.00(A0 (RS)) as alternate function push pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_Init(GPIOF, &GPIO_InitStructure);
  /* Set PG.12(NE4 (LCD/CS)) as alternate function push pull - CE3(LCD /CS) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_Init(GPIOG, &GPIO_InitStructure); 
}

/**
  * @brief  Initializes the LCD.
  * @param  None
  * @retval None
  */
void STM3210E_LCD_Init(void)
{ 
  u16 StartX;
  u8 i;
#if LCD_DEFAULT_FONT == Font16x24  
  const u8 str[]       = "  www.armjishu.com  ";
  const u8 ID8989str[] = " LCD SSD1289,ID8989 ";
#else
  const u8 str[]       = " Welcome to www.armjishu.com  ";
  const u8 ID8989str[] = "  LCD is SSD1289, ID is 8989  ";
#endif

  u8 len = sizeof(str)-1;

/* Configure the LCD Control pins --------------------------------------------*/
  LCD_CtrlLinesConfig();
/* Configure the FSMC Parallel interface -------------------------------------*/
  LCD_FSMCConfig();
  
  _delay_(5); /* delay 50 ms */
  DeviceCode = LCD_ReadReg(0x0000);
  _delay_(5); /* delay 50 ms */
  DeviceCode = LCD_ReadReg(0x0000);  
  printf("\n\r ###### LCD DeviceCode = LCD_ReadReg(0x0000) = 0x%x ###### ", DeviceCode);
  printf("\n\r ###### LCD DeviceCode = LCD->LCD_RAM = 0x%x ###### ", LCD->LCD_RAM);
  printf("\n\r ###### &LCD->LCD_REG =  0x%x ,  &LCD->LCD_RAM =  0x%x ###### ",  &LCD->LCD_REG,  &LCD->LCD_RAM);
  printf("\n\r ###### LCD DeviceCode = LCD_ReadReg(0x0000) = 0x%x ###### ", LCD_ReadReg(0x0000));
  
  /* Check if the LCD is SPFD5408B Controller */
  if(DeviceCode == 0x5408)
  {
    /* Start Initial Sequence ------------------------------------------------*/
    LCD_WriteReg(LCD_REG_1, 0x0100);  /* Set SS bit */
    LCD_WriteReg(LCD_REG_2, 0x0700);  /* Set 1 line inversion */
    LCD_WriteReg(LCD_REG_3, 0x1030);  /* Set GRAM write direction and BGR=1. */
    LCD_WriteReg(LCD_REG_4, 0x0000);  /* Resize register */
    LCD_WriteReg(LCD_REG_8, 0x0202);  /* Set the back porch and front porch */
    LCD_WriteReg(LCD_REG_9, 0x0000);  /* Set non-display area refresh cycle ISC[3:0] */
    LCD_WriteReg(LCD_REG_10, 0x0000); /* FMARK function */
    LCD_WriteReg(LCD_REG_12, 0x0000); /* RGB 18-bit System interface setting */
    LCD_WriteReg(LCD_REG_13, 0x0000); /* Frame marker Position */
    LCD_WriteReg(LCD_REG_15, 0x0000); /* RGB interface polarity, no impact */
    /* Power On sequence -----------------------------------------------------*/
    LCD_WriteReg(LCD_REG_16, 0x0000); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
    LCD_WriteReg(LCD_REG_17, 0x0000); /* DC1[2:0], DC0[2:0], VC[2:0] */
    LCD_WriteReg(LCD_REG_18, 0x0000); /* VREG1OUT voltage */
    LCD_WriteReg(LCD_REG_19, 0x0000); /* VDV[4:0] for VCOM amplitude */
    _delay_(20);                 /* Dis-charge capacitor power voltage (200ms) */
    LCD_WriteReg(LCD_REG_17, 0x0007);  /* DC1[2:0], DC0[2:0], VC[2:0] */
    _delay_(5);                   /* Delay 50 ms */
    LCD_WriteReg(LCD_REG_16, 0x12B0);  /* SAP, BT[3:0], AP, DSTB, SLP, STB */
    _delay_(5);                  /* Delay 50 ms */
    LCD_WriteReg(LCD_REG_18, 0x01BD);  /* External reference voltage= Vci */
    _delay_(5); 
    LCD_WriteReg(LCD_REG_19, 0x1400);  /* VDV[4:0] for VCOM amplitude */
    LCD_WriteReg(LCD_REG_41, 0x000E);  /* VCM[4:0] for VCOMH */
    _delay_(5);                   /* Delay 50 ms */
    LCD_WriteReg(LCD_REG_32, 0x0000); /* GRAM horizontal Address */
    LCD_WriteReg(LCD_REG_33, 0x013F); /* GRAM Vertical Address */
    /* Adjust the Gamma Curve (SPFD5408B)-------------------------------------*/
    LCD_WriteReg(LCD_REG_48, 0x0b0d);
    LCD_WriteReg(LCD_REG_49, 0x1923);
    LCD_WriteReg(LCD_REG_50, 0x1c26);
    LCD_WriteReg(LCD_REG_51, 0x261c);
    LCD_WriteReg(LCD_REG_52, 0x2419);
    LCD_WriteReg(LCD_REG_53, 0x0d0b);
    LCD_WriteReg(LCD_REG_54, 0x1006);
    LCD_WriteReg(LCD_REG_55, 0x0610);
    LCD_WriteReg(LCD_REG_56, 0x0706);
    LCD_WriteReg(LCD_REG_57, 0x0304);
    LCD_WriteReg(LCD_REG_58, 0x0e05);
    LCD_WriteReg(LCD_REG_59, 0x0e01);
    LCD_WriteReg(LCD_REG_60, 0x010e);
    LCD_WriteReg(LCD_REG_61, 0x050e);
    LCD_WriteReg(LCD_REG_62, 0x0403);
    LCD_WriteReg(LCD_REG_63, 0x0607);
    /* Set GRAM area ---------------------------------------------------------*/
    LCD_WriteReg(LCD_REG_80, 0x0000); /* Horizontal GRAM Start Address */
    LCD_WriteReg(LCD_REG_81, 0x00EF); /* Horizontal GRAM End Address */
    LCD_WriteReg(LCD_REG_82, 0x0000); /* Vertical GRAM Start Address */
    LCD_WriteReg(LCD_REG_83, 0x013F); /* Vertical GRAM End Address */
    LCD_WriteReg(LCD_REG_96,  0xA700); /* Gate Scan Line */
    LCD_WriteReg(LCD_REG_97,  0x0001); /* NDL, VLE, REV */
    LCD_WriteReg(LCD_REG_106, 0x0000); /* set scrolling line */
    /* Partial Display Control -----------------------------------------------*/
    LCD_WriteReg(LCD_REG_128, 0x0000);
    LCD_WriteReg(LCD_REG_129, 0x0000);
    LCD_WriteReg(LCD_REG_130, 0x0000);
    LCD_WriteReg(LCD_REG_131, 0x0000);
    LCD_WriteReg(LCD_REG_132, 0x0000);
    LCD_WriteReg(LCD_REG_133, 0x0000);
    /* Panel Control ---------------------------------------------------------*/
    LCD_WriteReg(LCD_REG_144, 0x0010); 
    LCD_WriteReg(LCD_REG_146, 0x0000);
    LCD_WriteReg(LCD_REG_147, 0x0003);
    LCD_WriteReg(LCD_REG_149, 0x0110);
    LCD_WriteReg(LCD_REG_151, 0x0000);
    LCD_WriteReg(LCD_REG_152, 0x0000);
    /* Set GRAM write direction and BGR=1
       I/D=01 (Horizontal : increment, Vertical : decrement)
       AM=1 (address is updated in vertical writing direction) */
    LCD_WriteReg(LCD_REG_3, 0x1018);
    LCD_WriteReg(LCD_REG_7, 0x0112); /* 262K color and display ON */
    LCD_SetFont(&LCD_DEFAULT_FONT);
    //return;
  }
  else if(DeviceCode==0x8989)
  {
    // power supply setting 
    // set R07h at 0021h (GON=1,DTE=0,D[1:0]=01)
    //LCD_WriteReg(0x0007,0x0021);       
    // set R00h at 0001h (OSCEN=1)       
    LCD_WriteReg(0x0000,0x0001);       
    // set R07h at 0023h (GON=1,DTE=0,D[1:0]=11)       
    // LCD_WriteReg(0x0007,0x0023);       
    // set R10h at 0000h (Exit sleep mode)       
    LCD_WriteReg(0x0010,0x0000);       
    _delay_(5);                  /* Delay 50 ms */      
    // set R07h at 0033h (GON=1,DTE=1,D[1:0]=11)       
    //LCD_WriteReg(0x0007,0x0033);       
	LCD_WriteReg(0x0007,0x0233);       
    // Entry mode setting (R11h)       
    // R11H Entry mode       
    // vsmode DFM1 DFM0 TRANS OEDef WMode DMode1 DMode0 TY1 TY0 ID1 ID0 AM LG2 LG2 LG0       
    //   0     1    1     0     0     0     0      0     0   1   1   1  *   0   0   0       
    //LCD_WriteReg(0x0011,0x6070);       
	LCD_WriteReg(0x0011,0x6078);       
    // LCD driver AC setting (R02h)       
    LCD_WriteReg(0x0002,0x0600);       
    // power control 1       
    // DCT3 DCT2 DCT1 DCT0 BT2 BT1 BT0 0 DC3 DC2 DC1 DC0 AP2 AP1 AP0 0       
    // 1     0    1    0    1   0   0  0  1   0   1   0   0   1   0  0       
    // DCT[3:0] fosc/4 BT[2:0]  DC{3:0] fosc/4       
    LCD_WriteReg(0x0003,0xA8A4);//0x0804  
    LCD_WriteReg(0x000C,0x0000);//
    LCD_WriteReg(0x000D,0x080C);//       
    // power control 4       
    // 0 0 VCOMG VDV4 VDV3 VDV2 VDV1 VDV0 0 0 0 0 0 0 0 0       
    // 0 0   1    0    1    0    1    1   0 0 0 0 0 0 0 0       
    LCD_WriteReg(0x000E,0x2900);       
    LCD_WriteReg(0x001E,0x00B8);       
    LCD_WriteReg(0x0001,0x293F);
    LCD_WriteReg(0x0010,0x0000);       
    LCD_WriteReg(0x0005,0x0000);       
    LCD_WriteReg(0x0006,0x0000);       
    LCD_WriteReg(0x0016,0xEF1C);     
    LCD_WriteReg(0x0017,0x0003);     
    LCD_WriteReg(0x0007,0x0233);		//0x0233       
    LCD_WriteReg(0x000B,0x0000|(3<<6));  //////     
    LCD_WriteReg(0x000F,0x0000);		//ɨ�迪ʼ��ַ
    LCD_WriteReg(0x0041,0x0000);     
    LCD_WriteReg(0x0042,0x0000);     
    LCD_WriteReg(0x0048,0x0000);     
    LCD_WriteReg(0x0049,0x013F);     
    LCD_WriteReg(0x004A,0x0000);     
    LCD_WriteReg(0x004B,0x0000);     
    LCD_WriteReg(0x0044,0xEF00);     
    LCD_WriteReg(0x0045,0x0000);     
    LCD_WriteReg(0x0046,0x013F);     
    LCD_WriteReg(0x0030,0x0707);     
    LCD_WriteReg(0x0031,0x0204);     
    LCD_WriteReg(0x0032,0x0204);     
    LCD_WriteReg(0x0033,0x0502);     
    LCD_WriteReg(0x0034,0x0507);     
    LCD_WriteReg(0x0035,0x0204);     
    LCD_WriteReg(0x0036,0x0204);     
    LCD_WriteReg(0x0037,0x0502);     
    LCD_WriteReg(0x003A,0x0302);     
    LCD_WriteReg(0x003B,0x0302);     
    LCD_WriteReg(0x0023,0x0000);     
    LCD_WriteReg(0x0024,0x0000);     
    LCD_WriteReg(0x0025,0x8000);     
    LCD_WriteReg(0x004e,0);        //��(X)��ַ0
    LCD_WriteReg(0x004f,0);        //��(Y)��ַ0
  }
  else if(DeviceCode==0x9320||DeviceCode==0x9300)
  {
    LCD_WriteReg(0x00,0x0000);
    LCD_WriteReg(0x01,0x0100);	//Driver Output Contral.
    LCD_WriteReg(0x02,0x0700);	//LCD Driver Waveform Contral.
    //		LCD_WriteReg(0x03,0x1030);	//Entry Mode Set.
    LCD_WriteReg(0x03,0x1018);	//Entry Mode Set.
    LCD_WriteReg(0x04,0x0000);	//Scalling Contral.
    LCD_WriteReg(0x08,0x0202);	//Display Contral 2.(0x0207)
    LCD_WriteReg(0x09,0x0000);	//Display Contral 3.(0x0000)
    LCD_WriteReg(0x0a,0x0000);	//Frame Cycle Contal.(0x0000)
    LCD_WriteReg(0x0c,(1<<0));	//Extern Display Interface Contral 1.(0x0000)
    LCD_WriteReg(0x0d,0x0000);	//Frame Maker Position.
    LCD_WriteReg(0x0f,0x0000);	//Extern Display Interface Contral 2.
    _delay_(5);                  /* Delay 50 ms */
    LCD_WriteReg(0x07,0x0101);	//Display Contral.
    _delay_(5);                  /* Delay 50 ms */
    LCD_WriteReg(0x10,(1<<12)|(0<<8)|(1<<7)|(1<<6)|(0<<4));	//Power Control 1.(0x16b0)
    LCD_WriteReg(0x11,0x0007);								//Power Control 2.(0x0001)
    LCD_WriteReg(0x12,(1<<8)|(1<<4)|(0<<0));					//Power Control 3.(0x0138)
    LCD_WriteReg(0x13,0x0b00);								//Power Control 4.
    LCD_WriteReg(0x29,0x0000);								//Power Control 7.
    
    LCD_WriteReg(0x2b,(1<<14)|(1<<4));
    
    LCD_WriteReg(0x50,0);		//Set X Start.
    LCD_WriteReg(0x51,239);	//Set X End.
    LCD_WriteReg(0x52,0);		//Set Y Start.
    LCD_WriteReg(0x53,319);	//Set Y End.
    
    LCD_WriteReg(0x60,0x2700);	//Driver Output Control.
    LCD_WriteReg(0x61,0x0001);	//Driver Output Control.
    LCD_WriteReg(0x6a,0x0000);	//Vertical Srcoll Control.
    
    LCD_WriteReg(0x80,0x0000);	//Display Position? Partial Display 1.
    LCD_WriteReg(0x81,0x0000);	//RAM Address Start? Partial Display 1.
    LCD_WriteReg(0x82,0x0000);	//RAM Address End-Partial Display 1.
    LCD_WriteReg(0x83,0x0000);	//Displsy Position? Partial Display 2.
    LCD_WriteReg(0x84,0x0000);	//RAM Address Start? Partial Display 2.
    LCD_WriteReg(0x85,0x0000);	//RAM Address End? Partial Display 2.
    
    LCD_WriteReg(0x90,(0<<7)|(16<<0));	//Frame Cycle Contral.(0x0013)
    LCD_WriteReg(0x92,0x0000);	//Panel Interface Contral 2.(0x0000)
    LCD_WriteReg(0x93,0x0001);	//Panel Interface Contral 3.
    LCD_WriteReg(0x95,0x0110);	//Frame Cycle Contral.(0x0110)
    LCD_WriteReg(0x97,(0<<8));	//
    LCD_WriteReg(0x98,0x0000);	//Frame Cycle Contral.
    LCD_WriteReg(0x07,0x0173);	//(0x0173)
  }
  else
  {
    printf("\n\r ###### Err: Unknow DeviceCode 0x%x ###### ", DeviceCode);
  /* Start Initial Sequence ----------------------------------------------------*/
    LCD_WriteReg(LCD_REG_229,0x8000); /* Set the internal vcore voltage */
    LCD_WriteReg(LCD_REG_0,  0x0001); /* Start internal OSC. */
    LCD_WriteReg(LCD_REG_1,  0x0100); /* set SS and SM bit */
    LCD_WriteReg(LCD_REG_2,  0x0700); /* set 1 line inversion */
    LCD_WriteReg(LCD_REG_3,  0x1030); /* set GRAM write direction and BGR=1. */
    LCD_WriteReg(LCD_REG_4,  0x0000); /* Resize register */
    LCD_WriteReg(LCD_REG_8,  0x0202); /* set the back porch and front porch */
    LCD_WriteReg(LCD_REG_9,  0x0000); /* set non-display area refresh cycle ISC[3:0] */
    LCD_WriteReg(LCD_REG_10, 0x0000); /* FMARK function */
    LCD_WriteReg(LCD_REG_12, 0x0000); /* RGB interface setting */
    LCD_WriteReg(LCD_REG_13, 0x0000); /* Frame marker Position */
    LCD_WriteReg(LCD_REG_15, 0x0000); /* RGB interface polarity */
  /* Power On sequence ---------------------------------------------------------*/
    LCD_WriteReg(LCD_REG_16, 0x0000); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
    LCD_WriteReg(LCD_REG_17, 0x0000); /* DC1[2:0], DC0[2:0], VC[2:0] */
    LCD_WriteReg(LCD_REG_18, 0x0000); /* VREG1OUT voltage */
    LCD_WriteReg(LCD_REG_19, 0x0000); /* VDV[4:0] for VCOM amplitude */
    _delay_(20);                 /* Dis-charge capacitor power voltage (200ms) */
    LCD_WriteReg(LCD_REG_16, 0x17B0); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
    LCD_WriteReg(LCD_REG_17, 0x0137); /* DC1[2:0], DC0[2:0], VC[2:0] */
    _delay_(5);                  /* Delay 50 ms */
    LCD_WriteReg(LCD_REG_18, 0x0139); /* VREG1OUT voltage */
    _delay_(5);                  /* Delay 50 ms */
    LCD_WriteReg(LCD_REG_19, 0x1d00); /* VDV[4:0] for VCOM amplitude */
    LCD_WriteReg(LCD_REG_41, 0x0013); /* VCM[4:0] for VCOMH */
    _delay_(5);                  /* Delay 50 ms */
    LCD_WriteReg(LCD_REG_32, 0x0000); /* GRAM horizontal Address */
    LCD_WriteReg(LCD_REG_33, 0x0000); /* GRAM Vertical Address */
  /* Adjust the Gamma Curve ----------------------------------------------------*/
    LCD_WriteReg(LCD_REG_48, 0x0006);
    LCD_WriteReg(LCD_REG_49, 0x0101);
    LCD_WriteReg(LCD_REG_50, 0x0003);
    LCD_WriteReg(LCD_REG_53, 0x0106);
    LCD_WriteReg(LCD_REG_54, 0x0b02);
    LCD_WriteReg(LCD_REG_55, 0x0302);
    LCD_WriteReg(LCD_REG_56, 0x0707);
    LCD_WriteReg(LCD_REG_57, 0x0007);
    LCD_WriteReg(LCD_REG_60, 0x0600);
    LCD_WriteReg(LCD_REG_61, 0x020b);
    
  /* Set GRAM area -------------------------------------------------------------*/
    LCD_WriteReg(LCD_REG_80, 0x0000); /* Horizontal GRAM Start Address */
    LCD_WriteReg(LCD_REG_81, 0x00EF); /* Horizontal GRAM End Address */
    LCD_WriteReg(LCD_REG_82, 0x0000); /* Vertical GRAM Start Address */
    LCD_WriteReg(LCD_REG_83, 0x013F); /* Vertical GRAM End Address */
    LCD_WriteReg(LCD_REG_96,  0x2700); /* Gate Scan Line */
    LCD_WriteReg(LCD_REG_97,  0x0001); /* NDL,VLE, REV */
    LCD_WriteReg(LCD_REG_106, 0x0000); /* set scrolling line */
  /* Partial Display Control ---------------------------------------------------*/
    LCD_WriteReg(LCD_REG_128, 0x0000);
    LCD_WriteReg(LCD_REG_129, 0x0000);
    LCD_WriteReg(LCD_REG_130, 0x0000);
    LCD_WriteReg(LCD_REG_131, 0x0000);
    LCD_WriteReg(LCD_REG_132, 0x0000);
    LCD_WriteReg(LCD_REG_133, 0x0000);
  /* Panel Control -------------------------------------------------------------*/
    LCD_WriteReg(LCD_REG_144, 0x0010);
    LCD_WriteReg(LCD_REG_146, 0x0000);
    LCD_WriteReg(LCD_REG_147, 0x0003);
    LCD_WriteReg(LCD_REG_149, 0x0110);
    LCD_WriteReg(LCD_REG_151, 0x0000);
    LCD_WriteReg(LCD_REG_152, 0x0000);
    /* Set GRAM write direction and BGR = 1 */
    /* I/D=01 (Horizontal : increment, Vertical : decrement) */
    /* AM=1 (address is updated in vertical writing direction) */
    LCD_WriteReg(LCD_REG_3, 0x1018);
    LCD_WriteReg(LCD_REG_7, 0x0173); /* 262K color and display ON */  
    LCD_SetFont(&LCD_DEFAULT_FONT); 
  }
  LCD_SetFont(&LCD_DEFAULT_FONT);
  LCD_SetColors(LCD_COLOR_YELLOW, LCD_COLOR_RED);

  printf("\n\r ###### LCD_PIXEL_WIDTH %d; LCD_PIXEL_HEIGHT %d. ###### ", LCD_PIXEL_WIDTH, LCD_PIXEL_HEIGHT);

  LCD_Clear(0x07e0);
  LCD_DisplayWelcomeStr(0x60);

  StartX = (320 - LCD_DEFAULT_FONT.Width*len)/2;
  for (i=0;i<len;i++)
  {
      LCD_DisplayChar(60, (StartX+LCD_DEFAULT_FONT.Width*i), str[i]);
  }
  
  if(DeviceCode==0x8989)
  {
      for (i=0;i<len;i++)
      {
          LCD_DisplayChar(144, (StartX+LCD_DEFAULT_FONT.Width*i), ID8989str[i]);
      }
  }
  
  _delay_(160);
  
}

/**
  * @brief  Sets the LCD Text and Background colors.
  * @param  _TextColor: specifies the Text Color.
  * @param  _BackColor: specifies the Background Color.
  * @retval None
  */
void LCD_SetColors(__IO uint16_t _TextColor, __IO uint16_t _BackColor)
{
  TextColor = _TextColor; 
  BackColor = _BackColor;
}

/**
  * @brief  Gets the LCD Text and Background colors.
  * @param  _TextColor: pointer to the variable that will contain the Text 
            Color.
  * @param  _BackColor: pointer to the variable that will contain the Background 
            Color.
  * @retval None
  */
void LCD_GetColors(__IO uint16_t *_TextColor, __IO uint16_t *_BackColor)
{
  *_TextColor = TextColor; *_BackColor = BackColor;
}

/**
  * @brief  Sets the Text color.
  * @param  Color: specifies the Text color code RGB(5-6-5).
  * @retval None
  */
void LCD_SetTextColor(__IO uint16_t Color)
{
  TextColor = Color;
}


/**
  * @brief  Sets the Background color.
  * @param  Color: specifies the Background color code RGB(5-6-5).
  * @retval None
  */
void LCD_SetBackColor(__IO uint16_t Color)
{
  BackColor = Color;
}

/**
  * @brief  Sets the Text Font.
  * @param  fonts: specifies the font to be used.
  * @retval None
  */
void LCD_SetFont(sFONT *fonts)
{
  LCD_Currentfonts = fonts;
}

/**
  * @brief  Gets the Text Font.
  * @param  None.
  * @retval the used font.
  */
sFONT *LCD_GetFont(void)
{
  return LCD_Currentfonts;
}

/**
  * @brief  Clears the selected line.
  * @param  Line: the Line to be cleared.
  *   This parameter can be one of the following values:
  *     @arg Linex: where x can be 0..n
  * @retval None
  */
void LCD_ClearLine(uint8_t Line)
{
  uint16_t refcolumn = 0;//LCD_PIXEL_WIDTH - 1;
  /* Send the string character by character on lCD */
  while ((refcolumn &0xFFFF) < LCD_PIXEL_WIDTH)
  {
    /* Display one character on LCD */
    LCD_DisplayChar(Line, refcolumn, ' ');
    /* Decrement the column position by 16 */
    refcolumn += LCD_Currentfonts->Width;
  }
}


/**
  * @brief  Clears the hole LCD.
  * @param  Color: the color of the background.
  * @retval None
  */
void LCD_Clear(uint16_t Color)
{
  uint32_t index = 0;
  
  LCD_SetCursor(0x00, 0x013F); 
  LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
  for(index = 0; index < 76800; index++)
  {
    LCD->LCD_RAM = Color;
  }  
}


/**
  * @brief  Sets the cursor position.
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position. 
  * @retval None
  */
void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos)
{
	if(DeviceCode==0x8989)
	{
	 	LCD_WriteReg(0x004e,Xpos);        //��
    	LCD_WriteReg(0x004f,Ypos);  //��
	}
	else if(DeviceCode==0x9919)
	{
		LCD_WriteReg(0x004e,Xpos); // ��
  		LCD_WriteReg(0x004f,Ypos); // ��	
	}
	else
	{
  		LCD_WriteReg(0x0020,Ypos); // ��
  		LCD_WriteReg(0x0021,0x13f-Xpos); // ��
	}
  //LCD_WriteReg(LCD_REG_32, Xpos);
  //LCD_WriteReg(LCD_REG_33, Ypos);
}


/**
  * @brief  Draws a character on LCD.
  * @param  Xpos: the Line where to display the character shape.
  * @param  Ypos: start column address.
  * @param  c: pointer to the character data.
  * @retval None
  */
void LCD_DrawChar(uint8_t Xpos, uint16_t Ypos, const uint16_t *c)
{
  uint32_t index = 0, i = 0;
  uint8_t Xaddress = 0;
   
  Xaddress = Xpos;
  
  LCD_SetCursor(Xaddress, Ypos);
  
  for(index = 0; index < LCD_Currentfonts->Height; index++)
  {
    LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
    for(i = 0; i < LCD_Currentfonts->Width; i++)
    {
      if((((c[index] & ((0x80 << ((LCD_Currentfonts->Width / 12 ) * 8 ) ) >> i)) == 0x00) &&(LCD_Currentfonts->Width <= 12))||
        (((c[index] & (0x1 << i)) == 0x00)&&(LCD_Currentfonts->Width > 12 )))

      {
        LCD_WriteRAM(BackColor);
      }
      else
      {
        LCD_WriteRAM(TextColor);
      }
    }
    Xaddress++;
    LCD_SetCursor(Xaddress, Ypos);
  }
}


/**
  * @brief  Displays one character (16dots width, 24dots height).
  * @param  Line: the Line where to display the character shape .
  *   This parameter can be one of the following values:
  *     @arg Linex: where x can be 0..9
  * @param  Column: start column address.
  * @param  Ascii: character ascii code, must be between 0x20 and 0x7E.
  * @retval None
  */
void LCD_DisplayChar(uint8_t Line, uint16_t Column, uint8_t Ascii)
{
  Ascii -= 32;
  LCD_DrawChar(Line, Column, &LCD_Currentfonts->table[Ascii * LCD_Currentfonts->Height]);
}


/**
  * @brief  Displays a maximum of 20 char on the LCD.
  * @param  Line: the Line where to display the character shape .
  *   This parameter can be one of the following values:
  *     @arg Linex: where x can be 0..9
  * @param  *ptr: pointer to string to display on LCD.
  * @retval None
  */
void LCD_DisplayStringLine(uint8_t Line, uint8_t *ptr)
{
  uint16_t refcolumn = 0;

  /* Send the string character by character on lCD */
  while ((*ptr != 0) & (((refcolumn) & 0xFFFF) < LCD_PIXEL_WIDTH))
  {
    /* Display one character on LCD */
    LCD_DisplayChar(Line, refcolumn, *ptr);
    /* Decrement the column position by 16 */
    refcolumn += LCD_Currentfonts->Width;
    /* Point on the next character */
    ptr++;
  }
}


/**
  * @brief  Sets a display window
  * @param  Xpos: specifies the X buttom left position.
  * @param  Ypos: specifies the Y buttom left position.
  * @param  Height: display window height.
  * @param  Width: display window width.
  * @retval None
  */
void LCD_SetDisplayWindow(uint8_t Xpos, uint16_t Ypos, uint8_t Height, uint16_t Width)
{
  /* Horizontal GRAM Start Address */
  if(Xpos >= Height)
  {
    LCD_WriteReg(LCD_REG_80, (Xpos - Height + 1));
  }
  else
  {
    LCD_WriteReg(LCD_REG_80, 0);
  }
  /* Horizontal GRAM End Address */
  LCD_WriteReg(LCD_REG_81, Xpos);
  /* Vertical GRAM Start Address */
  if(Ypos >= Width)
  {
    LCD_WriteReg(LCD_REG_82, (Ypos - Width + 1));
  }  
  else
  {
    LCD_WriteReg(LCD_REG_82, 0);
  }
  /* Vertical GRAM End Address */
  LCD_WriteReg(LCD_REG_83, Ypos);
  LCD_SetCursor(Xpos, Ypos);
}


/**
  * @brief  Disables LCD Window mode.
  * @param  None
  * @retval None
  */
void LCD_WindowModeDisable(void)
{
  LCD_SetDisplayWindow(239, 0x13F, 240, 320);
  LCD_WriteReg(LCD_REG_3, 0x1018);    
}


/**
  * @brief  Displays a line.
  * @param Xpos: specifies the X position.
  * @param Ypos: specifies the Y position.
  * @param Length: line length.
  * @param Direction: line direction.
  *   This parameter can be one of the following values: Vertical or Horizontal.
  * @retval None
  */
void LCD_DrawLine(uint8_t Xpos, uint16_t Ypos, uint16_t Length, uint8_t Direction)
{
  uint32_t i = 0;
  
  LCD_SetCursor(Xpos, Ypos);
  if(Direction == LCD_DIR_HORIZONTAL)
  {
    LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
    for(i = 0; i < Length; i++)
    {
      LCD_WriteRAM(TextColor);
    }
  }
  else
  {
    for(i = 0; i < Length; i++)
    {
      LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
      LCD_WriteRAM(TextColor);
      Xpos++;
      LCD_SetCursor(Xpos, Ypos);
    }
  }
}


/**
  * @brief  Displays a rectangle.
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position.
  * @param  Height: display rectangle height.
  * @param  Width: display rectangle width.
  * @retval None
  */
void LCD_DrawRect(uint8_t Xpos, uint16_t Ypos, uint8_t Height, uint16_t Width)
{
  LCD_DrawLine(Xpos, Ypos, Width, LCD_DIR_HORIZONTAL);
  LCD_DrawLine((Xpos + Height), Ypos, Width, LCD_DIR_HORIZONTAL);
  
  LCD_DrawLine(Xpos, Ypos, Height, LCD_DIR_VERTICAL);
  LCD_DrawLine(Xpos, (Ypos - Width + 1), Height, LCD_DIR_VERTICAL);
}


/**
  * @brief  Displays a circle.
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position.
  * @param  Radius
  * @retval None
  */
void LCD_DrawCircle(uint8_t Xpos, uint16_t Ypos, uint16_t Radius)
{
  int32_t  D;/* Decision Variable */ 
  uint32_t  CurX;/* Current X Value */
  uint32_t  CurY;/* Current Y Value */ 
  
  D = 3 - (Radius << 1);
  CurX = 0;
  CurY = Radius;
  
  while (CurX <= CurY)
  {
    LCD_SetCursor(Xpos + CurX, Ypos + CurY);
    LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
    LCD_WriteRAM(TextColor);
    LCD_SetCursor(Xpos + CurX, Ypos - CurY);
    LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
    LCD_WriteRAM(TextColor);
    LCD_SetCursor(Xpos - CurX, Ypos + CurY);
    LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
    LCD_WriteRAM(TextColor);
    LCD_SetCursor(Xpos - CurX, Ypos - CurY);
    LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
    LCD_WriteRAM(TextColor);
    LCD_SetCursor(Xpos + CurY, Ypos + CurX);
    LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
    LCD_WriteRAM(TextColor);
    LCD_SetCursor(Xpos + CurY, Ypos - CurX);
    LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
    LCD_WriteRAM(TextColor);
    LCD_SetCursor(Xpos - CurY, Ypos + CurX);
    LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
    LCD_WriteRAM(TextColor);
    LCD_SetCursor(Xpos - CurY, Ypos - CurX);
    LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
    LCD_WriteRAM(TextColor);
    if (D < 0)
    { 
      D += (CurX << 2) + 6;
    }
    else
    {
      D += ((CurX - CurY) << 2) + 10;
      CurY--;
    }
    CurX++;
  }
}


/**
  * @brief  Displays a monocolor picture.
  * @param  Pict: pointer to the picture array.
  * @retval None
  */
void LCD_DrawMonoPict(const uint32_t *Pict)
{
  uint32_t index = 0, i = 0;
  LCD_SetCursor(0, 0);
  LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
  for(index = 0; index < 2400; index++)
  {
    for(i = 0; i < 32; i++)
    {
      if((Pict[index] & (1 << i)) == 0x00)
      {
        LCD_WriteRAM(BackColor);
      }
      else
      {
        LCD_WriteRAM(TextColor);
      }
    }
  }
}


/**
  * @brief  Displays a bitmap picture loaded in the internal Flash.
  * @param  BmpAddress: Bmp picture address in the internal Flash.
  * @retval None
  */
void LCD_WriteBMP(uint32_t BmpAddress)
{
  uint32_t index = 0, size = 0;
  /* Read bitmap size */
  size = *(__IO uint16_t *) (BmpAddress + 2);
  size |= (*(__IO uint16_t *) (BmpAddress + 4)) << 16;
  /* Get bitmap data address offset */
  index = *(__IO uint16_t *) (BmpAddress + 10);
  index |= (*(__IO uint16_t *) (BmpAddress + 12)) << 16;
  size = (size - index)/2;
  BmpAddress += index;
  /* Set GRAM write direction and BGR = 1 */
  /* I/D=00 (Horizontal : decrement, Vertical : decrement) */
  /* AM=1 (address is updated in vertical writing direction) */
  LCD_WriteReg(LCD_REG_3, 0x1008);
 
  LCD_WriteRAM_Prepare();
 
  for(index = 0; index < size; index++)
  {
    LCD_WriteRAM(*(__IO uint16_t *)BmpAddress);
    BmpAddress += 2;
  }
 
  /* Set GRAM write direction and BGR = 1 */
  /* I/D = 01 (Horizontal : increment, Vertical : decrement) */
  /* AM = 1 (address is updated in vertical writing direction) */
  LCD_WriteReg(LCD_REG_3, 0x1018);
}

/**
  * @brief  Displays a full rectangle.
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position.
  * @param  Height: rectangle height.
  * @param  Width: rectangle width.
  * @retval None
  */
void LCD_DrawFullRect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height)
{
  LCD_SetTextColor(TextColor);

  LCD_DrawLine(Xpos, Ypos, Width, LCD_DIR_HORIZONTAL);
  LCD_DrawLine((Xpos + Height), Ypos, Width, LCD_DIR_HORIZONTAL);
  
  LCD_DrawLine(Xpos, Ypos, Height, LCD_DIR_VERTICAL);
  LCD_DrawLine(Xpos, (Ypos - Width + 1), Height, LCD_DIR_VERTICAL);

  Width -= 2;
  Height--;
  Ypos--;

  LCD_SetTextColor(BackColor);

  while(Height--)
  {
    LCD_DrawLine(++Xpos, Ypos, Width, LCD_DIR_HORIZONTAL);    
  }

  LCD_SetTextColor(TextColor);
}

/**
  * @brief  Displays a full circle.
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position.
  * @param  Radius
  * @retval None
  */
void LCD_DrawFullCircle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius)
{
  int32_t  D;    /* Decision Variable */ 
  uint32_t  CurX;/* Current X Value */
  uint32_t  CurY;/* Current Y Value */ 
  
  D = 3 - (Radius << 1);

  CurX = 0;
  CurY = Radius;
  
  LCD_SetTextColor(BackColor);

  while (CurX <= CurY)
  {
    if(CurY > 0) 
    {
      LCD_DrawLine(Xpos - CurX, Ypos + CurY, 2*CurY, LCD_DIR_HORIZONTAL);
      LCD_DrawLine(Xpos + CurX, Ypos + CurY, 2*CurY, LCD_DIR_HORIZONTAL);
    }

    if(CurX > 0) 
    {
      LCD_DrawLine(Xpos - CurY, Ypos + CurX, 2*CurX, LCD_DIR_HORIZONTAL);
      LCD_DrawLine(Xpos + CurY, Ypos + CurX, 2*CurX, LCD_DIR_HORIZONTAL);
    }
    if (D < 0)
    { 
      D += (CurX << 2) + 6;
    }
    else
    {
      D += ((CurX - CurY) << 2) + 10;
      CurY--;
    }
    CurX++;
  }

  LCD_SetTextColor(TextColor);
  LCD_DrawCircle(Xpos, Ypos, Radius);
}

/**
  * @brief  Displays an uni line (between two points).
  * @param  x1: specifies the point 1 x position.
  * @param  y1: specifies the point 1 y position.
  * @param  x2: specifies the point 2 x position.
  * @param  y2: specifies the point 2 y position.
  * @retval None
  */
void LCD_DrawUniLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
  int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0, 
  yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0, 
  curpixel = 0;
  
  deltax = ABS(x2 - x1);        /* The difference between the x's */
  deltay = ABS(y2 - y1);        /* The difference between the y's */
  x = x1;                       /* Start x off at the first pixel */
  y = y1;                       /* Start y off at the first pixel */
  
  if (x2 >= x1)                 /* The x-values are increasing */
  {
    xinc1 = 1;
    xinc2 = 1;
  }
  else                          /* The x-values are decreasing */
  {
    xinc1 = -1;
    xinc2 = -1;
  }
  
  if (y2 >= y1)                 /* The y-values are increasing */
  {
    yinc1 = 1;
    yinc2 = 1;
  }
  else                          /* The y-values are decreasing */
  {
    yinc1 = -1;
    yinc2 = -1;
  }
  
  if (deltax >= deltay)         /* There is at least one x-value for every y-value */
  {
    xinc1 = 0;                  /* Don't change the x when numerator >= denominator */
    yinc2 = 0;                  /* Don't change the y for every iteration */
    den = deltax;
    num = deltax / 2;
    numadd = deltay;
    numpixels = deltax;         /* There are more x-values than y-values */
  }
  else                          /* There is at least one y-value for every x-value */
  {
    xinc2 = 0;                  /* Don't change the x for every iteration */
    yinc1 = 0;                  /* Don't change the y when numerator >= denominator */
    den = deltay;
    num = deltay / 2;
    numadd = deltax;
    numpixels = deltay;         /* There are more y-values than x-values */
  }
  
  for (curpixel = 0; curpixel <= numpixels; curpixel++)
  {
    PutPixel(x, y);             /* Draw the current pixel */
    num += numadd;              /* Increase the numerator by the top of the fraction */
    if (num >= den)             /* Check if numerator >= denominator */
    {
      num -= den;               /* Calculate the new numerator value */
      x += xinc1;               /* Change the x as appropriate */
      y += yinc1;               /* Change the y as appropriate */
    }
    x += xinc2;                 /* Change the x as appropriate */
    y += yinc2;                 /* Change the y as appropriate */
  }
}

/**
  * @brief  Displays an polyline (between many points).
  * @param  Points: pointer to the points array.
  * @param  PointCount: Number of points.
  * @retval None
  */
void LCD_PolyLine(pPoint Points, uint16_t PointCount)
{
  int16_t X = 0, Y = 0;

  if(PointCount < 2)
  {
    return;
  }

  while(--PointCount)
  {
    X = Points->X;
    Y = Points->Y;
    Points++;
    LCD_DrawUniLine(X, Y, Points->X, Points->Y);
  }
}

/**
  * @brief  Displays an relative polyline (between many points).
  * @param  Points: pointer to the points array.
  * @param  PointCount: Number of points.
  * @param  Closed: specifies if the draw is closed or not.
  *           1: closed, 0 : not closed.
  * @retval None
  */
static void LCD_PolyLineRelativeClosed(pPoint Points, uint16_t PointCount, uint16_t Closed)
{
  int16_t X = 0, Y = 0;
  pPoint First = Points;

  if(PointCount < 2)
  {
    return;
  }  
  X = Points->X;
  Y = Points->Y;
  while(--PointCount)
  {
    Points++;
    LCD_DrawUniLine(X, Y, X + Points->X, Y + Points->Y);
    X = X + Points->X;
    Y = Y + Points->Y;
  }
  if(Closed)
  {
    LCD_DrawUniLine(First->X, First->Y, X, Y);
  }  
}

/**
  * @brief  Displays a closed polyline (between many points).
  * @param  Points: pointer to the points array.
  * @param  PointCount: Number of points.
  * @retval None
  */
void LCD_ClosedPolyLine(pPoint Points, uint16_t PointCount)
{
  LCD_PolyLine(Points, PointCount);
  LCD_DrawUniLine(Points->X, Points->Y, (Points+PointCount-1)->X, (Points+PointCount-1)->Y);
}

/**
  * @brief  Displays a relative polyline (between many points).
  * @param  Points: pointer to the points array.
  * @param  PointCount: Number of points.
  * @retval None
  */
void LCD_PolyLineRelative(pPoint Points, uint16_t PointCount)
{
  LCD_PolyLineRelativeClosed(Points, PointCount, 0);
}

/**
  * @brief  Displays a closed relative polyline (between many points).
  * @param  Points: pointer to the points array.
  * @param  PointCount: Number of points.
  * @retval None
  */
void LCD_ClosedPolyLineRelative(pPoint Points, uint16_t PointCount)
{
  LCD_PolyLineRelativeClosed(Points, PointCount, 1);
}


/**
  * @brief  Displays a  full polyline (between many points).
  * @param  Points: pointer to the points array.
  * @param  PointCount: Number of points.
  * @retval None
  */
void LCD_FillPolyLine(pPoint Points, uint16_t PointCount)
{
  /*  public-domain code by Darel Rex Finley, 2007 */
  uint16_t  nodes = 0, nodeX[MAX_POLY_CORNERS], pixelX = 0, pixelY = 0, i = 0,
  j = 0, swap = 0;
  uint16_t  IMAGE_LEFT = 0, IMAGE_RIGHT = 0, IMAGE_TOP = 0, IMAGE_BOTTOM = 0;

  IMAGE_LEFT = IMAGE_RIGHT = Points->X;
  IMAGE_TOP= IMAGE_BOTTOM = Points->Y;

  for(i = 1; i < PointCount; i++)
  {
    pixelX = POLY_X(i);
    if(pixelX < IMAGE_LEFT)
    {
      IMAGE_LEFT = pixelX;
    }
    if(pixelX > IMAGE_RIGHT)
    {
      IMAGE_RIGHT = pixelX;
    }
    
    pixelY = POLY_Y(i);
    if(pixelY < IMAGE_TOP)
    { 
      IMAGE_TOP = pixelY;
    }
    if(pixelY > IMAGE_BOTTOM)
    {
      IMAGE_BOTTOM = pixelY;
    }
  }
  
  LCD_SetTextColor(BackColor);  

  /*  Loop through the rows of the image. */
  for (pixelY = IMAGE_TOP; pixelY < IMAGE_BOTTOM; pixelY++) 
  {  
    /* Build a list of nodes. */
    nodes = 0; j = PointCount-1;

    for (i = 0; i < PointCount; i++) 
    {
      if (POLY_Y(i)<(double) pixelY && POLY_Y(j)>=(double) pixelY || POLY_Y(j)<(double) pixelY && POLY_Y(i)>=(double) pixelY) 
      {
        nodeX[nodes++]=(int) (POLY_X(i)+((pixelY-POLY_Y(i))*(POLY_X(j)-POLY_X(i)))/(POLY_Y(j)-POLY_Y(i))); 
      }
      j = i; 
    }
  
    /* Sort the nodes, via a simple "Bubble" sort. */
    i = 0;
    while (i < nodes-1) 
    {
      if (nodeX[i]>nodeX[i+1]) 
      {
        swap = nodeX[i]; 
        nodeX[i] = nodeX[i+1]; 
        nodeX[i+1] = swap; 
        if(i)
        {
          i--; 
        }
      }
      else 
      {
        i++;
      }
    }
  
    /*  Fill the pixels between node pairs. */
    for (i = 0; i < nodes; i+=2) 
    {
      if(nodeX[i] >= IMAGE_RIGHT) 
      {
        break;
      }
      if(nodeX[i+1] > IMAGE_LEFT) 
      {
        if (nodeX[i] < IMAGE_LEFT)
        {
          nodeX[i]=IMAGE_LEFT;
        }
        if(nodeX[i+1] > IMAGE_RIGHT)
        {
          nodeX[i+1] = IMAGE_RIGHT;
        }
        LCD_SetTextColor(BackColor);
        LCD_DrawLine(pixelY, nodeX[i+1], nodeX[i+1] - nodeX[i], LCD_DIR_HORIZONTAL);
        LCD_SetTextColor(TextColor);
        PutPixel(pixelY, nodeX[i+1]);
        PutPixel(pixelY, nodeX[i]);
        /* for (j=nodeX[i]; j<nodeX[i+1]; j++) PutPixel(j,pixelY); */
      }
    }
  } 

  /* draw the edges */
  LCD_SetTextColor(TextColor);
}

/**
  * @brief  Writes to the selected LCD register.
  * @param  LCD_Reg: address of the selected register.
  * @param  LCD_RegValue: value to write to the selected register.
  * @retval None
  */
void LCD_WriteReg(uint8_t LCD_Reg, uint16_t LCD_RegValue)
{
  /* Write 16-bit Index, then Write Reg */
  LCD->LCD_REG = LCD_Reg;
  /* Write 16-bit Reg */
  LCD->LCD_RAM = LCD_RegValue;
}


/**
  * @brief  Reads the selected LCD Register.
  * @param  LCD_Reg: address of the selected register.
  * @retval LCD Register Value.
  */
uint16_t LCD_ReadReg(uint8_t LCD_Reg)
{
  /* Write 16-bit Index (then Read Reg) */
  LCD->LCD_REG = LCD_Reg;
  /* Read 16-bit Reg */
  return (LCD->LCD_RAM);
}


/**
  * @brief  Prepare to write to the LCD RAM.
  * @param  None
  * @retval None
  */
void LCD_WriteRAM_Prepare(void)
{
  LCD->LCD_REG = LCD_REG_34;
}


/**
  * @brief  Writes to the LCD RAM.
  * @param  RGB_Code: the pixel color in RGB mode (5-6-5).
  * @retval None
  */
void LCD_WriteRAM(uint16_t RGB_Code)
{
  /* Write 16-bit GRAM Reg */
  LCD->LCD_RAM = RGB_Code;
}


/**
  * @brief  Reads the LCD RAM.
  * @param  None
  * @retval LCD RAM Value.
  */
uint16_t LCD_ReadRAM(void)
{
  /* Write 16-bit Index (then Read Reg) */
  LCD->LCD_REG = LCD_REG_34; /* Select GRAM Reg */
  /* Read 16-bit Reg */
  return LCD->LCD_RAM;
}


/**
  * @brief  Power on the LCD.
  * @param  None
  * @retval None
  */
void LCD_PowerOn(void)
{
/* Power On sequence ---------------------------------------------------------*/
  LCD_WriteReg(LCD_REG_16, 0x0000); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
  LCD_WriteReg(LCD_REG_17, 0x0000); /* DC1[2:0], DC0[2:0], VC[2:0] */
  LCD_WriteReg(LCD_REG_18, 0x0000); /* VREG1OUT voltage */
  LCD_WriteReg(LCD_REG_19, 0x0000); /* VDV[4:0] for VCOM amplitude*/
  _delay_(20);                 /* Dis-charge capacitor power voltage (200ms) */
  LCD_WriteReg(LCD_REG_16, 0x17B0); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
  LCD_WriteReg(LCD_REG_17, 0x0137); /* DC1[2:0], DC0[2:0], VC[2:0] */
  _delay_(5);                  /* Delay 50 ms */
  LCD_WriteReg(LCD_REG_18, 0x0139); /* VREG1OUT voltage */
  _delay_(5);                  /* Delay 50 ms */
  LCD_WriteReg(LCD_REG_19, 0x1d00); /* VDV[4:0] for VCOM amplitude */
  LCD_WriteReg(LCD_REG_41, 0x0013); /* VCM[4:0] for VCOMH */
  _delay_(5);                  /* Delay 50 ms */
  LCD_WriteReg(LCD_REG_7, 0x0173);  /* 262K color and display ON */
}


/**
  * @brief  Enables the Display.
  * @param  None
  * @retval None
  */
void LCD_DisplayOn(void)
{
  /* Display On */
  LCD_WriteReg(LCD_REG_7, 0x0173); /* 262K color and display ON */
}


/**
  * @brief  Disables the Display.
  * @param  None
  * @retval None
  */
void LCD_DisplayOff(void)
{
  /* Display Off */
  LCD_WriteReg(LCD_REG_7, 0x0); 
}


/**
  * @brief  Configures LCD Control lines (FSMC Pins) in alternate function mode.
  * @param  None
  * @retval None
  */
void LCD_CtrlLinesConfig(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  /* Enable FSMC, GPIOD, GPIOE, GPIOF, GPIOG and AFIO clocks */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE |
                         RCC_APB2Periph_GPIOF | RCC_APB2Periph_GPIOG |
                         RCC_APB2Periph_AFIO, ENABLE);
  /* Set PD.00(D2), PD.01(D3), PD.04(NOE), PD.05(NWE), PD.08(D13), PD.09(D14),
     PD.10(D15), PD.14(D0), PD.15(D1) as alternate function push pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 |
                                GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_14 | 
                                GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  /* Set PE.07(D4), PE.08(D5), PE.09(D6), PE.10(D7), PE.11(D8), PE.12(D9), PE.13(D10),
     PE.14(D11), PE.15(D12) as alternate function push pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | 
                                GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | 
                                GPIO_Pin_15;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  /* Set PF.00(A0 (RS)) as alternate function push pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_Init(GPIOF, &GPIO_InitStructure);
  /* Set PG.12(NE4 (LCD/CS)) as alternate function push pull - CE3(LCD /CS) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_Init(GPIOG, &GPIO_InitStructure);

  /*FSMC A21��A22���Ի������츴�����*/ 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6; 
  GPIO_Init(GPIOE, &GPIO_InitStructure); 

  /* Lcd_Light_Control */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  //Lcd_Light_OFF  
  Lcd_Light_ON
}


/**
  * @brief  Configures the Parallel interface (FSMC) for LCD(Parallel mode)
  * @param  None
  * @retval None
  */
void LCD_FSMCConfig(void)
{
  FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
  FSMC_NORSRAMTimingInitTypeDef  Timing_read, Timing_write;
/*-- FSMC Configuration ------------------------------------------------------*/
/*----------------------- SRAM Bank 4 ----------------------------------------*/
  /* FSMC_Bank1_NORSRAM4 configuration */
  Timing_read.FSMC_AddressSetupTime = 3;             
  Timing_read.FSMC_AddressHoldTime = 0;  
  Timing_read.FSMC_DataSetupTime = 3; 
  Timing_read.FSMC_BusTurnAroundDuration = 0;
  Timing_read.FSMC_CLKDivision = 0;
  Timing_read.FSMC_DataLatency = 0;
  Timing_read.FSMC_AccessMode = FSMC_AccessMode_A;    

  Timing_write.FSMC_AddressSetupTime = 1;             
  Timing_write.FSMC_AddressHoldTime = 0;  
  Timing_write.FSMC_DataSetupTime = 1; 
  Timing_write.FSMC_BusTurnAroundDuration = 0;
  Timing_write.FSMC_CLKDivision = 0;
  Timing_write.FSMC_DataLatency = 0;  
  Timing_write.FSMC_AccessMode = FSMC_AccessMode_A; 
  
  /* Color LCD configuration ------------------------------------
     LCD configured as follow:
        - Data/Address MUX = Disable
        - Memory Type = SRAM
        - Data Width = 16bit
        - Write Operation = Enable
        - Extended Mode = Enable
        - Asynchronous Wait = Disable */
  FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM4;
  FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
  FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_SRAM;
  FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
  FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
  FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait = FSMC_AsynchronousWait_Disable;  
  FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
  FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;
  FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
  FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
  FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
  FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;
  FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
  FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &Timing_read;
  FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &Timing_write;
  FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure);  
  /* BANK 4 (of NOR/SRAM Bank 1~4) is enabled */
  FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM4, ENABLE);
}

/**
  * @brief  Displays a pixel.
  * @param  x: pixel x.
  * @param  y: pixel y.  
  * @retval None
  */
static void PutPixel(int16_t x, int16_t y)
{ 
  if(x < 0 || x > 239 || y < 0 || y > 319)
  {
    return;  
  }
  LCD_DrawLine(x, y, 1, LCD_DIR_HORIZONTAL);
}

#ifndef USE_Delay
/**
  * @brief  Inserts a delay time.
  * @param  nCount: specifies the delay time length.
  * @retval None
  */
static void delay(vu32 nCount)
{
  vu32 index = 0; 
  for(index = (100000 * nCount); index != 0; index--)
  {
  }
}
#endif /* USE_Delay*/


/**
  * @}
  */ 

/****************************************************************************
* ��    �ƣ�u16 ili9320_GetPoint(u16 x,u16 y)
* ��    �ܣ���ȡָ���������ɫֵ
* ��ڲ�����x      ������
*           y      ������
* ���ڲ�������ǰ������ɫֵ
* ˵    ����
* ���÷�����i=ili9320_GetPoint(10,10);
****************************************************************************/
u16 LCD_GetPoint(u16 x,u16 y)
{
  LCD_SetCursor(x,y);
  //return (ili9320_BGR2RGB(LCD_ReadRAM()));
  return (LCD_ReadRAM());
}
/****************************************************************************
* ��    �ƣ�void ili9320_SetPoint(u16 x,u16 y,u16 point)
* ��    �ܣ���ָ�����껭��
* ��ڲ�����x      ������
*           y      ������
*           point  �����ɫ
* ���ڲ�������
* ˵    ����
* ���÷�����ili9320_SetPoint(10,10,0x0fe0);
****************************************************************************/
void LCD_SetPoint(u16 x,u16 y,u16 point)
{
  if ( (x>240)||(y>320) ) return;
  LCD_SetCursor(x,y);

  LCD_WriteRAM_Prepare();
  LCD_WriteRAM(point);
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

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

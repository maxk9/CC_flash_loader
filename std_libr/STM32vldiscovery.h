/**
  ******************************************************************************
  * @file    STM32vldiscovery.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    09/13/2010
  * @brief   Header file for STM32vldiscovery.c module.
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
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F100_Dicovery_H
#define __STM32F100_Dicovery_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "stm32f10x_conf.h"	// Keil::Device:StdPeriph Drivers:Framework
//#include "lcd-nokia1100.h"
/** @addtogroup Utilities
  * @{
  */ 
  
/** @addtogroup STM32vldiscovery
  * @{
  */ 

/** @defgroup STM32vldiscovery_Abstraction_Layer
  * @{
  */  

/** @defgroup STM32vldiscovery_HARDWARE_RESOURCES
  * @{
  */
  
/** @defgroup STM32vldiscovery_Exported_Types
  * @{
  */
  
  
  
	#define TIMER_DIV	1
  
	extern volatile u32 p0_len,p1_len,RX_RSSI;  
	extern volatile u16 period_capture, duty_cycle_capture;  
	extern volatile u32 RX_AM_STATE;
  
	extern volatile u8 status_sig;
	extern u8 	came_data[3],	came_bits,
				sher_fm_data[6], sher_fm_bits,
				klq_data[9], klq_bits;
	
//	extern u8 str[];
	
	

enum{
	sher_FM=0,
	sher_AM,
	keeloq,
	came_st,
	starline_st
	
};		
  
enum{
	button_signal=1,
	rx_bit_signal,
	LCD_out
};	
  
typedef enum 
{
  LED3 = 0,
  LED4 = 1
} Led_TypeDef;

typedef enum 
{  
  BUTTON_USER = 0
} Button_TypeDef;

typedef enum 
{  
  BUTTON_MODE_GPIO = 0,
  BUTTON_MODE_EXTI = 1
} ButtonMode_TypeDef;              

/** 
  * @brief  STM32F100 Button Defines Legacy  
  */ 

#define Button_USER          BUTTON_USER
#define Mode_GPIO            BUTTON_MODE_GPIO
#define Mode_EXTI            BUTTON_MODE_EXTI
#define Button_Mode_TypeDef  ButtonMode_TypeDef


/** @addtogroup STM32vldiscovery_LOW_LEVEL_LED
  * @{
  */
#define LEDn                             2
#define LED3_PIN                         GPIO_Pin_9  
#define LED3_GPIO_PORT                   GPIOC
#define LED3_GPIO_CLK                    RCC_APB2Periph_GPIOC  

#define LED4_PIN                         GPIO_Pin_8  
#define LED4_GPIO_PORT                   GPIOC
#define LED4_GPIO_CLK                    RCC_APB2Periph_GPIOC  

/**
  * @}
  */ 
  
/** @addtogroup STM32vldiscovery_LOW_LEVEL_BUTTON
  * @{
  */  
#define BUTTONn                          1

/* * @brief USER push-button
 */
#define USER_BUTTON_PIN                   GPIO_Pin_0
#define USER_BUTTON_GPIO_PORT             GPIOA
#define USER_BUTTON_GPIO_CLK              RCC_APB2Periph_GPIOA
#define USER_BUTTON_EXTI_PORT_SOURCE      GPIO_PortSourceGPIOA
#define USER_BUTTON_EXTI_PIN_SOURCE       GPIO_PinSource0
#define USER_BUTTON_EXTI_LINE             EXTI_Line0
#define USER_BUTTON_EXTI_IRQn             EXTI0_IRQn


//#define TX_ON							GPIO_ResetBits(GPIOA,(GPIO_Pin_10))
//#define TX_OFF							GPIO_SetBits(GPIOA,(GPIO_Pin_10))
#define TX_DATA_0						GPIO_ResetBits(GPIOA,(GPIO_Pin_10))
#define TX_DATA_1						GPIO_SetBits(GPIOA,(GPIO_Pin_10))
#define RX_ON							GPIO_SetBits(GPIOA,(GPIO_Pin_9))
#define RX_OFF							GPIO_ResetBits(GPIOA,(GPIO_Pin_9))
#define TX_ON							GPIO_SetBits(GPIOC,(GPIO_Pin_13))
#define TX_OFF							GPIO_ResetBits(GPIOC,(GPIO_Pin_13))
#define TX_TOGGLE						GPIOC->ODR ^= GPIO_Pin_13

#define FM_RX_STATE						GPIO_ReadOutputDataBit(GPIOA,(GPIO_Pin_9))

#define TEST_A11_ON						GPIOA->BSRR = GPIO_Pin_11
#define TEST_A11_OFF					GPIOA->BRR = GPIO_Pin_11
#define TEST_A11_TOGGLE					GPIOA->ODR ^= GPIO_Pin_11

#define TEST_A12_ON						GPIOA->BSRR = GPIO_Pin_12
#define TEST_A12_OFF					GPIOA->BRR = GPIO_Pin_12
#define TEST_A12_TOGGLE					GPIOA->ODR ^= GPIO_Pin_12

#define RST_LCD_RESET					GPIOA->BRR = GPIO_Pin_6
#define RST_LCD_SET						GPIOA->BSRR = GPIO_Pin_6
#define CS_LCD_RESET					GPIOA->BRR = GPIO_Pin_4
#define CS_LCD_SET						GPIOA->BSRR = GPIO_Pin_4
#define SCLK_LCD_RESET					GPIOA->BRR = GPIO_Pin_5
#define SCLK_LCD_SET					GPIOA->BSRR = GPIO_Pin_5
#define SDA_LCD_SET						GPIOA->BRR = GPIO_Pin_7
#define SDA_LCD_RESET					GPIOA->BSRR = GPIO_Pin_7


/**
  * @}
  */ 

/** @defgroup STM32vldiscovery_LOW_LEVEL__Exported_Functions
  * @{
  */ 
void STM32vldiscovery_LEDInit(Led_TypeDef Led);
void STM32vldiscovery_LEDOn(Led_TypeDef Led);
void STM32vldiscovery_LEDOff(Led_TypeDef Led);
void STM32vldiscovery_LEDToggle(Led_TypeDef Led);
void STM32vldiscovery_PBInit(Button_TypeDef Button, ButtonMode_TypeDef Button_Mode);
uint32_t STM32vldiscovery_PBGetState(Button_TypeDef Button);


void RX_init(void);
void FM_send_byte(u8 tx_byte);

/**
  * @}
  */ 
    
#ifdef __cplusplus
}
#endif



#endif /* __STM32vldiscovery_H */

/**
  * @}
  */ 

/**
  * @}
  */  

/**
  * @}
  */
  
/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/

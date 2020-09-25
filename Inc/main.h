/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define T9C1steppulse_Pin GPIO_PIN_5
#define T9C1steppulse_GPIO_Port GPIOE
#define encodectrA_Pin GPIO_PIN_0
#define encodectrA_GPIO_Port GPIOA
#define encodectrB_Pin GPIO_PIN_1
#define encodectrB_GPIO_Port GPIOA
#define encodertimeA_Pin GPIO_PIN_2
#define encodertimeA_GPIO_Port GPIOA
#define encodertimeB_Pin GPIO_PIN_3
#define encodertimeB_GPIO_Port GPIOA
#define solenoiddrive_Pin GPIO_PIN_6
#define solenoiddrive_GPIO_Port GPIOA
#define Stepper__DR__direction_Pin GPIO_PIN_0
#define Stepper__DR__direction_GPIO_Port GPIOB
#define Stepper__MF_not_enable_Pin GPIO_PIN_1
#define Stepper__MF_not_enable_GPIO_Port GPIOB
#define StepperBridge_Pin GPIO_PIN_9
#define StepperBridge_GPIO_Port GPIOE
#define LimitSw_inside_NO_Pin GPIO_PIN_10
#define LimitSw_inside_NO_GPIO_Port GPIOE
#define LimitSw_inside_NO_EXTI_IRQn EXTI15_10_IRQn
#define LimitSw_inside_NC_Pin GPIO_PIN_11
#define LimitSw_inside_NC_GPIO_Port GPIOE
#define LimitSw_inside_NC_EXTI_IRQn EXTI15_10_IRQn
#define LimitSw_outside_NO_Pin GPIO_PIN_12
#define LimitSw_outside_NO_GPIO_Port GPIOE
#define LimitSw_outside_NO_EXTI_IRQn EXTI15_10_IRQn
#define LimitSw_outside_NC_Pin GPIO_PIN_13
#define LimitSw_outside_NC_GPIO_Port GPIOE
#define LimitSw_outside_NC_EXTI_IRQn EXTI15_10_IRQn
#define OverrunSw_Inside_Pin GPIO_PIN_14
#define OverrunSw_Inside_GPIO_Port GPIOE
#define OverrunSw_Inside_EXTI_IRQn EXTI15_10_IRQn
#define OverrunSw_outside_Pin GPIO_PIN_15
#define OverrunSw_outside_GPIO_Port GPIOE
#define OverrunSw_outside_EXTI_IRQn EXTI15_10_IRQn
#define LED_GREEN_Pin GPIO_PIN_12
#define LED_GREEN_GPIO_Port GPIOD
#define LED_ORANGE_Pin GPIO_PIN_13
#define LED_ORANGE_GPIO_Port GPIOD
#define LED_RED_Pin GPIO_PIN_14
#define LED_RED_GPIO_Port GPIOD
#define LED_BLUE_Pin GPIO_PIN_15
#define LED_BLUE_GPIO_Port GPIOD
#define LimitSw_inside_NOC6_Pin GPIO_PIN_6
#define LimitSw_inside_NOC6_GPIO_Port GPIOC
#define LimitSw_inside_NCC7_Pin GPIO_PIN_7
#define LimitSw_inside_NCC7_GPIO_Port GPIOC
#define LimitSw_outside_NOC8_Pin GPIO_PIN_8
#define LimitSw_outside_NOC8_GPIO_Port GPIOC
#define LimitSw_outside_NCC9_Pin GPIO_PIN_9
#define LimitSw_outside_NCC9_GPIO_Port GPIOC
#define encodertimeZ_Pin GPIO_PIN_3
#define encodertimeZ_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
// ===============================================================================
// USART/UART assignments
#define HUARTMON  huart3 // uart  for PC monitoring

extern UART_HandleTypeDef huart3;
extern struct CAN_CTLBLOCK* pctl0;  // Pointer to CAN1 control block

// Gateway task (for Mailbox use)
//#define GATEWAYTASKINCLUDED // Include gateway

//#define USEUSBFORCANMSGS // Use USB for gateway to PC

//#define CONFIGCAN2	// Configure for CAN2

// ===============================================================================

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

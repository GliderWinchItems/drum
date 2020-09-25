/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
#include "morse.h"
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim9;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim12;

/* USER CODE BEGIN EV */

/* Registers from Hard Fault */

/* Saved registers -- in the following order:
0 - psr was stacked by Hard Fault; 4 - 10 remain.
  0, 1, 2, 3, 12, lr, pc, psr, 4, 5, 6, 7, 8, 9, 10 */
volatile uint32_t reg_stack[16];


volatile uint32_t sys_regs[5];
/* sys_regs: Four System registers saved. 0xE000ED30 skipped.
hence, 5 words reserved.

[0] 0xE000ED28 
  Memory Management Fault Status Register (byte)
   7 MMAR is valid
   4 Stacking error
   3 Unstacking error
   1 Data access violation
   0 Instruction access violation

  Bus Fault Status Register (byte)
   7 BFAR is valid
   4 Stacking error
   3 Unstacking error
   2 Imprecise data access violation
   1 Precise data access violation
   0 Instruction access violation
 
  Usage Fault Status Register (half word)
   9 Divide by zero (only set if DIV_)_TRP set)
   8 Unaligned access 
   3 Attempt to execut coprocessor instruction
   2 Attempt to do exception with bad value in EXC_RETURN
   1 Attempt to switch to invalid state (e.g. ARM)
   0 Attempt to execute an undefined instruction

[1] 0xE000ED2C Hard Fault Status Register (word)
 31 Triggered by debugg event
 30 Taken because bus fault/mem mngment fault/usage fault
  1 Vector fetch

[2] 0xE000ED30 Debug Fault Status Register (word)
 not saved

[3] 0xE000ED34 Memory Manage Address Register (word)
 Address that caused memory manage fault

[4] 0xE000ED38 Bus Fault Manage Address Register (word)
 Address that caused the bus fault
*/

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
/* The prototype shows it is a naked function - in effect this is just an
assembly function. */
void HardFault_Handler( void ) __attribute__( ( naked ) );

/* The fault handler implementation calls a function called
prvGetRegistersFromStack(). */
//void HardFault_Handler(void)
//{
  /* USER CODE BEGIN HardFault_IRQn 0 */
 __asm volatile
 (
   " tst lr, #4                       \n\t" /* Determine stack in use. */ 
   " ite eq                           \n\t" /* Set R0 with stack ptr */
   " mrseq r0, msp                    \n\t" 
   " mrsne r0, psp                    \n\t"
   " mov r1, r0                       \n\t"
   " ldr r2, handler2_address_const   \n\t" /* Save stacked regs. */
   " ldr r3, [r1, 0]                  \n\t" /* r0  */
	" str r3, [r2, 0]                  \n\t" 			
   " ldr r3, [r1, 4]                  \n\t" /* r1  */
	" str r3, [r2, 4]                  \n\t"  
   " ldr r3, [r1, 8]                  \n\t" /* r2  */
	" str r3, [r2, 8]                  \n\t" 				
   " ldr r3, [r1, 12]                 \n\t" /* r3  */			 
	" str r3, [r2, 12]                 \n\t" 				
   " ldr r3, [r1, 16]                 \n\t" /* r12 */
	" str r3, [r2, 16]                 \n\t"
   " ldr r3, [r1, 20]                 \n\t" /* lr  */
	" str r3, [r2, 20]                 \n\t"
   " ldr r3, [r1, 24]                 \n\t" /* pc  */
	" str r3, [r2, 24]                 \n\t"
   " ldr r3, [r1, 28]                 \n\t" /* psr */
	" str r3, [r2, 28]                 \n\t"
   " str r4, [r2, 32]     \n\t" /* r4 */ /* Save remaining regs. */
   " str r5, [r2, 36]     \n\t" /* r5 */
   " str r6, [r2, 40]     \n\t" /* r6 */
   " str r7, [r2, 48]     \n\t" /* r7 */
   " str r8, [r2, 52]     \n\t" /* r8 */
   " str r9, [r2, 56]     \n\t" /* r9 */
   " str r10, [r2, 60]    \n\t" /* r10 */
   " str r11, [r2, 64]    \n\t" /* r11 */
   " ldr r2, handler5_address_const  \n\t" /* system registers dest */
   " ldr r3, handler6_address_const  \n\t" /* system registers source */
   " ldr r1, [r3, 0]        \n\t" /* 0xE000ED28 Status regs: two bytes:one half word  */
	" str r1, [r2, 0]        \n\t" 			
   " ldr r1, [r3, 4]        \n\t" /* 0xE000ED2A HardFault Status (2 half word)  */
	" str r1, [r2, 4]        \n\t" 			
   " ldr r1, [r3, 12]       \n\t" /* 0xE000ED34 MMAR: Mem Mgmnt Addr Reg  */
	" str r1, [r2, 12]       \n\t" 			
   " ldr r1, [r3, 16]       \n\t" /* 0xE000ED38 BFAR: Bus Mgmnt Addr Reg    */
	" str r1, [r2, 16]       \n\t" 			
   " movs	r0, #111  \n\t"    /* Flash LEDs */
   " ldr r2, handler4_address_const   \n\t"
   " bx r2                            \n\t"
   " handler4_address_const: .word morse_trap \n\t"
   " handler2_address_const: .word reg_stack \n\t"
   " handler5_address_const: .word sys_regs \n\t"
   " handler6_address_const: .word 0xE000ED28 \n\t" /* Memory Mgmnt Fault Status Reg. */
 );
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */
	morse_trap(222);
  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */

    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */
	morse_trap(333);
  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */
	morse_trap(444);
  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream3 global interrupt.
  */
void DMA1_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */

  /* USER CODE END DMA1_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_tx);
  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

  /* USER CODE END DMA1_Stream3_IRQn 1 */
}

/**
  * @brief This function handles CAN1 TX interrupts.
  */
void CAN1_TX_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_TX_IRQn 0 */

  /* USER CODE END CAN1_TX_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_TX_IRQn 1 */

  /* USER CODE END CAN1_TX_IRQn 1 */
}

/**
  * @brief This function handles CAN1 RX0 interrupts.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */

  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles CAN1 RX1 interrupt.
  */
void CAN1_RX1_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX1_IRQn 0 */

  /* USER CODE END CAN1_RX1_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX1_IRQn 1 */

  /* USER CODE END CAN1_RX1_IRQn 1 */
}

/**
  * @brief This function handles TIM1 break interrupt and TIM9 global interrupt.
  */
void TIM1_BRK_TIM9_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 0 */


  /* USER CODE END TIM1_BRK_TIM9_IRQn 0 */
  HAL_TIM_IRQHandler(&htim9);
  /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 1 */

  /* USER CODE END TIM1_BRK_TIM9_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
extern void levelwind_items_TIM2_IRQHandler(void);
  levelwind_items_TIM2_IRQHandler();
  return;
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */
  
  extern void Stepper_EXTI15_10_IRQHandler(void);
  Stepper_EXTI15_10_IRQHandler();
  return;

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
  * @brief This function handles TIM8 break interrupt and TIM12 global interrupt.
  */
void TIM8_BRK_TIM12_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_BRK_TIM12_IRQn 0 */

  /* USER CODE END TIM8_BRK_TIM12_IRQn 0 */
  HAL_TIM_IRQHandler(&htim12);
  /* USER CODE BEGIN TIM8_BRK_TIM12_IRQn 1 */

  /* USER CODE END TIM8_BRK_TIM12_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

/******************************************************************************
* File Name          : tim2tim5common_init.c
* Date First Issued  : 01/12/25/
* Description        : Common timer inits for odometer and levelwind 
*******************************************************************************/

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "malloc.h"

#include "main.h"
#include "morse.h"

#include "drum_items.h"
#include "OdometerTask.h"
#include "MailboxTask.h"
#include "controlpanel_items.h"
#include "OdometerTask.h"
#include "odometer_items.h"
#include "levelwind_items.h"

extern TIM_HandleTypeDef htim2; // Timer FreeRTOS handle
extern TIM_HandleTypeDef htim5; // Timer FreeRTOS handle

static uint8_t oto;
/* *************************************************************************
 * void tim2tim5common_init(void);
 * @brief	: Init TIM2 & TIM5timers common to odometer and levelwind
 * *************************************************************************/
/*
TIM2 and TIM5 are used by both the odometer and levelwind functions, so the
initialization and set-up have to be coordinated.

Drum encoder: 
TIM5 configured for encoder A, B, Z
TIM2 congifured for input capture, encoder A, B, Z

Encoder channels connect to TIM2 and TIM5
TIM2 generates interrupts for encoder input capture of time.
TIM5 increments/decrements encoder counter

TIM5 PA0 = TIM2 PA2 - Encoder channel A Pullup resistors
TIM5 PA1 = TIM2 PA3 - Encoder channel B Pullup resistors

TIM2 32b (84 MHz) capture mode (interrupt)
   CH3 PA2 input capture: encoder A (TIM5 PA0)
   CH4 PA3 input capture: encoder B (TIM5 PA1)
   CH2 PB3 input capture: encoder Z
   CH1 no-pin Indexing interrupts

TIM5 32b encoder counter (no interrupt)
   CH3 PA0 encoder config: encoder A (TIM2 PA2)
   CH4 PA1 encoder config: encoder B (TIM2 PA3)


*/
TIM_TypeDef  *pT2base; // Register base address 
TIM_TypeDef  *pT5base; // Register base address 

void tim2tim5common_init(void)
{
	/* Only init once!. */
	if (oto != 0) return;
	oto = 1;

/* ### NOTE ### These might override STM32CubeMX settings. ### */

   pT2base  = htim2.Instance;
   pT5base  = htim5.Instance;

   /* TIM2 Shaft encoder input capture times & output capture indexing interrupts. */
   pT2base->CCER |= 0x1110; // Input capture active: CH2,3,4
#if LEVELWINDDEBUG   
   // Original: 0xE; // CH1,2,3 interrupt enable
   pT2base->DIER  = 0x1E; // CH1,2,3,4 interrupt enable
#else   
   // Original: // 0xA; // CH1,3 interrupt enable
   pT2base->DIER  = 0x1A;    // CH1,3,4 interrupt enable
#endif   
   pT2base->CCR1  = pT2base->CNT + 10000; // Short delay
   pT2base->ARR   = 0xffffffff; // (Max count - 1)

/* ### NOTE ### These might override STM32CubeMX settings. ### */
/* NOTE: TIM2 is used in levelwind, so this needs coordination */
   /* TIM2 Shaft encoder input capture times & output capture indexing interrupts. */
//   pT2base->CCER |= 0x1110; // Input capture active: CH2,3,4
//   pT2base->DIER  = 0x1A;    // CH1,3,4 interrupt enable
//   pT2base->CCR1  = pT2base->CNT + ODOMETER_T2C1_DUR; // 1/64 sec
//   pT2base->ARR   = 0xffffffff; // (Max count - 1)

   /* Make sure channel A & B counters are the same. */
   pT5base->CCR1 = 0; // jic
   pT5base->CCR2 = 0; // jic

   pT5base->CCMR1 |= (0x1<<8) || (0x1<<0); // TI mapping: CC2S, CC1S
   pT5base->CCER  |= 0x0011; // Input capture active: CH1,2
   pT5base->SMCR  |= 0x3; // Encoder counts on rising & falling edges

   /* TIM2 (which interrupts) started in task. */
   pT5base->CR1 |= 1;  // TIM5: encoder CH1 CH2 (no interrupt)

   /* ### NOTE ### These might override STM32CubeMX settings. ### */
/* NOTE: TIM2 is used in levelwind, so this needs coordination */
   /* TIM2 Shaft encoder input capture times & output capture indexing interrupts. */
 //  pT2base->CCER |= 0x1110; // Input capture active: CH2,3,4
 //  pT2base->DIER  = 0x1A;    // CH1,3,4 interrupt enable
 //  pT2base->CCR1  = pT2base->CNT + ODOMETER_T4C1_DUR; // 1/64 sec
 //
   /* Make sure channel A & B counters are the same. */
   pT5base->CCR1 = 0; // jic
   pT5base->CCR2 = 0; // jic

   pT5base->CCMR1 |= (0x1<<8) || (0x1<<0); // TI mapping: CC2S, CC1S
   pT5base->CCER  |= 0x0011; // Input capture active: CH1,2
   pT5base->SMCR  |= 0x3; // Encoder counts on rising & falling edges

   return;
}

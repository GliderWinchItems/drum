/******************************************************************************
* File Name          : stepper_items.c
* Date First Issued  : 07/31/2020
* Description        : Stepper motor associated items
*******************************************************************************/
/*

update100K

08/10/2020 - pins added to Control Panel for stepper testing

Control lines: Output pin drives FET gate, open drain to controller opto-isolator
PE5  - TIM9CH1 Stepper Pulse: PU (TIM1 Break shared interrupt vector)
   Interrupt vector: TIM1_BRK_TIM9_IRQHandler
PB0  - Direction: DR 
PB1  - Enable: EN  - High = enable (drive FET ON)

Limit switches: resistor pullup to +5v. Contact closes to gnd
   Interrupt vector: EXTI15_10_IRQHandler (common to PE10-PE15)
PE10 - EXTI10 Inside  Limit switch: NO contacts (switch connects to gnd)
PE11 - EXTI11 Inside  Limit switch: NC contacts (switch connects to gnd)
PE12 - EXTI12 Outside Limit switch: NO contacts (switch connects to gnd)
PE13 - EXTI13 Outside Limit switch: NC contacts (switch connects to gnd)
PE14 - EXTI14 Index   Limit switch: NO contacts (switch connects to gnd)
PE15 - EXTI15 Index   Limit switch: NC contacts (switch connects to gnd)

Drum encoder: TIM2CH1. Pullup resistors
PA0 - Encoder channel A
PA1 - Encoder channel B

*/
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "cmsis_os.h"
#include "malloc.h"
#include "stm32f4xx_hal.h"
#include "morse.h"
#include "yprintf.h"
#include "main.h"
#include "stepper_items.h"

#define TIM3CNTRATE 84000000   // TIM3 counter rate (Hz)
#define UPDATERATE 100000      // 100KHz interrupt/update rate
#define TIM3DUR  (TIM3CNTRATE/UPDATERATE) // 1680 counts per interrupt

#define TIM9CNTRATE 168000000 // TIM9 counter rate (Hz)
#define TIM9PWMCYCLE (168*10)   // 10us pwm cycle
#define TIM9PULSEDELAY (TIM9PWMCYCLE - (168*3))


TIM_TypeDef  *pT3base; // Register base address 
TIM_TypeDef  *pT9base; // Register base address 


/* Struct with all you want to know. */
struct STEPPERSTUFF stepperstuff;

static TIM_HandleTypeDef *ptim9;
static TIM_HandleTypeDef *ptim4;

/* *************************************************************************
 * void stepper_idx_v_struct_hardcode_params(void);
 * 
 * @brief       : Initialization
 * *************************************************************************/
void stepper_idx_v_struct_hardcode_params(void)
{
        stepperstuff.clfactor = 200.0f; // 100% gives max speed
        stepperstuff.ledctr   = 0;
        stepperstuff.accumpos = 0; // Position accumulator
        return;
}

/* *************************************************************************
 * void stepper_items_init(TIM_HandleTypeDef *phtim);
 * phtim = pointer to timer handle
 * @brief	: Initialization of channel increment
 * *************************************************************************/
void stepper_items_init(TIM_HandleTypeDef *phtim)
{
	stepper_idx_v_struct_hardcode_params();

	ptim9 = phtim; // Save locally for faster calling.
	pT9base = ptim9->Instance;

/* ### NOTE ### These might override STM32CubeMX settings. ### */
	pT9base->DIER = 0;// None //0x2; // CH1 interrupt enable
	pT9base->CCR1 = TIM9PULSEDELAY; // Delay count
	pT9base->ARR  = (TIM9PWMCYCLE - 1); // (10 us)
	pT9base->CCER = 0x3; // OC active high; signal on pin

	/* TIM3CH1 100 KHz interrupts. */
	
	extern TIM_HandleTypeDef htim4;
	ptim4 = &htim4;
	pT3base = ptim4->Instance;

	ptim4->Instance->DIER = 0x2; // CH1 interrupt enable, only.
	ptim4->Instance->CCR1 = ptim4->Instance->CNT + TIM3DUR;

	/* Start TIM3 counter. */
	ptim4->Instance->CR1 |= 1;

	return;
}
/* *************************************************************************
 * void stepper_items_clupdate(uint8_t dr);
 * @param 	: dr = direction: 0 = forward, not 0 = reverse
 * @brief	: Initialization of channel increment
 * *************************************************************************/
void stepper_items_clupdate(struct CANRCVBUF* pcan)
{
	union PAYFLT
	{
		float f;
		uint8_t u8[4];
	}pf;
	/* Extract float from payload */
	pf.u8[0] = pcan->cd.ui[1];
	pf.u8[1] = pcan->cd.ui[2];
	pf.u8[2] = pcan->cd.ui[3];
	pf.u8[3] = pcan->cd.ui[4];
	stepperstuff/clpos = pf.f;

	/* Bits are easy. */
	stepperstuff.clbits = pcan->cd.ui[0];



	stepperstuff.drflag = (Stepper__DR__direction_Pin << (dr*16)); // Save pushbutton state for direction control

	stepperstuff.speedcmdf = 1 * stepperstuff.clfactor;
	stepperstuff.speedcmdi = stepperstuff.speedcmdf;
	if (stepperstuff.speedcmdi > 65535)
		stepperstuff.speedinc = 65535;
	else
		stepperstuff.speedinc = stepperstuff.speedcmdi;

	return;
}


/*#######################################################################################
 * ISR routine for TIM9 [Normally no interrupt; interrupt for test purposes]
 *####################################################################################### */
uint32_t LEDx;
void stepper_items_IRQHandler(TIM_HandleTypeDef *phtim)
{
	__attribute__((__unused__))int temp;
	int32_t tmp;

	pT9base->SR = ~(0x1F);	// Reset CH1 flag (and all flags)

//phtim->Instance->CCR1 = ptim9->Instance->CNT + 504;
//morse_trap(574);

LEDx += 1;
if (LEDx > 1000)
{
  LEDx = 0;
  HAL_GPIO_TogglePin(GPIOD, LED_ORANGE_Pin);
}
	temp = phtim->Instance->SR;	// Avoid any tail-chaining
	return;
}
/*#######################################################################################
 * ISR routine for TIM3
 *####################################################################################### */
void stepper_items_TIM3_IRQHandler(void)
{
//	__attribute__((__unused__))int temp;
//	int32_t temp;

	pT3base->SR = ~(0x2);	// Reset CH1 flag

	pT3base->CCR1 += TIM3DUR; // Schedule next interrupt

	stepperstuff.accumpos += stepperstuff.speedinc;
	if ((stepperstuff.accumpos >> 16) != (stepperstuff.accumpos_prev))
	{
		// Update old position with new
		stepperstuff.accumpos_prev = (stepperstuff.accumpos >> 16);	

		// Set direction pin
		Stepper__DR__direction_GPIO_Port->BSRR = stepperstuff.drflag;

		// Start TIM9 to generated a delayed pulse.
		pT9base->CR1 = 0x9; 


HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5); // 'O-scope trigger'

// Visual check
stepperstuff.ledctr += 1;
if (stepperstuff.ledctr > 1000)
{
  stepperstuff.ledctr = 0;
  HAL_GPIO_TogglePin(GPIOD, LED_GREEN_Pin);
}
	}

//	temp = pT3base->SR;	// Avoid any tail-chaining
	return;

}

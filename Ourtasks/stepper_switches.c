/******************************************************************************
* File Name          : stepper_switches.c
* Date First Issued  : 09/16/2020
* Description        : Levelwind function w STM32CubeMX w FreeRTOS
*******************************************************************************/

/*
Notes:

PE9 - Input:pullup. Test sw bridges across overrun switches.

Limit switches: resistor pullup to +5v. Contact closes to gnd
   Interrupt vector: EXTI15_10_IRQHandler (common to PE10-PE15)
PE10 - EXTI10 Inside  Limit switch: NO contacts (switch connects to gnd)
PE11 - EXTI11 Inside  Limit switch: NC contacts (switch connects to gnd)
PE12 - EXTI12 Outside Limit switch: NO contacts (switch connects to gnd)
PE13 - EXTI13 Outside Limit switch: NC contacts (switch connects to gnd)

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
#include "main.h"
#include "stepper_items.h"
#include "DTW_counter.h"
#include "drum_items.h"
#include "stepper_switches.h"

static TIM_TypeDef  *pT2base; // Register base address 
static TIM_TypeDef  *pT5base; // Register base address 

/* Circular buffer for processing switch transitions. */
#define SWITCHXITIONSIZE 16
static struct SWITCHXITION switchxtion[SWITCHXITIONSIZE];
static struct SWITCHXITION* pbegin;
static struct SWITCHXITION* padd;
static struct SWITCHXITION* ptake;
static struct SWITCHXITION* pend;


/* *************************************************************************
 * int stepper_switches_defaultTaskcall(struct SERIALSENDTASKBCB* pbuf1);
 * @brief       : Call from main.c defaultTAsk jic
 * *************************************************************************/
int stepper_switches_defaultTaskcall(struct SERIALSENDTASKBCB* pbuf1)
{
	return 0;
}

/* *************************************************************************
 * void stepper_switches_init(void);
 * @brief       : Initialization
 * *************************************************************************/
void stepper_switches_init(void)
{
	struct STEPPERSTUFF* p = &stepperstuff; // Convenience pointer

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim5;
   pT2base  = htim2.Instance;
   pT5base  = htim5.Instance;

   /* Circular buffer pointers. */
   pbegin = &switchxtion[0];
   padd   = &switchxtion[0];
   ptake  = &switchxtion[0];
   pend   = &switchxtion[SWITCHXITIONSIZE];

	p->swbits = GPIOE->IDR & 0xfc00; // Save current switch bits PE10:15

	/* Initialize the debounced limit switch state & flags. */
	if ((p->swbits & LimitSw_inside_NO_Pin) == 0)
	{ // Here NO contact is now closed.
		p->sw[LIMITDBINSIDE].dbs = 1; // Set debounced R-S
		p->sw[LIMITDBINSIDE].flag1  = 1; // Flag for stepper ISR
	}
	if ((p->swbits & LimitSw_outside_NO_Pin) == 0)
	{ // Here NO contact is now closed.
		p->sw[LIMITDBOUTSIDE].dbs = 1; // Set debounced R-S
		p->sw[LIMITDBOUTSIDE].flag1  = 1; // Flag for stepper ISR
	}




	EXTI->RTSR |=  0xfc00;  // Trigger on rising edge
	EXTI->FTSR |=  0xfc00;  // Trigger on falling edge
	EXTI->IMR  &= ~0xf000;  // Interrupt mask reg: disable 10:15
	EXTI->EMR  |=  0xfc00;  // Event mask reg: enable 10:15
	EXTI->PR   |=  0xfc00;  // Clear any pending

	return;
}

/* *************************************************************************
 * struct SWITCHXITION* stepper_switches_get(void);
 * @brief       : Get pointer to circular buffer if reading available
 * @return      : pointer to buffer entry; NULL = no reading
 * *************************************************************************/
struct SWITCHXITION* stepper_switches_get(void)
{
	struct SWITCHXITION* ptmp;
	if (ptake == padd) return NULL;
	ptmp = ptake;
	ptake += 1;
	if (ptake >= pend) ptake = pbegin;
	return ptmp;
}

/*#######################################################################################
 * ISR routine for EXTI
 * CH1 = OC stepper reversal
 * CH2 = OC faux encoder interrupts
 *####################################################################################### */
/*
LimitSw_inside_NO_Pin    GPIO_PIN_10
LimitSw_inside_NC_Pin    GPIO_PIN_11
LimitSw_outside_NO_Pin   GPIO_PIN_12
LimitSw_outside_NC_Pin   GPIO_PIN_13
OverrunSw_Inside_Pin     GPIO_PIN_14
OverrunSw_outside_Pin    GPIO_PIN_15
*/
void Stepper_EXTI15_10_IRQHandler(void)
{
	struct STEPPERSTUFF* p = &stepperstuff; // Convenience pointer
	struct SWITCHXITION* ptmp;
//HAL_GPIO_TogglePin(GPIOD,LED_ORANGE_Pin);

	/* Here, one or more PE10-15 inputs changed. */
	p->swbits = GPIOE->IDR & 0xfc00; // Save latest switch bits 10:15

	padd->sws = p->swbits;		// Save all switch contact bits
	padd->tim = pT2base->CNT;   // 32b timer time
	padd->cnt = pT5base->CNT;   // Encoder counter
    ptmp = padd;  // Save in case R-S change
	padd += 1;    // Advance in circular buffer
	if (padd >= pend) padd = pbegin; // Wrap-around

	/* Do R-S flip-flop type switch debouncing for limit switches. */
	if ((EXTI->PR & (LimitSw_inside_NO_Pin)) != 0)
	{ // Here Pending Register shows this switch transitioned
		EXTI->PR = LimitSw_inside_NO_Pin; // Reset request
		if ((p->swbits & LimitSw_inside_NO_Pin) == 0)
		{ // Here NO contact is now closed.
			if (p->sw[0].dbs != 1)
			{ // Here R-S flip-flop was reset
				p->sw[LIMITDBINSIDE].dbs = 1; // Set debounced R-S
				p->sw[LIMITDBINSIDE].posaccum_NO = p->posaccum.s32;
				p->sw[LIMITDBINSIDE].flag1  = 1; // Flag for stepper ISR
				p->sw[LIMITDBINSIDE].flag2 += 1; // Flag for task(?)
				/* Notification goes here. */
				ptmp->sws |= LIMITDBINSIDE;
HAL_GPIO_WritePin(GPIOD,LED_ORANGE_Pin,GPIO_PIN_SET);				
			}
		}
		return;
	}
	if ((EXTI->PR & (LimitSw_inside_NC_Pin)) != 0)
	{ // Here Pending Register shows this switch transitioned
		EXTI->PR = LimitSw_inside_NC_Pin; // Reset request
		if ((p->swbits & LimitSw_inside_NC_Pin) == 0)
		{ // Here NC contact is now closed.
			if (p->sw[LIMITDBINSIDE].dbs != 0)
			{ // Here R-S flip-flop was set
				p->sw[LIMITDBINSIDE].dbs = 0; // Reset debounced R-S
				p->sw[LIMITDBINSIDE].posaccum_NC = p->posaccum.s32;
				p->sw[LIMITDBINSIDE].flag1  = 1; // Flag for stepper ISR
				p->sw[LIMITDBINSIDE].flag2 += 1; // Flag for task(?)
				/* Notification goes here. */
				ptmp->sws |= LIMITDBINSIDE;

HAL_GPIO_WritePin(GPIOD,LED_ORANGE_Pin,GPIO_PIN_RESET);				

			}
		}
		return;
	}

	if ((EXTI->PR & (LimitSw_outside_NO_Pin)) != 0)
	{ // Here Pending Register shows this switch transitioned
		EXTI->PR = LimitSw_outside_NO_Pin; // Reset request
		if ((p->swbits & LimitSw_outside_NO_Pin) == 0)
		{ // Here NO contact is now closed.
			if (p->sw[LIMITDBOUTSIDE].dbs != 1)
			{ // Here R-S flip-flop was reset
				p->sw[LIMITDBOUTSIDE].dbs = 1; // Set debounced R-S
				p->sw[LIMITDBOUTSIDE].posaccum_NO = p->posaccum.s32;
				p->sw[LIMITDBOUTSIDE].flag1  = 1; // Flag for stepper ISR
				p->sw[LIMITDBOUTSIDE].flag2 += 1; // Flag for task(?)
				/* Notification goes here. */	
				ptmp->sws |= LIMITDBOUTSIDE;

HAL_GPIO_WritePin(GPIOD,LED_RED_Pin,GPIO_PIN_SET);			
			}
		}
		return;
	}
	if ((EXTI->PR & (LimitSw_outside_NC_Pin)) != 0)
	{ // Here Pending Register shows this switch transitioned
		EXTI->PR = LimitSw_outside_NC_Pin; // Reset request
		if ((p->swbits & LimitSw_outside_NC_Pin) == 0)
		{ // Here NC contact is now closed.
			if (p->sw[LIMITDBOUTSIDE].dbs != 0)
			{ // Here R-S flip-flop was set
				p->sw[LIMITDBOUTSIDE].dbs = 0; // Reset debounced R-S
				p->sw[LIMITDBOUTSIDE].posaccum_NC = p->posaccum.s32;
				p->sw[LIMITDBOUTSIDE].flag1  = 1; // Flag for stepper ISR
				p->sw[LIMITDBOUTSIDE].flag2 += 1; // Flag for task(?)
				/* Notification goes here. */			
				ptmp->sws |= LIMITDBOUTSIDE;

HAL_GPIO_WritePin(GPIOD,LED_RED_Pin,GPIO_PIN_RESET);							
			}
		}
		return;
	}

	/* These are the NO contacts on overrun switches. */
	if ((EXTI->PR & (OverrunSw_Inside_Pin)) != 0)
	{ // Here Pending Register shows this switch transitioned
		EXTI->PR = OverrunSw_Inside_Pin; // Reset request
		/* Notification goes here. */
		return;
	}
		if ((EXTI->PR & (OverrunSw_outside_Pin)) != 0)
	{ // Here Pending Register shows this switch transitioned
		EXTI->PR = OverrunSw_outside_Pin; // Reset request
		/* Notification goes here. */
		return;
	}

	return;
}

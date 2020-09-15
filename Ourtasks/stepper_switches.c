/******************************************************************************
* File Name          : stepper_switches.c
* Date First Issued  : 09/16/2020
* Description        : Levelwind function w STM32CubeMX w FreeRTOS
*******************************************************************************/
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
//HAL_GPIO_TogglePin(GPIOD,LED_ORANGE_Pin);


	/* Here, one or more PE10-15 inputs changed. */
	p->swbits = GPIOE->IDR & 0xfc00; // Save latest switch bits 10:15

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

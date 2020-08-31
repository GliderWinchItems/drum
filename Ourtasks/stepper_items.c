/******************************************************************************
* File Name          : stepper_items.c
* Date First Issued  : 07/31/2020
* Description        : Stepper motor associated items
*******************************************************************************/
/*

<<<<<<< HEAD
08/23/2020 druminversion branch started
=======
08/23/2020 drumupdate100K
>>>>>>> 36cbe0c798364e1d025f7341a88f88b570609b2a

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

Drum encoder: TIM2CH1. Pullup resistors
PA0 - Encoder channel A
PA1 - Encoder channel B

TIM2 32b (84 MHz) capture mode (interrupt)
      CH3 PA2 input capture: encoder A (TIM5 PA0)
      CH4 PA3 input capture: encoder B (TIM5 PA1)
      CH2 PB3 input capture: encoder Z

  TIM5 32b encoder counter (no interrupt)
      CH3 PA0 encoder config: encoder A (TIM2 PA2)
      CH4 PA1 encoder config: encoder B (TIM2 PA3)

  TIM9 (168 MHz) Delayed stepper pulse (no interrupt)
      CH1 PE5 PWM/OPM: Stepper pulse

  TIM13 (84 MHz) Solenoid FET drive (no interrupt)
      CH1 PA6 PWM (4 KHz)

  TIM14 (84 MHz) Oscope sync (no interrupt)
      CH1 PA7 PWM/OPM; Scope pulse

  TIM4 (84 MHz) (Interrupts. Same priority as TIM2)
      CH1 output compare no output: Stepper reversal
      CH2 output compare no output: faux encoder transition

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
#define TIM9PWMCYCLE (168*10-30)   // 10us pwm cycle
#define TIM9PULSEDELAY (TIM9PWMCYCLE - (168*3))

TIM_TypeDef  *pT2base; // Register base address 
TIM_TypeDef  *pT4base; // Register base address 
TIM_TypeDef  *pT9base; // Register base address 
TIM_TypeDef  *pT14base; // Register base address 

/* Struct with all you want to know. */
struct STEPPERSTUFF stepperstuff;

/* CAN msgs */

enum cididx
{
	CID_STEPPER_HB	
};

/* *************************************************************************
 * void stepper_idx_v_struct_hardcode_params(void);
 * 
 * @brief       : Initialization
 * *************************************************************************/
void stepper_idx_v_struct_hardcode_params(struct STEPPERSTUFF* p)
{

    p->lc.clfactor  = 655.36f; // 100% gives max speed
    p->lc.cltimemax = 512;        // Timeout swtime ticks max
    p->lc.hbct      = 64;         // Number of swctr ticks between heartbeats
    p->ocinc        = 1680;       // 20 us OC interrupt duration

    /* Stepper sends these CAN msgs. */
    p->lc.cid_hb_stepper      = 0xE4A00000;   // CANID_HB_STEPPER: U8_U32, Heartbeat Status, stepper position accum');

    /* Pre-load CAN msg id and dlc. */
	p->canmsg[CID_STEPPER_HB].can.id  = p->lc.cid_hb_stepper;
	p->canmsg[CID_STEPPER_HB].can.dlc = 5; // U8_U32 payload
	p->canmsg[CID_STEPPER_HB].pctl = pctl0;	
	p->canmsg[CID_STEPPER_HB].maxretryct = 8;
    return;
}

/* *************************************************************************
 * void stepper_items_init(TIM_HandleTypeDef *phtim);
 * phtim = pointer to timer handle
 * @brief	: Initialization of channel increment
 * *************************************************************************/
void stepper_items_init(void)
{
	struct STEPPERSTUFF* p = &stepperstuff; // Convenience pointer
	stepper_idx_v_struct_hardcode_params(p);

    p->ledctr1   = 0;
    p->ledctr2   = 0;
    p->accumpos  = 0; // Position accumulator
    p->cltimectr = 0;
    p->hbctr     = 0;

	/* Bit positions for low overhead toggling. */
	p->ledbit1= (LED_GREEN_Pin);
	p->ledbit2= (LED_ORANGE_Pin);

	/* Save base addresses of timers for faster use later. */
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim14;
	pT2base  = htim2.Instance;
	pT4base  = htim4.Instance;
	pT9base  = htim9.Instance;
	pT14base = htim14.Instance;

/* ### NOTE ### These might override STM32CubeMX settings. ### */
	/* Generate pulse for stepper controller (PU line) */
	pT9base->DIER = 0;// None //0x2; // CH1 interrupt enable
	pT9base->CCR1 = TIM9PULSEDELAY; // Delay count
	pT9base->ARR  = (TIM9PWMCYCLE - 1); // (10 us)
	pT9base->CCER = 0x3; // OC active high; signal on pin

/* ### NOTE ### These might override STM32CubeMX settings. ### */
	/* Generate pulse for trigger scope swseep. */
	pT14base->DIER = 0;// None //0x2; // CH1 interrupt enable
	pT14base->CCR1 = 1; // Delay count
	pT14base->ARR  = (TIM9PWMCYCLE - 1); // (10 us)
	pT14base->CCER = 0x3; // OC active high; signal on pin

	/* TIM4 (will become the stepper indexing interrupt source (~2KHz)). */
	pT4base->DIER = 0x4; // CH2 interrupt enable, only.
	pT4base->CCR1 = pT4base->CNT + 100; // 1 ms delay
	pT4base->ARR  = 0xffff;

	/* TIM2 CH1 runs at fixed interrupt rate. */
	pT2base->DIER = 0x2; // CH1 interrupt enable, only.
	pT2base->CCR1 = pT2base->CNT + 100; // OC2 register: set short delay
//	pT2base->ARR  = 0xffffffff; // Auto-reload-register

	/* Start TIM2 counter. */
	pT2base->CR1 |= 1;
	return;
}
/* *************************************************************************
 * void stepper_items_timeout(void);
 * @brief	: Check for loss of CL CAN msgs
 * *************************************************************************/
void stepper_items_timeout(void)
{
	struct STEPPERSTUFF* p = &stepperstuff; // Convenience pointer

/* This is shamefull, but works until MailboxTask problem fixed. */		
extern struct CANRCVBUF dbgcan;
struct CANRCVBUF* pcan = &dbgcan;

	p->cltimectr += 1;
	if (p->cltimectr >= p->lc.cltimemax)
	{ // We timed out! Stop the stepper
		p->cltimectr = p->lc.cltimemax;

		/* Set enable bit which turns FET on, which disables stepper. */
		p->enflag = (2 << 16); // Set bit with BSRR storing

		/* Set last CAN msg to have enable bit off so that
		   stepperCLupdate execution will continue to disable. */
		pcan->cd.uc[0] &= ~ENBIT; // Clear enable bit in last CAN msg

		/* Bits positioned for updating PB BSRR register. */
		p->iobits = p->drflag | p->enflag;
	}
	return;
}
/* *************************************************************************
 * void stepper_items_CANsend(void);
 * @brief   : Send CAN msgs for stepper
 * *************************************************************************/
 void stepper_items_CANsend(void)
 {
 	struct STEPPERSTUFF* p = &stepperstuff; // Convenience pointer
 	p->hbctr += 1;
 	if (p->hbctr >= p->lc.hbct)
 	{
 		p->hbctr = 0;

 		/* Setup CAN msg */
 		p->canmsg[CID_STEPPER_HB].can.cd.uc[0] = p->stepperstatus;
 		p->canmsg[CID_STEPPER_HB].can.cd.uc[1] = p->accumpos >>  0;
 		p->canmsg[CID_STEPPER_HB].can.cd.uc[2] = p->accumpos >>  8;
 		p->canmsg[CID_STEPPER_HB].can.cd.uc[3] = p->accumpos >> 16;
 		p->canmsg[CID_STEPPER_HB].can.cd.uc[4] = p->accumpos >> 24;

 		/* Queue CAN msg to send. */
		xQueueSendToBack(CanTxQHandle,&p->canmsg[CID_STEPPER_HB],4);	
 	}
 	return;
 }
/* *************************************************************************
 * void stepper_items_clupdate(struct CANRCVBUF* pcan);
 * @param 	: pcan = pointer to CAN msg struct
 * @brief	: Initialization of channel increment
 * *************************************************************************/
uint32_t dbgid;
uint32_t dbgstep1;
uint32_t dbgstep2;
struct CANRCVBUF* dbgpcan;

void stepper_items_clupdate(struct CANRCVBUF* pcan)
{
if (pcan == NULL) 
{
	dbgstep1 += 1;
		return;
}
dbgstep2 += 1;

//if (pcan->id != 0xE4600000) morse_trap(7373);
dbgpcan = pcan;
dbgid = pcan->id;

/* This is shamefull, but works until MailboxTask problem fixed. */		
//extern struct CANRCVBUF dbgcan;
//pcan = &dbgcan;

		struct STEPPERSTUFF* p = &stepperstuff; // Convenience pointer

	/* Reset loss of CL CAN msgs timeout counter. */
	p->cltimectr = 0; 

	/* Extract float from payload */
	p->pf.u8[0] = pcan->cd.uc[1];
	p->pf.u8[1] = pcan->cd.uc[2];
	p->pf.u8[2] = pcan->cd.uc[3];
	p->pf.u8[3] = pcan->cd.uc[4];

	p->clpos = p->pf.f; // Redundant
	p->pay0 = pcan->cd.uc[0];

	/* Save bits for direction and enable. */
	// Direction bit
	if ((pcan->cd.uc[0] & DRBIT) == 0)
	{
		p->drflag = (1 << 16); // Reset
		p->drsign = 1;
	}
	else
	{
		p->drflag = 1; // Set
		p->drsign = -1;
	}

	// Enable bit
	if ((pcan->cd.uc[0] & ENBIT) != 0)
		p->enflag = (2 << 16); // Reset
	else
		p->enflag = 2; // Set

	/* Bits positioned for updating PB BSRR register. */
	p->iobits = p->drflag | p->enflag;

/* Convert CL position (0.0 - 100.0) to output comnpare duration increment. */

	p->speedcmdf = p->clpos * p->lc.clfactor;

	p->ocnxt = p->speedcmdf; // Convert to integer

return;
}
/*#######################################################################################
 * ISR routine for TIM9 [Normally no interrupt; interrupt for test purposes]
 *####################################################################################### */
void stepper_items_TIM9_IRQHandler(void)
{
	pT9base->SR = ~(0x1F);	// Reset CH1 flag (and all flags)
		// Visual check for debugging
			stepperstuff.ledctr2 += 1; // Slow LED toggling rate
			if (stepperstuff.ledctr2 > 1000)
			{
	  			stepperstuff.ledctr2 = 0;
  				// Toggle LED on/off
				stepperstuff.ledbit2 ^= (LED_ORANGE_Pin | (LED_ORANGE_Pin << 16));
			}
	return;
}
/*#######################################################################################
 * ISR routine for TIM2
 * CH3 - IC encoder channel A
 * CH4 - IC encoder channel B
 * CH1 - IC encoder channel Z
 *####################################################################################### */
void stepper_items_TIM2_IRQHandler(void)
{

	struct STEPPERSTUFF* p = &stepperstuff; // Convenience pointer

	/* Fix rate for this OC interrupt. */
	if ((pT2base->SR & 0x2) != 0)
	{
		pT2base->SR = ~(0x2);	// Reset CH1 flag

		// Duration increment computed from CL CAN msg
		pT2base->CCR1 += p->ocinc; // Schedule next interrupt

		// Update direction and enable i/o pins
		Stepper__DR__direction_GPIO_Port->BSRR = p->iobits;

		/* If enable flag resets i/o pin, then send step pulses. */
		if ((p->enflag & (2 << 0)) == 0) 
		{ // Here, motor is enabled

			// Drum encoder accumulator: add increment to accumulator
			p->drumaccum += p->ocnxt * p->drsign;

			/* When accumulator upper 16 changes generate a pulse. */
			if ((p->drumaccum & 0xffff0000) != (p->drumaccum_prev))
			{ // Here carry from low 16b to high 16b
				p->drumaccum_prev = (p->drumaccum & 0xffff0000);

				// Start TIM9 to generate a delayed pulse.
				pT9base->CR1 = 0x9; 

				// Start TIM14 to start scope sync pulse (PE5)
				pT14base->CR1 = 0x9; 

				// Visual check for debugging
				p->ledctr1 += 1; // Slow LED toggling rate
				if (p->ledctr1 > 1000)
				{
	 					p->ledctr1 = 0;

  					// Toggle LED on/off
					p->ledbit1 ^= (LED_GREEN_Pin | (LED_GREEN_Pin << 16));
					LED_GREEN_GPIO_Port->BSRR = p->ledbit1;
				}
			}
		}
	}
// NOTE: should not come here as only CH1 interrupt enabled	
	if ((pT2base->SR & 0x1D) != 0)
	{
		pT2base->SR = ~(0x1D);	// Reset all but CH1 flag
	}
	return;
}
/*#######################################################################################
 * ISR routine for TIM4
 * CH1 = OC stepper reversal
 *####################################################################################### */

void stepper_items_TIM4_IRQHandler(void)
{
	pT4base->SR = ~(0x1F);	// Reset all flags
	return;
}

/******************************************************************************
* File Name          : stepper_items.c
* Date First Issued  : 07/31/2020
* Description        : Stepper motor associated items
*******************************************************************************/
/*
09/10/2020 realfaux branch

09/03/2020 testencoder branch 
Mods reminder list 
- CH2 fauxencoder -> CH1 testencoder (output compare)
- PA0-PA2 high is about 3.9v because PA0 pushbutton has 220K pull-down resistor
  PA1-PA3 high is 4.48v 
  PB3 - high is 5.05v

08/23/2020 druminversion branch started

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
#include "DTW_counter.h"
#include "drum_items.h"
#include "stepper_switches.h"

#define TIM3CNTRATE 84000000   // TIM3 counter rate (Hz)
#define UPDATERATE 100000      // 100KHz interrupt/update rate
#define TIM3DUR  (TIM3CNTRATE/UPDATERATE) // 1680 counts per interrupt

#define TIM9CNTRATE 168000000 // TIM9 counter rate (Hz)
#define TIM9PWMCYCLE (168*10-30)   // 10us pwm cycle
#define TIM9PULSEDELAY (TIM9PWMCYCLE - (168*3))

#define GSM
#define DEBUG

TIM_TypeDef  *pT2base; // Register base address 
TIM_TypeDef  *pT4base; // Register base address 
TIM_TypeDef  *pT5base; // Register base address 
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
 * @brief       : Initialization of parameters
 * *************************************************************************/
void stepper_idx_v_struct_hardcode_params(struct STEPPERSTUFF* p)
{
   p->lc.clfactor    = 168E3;    // CL scaling: 100% = 50 us
   p->lc.cltimemax   = 512;   // Number of software timeout ticks max
   p->lc.hbct        = 64;    // Number of swctr ticks between heartbeats
   p->lc.Ka          = 8;     // Reversal rate
   p->lc.Nr          = 3500;  // Sweep rate to reversal rate ratio
   p->lc.Ks          = p->lc.Nr *  p->lc.Ka; // Sweep rate (Ks/65536) = stepper pulses per encoder edge
   p->lc.Lplus       =   8000;
   p->lc.Lminus      =  -8000;

   /* Stepper sends these CAN msgs. */
   p->lc.cid_hb_stepper      = 0xE4A00000;   // CANID_HB_STEPPER: U8_U32, Heartbeat Status, stepper position accum');

   /* Pre-load CAN msg id and dlc. */
   // Stepper heartbeat
   p->canmsg[CID_STEPPER_HB].can.id  = p->lc.cid_hb_stepper; // CAN id.
   p->canmsg[CID_STEPPER_HB].can.dlc = 5;    // U8_U32 payload
   p->canmsg[CID_STEPPER_HB].pctl = pctl0;     // CAN1 control block pointer
   p->canmsg[CID_STEPPER_HB].maxretryct = 8; // Max retry count
   return;
}
/* *************************************************************************
 * static void  switches_init(struct STEPPERSTUFF* p);
 * @brief   : Initialization for limit and overrun switches
 * *************************************************************************/
static void  switches_init(struct STEPPERSTUFF* p)
{
	p->swbits = GPIOE->IDR & 0xfc00; // Save current switch bits PE10:15

	EXTI->RTSR |=  0xfc00;  // Trigger on rising edge
	EXTI->FTSR |=  0xfc00;  // Trigger on falling edge
	EXTI->IMR  &= ~0xf000;  // Interrupt mask reg: disable 10:15
	EXTI->EMR  |=  0xfc00;  // Event mask reg: enable 10:15
	EXTI->PR   |=  0xfc00;  // Clear any pending

	return;
}
/* *************************************************************************
 * void stepper_items_init(void);
 * @brief   : Initialization
 * *************************************************************************/
void stepper_items_init(void)
{
   struct STEPPERSTUFF* p = &stepperstuff; // Convenience pointer
   
   // Initialize hardcoded parameters (used in some computations below)
   stepper_idx_v_struct_hardcode_params(p);

   // Initialize for limit and overrun switches
   switches_init(p);

   p->ledctr1   = 0;
   p->ledctr2   = 0;
   // Position accumulator initial value. Reference paper for the value employed.
   p->posaccum.s32 = (p->lc.Lminus << 16) - (p->lc.Nr * (p->lc.Nr - 1) * p->lc.Ka) / 2;   
   p->posaccum_prev = p->posaccum.s32;
   // initialize 32-bit values for Lplus32 and Lminus32. Reference paper.
   p->Lminus32 = p->lc.Lminus << 16;
   p->Lplus32  = p->Lminus32 
      + (((p->lc.Lplus - p->lc.Lminus) << 16) / p->lc.Ks) * p->lc.Ks;
   p->velaccum.s32 = 0;      // Velocity accumulator initial value  
   p->drbit      = 0;        // Drum direction bit
   p->drbit_prev = p->drbit;     
   p->cltimectr  = 0;
   p->hbctr      = 0;
   p->ocinc      = 8400000;	// Default 1/10 sec duration
   p->ocidx      =   42000; // Default indexing increment 500 ms
   p->dtwmin     = 0xffffffff;

   /* Bit positions for low overhead toggling. */
   p->ledbit1= (LED_GREEN_Pin);
   p->ledbit2= (LED_ORANGE_Pin);

   /* Save base addresses of timers for faster use later. */
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim14;
   pT2base  = htim2.Instance;
   pT4base  = htim4.Instance;
   pT5base  = htim5.Instance;
   pT9base  = htim9.Instance;
   pT14base = htim14.Instance;

/* ### NOTE ### These might override STM32CubeMX settings. ### */
   /* Generate pulse for stepper controller (PU line) */
   pT9base->DIER = 0;// No interrupt
   pT9base->CCR1 = TIM9PULSEDELAY; // Delay count
   pT9base->ARR  = (TIM9PWMCYCLE - 1); // (10 us)
   pT9base->CCER = 0x3; // OC active high; signal on pin

/* ### NOTE ### These might override STM32CubeMX settings. ### */
   /* Generate pulse for trigger scope swseep. */
   pT14base->DIER = 0;// No interrupt
   pT14base->CCR1 = 1; // Delay count
   pT14base->ARR  = (TIM9PWMCYCLE - 1); // (10 us)
   pT14base->CCER = 0x3; // OC active high; signal on pin

   /* TIM4 (will become the stepper indexing interrupt source (~2KHz)). */
   pT4base->DIER = (1<<3) || (1<<1); // CH1,CH3
   pT4base->CCR2 = pT4base->CNT + 100; // 1 ms delay
   pT4base->ARR  = 0xffff;

	/* TIM2 Shaft encoder input capture times & output caputre indexing interrupts. */
	pT2base->CCER |= 0x1100; // Input capture active: CH3,4
	pT2base->DIER = 0xA; // CH1,3 interrupt enable
	pT2base->CCR1 = pT2base->CNT + 1000; // 1 short delay
	pT2base->ARR  = 0xffffffff;

	/* Start counters. */
	pT2base->CR1 |= 1;  // TIM2 CH1 oc, CH3 ic/oc
	pT5base->CR1 |= 1;  // TIM5 encoder CH1 CH2 (not interrupt)

	/* Enable limite and overrun switch interrupts EXTI15_10. */
	EXTI->IMR  |= 0xf000;  // Interrupt mask reg: enable 10:15
   return;
}
/* *************************************************************************
 * void stepper_items_timeout(void);
 * @brief   : Check for loss of CL CAN msgs
 * *************************************************************************/
void stepper_items_timeout(void)
{
   struct STEPPERSTUFF* p = &stepperstuff; // Convenience pointer

   p->cltimectr += 1;
   if (p->cltimectr >= p->lc.cltimemax)
   { // We timed out! Stop the stepper
      p->cltimectr = p->lc.cltimemax;

      /* Set enable bit which turns FET on, which disables stepper. */
      p->enflag = (2 << 16); // Set bit with BSRR storing

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
      p->canmsg[CID_STEPPER_HB].can.cd.uc[1] = p->posaccum.s32 >>  0;
      p->canmsg[CID_STEPPER_HB].can.cd.uc[2] = p->posaccum.s32 >>  8;
      p->canmsg[CID_STEPPER_HB].can.cd.uc[3] = p->posaccum.s32 >> 16;
      p->canmsg[CID_STEPPER_HB].can.cd.uc[4] = p->posaccum.s32 >> 24;

      /* Queue CAN msg to send. */
      xQueueSendToBack(CanTxQHandle,&p->canmsg[CID_STEPPER_HB],4);   
   }
   return;
 }
/* *************************************************************************
 * void stepper_items_clupdate(struct CANRCVBUF* pcan);
 * @param   : pcan = pointer to CAN msg struct
 * @brief   : Initialization of channel increment
 * *************************************************************************/
void stepper_items_clupdate(struct CANRCVBUF* pcan)
{
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

   /* Convert CL position (0.0 - 100.0) to output comnpare duration increment. */
#define MAXDURF (84E5f) // 1/10sec per faux encoder interrupt max duration
   p->focdur = (p->lc.clfactor / p->clpos);
   if ( p->focdur > (MAXDURF))
   { 
      p->focdur = MAXDURF; // Hold at max
   }
   p->ocinc = p->focdur;   // Convert to integer

  /* Configure TIM2CH3 to be either input capture from encoder, or output compare (no pin). */
   // Each ZTBIT pushbutton press toggles beween IC and OC modes
   p->ocicbit = (pcan->cd.uc[0] & ZTBIT);
   if (p->ocicbit != p->ocicbit_prev)
   { // Here, the PREP bit changed
   	  	p->ocicbit_prev = p->ocicbit;
   	  	if (p->ocicbit != 0)
   	  	{ // Here. PREP bit went from off to on
   	  		pT2base->DIER &= ~0x80; // Disable TIM2CH3 interrupt
			if ((pT2base->CCMR2 & 0x1) != 0)
   	  		{ // Here, currently using encoder input capture
p->xbit =2;   	  			
	   	  		// Setup for output compare
	   	  		pT2base->CCER  |=  (1<<11); // CC3NP: Configure as output
	   	  		pT2base->CCER  &= ~(1<<8);  // CC3E = 0; Turn channel off
   		  		pT2base->CCMR2 &= ~(0xff << 0); // Change to Output compare, no pin
   	  			pT2base->CCR3 = pT2base->CNT + p->ocinc; // Schedule next faux encoder interrupt
   		  	}
   	  		else
	   	  	{ // Here, currently using output compare
   		  		// Setup for input capture
p->xbit = 1;
	   	  		pT2base->CCER  &= ~((1<<8) || (1<<11)); // CC3E, CC3NP = input
				pT2base->CCMR2 |= 0x01; // Input capture mapped to TI3
				pT2base->SR = ~(1<<3);	// Reset CH3 flag if on
	   	  		pT2base->CCER  |= (1<<8); // Capture enabled on pin.
   		  	}
   		  	pT2base->DIER |= 0x80; // Enable TIM2CH3 interrupt
   		}
   }

   /* Payload byte bits for direction and enable. */

   /* Direction bit of output compare (CL control) comes from
        CAN msg. 
      Direction bit for Input capture (encoder control) comes from
        TIM5 CR1 bit DIR which is setup in the ISR. */
   if ((pT2base->CCMR2 & 0x1) == 0) // Which mode?
   { // Here TIM2CH3 mode is output compare. Use CAN payload bit
   	   // Output capture (no pin) is TIM2CH3 mode
   	   if ((pcan->cd.uc[0] & DRBIT) == 0)
   	   {
      		p->drflag = (1 << 16); // Reset
      		p->drbit  = 0;
   		}
   		else
   		{
   			p->drflag = 1; // Set
      		p->drbit  = 1;
   		}
   	}

   // Motor Enable bit
   if ((pcan->cd.uc[0] & ENBIT) != 0)
      p->enflag = (2 << 16); // Reset
   else
      p->enflag = 2; // Set

   /* Bits positioned for updating PB BSRR register. */
   p->iobits = p->drflag | p->enflag;
   return;  
}
/*#######################################################################################
 * ISR routine for TIM2
 * CH1 - OC timed interrupts  indexing interrupts
 * CH2 - OC timed interrupts  or, FreeRTOS task forces this interrupt?
 * CH3 - IC encoder channel A or, OC generates faux encoder interrupts
 * CH4 - IC encoder channel B not used in this version
 *####################################################################################### */
void stepper_items_TIM2_IRQHandler(void)
{
   struct STEPPERSTUFF* p = &stepperstuff; // Convenience pointer

   // Capture DTW timer for cycle counting
      p->dtwentry = DTWTIME;

     /* TIM2CH3 = encodertimeA PA2 TIM5CH1 PA0	*/
	if ((pT2base->SR & (1<<3)) != 0) // CH3 Interrupt flag?
	{ // Yes, either encoder channel A, or output compare
		pT2base->SR = ~(1<<3);	// Reset CH3 flag

		/* Was this interrupt due to encoder input capture or output compare?. */
		if ((pT2base->CCMR2 & 0x1) == 0)
		{ // Here we are using TIM2CH3 as OC compare instead of input capture. */
	 		// Duration increment computed from CL CAN msg
    		pT2base->CCR3 += p->ocinc; // Schedule next faux encoder interrupt
    		// Make faux encoder counter 
    		drumstuff.decA.cur.tim = pT2base->CCR3;   // Save current time
    		if (p->drbit == 0)
				drumstuff.decA.cur.cnt += 1;  
			else
				drumstuff.decA.cur.cnt -= 1; 
		}
		else
		{ // Here, encoder driven input capture. Save for odometer and speed
			drumstuff.decA.cur.tim = pT2base->CCR3;   // Save current time
			drumstuff.decA.cur.cnt = pT5base->CNT;    // Save current encoder count
			p->drbit = (pT5base->CR1 & 0x10); // Encoding DIR (direction) (0|non-zero)
		}

		/* Here: either encoder channel A driven input capture interrupt, or 
		    CL controlled timer output compare interrupt. */

		/* During indexing the encoder input capture or output compare
		     interrupts do not drive
		   the stepper. */
		if (p->flagindexing == 0)
		{ // Here, not indexing
/* ------------ Tracking/sweep --------------------------------------------------- */
			// Update enable i/o pin
      		Stepper__DR__direction_GPIO_Port->BSRR = p->enflag;

      // forward (stepper) direction means position accumulator is increasing
      // negative direction means position accumulator is decreasing
      // drbit = 0 means positive drum direction

      // update velocity integrator
      
      // invert velocity integrator if Drum Direction has changed
      		if (p->drbit != p->drbit_prev)
      		{  // Drum direction has changed
         		p->drbit_prev = p->drbit;   // save new direction
         		p->velaccum.s32 = -p->velaccum.s32; // invert velocity value
      		} 
            
      		else if (p->posaccum.s32 >= p->Lplus32 || p->posaccum.s32 <= p->Lminus32)     
      		{
         		if (p->posaccum.s32 >= p->Lplus32)
         		{  // in positive level-wind reversal region
            		p->velaccum.s32 -= p->lc.Ka;  // apply negative acceleration 
		        }
        		else
	        	{  // in negative level-wind reversal region
    	        	p->velaccum.s32 += p->lc.Ka;  // apply positive acceleration
        	 	}
      		}
         
	      	p->dbg1 = p->velaccum.s32;
    	  	p->dbg2 = p->posaccum.s16[1];
	      	p->dbg3 = p->posaccum.u16[0];
         
    	  	// update position integrator
	      	p->posaccum.s32 += p->velaccum.s32;
      
    	  	/* When accumulator upper 16b changes generate a stepper pulse. */
      		if ((p->posaccum.s16[1]) != (p->posaccum_prev))
      		{ // Here carry/borrow from low 16b to high 16b
         		p->posaccum_prev = (p->posaccum.s16[1]);

         		/* Skip stepper pulses if motor not enabled. */
         		if ((p->enflag & (2 << 0)) == 0) 
         		{  // set direction based on sign of Velocity integrator
            		// depends on velocity magnitude being <= 2^16
            		Stepper__DR__direction_GPIO_Port->BSRR = (p->velaccum.s16[1])
               			? 1 : (1 << 16);

            		// Start TIM9 to generate a delayed pulse.
            		pT9base->CR1 = 0x9;
         		}
      		}
/* ------------ End tracking section --------------------------------------------- */
		}

// Debugging
p->ledctr1 += 1;
if ((pT2base->CCMR2 & 0x1) != 0)
  p->ledctr2 = 0;
else
  p->ledctr2 = 500;
if (p->ledctr1 > p->ledctr2)			
{
	p->ledctr1 = 0;
	HAL_GPIO_TogglePin(GPIOD,LED_GREEN_Pin); // GREEN LED			
}

	}

	/* Indexing timer interrupt. */
	if ((pT2base->SR & (1<<1)) != 0) // CH1 Interrupt flag?
	{ // Yes, OC drive 
		pT2base->SR = ~(1<<1);	// Reset CH1 flag

 		// Duration increment computed from CL CAN msg
   		pT2base->CCR1 += p->ocidx; // Schedule next indexing interrupt

		if (p->flagindexing != 0)
		{ // Here, indexing
			/* == indexing section goes here. == */
		}
   	}

   p->dtwdiff = DTWTIME - p->dtwentry;
   if (p->dtwdiff > p->dtwmax) p->dtwmax = p->dtwdiff;
   else if (p->dtwdiff < p->dtwmin) p->dtwmin = p->dtwdiff;

   return;
}

/*#######################################################################################
 * ISR routine for TIM4
 * CH1 = OC stepper reversal
 * CH2 = OC faux encoder interrupts
 *####################################################################################### */
void stepper_items_TIM4_IRQHandler(void)
{
   pT4base->SR = ~(0x1F);  // Reset all flags
   return;
}

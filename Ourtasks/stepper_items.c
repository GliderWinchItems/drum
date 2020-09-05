/******************************************************************************
* File Name          : stepper_items.c
* Date First Issued  : 07/31/2020
* Description        : Stepper motor associated items
*******************************************************************************/
/*

09/03/2020 testencoder branch started
Mods reminder list 
- CH2 fauxencoder -> CH1 testencoder (output compare)


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
   p->lc.clfactor  	= 168E3; 	// CL scaling: 100% = 50 us
   p->lc.cltimemax 	= 512; 	// Number of software timeout ticks max
   p->lc.hbct      	= 64;    // Number of swctr ticks between heartbeats
   p->lc.Ka 			= 25;		// Reversal rate
   p->lc.Nr 			= 1000;	//	Sweep rate to reversal rate ratio
   p->lc.Ks        	= p->lc.Nr *  p->lc.Ka; // Sweep rate (Ks/65536) = stepper pulses per encoder edge
   p->lc.Lplus			=  2000;
   p->lc.Lminus		= -2000;
   /* Stepper sends these CAN msgs. */
   p->lc.cid_hb_stepper      = 0xE4A00000;   // CANID_HB_STEPPER: U8_U32, Heartbeat Status, stepper position accum');

   /* Pre-load CAN msg id and dlc. */
  	// Stepper heartbeat
	p->canmsg[CID_STEPPER_HB].can.id  = p->lc.cid_hb_stepper; // CAN id.
	p->canmsg[CID_STEPPER_HB].can.dlc = 5;    // U8_U32 payload
	p->canmsg[CID_STEPPER_HB].pctl = pctl0;	  // CAN1 control block pointer
	p->canmsg[CID_STEPPER_HB].maxretryct = 8; // Max retry count
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
	
	//	Initialize hardcoded parameters (used in some computations below)
	stepper_idx_v_struct_hardcode_params(p);

   p->ledctr1   = 0;
   p->ledctr2   = 0;
   // Position accumulator initial value. Reference paper for the value employed.
   p->posaccum.s32 = (p->lc.Lminus << 16) - (p->lc.Nr * (p->lc.Nr - 1) * p->lc.Ka) / 2; 	
   p->posaccum_prev = p->posaccum.s32;
   p->velaccum.s32 = 0;	//p->lc.Ks;	//	Velocity accumulator initial value
   p->drbit = 0;				//	Drum direction bit
   p->drbit_prev = p->drbit;		
   p->cltimectr = 0;
   p->hbctr     = 0;
	p->ocinc = 8400000;
	p->dtwmin = 0xffffffff;

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
	pT5base  = htim4.Instance;
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
	pT4base->CCR2 = pT4base->CNT + 100; // 1 ms delay
	pT4base->ARR  = 0xffff;

	/* TIM2 Stepper reversal timer and faux encoder transitions. */
	pT2base->DIER = 0x1E; // CH1,2,3,4 interrupt enables
	pT2base->CCR1 = pT2base->CNT + 100; // 1 ms delay
	pT2base->ARR  = 0xffffffff;

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
 * @param 	: pcan = pointer to CAN msg struct
 * @brief	: Initialization of channel increment
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

	/* Save bits for direction and enable. */
	// Direction bit
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

	// Enable bit
	if ((pcan->cd.uc[0] & ENBIT) != 0)
		p->enflag = (2 << 16); // Reset
	else
		p->enflag = 2; // Set

	/* Bits positioned for updating PB BSRR register. */
	p->iobits = p->drflag | p->enflag;

/* Convert CL position (0.0 - 100.0) to output comnpare duration increment. */
#define MAXDURF (84E5f) // 1/10sec per faux encoder interrupt
	p->focdur = (p->lc.clfactor / p->clpos);
	if ( p->focdur > (MAXDURF))
	{ 
		p->focdur = MAXDURF; // Hold at max
	}
	p->ocinc = p->focdur;	// Convert to integer
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

	/* TIM2CH3 = encodertimeA PA2	*/
	if ((pT2base->SR & (1<<3)) != 0) // Output compare?
	{
				pT2base->SR = ~(1<<3);	// Reset CH3 flag

	}

	/* TIM2CH4 = encodertimeB PA3	*/
	if ((pT2base->SR & (1<<4)) != 0) // Output compare?
	{
				pT2base->SR = ~(1<<4);	// Reset CH4 flag

	}

	/* TIM2CH2 = encodertimeZ PB3	*/
	if ((pT2base->SR & (1<<2)) != 0) // Output compare?
	{
				pT2base->SR = ~(1<<2);	// Reset CH1 flag

	}



	/* Faux encoder transition interrupt. */
	/* TIM2CH1 = output compare, no pin. */
	if ((pT2base->SR & 0x2) != 0) // Output compare?
	{ // Yes. Faux encoder, for now
		pT2base->SR = ~(0x2);	// Reset CH1 flag

		// Duration increment computed from CL CAN msg
		pT2base->CCR1 += p->ocinc; // Schedule next interrupt

		// Update enable i/o pin
		Stepper__DR__direction_GPIO_Port->BSRR = p->enflag;

		//	Capture DTW timer for cycle counting
		p->dtwentry = DTWTIME;

#if 1 // GSM 1 DEH 0		
 // new code for sweep and reversal
      // forward direction means position accumulator is increasing
      // negative direction means position accumulator is decreasing
      //	drbit = 0 means positive direction

      // update velocity integrator
   #if 0	//	only invert velocity in linear region
      if (p->posaccum.s16[1] >= p->lc.Lplus || p->posaccum.s16[1] <= p->lc.Lminus)
      {  // in a level-wind reversal region
         #if 0	//	original treatement of drum direction reversal in reversal region
         if (p->posaccum.s16[1] >= p->lc.Lplus)
         {  // in positive level-wind reversal region
         	p->velaccum.s32 = (p->drbit) 
            	? p->velaccum.s32 + p->lc.Ka : p->velaccum.s32 - p->lc.Ka;
         }
         else
         {  // in negative level-wind reversal region
         	p->velaccum.s32 = (p->drbit) 
            	? p->velaccum.s32 - p->lc.Ka : p->velaccum.s32 + p->lc.Ka;            
         }
         #else	// revised treatment
         if (p->posaccum.s16[1] >= p->lc.Lplus)
         {  // in positive level-wind reversal region
         	p->velaccum.s32 -= p->lc.Ka; 
         }
         else
         {  // in negative level-wind reversal region
         	p->velaccum.s32 += p->lc.Ka;            
         }
         #endif
      }
      else 
      {  	// in a linear sweep region
         if (p->drbit != p->drbit_prev)
         {  // drum direction has changed
            p->drbit_prev = p->drbit;   // store new direction
            p->velaccum.s32 = -p->velaccum.s32; // invert velocity value
         }        
      }
   #else	//	always invert velocity when drum direction changes

      if (p->drbit != p->drbit_prev)
      	{  // Drum direction has changed
            p->drbit_prev = p->drbit;   // store new direction
            p->velaccum.s32 = -p->velaccum.s32; // invert velocity value
         } 

      if (p->posaccum.s16[1] >= p->lc.Lplus || p->posaccum.s16[1] <= p->lc.Lminus)
      #if 0	//	original treatement of drum direction reversal in reversal region
         if (p->posaccum.s16[1] >= p->lc.Lplus)
         {  // in positive level-wind reversal region
         	p->velaccum.s32 = (p->drbit) 
            	? p->velaccum.s32 + p->lc.Ka : p->velaccum.s32 - p->lc.Ka;
         }
         else
         {  // in negative level-wind reversal region
         	p->velaccum.s32 = (p->drbit) 
            	? p->velaccum.s32 - p->lc.Ka : p->velaccum.s32 + p->lc.Ka;            
         }
         #else	// revised treatment
         if (p->posaccum.s16[1] >= p->lc.Lplus)
         {  // in positive level-wind reversal region
         	p->velaccum.s32 -= p->lc.Ka; 
         }
         else
         {  // in negative level-wind reversal region
         	p->velaccum.s32 += p->lc.Ka;            
         }
         #endif
   #endif

      #if 1	//	some are all may be temporary for development
      if (p->velaccum.s32 >  p->lc.Ks) p->velaccum.s32 =  p->lc.Ks;
      if (p->velaccum.s32 < -p->lc.Ks) p->velaccum.s32 = -p->lc.Ks;
      p->dbg1 = p->velaccum.s32;
      p->dbg2 = p->posaccum.s16[1];
      #endif
   
      
      // update position integrator
      p->posaccum.s32 += p->velaccum.s32;
      //= (p->drbit) ? p->posaccum.s32 + p->velaccum.s32 : p->posaccum.s32 - p->velaccum.s32;
      
      /* When accumulator upper 16b changes generate a stepper pulse. */
      if ((p->posaccum.s16[1]) != (p->posaccum_prev))
      { // Here carry/borrow from low 16b to high 16b
         p->posaccum_prev = (p->posaccum.s16[1]);
         	


         /* Skip stepper pulses if motor not enabled. */
         if ((p->enflag & (2 << 0)) == 0) 
         {  // set direction based on sign of Velocity intergrator
            // depends on velocity magnitude being <= 2^16
            Stepper__DR__direction_GPIO_Port->BSRR = (p->velaccum.s16[1])
               ? 1 : (1 << 16);

#else	//	DEH orignal code

/* Using 'if' drbit (0|1) rather than multiply with drsign (+/- 1)
   saves two machine cycles, but requires more flash */
		if (p->drbit == 0)
			p->posaccum.s32 += p->lc.Ks;
		else
			p->posaccum.s32 -= p->lc.Ks;


/* Using a UNION to test & store the upper 16b saves compiled
   machine cycles (at least 8) over using '&' or '>>' techniques. */
		/* When accumulator upper 16b changes generate a pulse. */
		if ((p->posaccum.s16[1]) != (p->posaccum_prev))
		{ // Here carry from low 16b to high 16b
			p->posaccum_prev = (p->posaccum.s16[1]);


			/* Skip stepper pulses if motor not enabled. */
			if ((p->enflag & (2 << 0)) == 0) 
			{				
					Stepper__DR__direction_GPIO_Port->BSRR = p->drflag;

#endif
				// Start TIM9 to generate a delayed pulse.
				pT9base->CR1 = 0x9;


#if 0	//	1 for debug, 0 for operational
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
#endif
			}
		}
	}

	p->dtwdiff = DTWTIME - p->dtwentry;
	if (p->dtwdiff > p->dtwmax) p->dtwmax = p->dtwdiff;
	else if (p->dtwdiff < p->dtwmin) p->dtwmin = p->dtwdiff;

	return;
}



#if 0 // Original fauxinterrupt release code
void stepper_items_TIM2_IRQHandler(void)
{
	struct STEPPERSTUFF* p = &stepperstuff; // Convenience pointer
	p->dtwentry = DTWTIME;

	/* Faux encoder transition interrupt. */
	if ((pT2base->SR & 0x4) != 0)
	{
		pT2base->SR = ~(0x4);	// Reset CH2 flag

		// Duration increment computed from CL CAN msg
		pT2base->CCR2 += p->ocinc; // Schedule next interrupt

		// Update enable i/o pin
		Stepper__DR__direction_GPIO_Port->BSRR = p->enflag;

/* With real encoder interrupts the direction will be determined
   by comparing the previous input capture interrupt count with
   the new count. For now, drsign is set from the CAN msg from
   the Control Panel containing the direction pushbutton state. */
		// Each encoder edge generates a ratio less than 1 step pulses

/* Using 'if' drbit (0|1) rather than multiply with drsign (+/- 1)
   saves two machine cycles, but requires more flash */
		if (p->drbit == 0)
			p->posaccum.s32 += p->lc.Ks;
		else
			p->posaccum.s32 -= p->lc.Ks;

/* Using a UNION to test & store the upper 16b saves compiled
   machine cycles (at least 8) over using '&' or '>>' techniques. */
		/* When accumulator upper 16b changes generate a pulse. */
		if ((p->posaccum.s16[1]) != (p->posaccum_prev))
		{ // Here carry from low 16b to high 16b
			p->posaccum_prev = (p->posaccum.s16[1]);


			/* Skip stepper pulses if motor not enabled. */
			if ((p->enflag & (2 << 0)) == 0) 
			{
				// Change direction when accumulator passes through zero
//				if (p->posaccum.s16[1] < 0)
					Stepper__DR__direction_GPIO_Port->BSRR = p->drflag;
//				else
//					Stepper__DR__direction_GPIO_Port->BSRR = DRBIT << 16;

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
	p->dtwdiff = DTWTIME - p->dtwentry;
	if (p->dtwdiff > p->dtwmax) p->dtwmax = p->dtwdiff;
	else if (p->dtwdiff < p->dtwmin) p->dtwmin = p->dtwdiff;

	return;
}
#endif	
/*#######################################################################################
 * ISR routine for TIM4
 * CH1 = OC stepper reversal
 * CH2 = OC faux encoder interrupts
 *####################################################################################### */

void stepper_items_TIM4_IRQHandler(void)
{
	pT4base->SR = ~(0x1F);	// Reset all flags
	return;
}

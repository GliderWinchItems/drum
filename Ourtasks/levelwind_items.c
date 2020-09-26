/******************************************************************************
* File Name          : levelwind_items.c
* Date First Issued  : 09/23/2020
* Description        : Levelwind levelwind motor algorithm and items
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

08/10/2020 - pins added to Control Panel for levelwind testing

Control lines: Output pin drives FET gate, open drain to controller opto-isolator
PE5  - TIM9CH1 Stepper Pulse: PU (TIM1 Break shared interrupt vector)
   Interrupt vector: TIM1_BRK_TIM9_IRQHandler
PB0  - Direction: DR 
PB1  - Enable: EN  - High = enable (drive FET ON)

Drum encoder: TIM2CH1. Pullup resistors
PA0 - Encoder channel A
PA1 - Encoder channel B

TIM2 32b (84 MHz) capture mode (interrupt)
   CH3 PA2 input capture: encoder A (TIM5 PA0)
   CH4 PA3 input capture: encoder B (TIM5 PA1)
   CH2 PB3 input capture: encoder Z
   CH1 no-pin Indexing interrupts

TIM5 32b encoder counter (no interrupt)
   CH3 PA0 encoder config: encoder A (TIM2 PA2)
   CH4 PA1 encoder config: encoder B (TIM2 PA3)

TIM9 (168 MHz) Delayed levelwind pulse (no interrupt)
   CH1 PE5 PWM/OPM: Stepper pulse

TIM13 (84 MHz) Solenoid FET drive (no interrupt)
   CH1 PA6 PWM (4 KHz)

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
#include "levelwind_items.h"
#include "DTW_counter.h"
#include "drum_items.h"
#include "levelwind_switches.h"
#include "levelwind_items.h"

#define DTW    1  // True to keep DTW timing Code

struct LEVELWINDDBGBUF levelwinddbgbuf[LEVELWINDDBGBUFSIZE];

TIM_TypeDef  *pT2base; // Register base address 
TIM_TypeDef  *pT5base; // Register base address 
TIM_TypeDef  *pT9base; // Register base address 

/* Struct with all you want to know. */
struct LEVELWINDFUNCTION levelwindfunction;

/* *************************************************************************
 * void levelwind_items_timeout(void);
 * @brief   : Check for loss of CL CAN msgs
 * *************************************************************************/
void levelwind_items_timeout(void)
{
   struct LEVELWINDFUNCTION* p = &levelwindfunction; // Convenience pointer

   p->cltimectr += 1;
   if (p->cltimectr >= p->lc.cltimemax)
   { // We timed out! Stop the levelwind
      p->cltimectr = p->lc.cltimemax;

      /* Set enable bit which turns FET on, which disables levelwind. */
      p->enflag = (2 << 16); // Set bit with BSRR storing

      /* Bits positioned for updating PB BSRR register. */
      p->iobits = p->drflag | p->enflag;
   }
   return;
}
/* *************************************************************************
 * void levelwind_items_CANsendHB(void);
 * @brief   : Send CAN heartbeat for levelwind
 * *************************************************************************/
 void levelwind_items_CANsendHB(void)
 {
   struct LEVELWINDFUNCTION* p = &levelwindfunction; // Convenience pointer
   /* Setup CAN msg */
   p->canmsg[CID_LEVELWIND_HB].can.cd.uc[0] = p->levelwindstatus;
   p->canmsg[CID_LEVELWIND_HB].can.cd.uc[1] = p->posaccum.s32 >>  0;
   p->canmsg[CID_LEVELWIND_HB].can.cd.uc[2] = p->posaccum.s32 >>  8;
   p->canmsg[CID_LEVELWIND_HB].can.cd.uc[3] = p->posaccum.s32 >> 16;
   p->canmsg[CID_LEVELWIND_HB].can.cd.uc[4] = p->posaccum.s32 >> 24;

   /* Queue CAN msg to send. */
   xQueueSendToBack(CanTxQHandle,&p->canmsg[CID_LEVELWIND_HB],4);   
   return;
 }
/* *************************************************************************
 * void levelwind_items_clupdate(struct CANRCVBUF* pcan);
 * @param   : pcan = pointer to CAN msg struct
 * @brief   : Initialization of channel increment
 * *************************************************************************/
void levelwind_items_clupdate(struct CANRCVBUF* pcan)
{
   struct LEVELWINDFUNCTION* p = &levelwindfunction; // Convenience pointer

   /* Reset loss of CL CAN msgs timeout counter. */
   p->cltimectr = 0; 

   /* Extract float from payload */
   p->pf.u8[0] = pcan->cd.uc[1];
   p->pf.u8[1] = pcan->cd.uc[2];
   p->pf.u8[2] = pcan->cd.uc[3];
   p->pf.u8[3] = pcan->cd.uc[4];

   p->clpos = p->pf.f; // Redundant???
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
   { // Here, the PREP??? bit changed
      p->ocicbit_prev = p->ocicbit;
      if (p->ocicbit != 0)
      { // Here. PREP??? bit went from off to on
         pT2base->DIER &= ~0x80; // Disable TIM2CH3 interrupt
         if ((pT2base->CCMR2 & 0x1) != 0)
         { // Here, currently using encoder input capture
            // Setup for output compare
            pT2base->CCER  |=  (1<<11); // CC3NP: Configure as output
            pT2base->CCER  &= ~(1<<8);  // CC3E = 0; Turn channel off
            pT2base->CCMR2 &= ~(0xff << 0); // Change to Output compare, no pin
            pT2base->CCR3 = pT2base->CNT + p->ocinc; // Schedule next faux encoder interrupt
         }
         else
         { // Here, currently using output compare
            // Setup for input capture
            pT2base->CCER  &= ~((1<<8) || (1<<11)); // CC3E, CC3NP = input
            pT2base->CCMR2 |= 0x01; // Input capture mapped to TI3
            pT2base->SR = ~(1<<3);  // Reset CH3 flag if on
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
      if ((pcan->cd.uc[0] & DRBIT) == 0)  p->drflag = (1 << 16);  // Reset
      else  p->drflag = 1;                                        // Set
   }

   // Motor Enable bit
   if ((pcan->cd.uc[0] & ENBIT) != 0)
   {
      p->enflag = (2 << 16); // Reset
   }
   else
   {
      p->enflag = 2; // Set
   }

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
void levelwind_items_TIM2_IRQHandler(void)
{
   struct LEVELWINDFUNCTION* p = &levelwindfunction; // Convenience pointer

   /* This block for z channel (index) processing. It will be removed in operational
      code. */

   // TIM2CH2 = encodertimeZ
#if LEVELWINDDEBUG   
   if ((pT2base->SR & (1<<2)) != 0) // CH2 Interrupt flag?
   { // Yes, encoder channel Z transition
      pT2base->SR = ~(1<<2);  // Reset CH2 flag

      if ((GPIOB->IDR & (1<<3)) == 0)
      {
         HAL_GPIO_WritePin(GPIOD,LED_ORANGE_Pin,GPIO_PIN_SET);
      }
      else
      {
         HAL_GPIO_WritePin(GPIOD,LED_ORANGE_Pin,GPIO_PIN_RESET);
      }
      return;
   }
#endif

#if DTW
   // Capture DTW timer for cycle counting
   p->dtwentry = DTWTIME;
#endif 

   /* If we get here, we must have a TIM2 interrupt but it could be from channel 1
   or channel 3 or both. We service both in a single pass if needed. If the reversing 
   screw eumulation should be run, its switch block sets emulation_run to 1.*/   
   
   uint8_t  emulation_run = 0;   

   /* TIM2CH3 = encodertimeA PA2 TIM5CH1 PA0  */
   if ((pT2base->SR & (1<<3)) != 0) // CH3 Interrupt flag?
   { // Yes, either encoder channel A, or output compare emulating an encoder edge
      uint8_t  ddir; // temporary drum direction 

      pT2base->SR = ~(1<<3);  // Reset CH3 flag

      /* Was this interrupt due to encoder input capture or output compare?. */
      if ((pT2base->CCMR2 & 0x1) == 0)
      { // Here we are using TIM2CH3 as OC compare instead of input capture. */
         // Duration increment computed from CL CAN msg
         pT2base->CCR3 += p->ocinc; // Schedule next faux encoder interrupt
         // Make faux encoder counter 
         if (p->drflag == 1)
         {  
            ddir = 1;   //drum direction reverse
         }         
         else
         {
            ddir = 0;   // drum direction forward
         } 
      }      
      else
      { // Here, encoder driven input capture. Save for odometer and speed
         ddir = (pT5base->CR1 & 0x10) ? 1 : 0;     // Encoding DIR (direction) (0|1)
      }

      /* Here: either encoder channel A driven input capture interrupt, or 
         CL controlled timer output compare interrupt. */

      /* These encoder input capture or output compare interrupts do not drive the levelwind
         during indexing, sweeping, moving, and off states,  */
      switch (p->lw_state & 0xF0)   // deal with interrupts based on lw_state msn
      {
         case (LW_TRACK & 0xF0):
         {
            /* code here to check if LOS has occured. if so, switch to LOS recovery
               state. */
            emulation_run = 1;
            p->drbit = ddir;
            break;
         }

         case (LW_LOS & 0xF0):
         {            
            // code here looking for limit switch clousure to re-index on
            // then switch back to tracking state 
            emulation_run = 1;
            p->drbit = ddir;
            break;
         }
      }
   }

   // Indexing timer interrupt processing
   if ((pT2base->SR & (1<<1)) != 0) // CH1 Interrupt flag
   { // Yes, OC drive 
      pT2base->SR = ~(1<<1);  // Reset CH1 flag

      // Duration increment computed from CL CAN msg (during development)
      pT2base->CCR1 += p->ocidx; // Schedule next indexing interrupt
     
      switch (p->lw_state & 0xF0)   // deal with interrupts based on lw_state
      {
         case (LW_INDEX & 0xF0):
         {  
            // on indexing, switch to sweep state for limit switch testing
            if (p->sw[LIMITDBINSIDE].flag1) // limit switch has activated
            {
               p->posaccum.s32 = p->Lplus32 - p->Ks * 1000;
               p->posaccum_prev = p->posaccum.s16[1];
               p->lw_state = LW_SWEEP; // move to sweep state
               pT5base->CNT = 0; // reset odometer to 0 for testing
            }

            emulation_run = 1;            
            break;
         }         

         case (LW_SWEEP & 0xF0):
         {            
            /* code here testing limit switches in operational code and characterizing  
            their behavior during development. If needed, this mode can be used for
            calibrating limit switches for speed dependent corrections in LOS recovery. */
            
            
            if (p->velaccum.s32 == 0)
               {
                  p->ocidx = 21000/8;  // speed up sweep interrupt rate 
               }
            
                        
            if (p->sw[LIMITDBOUTSIDE].flag1)
            {
               p->lw_state = LW_MOVE;   
            }


            emulation_run = 1;
            break;
         }

         case (LW_MOVE & 0xF0):
         {            
            // code here dealing with stopping at specified location
            
            // transition to off state on completion
            
            if (p->velaccum.s32 == 0)
               {
                  p->lw_state = LW_TRACK;
                  break;
               }

            emulation_run = 1;
            break;
         }
      }      
   }

   /* reversing screw emulation code */
   if (emulation_run)
   {
#if LEVELWINDDEBUG   // move this out of ISR at some point
   // Update enable i/o pin
      Stepper__DR__direction_GPIO_Port->BSRR = p->enflag;
#endif     

      // forward (levelwind) direction means position accumulator is increasing
      // negative direction means position accumulator is decreasing
      // drbit = 0 means positive drum direction

      // update velocity integrator         
      if (p->drbit != p->drbit_prev)   
      {  // Drum direction has changed
         p->drbit_prev = p->drbit;   // save new direction
         p->velaccum.s32 = -p->velaccum.s32; // invert velocity value
      }
      else if (p->posaccum.s32 >= p->Lplus32)   // in positive level-wind region ?
      {
         p->velaccum.s32 -= p->lc.Ka;  // apply negative acceleration 
      }      
      else if (p->posaccum.s32 <= p->Lminus32)  // in negative level-wind region ?
      {
         p->velaccum.s32 += p->lc.Ka;  // apply positive acceleration
      }
      
      // update position integrator
      p->posaccum.s32 += p->velaccum.s32;

#if LEVELWINDDEBUG    
   p->pdbgadd->tim5cnt = pT5base->CNT;
   p->intcntr++;         
   /* p->dbg1 = p->velaccum.s32;
      p->dbg2 = p->posaccum.s16[1];
      p->dbg3 = p->posaccum.u16[0]; */

   // Store vars in buffer location. 
   p->pdbgadd->intcntr   = p->intcntr;
   p->pdbgadd->dbg1      = p->velaccum.s32;
   p->pdbgadd->dbg2      = p->posaccum.s16[1];
   p->pdbgadd->dbg3      = p->posaccum.u16[0];
   p->pdbgadd += 1;    // Advance add pointer
   if (p->pdbgadd >= p->pdbgend) p->pdbgadd = p->pdbgbegin;

#endif
         
      /* When accumulator upper 16b changes generate a levelwind pulse. */
      if ((p->posaccum.s16[1]) != (p->posaccum_prev))
      { // Here carry/borrow from low 16b to high 16b
         p->posaccum_prev = p->posaccum.s16[1];

        // set direction based on sign of Velocity integrator
         Stepper__DR__direction_GPIO_Port->BSRR = (p->velaccum.s16[1])
            ? 1 : (1 << 16);

         // Start TIM9 to generate a delayed pulse.
         pT9base->CR1 = 0x9;         
      }
   }

#if LEVELWINDDEBUG
   p->ledctr1 += 1;
   if ((pT2base->CCMR2 & 0x1) != 0)
   {
      p->ledctr2 = 0;
   }   
   else
   {
      p->ledctr2 = 500;
   }

   if (p->ledctr1 > p->ledctr2)        
   {
      p->ledctr1 = 0;
      if ((GPIOA->IDR & (1<<0)) == 0)
      {
         HAL_GPIO_WritePin(GPIOD,LED_GREEN_Pin,GPIO_PIN_SET); // GREEN LED       
      }
      else
      {
         HAL_GPIO_WritePin(GPIOD,LED_GREEN_Pin,GPIO_PIN_RESET); // GREEN LED    
      }  
   }
#endif

#if DTW
   p->dtwdiff = DTWTIME - p->dtwentry;
   if (p->dtwdiff > p->dtwmax) p->dtwmax = p->dtwdiff;
   else if (p->dtwdiff < p->dtwmin) p->dtwmin = p->dtwdiff;
#endif

   return;
}


/* *************************************************************************
 * struct LEVELWINDDBGBUF* levelwind_items_getdbg(void);
 * @brief   : Get pointer to debug buffer
 * @return  : NULL = no new data; otherwise ptr struct with data
 * *************************************************************************/
#if LEVELWINDDEBUG 
struct LEVELWINDDBGBUF* levelwind_items_getdbg(void)
{
   struct LEVELWINDFUNCTION* p = &levelwindfunction; // Convenience pointer
   struct LEVELWINDDBGBUF* ptmp;
   if (p->pdbgadd == p->pdbgtake) return NULL;
   ptmp = p->pdbgtake;
   p->pdbgtake += 1;
   if (p->pdbgtake >= p->pdbgend) p->pdbgtake = p->pdbgbegin;
   return ptmp;
}
#endif
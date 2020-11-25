/******************************************************************************
* File Name          : LevelwindTask.c
* Date First Issued  : 09/15/2020
* Description        : Levelwind function w STM32CubeMX w FreeRTOS
*******************************************************************************/

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "malloc.h"

#include "main.h"
#include "morse.h"

#include "drum_items.h"
#include "LevelwindTask.h"
#include "levelwind_func_init.h"
#include "levelwind_switches.h"
#include "MailboxTask.h"
#include "controlpanel_items.h"
#include "levelwind_items.h"

/* Private functions and macros to the file */

/* ************************************************************************/
 void levelwind_task_move_to_off(uint8_t);
/* @brief   : move to level-wind off state
 * @param   : error flag; 1 error, 0 no error
 * *************************************************************************/

#define enable_stepper  p->enflag = Stepper_MF_Pin << 16;         \
                        Stepper_MF_GPIO_Port->BSRR = p->enflag    

#define disable_stepper p->enflag = Stepper_MF_Pin;               \
                        Stepper_MF_GPIO_Port->BSRR = p->enflag
                  

/* Open Questions:
      Is a delay needed after stepper enable
      Is the variable indexed needed

*/


osThreadId LevelwindTaskHandle;

uint32_t dbgEth;

struct LEVELWINDFUNCTION levelwindfunction;
struct CONTROLPANELSTATE cp_state;

/* *************************************************************************
 * void swtim1_callback(TimerHandle_t tm);
 * @brief	: Software timer 1 timeout callback
 * *************************************************************************/
static void swtim1_callback(TimerHandle_t tm)
{
	xTaskNotify(LevelwindTaskHandle, LEVELWINDSWSNOTEBITSWT1, eSetBits);
	return;
}

/* *************************************************************************
 * void StartLevelwindTask(void const * argument);
 *	@brief	: Task startup
 * *************************************************************************/
struct SWITCHPTR* psw_safeactivex; // Debugging

void StartLevelwindTask(void const * argument)
{
	struct LEVELWINDFUNCTION* p = &levelwindfunction; // Convenience pointer
   struct CONTROLPANELSTATE* pcp = &cp_state;   // Convenience pointer

	/* A notification copies the internal notification word to this. */
	uint32_t noteval = 0;    // Receives notification word upon an API notify
	uint32_t noteuse = 0xffffffff;

	/* Initialize levelwind function struct. */
	levelwind_func_init_init(p);

   /* Initialize control panel state struct. */
   levelwind_task_cp_state_init();

	/* Hardware filter CAN msgs. */
	levelwind_func_init_canfilter(p);

   // move level-wind state machine to off with no error flag set
   levelwind_task_move_to_off(0);   


   pcp->mode = LW_MODE_CENTER;   // CP LW Mode selection 
   p->state = LW_OFF;

 #if 1   // initial conditions for indexing demo
   p->mc_state = MC_PREP;
   p->isr_state = LW_ISR_OFF;
   p->mode = LW_MODE_CENTER;
#endif
   
#if 0 // intial conditions for testing centering operation
   p->mc_state = MC_RETRIEVE;
   p->state = LW_TRACK;
   p->isr_state = LW_ISR_TRACK;   
   enable_stepper;
   
   /* Move the position value around to simulate where the LW starts before 
      centering operation. Center corresponds to 7500 with the current 
      parameters. If it is within ~1500 microsteps of center (corresponding 
      to about +/-15 mm), it says close enough and does nothing but disable 
      the stepper (disabling the stepper happens in all cases. That limitation
      is planned to be removed shortly.  */

   p->posaccum.s32 = (7500 - 500) << 16;
   p->velaccum.s32 = 0;
#endif

/* Manaual mode test. Close the manaul switch either with a acutal switch or 
   by setting the test for that switch to true in the  code for it below. Either
   use a real sensed switch or set the test for the desired switch direction
   in levelwind_items to 1 in its Manual case. (Current code has the right switch
   indicating asserted.)  */  

#if 0  // initial condtions for exit Manual demo
   p->mc_state = MC_PREP;
   p->state = LW_MANUAL;
   p->isr_state = LW_ISR_MANUAL;
   p->mode = LW_MODE_CENTER;

   /* When Manual switch test is Off in code below and the move_to_off error 
   is not asserted, LW Manual case below should 
   transition to Off and then start indexing demo */

#endif

#if 0 // Overrun test setup

   p->mc_state = MC_PREP;
   p->mode = LW_MODE_CENTER;
   p->state = LW_TRACK;
   p->isr_state = LW_ISR_TRACK;   
   enable_stepper;

   /* Assert Overrun switch and system should go to Off with stepper disabled.
      Insufficient setup for track to actually run but stepper should be enabled
      until Overrun switch is asserted.

      When the overrun switch is cleared, the system should go back to Off. If 
      the error flag is cleared, it should re-index.
    */

#endif
   
	/* Limit and overrun switches. */
	levelwind_switches_init();   

	/* Notifications from levelwind_items ISR via intermediary vector. 
	   with a priority within, but at the top of the FreeRTOS range. */
	HAL_NVIC_SetPriority(ETH_IRQn, 5, 0); 
   HAL_NVIC_EnableIRQ(ETH_IRQn);

    /* Levelwind ISR uses the following to trigger a notification */
  //NVIC_SetPendingIRQ(ETH_IRQn);

    /* Create timer #1: hearbeat (2 per sec) */
	levelwindfunction.swtim1 = xTimerCreate("swtim1",
		   pdMS_TO_TICKS(p->lc.hbct_t), 
		   pdTRUE, (void *) 0, 
		   swtim1_callback);
	if (levelwindfunction.swtim1 == NULL) {morse_trap(404);}

	/* Start command/keep-alive timer */
	BaseType_t bret = xTimerReset(p->swtim1, 10);
	if (bret != pdPASS) {morse_trap(405);}

extern CAN_HandleTypeDef hcan1;
	HAL_CAN_Start(&hcan1); // CAN1

	for (;;)
	{
		/* Wait for notifications 
         250 ms timeout insures Manual and Overrun switches get polled at least 
         4 times per second. */      
		xTaskNotifyWait(0,noteuse, &noteval, pdMS_TO_TICKS(250));
		noteuse = 0;	// Accumulate bits in 'noteval' processed.

      if ((noteval & LEVELWINDSWSNOTEBITISR) != 0)
		{ // Here levelwind_items.c triggered the ETH_IRQHandler
			noteuse |= LEVELWINDSWSNOTEBITISR;
         dbgEth += 1;

         /* Code here to figure out which ISR initated the notification 
            and accordingly do  any preliminary processing. It  is possible 
            that a switch statement for each ISR would be used. Also, check
            if an overrun switch has activated and do a state change to 
            MANUAL instead of repeating that code everywhere.  */                  
		}

		if ((noteval & LEVELWINDSWSNOTEBITCAN2) != 0) 
		{  // CAN:  'CANID_MC_STATE','26000000', 'MC', 'UNDEF','MC: Launch state msg');
			// clupdate( ) should not be called. The MC state should be extracted from message
         // levelwind_items_clupdate(&p->pmbx_cid_drum_tst_stepcmd->ncan.can);
			noteuse |= LEVELWINDSWSNOTEBITCAN2;
		}

		if ((noteval & LEVELWINDSWSNOTEBITCAN1) != 0) 
		{   // CAN:  CANID_TST_STEPCMD: U8_FF DRUM1: U8: Enable,Direction, FF: CL position: E4600000
		    // Received CAN msg with Control Lever position, direction and enable bits 
			levelwind_items_clupdate(&p->pmbx_cid_drum_tst_stepcmd->ncan.can);
         // this will process the control panel state messages and above clupdate scraped
         levelwind_task_cp_state_update(&p->pmbx_cid_drum_tst_stepcmd->ncan.can);
			noteuse |= LEVELWINDSWSNOTEBITCAN1;   
      }       
// REVIST!!! Is this already being taken care of with status messages???
		if ((noteval & LEVELWINDSWSNOTEBITSWT1) != 0) 
		{ // Software timer #1: Send heartbeat
			levelwind_items_CANsendHB();
			noteuse |= LEVELWINDSWSNOTEBITSWT1;
		}

      if ((p->sw[LIMITDBMS].flag2))  // To test a logical item (e.g., pin) when needed
      {
         //HAL_GPIO_WritePin(GPIOD,LED_RED_Pin,GPIO_PIN_SET);
      }
      else
      {
         //HAL_GPIO_WritePin(GPIOD,LED_RED_Pin,GPIO_PIN_RESET);
      }


      if (!(GPIOE->IDR & ManualSw_NO_Pin) && (p->state != LW_MANUAL))   // here test for Manual switch closure (no associated task notification)
      {  // Manual (bypass) switch is activated (closed); go to Manual state
         p->ocinc = p->ocman;
         p->isr_state = LW_ISR_MANUAL;
         p->state = LW_MANUAL;
         p->indexed = 0;       // REVISIT: indexed flag may not be needed???
         p->status = LW_STATUS_MANUAL;                  
         enable_stepper;
      }

      else if (!(GPIOE->IDR & OverrunSwes_NO_Pin) && (p->state != LW_OVERRUN)) // here test for Overrun switches activation
      {  // An Overrun switch is closed; go to Overrun state           
         p->ocinc = p->ocman;    // slow down output compare interrupt rate
         p->isr_state = LW_ISR_OFF;
         p->state = LW_OVERRUN;
         p->indexed = 0;                  // indexed flag may not be needed???
         p->status = LW_STATUS_OVERRUN;
         disable_stepper;
      }

      // is this drum not enabled for operation on the control panel
      else if  (!(pcp->op_drums & (0x01 << (p->lc.mydrum - 1))) 
         || ((pcp->mode == LW_MODE_OFF) && (pcp->active_drum == p->lc.mydrum)))
      {  // move level-wind state machine to off with error flag clear
         levelwind_task_move_to_off(0);           
      }

      else 
      {  // this drum node is in use and mode is not Off
         switch (p->state & 0xF0)   // deal with CAN notification based on LW super-state
         {
            case (LW_OFF):
            {  
               // clear error flag and status if LW mode is set to Off
               if (p->mode == LW_MODE_OFF) 
               {  
                  p->status = LW_STATUS_GOOD; 
               } 
               
               else if ((p->mc_state == MC_PREP) && ((p->state & 0x0F) == 0) 
                  && (p->sw[LIMITDBMS].flag2)) // last condition temporary for early development
               {  // we are in MC Prep state on an operational drum with error flag clear  
                  p->sw[LIMITDBMS].flag2 = 0; // TEMPORARY: clear LS motorside latching flag
                  
                  p->ocinc = p->lc.ocidx; // set oc  interrupt rate for indexing
                  p->isr_state = LW_ISR_INDEX;
                  p->state = LW_INDEX;
                  enable_stepper;

                  // these values are set up temporarily for development to make leftost position 0
                  // Need padding for to provide margin for initial sweep
                  // Position accumulator initial value. Reference paper for the value employed.
                  // p->posaccum.s32 = (p->lc.Lminus << 16) - p->rvrsldx;
                  p->posaccum.s32 = 0; // temporary to have it start at 0.
                  p->pos_prev = p->posaccum.s32;
                  // initialize 32-bit values for Lplus32 and Lminus32. Reference paper
                  // p->Lminus32 = p->lc.Lminus << 16;
                  p->Lminus32 = (p->lc.Lminus << 16) + p->rvrsldx;
                  p->Lplus32  = p->Lminus32 
                     + (((p->lc.Lplus - p->lc.Lminus) << 16) / p->Ks) * p->Ks;
                  p->velaccum.s32 = 0; // Velocity accumulator initial value
               }              
               break;
            }

            case (LW_OVERRUN):
            {  
               if(1) // poll overrun switch to see if it is cleared
               {  // move level-wind state machine to off with error flag set
                  levelwind_task_move_to_off(1);  
               }
               break;
            }         

            case (LW_MANUAL): // poll Manual switch to see if it is cleared
            {
               if (GPIOE->IDR & ManualSw_NO_Pin)   // poll if an NO Manual switch is still activated
               {  // move level-wind state machine to off with error flag set
                  levelwind_task_move_to_off(1);           
               }
               break;
            }

            case (LW_INDEX):
            {  // Here check if ISR has finished the indexing
               if (p->isr_state == LW_ISR_TRACK)
               {
                  p->state = LW_TRACK;
                  p->status = LW_STATUS_GOOD;
               }
               
               if(p->mc_state == MC_SAFE) 
               {  // move level-wind state machine to off with error flag not set
                  levelwind_task_move_to_off(0);    
               }               
               break;
            }
            case (LW_TRACK):
            {
               if (p->mc_state == MC_SAFE) 
               {  // move level-wind state machine to off with error flag not set
                  levelwind_task_move_to_off(0);    
               }
               else if ((p->mc_state == MC_RETRIEVE) && (p->mode == LW_MODE_CENTER))                  
               {  /* setup to center for retrieve and transition to Center  */    
                  int32_t center = (p->lc.Lplus + p->lc.Lminus) << 15; // calculate center position
                  int32_t distance = center - p->posaccum.s32;   // signed distance to center
                  if (distance > (2 * p->rvrsldx))          
                  {  // move to increase posaccum  from 0 to distance                     
                     /* start 1 iteration in to avoid immediate zero velocity termination 
                        in LW_ISR_ARREST  */
                     p->posaccum.s32 = p->lc.Ka;        
                     p->velaccum.s32 = p->lc.Ka;
                     p->Lminus32 = p->rvrsldx;
                     p->Lplus32  = p->rvrsldx
                        + ((distance - (2 * p->rvrsldx)) / p->Ks) * p->Ks;                     
                     p->ocinc = p->ocswp; // center at sweep speed
                     p->state = LW_CENTER;
                     // transition to Arrest with next isr state Off
                     p->isr_state = LW_ISR_ARREST | (LW_ISR_OFF >> 4);                             
                  }
                  else if (-distance > (2 * p->rvrsldx))
                  {  // move to decrease posaccum from 0 to -distance                                          
                     /* start 1 iteration in to avoid immediate zero velocity termination 
                        in LW_ISR_ARREST*/
                     p->posaccum.s32 = -p->lc.Ka;        
                     p->velaccum.s32 = -p->lc.Ka;
                     p->Lplus32  = -p->rvrsldx;
                     p->Lminus32 = -p->rvrsldx  
                        + ((distance + (2 * p->rvrsldx)) / p->Ks) * p->Ks;                     
                     // transition to Arrest with next isr state Off
                     p->isr_state = LW_ISR_ARREST | (LW_ISR_OFF >> 4); 
                     p->state = LW_CENTER;
                     p->ocinc = p->ocswp; // center at sweep speed
                  }
                  else if (distance > ((int32_t) 2) << 16)          
                  {  // distance greater than 2 stepper steps
                     // compute truncated accell/deceration profile
                     int16_t Nr = sqrtf((float)(distance / (p->lc.Ka)));
                     int32_t rvrsldx = Nr * (Nr - 1) * p->lc.Ka / 2;
                     int16_t Ks = Nr * p->lc.Ka;
                     /* start 1 iteration in to avoid immediate zero velocity termination 
                        in LW_ISR_ARREST  */
                     p->posaccum.s32 = p->lc.Ka;        
                     p->velaccum.s32 = p->lc.Ka;
                     p->Lminus32 = rvrsldx;
                     p->Lplus32  = rvrsldx
                        + ((distance - (2 * rvrsldx)) / Ks) * Ks;                     
                     p->ocinc = p->ocswp; // center at sweep speed
                     p->state = LW_CENTER;
                     // transition to Arrest with next isr state Off
                     p->isr_state = LW_ISR_ARREST | (LW_ISR_OFF >> 4);
                  }
                  else if (-distance > ((int32_t) 2) << 16)          
                  {  // distance greater than 2 stepper steps
                     // compute truncated accell/deceration profile
                     int16_t Nr = sqrtf((float)(-distance / (p->lc.Ka)));
                     int32_t rvrsldx = Nr * (Nr - 1) * p->lc.Ka / 2;
                     int16_t Ks = Nr * p->lc.Ka;
                     /* start 1 iteration in to avoid immediate zero velocity termination 
                        in LW_ISR_ARREST  */
                     p->posaccum.s32 = -p->lc.Ka;        
                     p->velaccum.s32 = -p->lc.Ka;

                     p->Lminus32 = -rvrsldx;
                     p->Lplus32  = -rvrsldx
                        + ((distance + (2 * rvrsldx)) / Ks) * Ks;                     
                     p->ocinc = p->ocswp; // center at sweep speed
                     p->state = LW_CENTER;
                     // transition to Arrest with next isr state Off
                     p->isr_state = LW_ISR_ARREST | (LW_ISR_OFF >> 4);
                  } 
                  else
                  {  //  distance close enough, don't bother to move
                     p->ocinc = p->ocman; // reduce output compare interrupt rate
                     p->isr_state = LW_ISR_OFF;
                     p->state = LW_CENTER;
                     disable_stepper;
                  }                  
               }
               break;
            }

            case (LW_CENTER):
            {  
               if (p->isr_state == LW_ISR_OFF)
               {  // movement to center has concluded
                  disable_stepper;
               }

               if((p->mc_state == MC_PREP))  
               {  // move to Index state with stepper driver enabled
                  
                  /* This code is essentially the same as is used in the Off state
                  to start indexing. We could just move through Off but then that 
                  would generate two status-state messages. That could be circumvented
                  with a trick but this is cleaner. */                  
                  p->ocinc = p->lc.ocidx;
                  p->isr_state = LW_ISR_INDEX;
                  p->state = LW_INDEX;
                  enable_stepper;

                  // REVISIT: initialize trajectory integrators and associated values
                  // these values are set up temporarily for development to make leftost position 0
                  // Need padding for to provide margin for initial sweep
                  // Position accumulator initial value. Reference paper for the value employed.
                  // p->posaccum.s32 = (p->lc.Lminus << 16) - p->rvrsldx;
                  p->posaccum.s32 = 0; // temporary to have it start at 0.
                  p->pos_prev = p->posaccum.s32;
                  // initialize 32-bit values for Lplus32 and Lminus32. Reference paper
                  // p->Lminus32 = p->lc.Lminus << 16;
                  p->Lminus32 = (p->lc.Lminus << 16) + p->rvrsldx;
                  p->Lplus32  = p->Lminus32 
                     + (((p->lc.Lplus - p->lc.Lminus) << 16) / p->Ks) * p->Ks;
                  p->velaccum.s32 = 0;          // Velocity accumulator initial value      
               }
               break;
            }

            case (LW_LOS):
            {
               // exit from LOS handled by switches or Tim2 ISR?
               // code here TBD
               break;
            }
         }               
      }
      

      /* see if status or super-state have changed or HB timer has expired
         and send appropriate status-state message */
      if ((p->state != p->state_prev) || (p->status != p->status_prev))
      {  
         p->state_prev = p->state;
         p->status_prev = p->status;
         // need  to initate an automonous status-state message
      }
      else if (0 && (p->hbctr >= xTaskGetTickCount()))
      {
         // need  to initate a HB status-state message
      }    
	}
}

/* *************************************************************************
 * void levelwind_task_move_to_off (void);
 * @brief   : move to level-wind off state
 * @param   : err; 1 error, 0 no error
 * *************************************************************************/
void levelwind_task_move_to_off(uint8_t err)
{   
   struct LEVELWINDFUNCTION* p = &levelwindfunction; // Convenience pointer

   p->isr_state = LW_ISR_OFF;
   p->ocinc = p->ocman;    // slow down output compare interrupt rate
   p->state = LW_OFF;
   p->indexed = 0;   // may not be needed
   p->state = LW_MODE_OFF & err;
   p->status = (err) ? LW_STATUS_OFF_AFTER_ERROR : LW_STATUS_GOOD;
   disable_stepper;

   return;  
}

/* *************************************************************************
 * void levelwind_task_cp_state_update(struct CANRCVBUF* pcan);
 * @param   : pcan = pointer to CAN msg struct
 * @brief   : update of control panel state structure
 * *************************************************************************/
void levelwind_task_cp_state_init(void)
{   
   struct CONTROLPANELSTATE* pcp = &cp_state;   // Convenience pointer

   pcp->init = 0;

   // initially assumed input states
   pcp->clpos = 0.0f;
   pcp->safe_active = 0;
   pcp->arm = 0;
   pcp->rtrv_prep = 0;     
   pcp->zero_tension = 0;  
   pcp->zero_odometer = 0;     
   pcp->brake = 0;  
   pcp->guillotine = 0;     
   pcp->emergency = 0;  
   pcp->mode = LW_MODE_OFF;
   pcp->index = 0;
   pcp->rev_fwd = 1;     
   pcp->rmt_lcl = 1;  
   pcp->active_drum = 1;   // assumes single drum system     
   pcp->op_drums = 0x01;   // bit 0 corresponds to drum #1
   
   
   // initially assumed output states
   pcp->safe_led = 0;
   pcp->prep_led = 0;
   pcp->armed_led = 0;
   pcp->grndrtn_led = 0;
   pcp->ramp_led = 0;
   pcp->climb_led = 0;
   pcp->recovery_led = 0;
   pcp->retrieve_led = 0;
   pcp->abort_led= 0;
   pcp->stop_led = 0;
   pcp->arm_pb_led = 0;
   pcp->prep_rcvry_led = 0;
   // pcp->beeper = 0;  // beeper will likely be getting its own CAN message
                        // until that is agreed to don't handle

   return;  
}


/* *************************************************************************
 * void levelwind_task_cp_state_update(void);
 * @brief   : update of control panel state structure
 * *************************************************************************/
void levelwind_task_cp_state_update(struct CANRCVBUF* pcan)
{
   struct CONTROLPANELSTATE* pcp = &cp_state;   // Convenience pointer
   
#define  CL       1  // turn on control lever position updates
#define  INPUTS   1  // turn on input updates
#define  OUTPUTS  1  // turn on output updates

return;  // DEBUG!!!!do nothing until real cp state CAN messages are present

   /* This does a full update of the control panel state struct. If speed is 
      important, only the items used in the function could be extracted by 
      simply commenting out unneeded updates. Order doesn't matter so unused
      could be grouped in a #if 0 block to preserve option to restore.  */ 

#if CL
   /* Extract and convert CL position from payload */
   pcp->clpos = pcan->cd.uc[SAFEACTIVE_BYTE] * CLPOS_SCL;
#endif

#if INPUTS
   /* extract switch and pb called from StartLevelwindTask*/
   // the below produces 0/1 logicals. a slight speed-up could be attained
   // if 0/not-0 logicals were used eliminating right shifts for 
   // the single bit items. most of the operations shown are acutally
   // completed at compile time.   

   pcp->safe_active = (pcan->cd.uc[SAFEACTIVE_BYTE] 
      & (SAFEACTIVE_MASK << SAFEACTIVE_BIT)) >> SAFEACTIVE_BIT;

   pcp->arm = (pcan->cd.uc[ARMED_BYTE] 
      & (ARMED_MASK << ARMED_BIT)) >> ARMED_BIT;

   pcp->rtrv_prep = (pcan->cd.uc[RTRVPREP_BYTE] 
      & (RTRVPREP_MASK << RTRVPREP_BIT)) >> RTRVPREP_BIT;

   pcp->zero_tension = (pcan->cd.uc[ZEROTEN_BYTE] 
      & (ZEROTEN_MASK << ZEROTEN_BIT)) >> ZEROTEN_BIT;

   pcp->zero_odometer = (pcan->cd.uc[ZEROODOM_BYTE] 
      & (ZEROODOM_MASK << ZEROODOM_BIT)) >> ZEROODOM_BIT;

   pcp->brake = (pcan->cd.uc[BRAKE_BYTE] 
      & (BRAKE_MASK << BRAKE_BIT)) >> BRAKE_BIT;

   pcp->guillotine = (pcan->cd.uc[GUILLOTINE_BYTE] 
      & (GUILLOTINE_MASK << GUILLOTINE_BIT)) >> GUILLOTINE_BIT;

   pcp->emergency = (pcan->cd.uc[EMERGENCY_BYTE] 
      & (EMERGENCY_MASK << EMERGENCY_BIT)) >> EMERGENCY_BIT;

   pcp->mode = (pcan->cd.uc[LWMODE_BYTE] 
      & (LWMODE_MASK << LWMODE_BIT)) >> LWMODE_BIT;

   pcp->index = (pcan->cd.uc[LWINDEX_BYTE] 
      & (LWINDEX_MASK << LWINDEX_BIT)) >> LWINDEX_BIT;

   pcp->rev_fwd = (pcan->cd.uc[REVFWD_BYTE] 
      & (REVFWD_MASK << REVFWD_BIT)) >> REVFWD_BIT;
   
   pcp->rmt_lcl = (pcan->cd.uc[RMTLCL_BYTE] 
      & (RMTLCL_MASK << RMTLCL_BIT)) >> RMTLCL_BYTE;

   pcp->active_drum = (pcan->cd.uc[ACTIVEDRUM_BYTE] 
      & (ACTIVEDRUM_MASK << ACTIVEDRUM_BIT)) >> ACTIVEDRUM_BIT;

   pcp->op_drums = (pcan->cd.uc[OPDRUMS_BYTE] 
      & (OPDRUMS_MASK << OPDRUMS_BIT)) >> OPDRUMS_BIT;
#endif

#if OUTPUTS
   // output updates

   pcp->safe_led = (pcan->cd.uc[SAFELED_BYTE] 
      & (SAFELED_MASK << SAFELED_BIT)) >> SAFELED_BIT;

   pcp->prep_led = (pcan->cd.uc[PREPLED_BYTE] 
      & (PREPLED_MASK << PREPLED_BIT)) >> PREPLED_BIT;


   pcp->armed_led = (pcan->cd.uc[ARMEDLED_BYTE] 
      & (ARMEDLED_MASK << ARMEDLED_BIT)) >> ARMEDLED_BIT;
   

   pcp->grndrtn_led = (pcan->cd.uc[GRNDRTNLED_BYTE] 
      & (GRNDRTNLED_MASK << GRNDRTNLED_BIT)) >> GRNDRTNLED_BIT;
   

   pcp->ramp_led = (pcan->cd.uc[RAMPLED_BYTE] 
      & (RAMPLED_MASK << RAMPLED_BIT)) >> RAMPLED_BIT;
   

   pcp->climb_led = (pcan->cd.uc[CLIMBLED_BYTE] 
      & (CLIMBLED_MASK << CLIMBLED_BIT)) >> CLIMBLED_BIT;
   

   pcp->recovery_led = (pcan->cd.uc[RECOVERYLED_BYTE] 
      & (RECOVERYLED_MASK << RECOVERYLED_BIT)) >> RECOVERYLED_BIT;
   

   pcp->retrieve_led = (pcan->cd.uc[RETRIEVELED_BYTE] 
      & (RETRIEVELED_MASK << RETRIEVELED_BIT)) >> RETRIEVELED_BIT;
   

   pcp->abort_led = (pcan->cd.uc[ABORTLED_BYTE] 
      & (ABORTLED_MASK << ABORTLED_BIT)) >> ABORTLED_BIT;
   

   pcp->stop_led = (pcan->cd.uc[STOPLED_BYTE] 
      & (STOPLED_MASK << STOPLED_BIT)) >> STOPLED_BIT;
   

   pcp->arm_pb_led = (pcan->cd.uc[ARMPBLED_BYTE] 
      & (ARMPBLED_MASK << ARMPBLED_BIT)) >> ARMPBLED_BIT;
   
   pcp->prep_rcvry_led = (pcan->cd.uc[PREPRCRYPBLED_BYTE] 
      & (PREPRCRYPBLED_MASK << PREPRCRYPBLED_BIT)) >> PREPRCRYPBLED_BIT;

   /* beeper will likely get its own CAN messages
   pcp->beeper = (pcan->cd.uc[BEEPER_BYTE] 
      & (BEEPER_MASK << BEEPER_BIT)) >> BEEPER_BIT; */
#endif

   return;  
}

/* *************************************************************************
 * osThreadId xLevelwindTaskCreate(uint32_t taskpriority);
 * @brief	: Create task; task handle created is global for all to enjoy!
 * @param	: taskpriority = Task priority (just as it says!)
 * @return	: LevelwindTaskHandle
 * *************************************************************************/
osThreadId xLevelwindTaskCreate(uint32_t taskpriority)
{
 	osThreadDef(LevelwindTask, StartLevelwindTask, osPriorityNormal, 0, (192));
	LevelwindTaskHandle = osThreadCreate(osThread(LevelwindTask), NULL);
	vTaskPrioritySet( LevelwindTaskHandle, taskpriority );
	return LevelwindTaskHandle;
}
/* ##################################################################
 * Commandeered vector for levelwind_items_TIM2_IRQHandler notifications
 * ################################################################## */
void ETH_IRQHandler(void)
{	
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if (LevelwindTaskHandle != NULL)
	{ // Here, notify one task a new msg added to circular buffer
		xTaskNotifyFromISR(LevelwindTaskHandle,\
		LEVELWINDSWSNOTEBITISR, eSetBits,\
		&xHigherPriorityTaskWoken );
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken ); // Trigger scheduler
	}
	return;
}
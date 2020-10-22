/******************************************************************************
* File Name          : LevelwindTask.c
* Date First Issued  : 09/15/2020
* Description        : Levelwind function w STM32CubeMX w FreeRTOS
*******************************************************************************/

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "malloc.h"

#include "main.h"
#include "morse.h"

#include "levelwind_items.h"
#include "drum_items.h"
#include "LevelwindTask.h"
#include "levelwind_func_init.h"
#include "levelwind_switches.h"
#include "MailboxTask.h"
#include "controlpanel_items.h"


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


   /* Set initial_state/mode until MC messages arrive. Initial conditions for
      CP state are handled in ...cp_state_init above. Below should
      be moved into levelwind_func_init_init( ) at some point but useful here
      for development   */
   p->lw_state = LW_OFF;
   p->lw_mode = LW_ISR_OFF;
   p->lw_error = 0;
   p->lw_indexed = 0;
   // disable stepper by resetting output with BSRR storing
   p->enflag = Stepper_MF_Pin;         // configure for set
   Stepper_MF_GPIO_Port->BSRR = p->enflag;   // write to port
   

   /* TEMPORARY Unil message are actually present  */
   p->mc_prev_state = MC_SAFE;
   p->mc_pres_state = MC_PREP;  

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
         250 ms timeout insures Manual switch gets polled at least 4 times
         per second. */      
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

		if ((noteval & LEVELWINDSWSNOTEBITSWT1) != 0) 
		{ // Software timer #1: Send heartbeat
			levelwind_items_CANsendHB();
			noteuse |= LEVELWINDSWSNOTEBITSWT1;
		}

      if ((0) && (p->lw_state != LW_MANUAL))   // here test for Manual switch closure (no associated task notification)
      {  // Manual (bypass) switch is closed
         p->lw_state = LW_MANUAL;
         p->lw_mode = LW_ISR_MANUAL;
         p->lw_indexed = 0;         
         p->lw_error = 1;  // indicate an Overrun switch has tripped
         p->ocinc = p->ocman;
         // enable stepper by resetting output with BSRR storing
         p->enflag = Stepper_MF_Pin << 16;         // configure for reset
         Stepper_MF_GPIO_Port->BSRR = p->enflag;   // write to port

         // ADD initiate LW status-state message
      }
      else if ((0) && (p->lw_state != LW_OVERRUN)) // here test for Overrun switch activations
      {
         p->lw_state = LW_OVERRUN;
         p->lw_mode = LW_ISR_OFF;
         p->lw_indexed = 0;         
         p->lw_error = 1;  // indicate an Overrun switch has tripped
         // disable stepper by setting output with BSRR storing
         p->enflag = Stepper_MF_Pin;         // configure for reset
         Stepper_MF_GPIO_Port->BSRR = p->enflag;   // write to port

         // ADD initiate LW status-state message
      }
      // check to see if this drum is enabled for operation on the control panel
      else if (pcp->op_drums & (0x01 << (p->lc.mydrum - 1)))
      {  // this node is operational

         switch (p->lw_state & 0xF0)   // deal with CAN notification based on LW super-state
         {
            case (LW_OFF):
            {  
               if ((p->mc_pres_state == MC_PREP) && (p->mc_prev_state == MC_SAFE))
               {  // if here we have moved into MC Prep state from Safe state
                  p->lw_state = LW_INDEX;
                  p->lw_mode = LW_ISR_INDEX;
                  p->lw_error = 0;  // clear error flag
                  // enable stepper by resetting output with BSRR storing
                  p->enflag = Stepper_MF_Pin << 16;         // configure for reset
                  Stepper_MF_GPIO_Port->BSRR = p->enflag;   // write to port

                  // initialize trajectory integrators and associated values
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
                  p->velaccum.s32 = 0;             // Velocity accumulator initial value  
                  p->drbit = p->drbit_prev = 0;    // Drum direction bit
                  
                  // ADD initiate LW status-state message
               }              
               break;
            }

            case (LW_OVERRUN):
            {  
               if(1) // test overrun switch to see if it is still activated
               {  // switch cleared: move to off with error set
                  p->lw_state = LW_OFF;
                  p->lw_mode = LW_ISR_OFF;
                  p->lw_error = 1;  // set error flag
                  // disable stepper by resetting output with BSRR storing
                  p->enflag = Stepper_MF_Pin;         // configure for set
                  Stepper_MF_GPIO_Port->BSRR = p->enflag;   // write to port

                  // ADD initiate LW status-state message
               }
               break;
            }         

            case (LW_MANUAL):
            {
               if (1)   // test if an Manual switch is still activated
               {  // switch cleared: move to Off with error set
                  p->lw_state = LW_OFF;
                  p->lw_mode = LW_ISR_OFF;
                  p->lw_error = 1;  // set error flag
                  // disable stepper by resetting output with BSRR storing
                  p->enflag = Stepper_MF_Pin;         // configure for set
                  Stepper_MF_GPIO_Port->BSRR = p->enflag;   // write to port

                  // ADD initiate LW status-state message                  
               }
               break;
            }


            case (LW_INDEX):
            {  // Tim2 ISR moves LW state to Track when done

               // Here check if ISR has finished the indexing
               // ADD initiate LW status-state message      

               if(p->mc_pres_state == MC_SAFE) 
               {  // move to Off state
                  p->lw_mode = LW_ISR_OFF;
                  p->lw_state = LW_OFF;

                  // ADD initiate LW status-state message   
               }               
               break;
            }

            case (LW_TRACK):
            {
               if ((p->mc_pres_state == MC_RETRIEVE) && (p->lw_mode == LW_MODE_CENTER))
               {
                  p->lw_state = LW_CENTER;
                  // may setp up centering and get it started here
               }
               else if(p->mc_pres_state == MC_SAFE) 
               {  // move to Off state
                  p->lw_mode = LW_ISR_OFF;
                  p->lw_state = LW_OFF;
               }
               break;
            }

            case (LW_CENTER):
            {
               /*
                  I think when the Track state transition to Center, it will modify
                  Lplus or Lminus and switch the mode to Arrest to move it to the 
                  center   */

               /* code here will see if centering is complete and sending a CAN
                  status-state message once  */


               /* This code deals with moving back to Index when the Retreive is 
                  completed*/
               if((p->mc_pres_state == MC_PREP))  
               {  // move to Index state with stepper driver enabled
                  p->lw_state = LW_INDEX;
                  p->lw_mode = LW_ISR_INDEX;
                  // enable stepper by resetting output with BSRR storing
                  p->enflag = Stepper_MF_Pin << 16;         // configure for reset
                  Stepper_MF_GPIO_Port->BSRR = p->enflag;   // write to port
                  
                  // ADD initiate LW status-state message
               }
               break;
            }

            case (LW_LOS):
            {
               // exit from LOS handled by switches or Tim2 ISR?
               break;
            }
         }               
      }
      else 
      {// drum not in operational use. move level-wind state to OFF.
         p->lw_state = LW_OFF;
         p->lw_mode = LW_ISR_OFF;
         p->lw_indexed = 0;
         p->lw_error = 0;           
      }


      /* Need check for state-status chages to send automatic messages*/
	}
}




/* *************************************************************************
 * void levelwind_task_cp_sate_update(struct CANRCVBUF* pcan);
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
   pcp->lw_mode = 0;     
   pcp->lw_index = 0;
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
   pcp->beeper = 0;

   return;  
}


/* *************************************************************************
 * void levelwind_task_cp_sate_update(struct CANRCVBUF* pcan);
 * @param   : pcan = pointer to CAN msg struct
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

   pcp->lw_mode = (pcan->cd.uc[LWMODE_BYTE] 
      & (LWMODE_MASK << LWMODE_BIT)) >> LWMODE_BIT;

   pcp->lw_index = (pcan->cd.uc[LWINDEX_BYTE] 
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
   
   pcp->beeper = (pcan->cd.uc[BEEPER_BYTE] 
      & (BEEPER_MASK << BEEPER_BIT)) >> BEEPER_BIT;
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
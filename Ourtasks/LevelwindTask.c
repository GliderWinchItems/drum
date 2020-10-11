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
#include "levelwind_items.h"
//#include "controlpanel_items.h"


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

	/* A notification copies the internal notification word to this. */
	uint32_t noteval = 0;    // Receives notification word upon an API notify
	uint32_t noteuse = 0xffffffff;

	/* Initialize levelwind function struct. */
	levelwind_func_init_init(p);

   /* Initialize control panel state struct. */
   levelwind_task_cp_state_init();

	/* Hardware filter CAN msgs. */
	levelwind_func_init_canfilter(p);

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
		/* Wait for notifications */      
		xTaskNotifyWait(0,noteuse, &noteval, portMAX_DELAY);
		noteuse = 0;	// Accumulate bits in 'noteval' processed.

		if ((noteval & LEVELWINDSWSNOTEBITISR) != 0)
		{ // Here levelwind_items.c triggered the ETH_IRQHandler
			noteuse |= LEVELWINDSWSNOTEBITISR;
         dbgEth += 1;

         /* Code here to figure out which ISR initated the notification 
            and accordingly do  any preliminary processing. It  is possible 
            that a switch statement for each ISR would be used. Also, check
            if an averrun switch has activated and do a state change to 
            MANUAL instead of repeating that code everywhere. Similar check
            may be appropriate for Off state */   
              
         switch (p->lw_state & 0xF0)   // deal with ISR notification based on lw_state
         {
            case (LW_OFF & 0xF0):
            {
               break;
            }

            case (LW_OVERRUN & 0xF0):
            {  
               break;
            }         

            case (LW_MANUAL & 0xF0):
            { 
               break;
            }

            case (LW_CENTER & 0xF0):
            {
               break;
            }

            case (LW_INDEX & 0xF0):
            {
               break;
            }

            case (LW_TRACK & 0xF0):
            {
               break;
            }

            case (LW_LOS & 0xF0):
            {
               break;
            }
         }               
		}

		if ((noteval & LEVELWINDSWSNOTEBITCAN2) != 0) 
		{  // CAN:  'CANID_MC_STATE','26000000', 'MC', 'UNDEF','MC: Launch state msg');
			// clupdate( ) should not be called. The MC state should be extracted from message
         levelwind_items_clupdate(&p->pmbx_cid_drum_tst_stepcmd->ncan.can);
			noteuse |= LEVELWINDSWSNOTEBITCAN2;

		}

		if ((noteval & LEVELWINDSWSNOTEBITCAN1) != 0) 
		{   // CAN:  CANID_TST_STEPCMD: U8_FF DRUM1: U8: Enable,Direction, FF: CL position: E4600000
		    // Received CAN msg with Control Lever position, direction and enable bits 
			levelwind_items_clupdate(&p->pmbx_cid_drum_tst_stepcmd->ncan.can);
			noteuse |= LEVELWINDSWSNOTEBITCAN1;

         switch (p->lw_state & 0xF0)   // deal with CAN notification based on lw_state
         {
            

            case (LW_OFF & 0xF0):
            {
               // if Track or Center is selected on CP move to appropriate state
               break;
            }

            case (LW_OVERRUN & 0xF0):
            {  
               // this handled by switch changes?
               break;
            }         

            case (LW_MANUAL & 0xF0):
            {
               // this handled by switch changes alone?
               break;
            }

            case (LW_CENTER & 0xF0):
            {
               // only allowed in Retrieve state
               break;
            }

            case (LW_INDEX & 0xF0):
            {
               // code here sequences through indexing process
               break;
            }

            case (LW_TRACK & 0xF0):
            {
               // code here monitors MC and CP state to move to appropriate state
               break;
            }

            case (LW_LOS & 0xF0):
            {
               // exit from LOS handled by switches or Tim2 ISR?
               break;
            }
         }               
      }

		if ((noteval & LEVELWINDSWSNOTEBITSWT1) != 0) 
		{ // Software timer #1: Send heartbeat
			levelwind_items_CANsendHB();
			noteuse |= LEVELWINDSWSNOTEBITSWT1;
		}
	}
}

/* *************************************************************************
 * void levelwind_task_cp_sate_update(struct CANRCVBUF* pcan);
 * @param   : pcan = pointer to CAN msg struct
 * @brief   : update of control panel state structure
 * *************************************************************************/
void levelwind_task_cp_state_init(void)
{
   
   struct CONTROLPANELSTATE* pcp = &cp_state;         // Convenience pointer

   pcp->init = INITIALIZED;

   // initially assumed input states
   pcp->clpos = 0.0f;
   pcp->safe_active = SAFE;
   pcp->armed = 0;
   pcp->retrieve_prep = 0;     
   pcp->zero_tension = 0;  
   pcp->zero_odometer = 0;     
   pcp->brake = 0;  
   pcp->guillotine = 0;     
   pcp->emergency = NORMAL;  
   pcp->lw_mode = 0;     
   pcp->lw_index = 0;
   pcp->reverse = FORWARD;     
   pcp->local_remote = LOCAL;  
   pcp->active_drum = 1;         // assumes single drum system     
   pcp->operating_drums = 0x01;   // REVIST assumes bit 0 corresponds to drum #1
   
   
   // initially assumed output states
   pcp->safe_led = 0;
   pcp->prep_led = 0;
   pcp->armed_led = 0;
   pcp->grndrllrtn_led = 0;
   pcp->ramp_led = 0;
   pcp->climb_led = 0;
   pcp->recovery_led = 0;
   pcp->retrieve_led = 0;
   pcp->abort_led= 0;
   pcp->stop_led = 0;
   pcp->arm_pb_led = 0;
   pcp->prep_recovery_led = 0;
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
   struct CONTROLPANELSTATE* pcp = &cp_state;         // Convenience pointer


   /* Extract and convert CL position from payload */
   pcp->clpos = pcan->cd.uc[SAFEACTIVE_BYTE] * CLPOS_SCL;



   /* extract switch and pb StartLevelwindTask*/
   pcp->safe_active = (pcan->cd.uc[SAFEACTIVE_BYTE] & SAFEACTIVE_MASK) 
      ? ACTIVE:SAFE;

   pcp->armed = (pcan->cd.uc[ARMED_BYTE] & ARMED_MASK)
      ? 1:0;

   pcp->retrieve_prep = (pcan->cd.uc[RETRIEVEPREP_BYTE] & RETRIEVEPREP_MASK) 
      ? 1:0;

   pcp->zero_tension = (pcan->cd.uc[ZEROTEN_BYTE] & ZEROTEN_MASK) 
      ? 1:0;

   pcp->zero_odometer = (pcan->cd.uc[ZEROODOM_BYTE] & ZEROODOM_MASK)
      ? 1:0;

   pcp->brake = (pcan->cd.uc[BRAKE_BYTE] & BRAKE_MASK)
      ? 1:0;

   pcp->guillotine = (pcan->cd.uc[GUILLOTINE_BYTE] & GUILLOTINE_MASK) 
      ? 1:0;

   pcp->emergency = (pcan->cd.uc[EMERGENCY_BYTE] & EMERGENCY_MASK) 
      ? EMERGENCY:NORMAL;

   pcp->lw_mode = pcan->cd.uc[LWMODE_BYTE] & LWMODE_MASK;

   pcp->lw_index = (pcan->cd.uc[LWINDEX_BYTE] & LWINDEX_MASK) 
      ? 1:0;

   pcp->reverse = (pcan->cd.uc[REVERSE_BYTE] & REVERSE_MASK) 
      ? 1:0;
   
   pcp->local_remote = (pcan->cd.uc[LOCALREMOTE_BYTE] & LOCALREMOTE_MASK) 
      ? REMOTE:LOCAL;

   pcp->active_drum = pcan->cd.uc[ACTIVEDRUM_BYTE] & ACTIVEDRUM_MASK;

   pcp->operating_drums = pcan->cd.uc[OPDRUMS_BYTE] & OPDRUMS_MASK;

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
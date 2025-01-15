/******************************************************************************
* File Name          : OdometerTask.c
* Date First Issued  : 12/12/2024
* Description        : Odometer function w STM32CubeMX w FreeRTOS
*******************************************************************************/

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "malloc.h"

#include "main.h"
#include "morse.h"
#include "drum_items.h"
#include "MailboxTask.h"
#include "controlpanel_items.h"
#include "OdometerTask.h"
#include "odometer_items.h"
#include "../../../GliderWinchCommons/embed/svn_common/trunk/db/gen_db.h"
#include "levelwind_func_init.h"

uint32_t debugodo1;

static void do_req_codes(struct CANRCVBUF* pcan);
void odometer_func_init_canfilter(struct ODOMETERFUNCTION* p);

osThreadId OdometerTaskHandle;

uint32_t dbgEth;

struct ODOMETERFUNCTION odometerfunction;
struct CONTROLPANELSTATE cp_state;

#if 1
/* *************************************************************************
 * static void toggle_led(void);
 * @brief   : Debugging 
 * *************************************************************************/
static uint32_t zledctr1;
static void toggle_led(void)
{
   zledctr1 += 1;
   if (zledctr1 >= 32)        
   {
      zledctr1 = 0;
      HAL_GPIO_TogglePin(GPIOD,LED_RED_Pin);
   }
}   
#endif

/* *************************************************************************
 * void swtim1_callback(TimerHandle_t tm);
 * @brief	: Software timer 1 timeout callback
 * *************************************************************************/
static void swtim1_callback(TimerHandle_t tm)
{
	xTaskNotify(OdometerTaskHandle, ODOMETERNOTBITSWT1, eSetBits);
	return;
}

/* *************************************************************************
 * void StartOdometerTask(void const * argument);
 *	@brief	: Task startup
 * *************************************************************************/
struct SWITCHPTR* psw_safeactivex; // Debugging

void StartOdometerTask(void const * argument)
{
	struct CANRCVBUF* pcan;
	struct ODOMETERFUNCTION* p = &odometerfunction; // Convenience pointer

	/* A notification copies the internal notification word to this. */
	uint32_t noteval = 0;    // Receives notification word upon an API notify

   // Load parameters
   odometer_idx_v_struct_hardcode_params(&p->lc);

   /* Add CAN Mailboxes                        CAN           CAN ID        Notify bit   Paytype */
   p->pmbx_cid_gps_sync    =  MailboxTask_add(pctl0,p->lc.cid_gps_sync,NULL,ODOMETERNOTBITSYNC,0,U8);
   p->pmbx_cid_mc_state    =  MailboxTask_add(pctl0,p->lc.cid_mc_state,NULL,ODOMETERNOTBITMC,0,U8);
   p->pmbx_cid_cmd_encoder =  MailboxTask_add(pctl0,p->lc.cid_cmd_encoder ,NULL,ODOMETERNOTBITCMD,0,U8);
   /* NOTE: The following handles the RESET for program loading over CAN. */
   p->pmbx_cid_cmd_uni_bms_pc_i = MailboxTask_add(pctl0,p->lc.cid_cmd_uni_bms_pc_i ,NULL,ODOMETERNOTBITRESET,0,U8);

extern void odometer_items_init(struct ODOMETERFUNCTION* p);
   odometer_items_init(p);

    /* Create timer #1: hearbeat (2 per sec) */
	odometerfunction.swtim1 = xTimerCreate("swtim1",
		   pdMS_TO_TICKS(100), 
		   pdTRUE, (void *) 0, 
		   swtim1_callback);
	if (odometerfunction.swtim1 == NULL) {morse_trap(404);}

	/* Start heart-beat timer 100 ms ticks */
	BaseType_t bret = xTimerReset(p->swtim1, pdMS_TO_TICKS(100));
	if (bret != pdPASS) {morse_trap(405);}

// LevelwindTask starts CAN
extern CAN_HandleTypeDef hcan1;
	HAL_CAN_Start(&hcan1); // CAN1
	odometer_func_init_canfilter(p);

	for (;;)
	{
		/* Wait for notifications */
		xTaskNotifyWait(0,0xffffffff, &noteval, portMAX_DELAY);

      if ((noteval & ODOMETERNOTBITSWT1) != 0)
		{ // Software timer callback caused this
debugodo1 += 1;         
//         if (p->hboctr > 0)
//            p->hboctr -= 1;
		}

      if ((noteval & ODOMETERNOTBITSYNC) != 0)
      { // gps/logger time sync msg ('00400000')
      }

      if ((noteval & ODOMETERNOTBITMC) != 0)
      { // Master controller state msg ('26000000')
      }

      if ((noteval & ODOMETERNOTBITCMD) != 0)
      { // command/request ('83600000'|'83800000')
      	pcan = &p->pmbx_cid_cmd_encoder->ncan.can;
      	do_req_codes(pcan);
      }

      if ((noteval & ODOMETERNOTBITTIM2) != 0)
      { // New set of encoder readings
toggle_led();         
         odometer_items_compute();
         odometer_items_hearbeat();
      }

      if ((noteval & ODOMETERNOTBITRESET) != 0)
      { // command/request: LDR_RESET ('AEC00000')
      	pcan = &p->pmbx_cid_cmd_uni_bms_pc_i->ncan.can;
      	if (pcan->cd.uc[0] == LDR_RESET) // Execute a RESET ###############################
      	{
      		if (pcan->cd.ui[1] == I_AM_CANID)
      		{
					#define SCB_AIRCR 0xE000ED0C
					*(volatile unsigned int*)SCB_AIRCR = (0x5FA << 16) | 0x4;// Cause a RESET
					//	while (1==1);// Redundant. Reset means it is "gone"
				}
      	}
//			morse_trap(888);
      }
	}
}

/* *************************************************************************
 * osThreadId xOdometerTaskCreate(uint32_t taskpriority);
 * @brief	: Create task; task handle created is global for all to enjoy!
 * @param	: taskpriority = Task priority (just as it says!)
 * @return	: OdometerTaskHandle
 * *************************************************************************/
osThreadId xOdometerTaskCreate(uint32_t taskpriority)
{
 	osThreadDef(OdometerTask, StartOdometerTask, osPriorityNormal, 0, (96));
	OdometerTaskHandle = osThreadCreate(osThread(OdometerTask), NULL);
	vTaskPrioritySet( OdometerTaskHandle, taskpriority );
	return OdometerTaskHandle;
}
/* *************************************************************************
 * static void do_req_codes(struct CANRCVBUF* pcan);
 *	@brief	: Respond to the CAN msg request code
 *  @param  : pcan = point to CAN msg struct
 * *************************************************************************/
static void do_req_codes(struct CANRCVBUF* pcan)
{
	/* First payload byte holds root request code. */
	switch (pcan->cd.uc[0])
	{
	case LDR_RESET: // Execute a RESET ###############################
		#define SCB_AIRCR 0xE000ED0C
//		*(volatile unsigned int*)SCB_AIRCR = (0x5FA << 16) | 0x4;// Cause a RESET
//		while (1==1);// Redundant. Reset means it is "gone"
		morse_trap(888);
		break; 

	case CMD_CMD_TYPE2: // 
		break;

	case LDR_SQUELCH:
		break;		

	default:
//		bqfunction.warning = 551;
morse_trap(551);
		break;
	}
	return;
}
/* *************************************************************************
 * void odometer_func_init_canfilter(struct ODOMETERFUNCTION* p);
 *	@brief	: Setup CAN hardware filter with CAN addresses to receive
 * @param	: p = pointer to odometer function struct
 * *************************************************************************/
void odometer_func_init_canfilter(struct ODOMETERFUNCTION* p)
{
	canfilt(666,p->pmbx_cid_gps_sync);
   canfilt(667,p->pmbx_cid_mc_state);
   canfilt(668,p->pmbx_cid_cmd_encoder);
   canfilt(669,p->pmbx_cid_cmd_uni_bms_pc_i);

	return;
}
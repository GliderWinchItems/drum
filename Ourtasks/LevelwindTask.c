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


osThreadId LevelwindTaskHandle;

uint32_t dbgEth;

struct LEVELWINDFUNCTION levelwindfunction;

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

	/* Initialize function struct. */
	levelwind_func_init_init(p);

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
		}
		if ((noteval & LEVELWINDSWSNOTEBITCAN1) != 0) 
		{ // CAN:  CANID_TST_STEPCMD: U8_FF DRUM1: U8: Enable,Direction, FF: CL position: E4600000
		    // Received CAN msg with Control Lever position, direction and enable bits 
			levelwind_items_clupdate(&p->pmbx_cid_drum_tst_stepcmd->ncan.can);
			noteuse |= LEVELWINDSWSNOTEBITCAN1;
		}	
		if ((noteval & LEVELWINDSWSNOTEBITSWT1) != 0) 
		{ // Software timer #1: Send heartbeat
			levelwind_items_CANsendHB();
			noteuse |= LEVELWINDSWSNOTEBITSWT1;
		}

	/* =========== States go here ============ */

	}
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
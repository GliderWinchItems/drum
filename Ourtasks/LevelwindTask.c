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

#include "stepper_items.h"
#include "drum_items.h"
#include "LevelwindTask.h"
#include "stepper_switches.h"

osThreadId LevelwindTaskHandle;

uint32_t dbgEth;

/* *************************************************************************
 * void StartLevelwindTask(void const * argument);
 *	@brief	: Task startup
 * *************************************************************************/
struct SWITCHPTR* psw_safeactivex; // Debugging

void StartLevelwindTask(void const * argument)
{
	/* A notification copies the internal notification word to this. */
	uint32_t noteval = 0;    // Receives notification word upon an API notify
	uint32_t noteuse = 0xffffffff;

	stepper_switches_init();

	HAL_NVIC_SetPriority(ETH_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(ETH_IRQn);

    /* Set interrupt pending */
  //NVIC_SetPendingIRQ(ETH_IRQn);

	for (;;)
	{
		/* Wait for notifications */
		xTaskNotifyWait(0,noteuse, &noteval, portMAX_DELAY);
		noteuse = 0;	// Accumulate bits in 'noteval' processed.
		if ((noteval & STEPPERSWSNOTEBITISR) != 0)
		{ // Here stepper_items.c triggered the ETH_IRQHandler
			noteuse |= STEPPERSWSNOTEBITISR;
dbgEth += 1;

		}
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
 * Commandeered vector for stepper_items_TIM2_IRQHandler notifications
 * ################################################################## */
void ETH_IRQHandler(void)
{	
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if (LevelwindTaskHandle != NULL)
	{ // Here, notify one task a new msg added to circular buffer
		xTaskNotifyFromISR(LevelwindTaskHandle,\
		STEPPERSWSNOTEBITISR, eSetBits,\
		&xHigherPriorityTaskWoken );
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken ); // Trigger scheduler
	}
	return;
}
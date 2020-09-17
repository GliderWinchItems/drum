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

	for (;;)
	{
		/* Wait for notifications */
		xTaskNotifyWait(0,0xffffffff, &noteval, portMAX_DELAY);
		noteuse = 0;	// Accumulate bits in 'noteval' processed.
		osDelay(10);
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
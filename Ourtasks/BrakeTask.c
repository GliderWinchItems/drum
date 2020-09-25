/******************************************************************************
* File Name          : BrakeTask.c
* Date First Issued  : 09/15/2020
* Description        : Brake function w STM32CubeMX w FreeRTOS
*******************************************************************************/

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "malloc.h"

#include "main.h"
#include "morse.h"

#include "stepper_items.h"
#include "drum_items.h"
#include "BrakeTask.h"



osThreadId BrakeTaskHandle;

/* *************************************************************************
 * void StartBrakeTask(void const * argument);
 *	@brief	: Task startup
 * *************************************************************************/
void StartBrakeTask(void const * argument)
{
	for (;;)
	{
		osDelay(10);
	}
}
/* *************************************************************************
 * osThreadId xBrakeTaskCreate(uint32_t taskpriority);
 * @brief	: Create task; task handle created is global for all to enjoy!
 * @param	: taskpriority = Task priority (just as it says!)
 * @return	: BrakeTaskHandle
 * *************************************************************************/
osThreadId xBrakeTaskCreate(uint32_t taskpriority)
{
 	osThreadDef(BrakeTask, StartBrakeTask, osPriorityNormal, 0, (192));
	BrakeTaskHandle = osThreadCreate(osThread(BrakeTask), NULL);
	vTaskPrioritySet( BrakeTaskHandle, taskpriority );
	return BrakeTaskHandle;
}
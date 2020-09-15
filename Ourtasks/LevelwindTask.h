/******************************************************************************
* File Name          : LevelwindTask.h
* Date First Issued  : 09/15/2020
* Description        : Levelwind function w STM32CubeMX w FreeRTOS
*******************************************************************************/

#ifndef __LEVELWINDTASK
#define __LEVELWINDTASK

#include <stdint.h>
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "CanTask.h"

/* *************************************************************************/
 osThreadId xLevelwindTaskCreate(uint32_t taskpriority);
/* @brief	: Create task; task handle created is global for all to enjoy!
 * @param	: taskpriority = Task priority (just as it says!)
 * @return	: LevelwindTaskHandle
 * *************************************************************************/

 extern osThreadId LevelwindTaskHandle;

#endif


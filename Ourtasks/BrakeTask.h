/******************************************************************************
* File Name          : BrakeTask.h
* Date First Issued  : 09/15/2020
* Description        : Brake function w STM32CubeMX w FreeRTOS
*******************************************************************************/

#ifndef __BRAKETASK
#define __BRAKETASK

#include <stdint.h>
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "CanTask.h"

/* *************************************************************************/
 osThreadId xBrakeTaskCreate(uint32_t taskpriority);
/* @brief	: Create task; task handle created is global for all to enjoy!
 * @param	: taskpriority = Task priority (just as it says!)
 * @return	: DrumTaskHandle
 * *************************************************************************/

 extern osThreadId BrakeTaskHandle;

#endif


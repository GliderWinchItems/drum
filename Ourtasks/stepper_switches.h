/******************************************************************************
* File Name          : stepper_switches.h
* Date First Issued  : 09/16/2020
* Description        : Levelwind function w STM32CubeMX w FreeRTOS
*******************************************************************************/


#ifndef __STEPPERSWITCHES
#define __STEPPERSWITCHES

#include <stdint.h>
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "CanTask.h"

#define LIMITDBINSIDE  0
#define LIMITDBOUTSIDE 1

/* Parameters for switch contact closures versus position accumulator. */
struct STEPPERSWCONTACT
{
   int32_t close;
   int32_t open;
   uint32_t delay;
};

struct EXTISWITCHSTATUS
{
   uint32_t tim2;   // TIM2 CNT
    int32_t posaccum_NO;  // Position accumulator
    int32_t posaccum_NC;  // Position accumulator
   uint8_t  cur;    // Current
   uint8_t  prev;   // Previous
   uint8_t  dbs;    // Debounced state
   uint8_t  flag1;   // 0 = handled; 1 = not handled
   uint8_t  flag2;   // 0 = handled; 1 = not handled

};

#endif


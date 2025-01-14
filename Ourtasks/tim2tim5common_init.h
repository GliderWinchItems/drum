/******************************************************************************
* File Name          : tim2tim5common_init.h
* Date First Issued  : 01/12/25/
* Description        : Common timer inits for odometer and levelwind 
*******************************************************************************/

#ifndef __TIM2TIM5COMMONINIT
#define __TIM2TIM5COMMONINIT

#include "LevelwindTask.h"
#include "OdometerTask.h"

/* *************************************************************************/
 void tim2tim5common_init(void);
/* @brief	: Init timers common to TIM2 and TIM5
 * *************************************************************************/

extern TIM_TypeDef  *pT2base; // Register base address 
extern TIM_TypeDef  *pT5base; // Register base address 

#endif


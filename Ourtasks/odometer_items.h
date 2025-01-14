/******************************************************************************
* File Name          : odometer_items.h
* Date First Issued  : 12/12/2024
* Description        : Odometer function w STM32CubeMX w FreeRTOS
*******************************************************************************/

#ifndef __ODOMETERITEMS
#define __ODOMETERITEMS

#include <stdint.h>
#include <math.h>
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
//#include "CanTask.h"
#include "controlpanel_items.h"
#include "mastercontroller_states.h"
#include "OdometerTask.h"

/* *************************************************************************/
// void odometer_items_init(struct ODOMETERFUNCTION* p);
/* @brief	: Init for Odometer Tast
 * *************************************************************************/
 void odometer_items_compute(void);
/* @brief   : Do computations on new reading set
 * *************************************************************************/
 void odometer_items_hearbeat(void);
/* @brief   : Do speed and acceleration computations on new reading set
 * *************************************************************************/

 /*#######################################################################################*/
 void odometer_items_TIM2_IRQHandler(void);
/* ISR routine for TIM2
 * CH1 - OC measurement duration 
 * CH2 - IC ecnoder channel Z reference (reserved: not implmented)
 * CH3 - IC encoder channel A 
 * CH4 - IC encoder channel B 
 *####################################################################################### */
 void I2C3_ER_IRQHandler(void);
/* ISR routine for I2C3_ER
 * Intermediate ISR triggered by TIM2 ISR OC, for notifying OdometerTask
 *####################################################################################### */

#endif
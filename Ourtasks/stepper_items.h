/******************************************************************************
* File Name          : stepper_items.h
* Date First Issued  : 07/31/2020
* Description        : Stepper motor associated items
*******************************************************************************/

#ifndef __STEPPER_ITEMS
#define __STEPPER_ITEMS

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "stepper_items.h"

/* Port and pin numbers for stepper controller. */
#define PU_port  GPIOA      // Pulse
#define PU_pin   GPIO_PIN_5 // Pulse
#define DR_port  GPIOB      // Direction
#define DR_pin   GPIO_PIN_0 // Direction
#define EN_port  GPIOB      // Enable
#define EN_pin   GPIO_PIN_1 // Enable
#define LMIN_port  GPIOE       // Limit switch inner
#define LMIN_pin   GPIO_PIN_5  // Limit switch inner
#define LMOUT_port GPIOE       // Limit switch outside
#define LMOUT_pin  GPIO_PIN_10 // Limit switch outside


struct STEPPERSTUFF
{
	int64_t  position;	 // Step count of position
	float	 clfactor;	 // Constant to compute oc duration at CL = 100.0
	float    speedcmdf;  // Speed command (float)
	float    clpos;      // CL position extracted from CAN msg
	uint32_t ledctr;     // Counter for throttling green LED
	uint32_t speedcmdi;	 // Commanded speed (integer)
	int32_t  accumpos;   // Position accumulator in upper 16b
	uint16_t speedinc;   // Low 16b of position accumulator
	int16_t  accumpos_prev; // Previous accumpos (hi-ord 16b)
	uint32_t drflag;     // BSRR pin set/reset bit position
	uint8_t  clbits;     // Bits from CL CAN msg

};

/* *************************************************************************/
 void stepper_items_init(TIM_HandleTypeDef *phtim);
/* phtim = pointer to timer handle
 * @brief	: Initialization of channel increment
 * *************************************************************************/
 void stepper_items_clupdate(uint8_t dr);
 /* @param 	: dr = direction: 0 = forward, not 0 = reverse
  * @brief	: Initialization of channel increment
 * *************************************************************************/
 void stepper_items_IRQHandler(TIM_HandleTypeDef *phtim);


 extern struct STEPPERSTUFF stepperstuff;

#endif

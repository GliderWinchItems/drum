/******************************************************************************
* File Name          : GevcuUpdates.c
* Date First Issued  : 07/02/2019
* Description        : Update outputs in Gevcu function w STM32CubeMX w FreeRTOS
*******************************************************************************/

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "malloc.h"
#include "ADCTask.h"
#include "adctask.h"

#include "GevcuTask.h"
#include "gevcu_idx_v_struct.h"
#include "CanTask.h"
#include "gevcu_msgs.h"

#include "morse.h"

/* From 'main.c' */
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

static void payloadfloat(uint8_t *po, float f)
{
	union FFUI
	{
		float f;
		uint8_t u8[4];
	}ffui;
	ffui.f = f;

	*(po + 0) = ffui.u8[0];
	*(po + 1) = ffui.u8[1];
	*(po + 2) = ffui.u8[2];
	*(po + 3) = ffui.u8[3];
	return;
}

/* *************************************************************************
 * void GevcuUpdates(void);
 * @brief	: Update outputs based on bits set
 * *************************************************************************/
void GevcuUpdates(void)
{
	
	
	/* Reset new & various flags. */
	gevcufunction.evstat &= ~(
		EVSWTIM1TICK | /* Timer tick */
		EVNEWADC       /* new ADC readings */
		);

	return;
}

	

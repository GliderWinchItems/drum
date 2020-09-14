/******************************************************************************
* File Name          : drum_items.c
* Date First Issued  : 09/08/2020
* Description        : Drum function
*******************************************************************************/

#include "drum_items.h"
#include "stm32f4xx_hal.h"

/* Struct with many things for the drum function. */
struct DRUMSTUFF drumstuff;

extern TIM_HandleTypeDef htim2;


/* *************************************************************************
 * void drum_idx_v_struct_hardcode_params(struct DRUMSTUFF* p);
 * @brief       : Initialization of parameters
 * *************************************************************************/
void drum_idx_v_struct_hardcode_params(struct DRUMLC* p)
{
	p->Kencodercountperrev = 1440; // Encoder counts per revolution
	p->Kdrumdia    = 0.9947f;     // Drum diameter (meters)
	p->Kgearratio  = 3.80f;       // Encoder:drum axle gear ratio
	return;
}
/* *************************************************************************
 * void drum_items_init(struct DRUMSTUFF* p);
 * @brief       : Initialization of parameters
 * @param  	: p = pointer to struct with everything for this drum
 * *************************************************************************/
void drum_items_init(struct DRUMSTUFF* p)
{
	/* Load parameters */
	drum_idx_v_struct_hardcode_params(&p->lc);

	/* Computed from parameters. */
	// Factor: Encoder counts to encoder rev/min
	p->Fspeed_rpm_encoder  = ( (60.0f * 84000000.0f) / p->lc.Kencodercountperrev);

	// Factor: Cable distance = (drum_dia * pi) / (enccoder_counts_per_rev * encoder:drum_axle ratio)
	p->Fcable_distance  = (p->lc.Kdrumdia * 3.14159265f) / (p->lc.Kencodercountperrev * p->lc.Kgearratio);

	// Factor: Cable speed = encoder_rpm * ((drum_dia * pi) / (gear_ratio * 60(sec/min) )
	p->Fspeed_cable     = (p->lc.Kdrumdia * 3.14159265f) / (p->lc.Kgearratio * 60.0f);

	p->cablezeroref  = 0; // Encoder counter reference/offset for zero cable

	return;	
}

/* *************************************************************************
 * void drum_items_computespeed(struct DRUMSTUFF* p);
 * @brief       : Compute speed for the encoder channel
 * @param 		: p = pointer to data for channel
 * *************************************************************************/
void drum_items_computespeed(struct DRUMSTUFF* p)
{
	/* Copy latest time and count, and be sure it didn't change mid-copy. */
	do
	{ 
		p->decA.tmp = p->decA.cur; // Copy time and count set by ISR

		// Loop until re-read results in same two readings.
	} while ( (p->decA.tmp.tim != p->decA.cur.tim) || (p->decA.tmp.cnt != p->decA.cur.cnt) );


	/* Get input capture time now */
	//p->sampletim = htim2.Instance->CNT;

	/* Compute number of encoder ticks between latest encoder count
	     and latest encoder count on previous pass thru this routine.  */
	p->decA.diff.cnt = p->decA.tmp.cnt - p->decA.prev.cnt;
	if (p->decA.diff.cnt == 0)
	{ // Here, no encoder counts between sample times.
		p->Cspeed_rpm_encoder = 0; // Assume the speed is zero.
		p->Cspeed_cable = 0;
		return;
	}
	/* Here, one or more encoder counts between sample times. */

	/* Compute time difference between latest encoder input capture
	     and latest encoder input capture on previous pass thru 
	     this routine */
	p->decA.diff.tim = p->decA.tmp.tim - p->decA.prev.tim;

	/* The current readings are now the previous readings. */
	p->decA.prev = p->decA.tmp;

	/* Encoder speed (rpm). Note that diff.cnt is signed. */
	p->Cspeed_rpm_encoder = p->Fspeed_rpm_encoder * (float)p->decA.diff.cnt / (float)p->decA.diff.tim;

	/* Cable speed (meters/sec) = Encoder_rpm * Factor */
	p->Cspeed_cable = p->Cspeed_rpm_encoder * p->Fspeed_cable;

	// Cable out (meters) = Factor * net_encoder_counts
	p->Ccable_distance = p->Fcable_distance * (float)(p->cablezeroref + p->decA.tmp.cnt);

	return;
}
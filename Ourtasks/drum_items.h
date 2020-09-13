/******************************************************************************
* File Name          : drum_items.h
* Date First Issued  : 09/08/2020
* Description        : Drum function
*******************************************************************************/

#ifndef __DRUM_ITEMS
#define __DRUM_ITEMS

#include <stdint.h>

struct DRUMLC
{
	float Kgearratio; // Encoder shaft:drum axle gear ratio
	float Kdrumdia;   // Nominal drum diameter (meters)
	float Kencodercountperrev;
};


/* Input Capture time and encoder count. */
struct DRUMTIMCNT
{
   uint32_t tim; // Input capture time
    int32_t cnt; // Encoder count at capture time	
};


struct DRUMENCODERCH /* Drum Encoder Channel */
{
   struct DRUMTIMCNT cur;
   struct DRUMTIMCNT prev;
   struct DRUMTIMCNT diff;
   struct DRUMTIMCNT tmp;

};

struct DRUMSTUFF
{
   struct DRUMLC lc;          // Parameters
   struct DRUMENCODERCH decA; // Time & Count Encoder Channel A: even

//   struct DRUMENCODERCH decB; // Time & Count Encoder Channel B
//   struct DRUMENCODERCH decZ; // Time & Count Encoder Channel Z
   uint32_t encoder; // Latest encoder (needed?)
   uint32_t CR1; // Latest direction bit (needed?)

   int32_t cablezeroref;     // Encoder count for zero line out.

	float Fspeed_rpm_encoder; // Factor to convert encoder counts to rpm
   	float Fspeed_cable;	      // Factor to convert encoder rpm to meters/sec
	float Fcable_distance;    // Factor to convert encoder count to meters

	float Cspeed_rpm_encoder; // Computed encoder counts to rpm
   	float Cspeed_cable;	      // Computed encoder rpm to meters/sec
	float Ccable_distance;    // Computed encoder count to meters

};

/* *************************************************************************/
 void drum_items_computespeed(struct DRUMSTUFF* p);
/* @brief       : Compute speed for the encoder channel
 * @param 		: p = pointer to data for channel
 * *************************************************************************/
 void drum_items_init(struct DRUMSTUFF* p);
/* @brief       : Initialization of parameters
 * @param  	: p = pointer to struct with everything for this drum
 * *************************************************************************/

extern struct DRUMSTUFF drumstuff;

#endif
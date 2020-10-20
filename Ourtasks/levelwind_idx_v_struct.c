/******************************************************************************
* File Name          : levelwind_idx_v_struct.c
* Date First Issued  : 09/23/2020
* Board              :
* Description        : Load parameter struct
*******************************************************************************/

#include "levelwind_idx_v_struct.h"
#include "SerialTaskReceive.h"
#include "../../../GliderWinchCommons/embed/svn_common/trunk/db/gen_db.h"

/* *************************************************************************
 * void levelwind_idx_v_struct_hardcode_params(truct LEVELWINDLC* p);
 * @brief	: Init struct from hard-coded parameters (rather than database params in highflash)
 * @return	: 0
 * *************************************************************************/
void levelwind_idx_v_struct_hardcode_params(struct LEVELWINDLC* p)
{
	p->size    = 32;
	p->crc     = 0;   // TBD
   p->version = 1;   // 

	/* Timings in milliseconds. Converted later to timer ticks. */

/* GevcuTask counts 'sw1timer' ticks for various timeouts.
 We want the duration long, but good enough resolution(!)
 With systick at 512/sec, specifying 8 ms yields a 4 tick duration
 count = 4 -> 64/sec (if we want to approximate the logging rate)
 count = 64 -> 1/sec 
*/ 
	//    my drum should be associated with whole node and not just the
   //    level-wind function. need to figure out where the parameters for 
   //    the whole node should be placed       
   p->mydrum      = 1;     // drum this node is assigned to     

   p->hbct_t      = 500;   // Heartbeat ct: milliseconds between sending 
   
   p->hbct        = 64;    // Number of swctr ticks between heartbeats
   p->Ka          = 8;     // Reversal rate
   p->Nr          = 3500;  // Sweep rate to reversal rate ratio
   p->Lplus       = 15000; // Calibrated start of positive reversal region
   p->Lminus      =    0;  // Calibrated start of negative reversal region

   p->ocidx       = 21000; // Indexing increment for 250 ms
   p->Nswp        = 5;     // Sweeping increment
   p->Nman        = 5;     // Sweeping increment


// For development; these will likely not be in operational code
   p->clfactor    = 168E3; // CL scaling: 100% = 50 us
   p->ka_levelwind_t = 2555;  // keep-alive timeout (timeout delay ms)
   p->cltimemax   = 512;   // Number of software timeout ticks max


// CAN ids levelwind sends
   //                      CANID_HEX      CANID_NAME             CAN_MSG_FMT     DESCRIPTION
   // Others receive
   p->cid_hb_levelwind  = 0xE4A00000;   // CANID_HB_LEVELWIND: U8_U32, Heartbeat Status, levelwind position accum');

// List of CAN ID's for setting up hw filter for incoming msgs
   	// We receive: Logger/gps 
//	p->cid_gps_sync     = 0x00400000; // CANID_HB_TIMESYNC:  U8 : GPS_1: U8 GPS time sync distribution msg-GPS time sync msg
	// We receive stepper repo: update100K sends
	p->cid_drum_tst_stepcmd	=  CANID_TST_STEPCMD; //0xE4600000; // CANID_TST_STEPCMD: U8_FF DRUM1: U8: Enable,Direction, FF: CL position:
   p->cid_mc_state = CANID_MC_STATE; //'CANID_MC_STATE','26000000', 'MC', 'UNDEF','MC: Launch state msg');

	return;
}

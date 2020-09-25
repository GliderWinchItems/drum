/******************************************************************************
* File Name          : levelwind_idx_v_struct.h
* Date First Issued  : 09/23/2020
* Board              :
* Description        : Load parameter struct
*******************************************************************************/

#include <stdint.h>
#include "common_can.h"
#include "LevelwindTask.h"

#ifndef __LEVELWIND_IDX_V_STRUCT
#define __LEVELWIND_IDX_V_STRUCT

/* GevcuTask counts 'sw1timer' ticks for various timeouts.
 SWTIM1TICKDURATION
 We want the duration long, but good enough resolution(!)
 With systick at 512/sec, specifying 8 ms yields a 4 tick duration
 count = 4 -> 64/sec (if we want to approximate the logging rate)
 count = 64 -> 1/sec 
*/ 
#define SWTIM1TICKDURATION 8
#define SWTIM1TICKPERSEC (1000/SWTIM1TICKDURATION)

#define SWTIM1_64PERSEC (configTICK_RATE_HZ/64) // swtim1 ticks 

/* Parameters levelwind instance (LC = Local Copy) */
struct LEVELWINDLC
 {
/* NOTE: all suffix _t parameters are times in milliseconds */

	uint32_t size;
	uint32_t crc;   // TBD
   uint32_t version;   // 

	/* Timings in milliseconds. Converted later to 'swtim1' ticks. */
	uint32_t ka_levelwind_t; // keepalive from PC (ms) 
	uint32_t hbct_t;      // Heartbeat ct: ms between sending 

   float    clfactor;   // Constant to compute oc duration at CL = 100.0
   uint32_t cltimemax;  // Max timer count for shutdown
   int32_t  Lplus;      //
   int32_t  Lminus;     //
   uint32_t hbct;       // Number of ticks between hb msgs
   int32_t  Ka;         // reversal rate
   int32_t  Nr;         // ratio of reversal rate to sweep rate      

 // CAN ids ...........................................................................
   //                                  CANID_NAME             CAN_MSG_FMT     DESCRIPTION
    // Levelwind sends; PC receives
   uint32_t cid_hb_levelwind;        // CANID_HB_LEVELWIND: U8_U32','LEVELWIND: U8: Status, U32: stepper position accum


 // List of CAN ID's for setting up hw filter for incoming msgs
	// stepper test repo sends: drum receives
	uint32_t cid_drum_tst_stepcmd; // CANID_TST_STEPCMD: U8_FF DRUM1: U8: Enable,Direction, FF: CL position: E4600000


 };

/* *************************************************************************/
void levelwind_idx_v_struct_hardcode_params(struct LEVELWINDLC* p);
/* @brief	: Init struct from hard-coded parameters (rather than database params in highflash)
 * @return	: 0
 * *************************************************************************/
 
#endif


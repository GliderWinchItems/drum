/******************************************************************************
* File Name          : odometer_idx_v_struct.h
* Date First Issued  : 12/10/2024
* Description        : Odometer function
*******************************************************************************/

#ifndef __ODOMETERIDXVSTRUCT
#define __ODOMETERIDXVSTRUCT

#include <stdint.h>
#include <math.h>
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "CanTask.h"
#include "controlpanel_items.h"
#include "mastercontroller_states.h"
//#include "OdometerTask.h"

/* Parameters levelwind instance (LC = Local Copy) */
struct ODOMETERLC
{
	/* Timings in milliseconds. Converted later to 'swtim1' ticks. */
   uint32_t mydrum;  // drum this node is assigned to     
   uint32_t hb_t;    // Heartbeat: milliseconds between sending 
   uint32_t zspd_t;  // Zero drum speed detection duration (ms)

// NOTE: These could be exchanged for simply drum hub diameter or radius
	uint32_t rim_to_hub;  // Distance (mm), drum rim -to- drum hub
	uint32_t rim_to_rim;  // Distance (mm), drum diameter

	// Default Ephemeral parameters
	uint32_t rope_out;   // Distance (cm), 
	uint32_t rim_to_rope; // Distance (mm), drum rim -to- loaded rope

	float rope_total_default;  // Total rope loaded on drum (meters)
	float rim_to_rope_default; // Distance (mm), drum rim -to- loaded rope
   float rim_to_cushion;      // Distance (mm), drum rim -to- cushion layer
	float hub_bare_dia;        // Drum without rope diameter (meters)
	float rope_dia; // Effective dia for drum loading computations (mm)
   float drum_outer_dia;  // Drum flange outer diameter(meters)
   float drum_width;      // Drum inside width (meters)
   float encoder_ratio;   // Gear ratio--motor:drum
   float scale_en_mtr;    // encoder ct/time scale to motor rpm
   float scale_mtr_drum;  // motor rpm scale to drum rpm
   float scale_en_circum; // encoder to drum circumference (meters)	
   float scale_speediff_accel; // speed diff sum to motor accel (rad/sec/sec)

	// gps time ticks per CAN msg (WHAT IS THIS FOR!)
//	uint8_t gpstick_ct[ODO_MSG_MAX]; // 0 = hb; 1-255 TIMESYNC msg count

// CAN ids ...........................................................................
   //                                  CANID_NAME             CAN_MSG_FMT     DESCRIPTION
 // List of CAN ID's for setting up hw filter for incoming msgs
   //                      CANID_HEX      CANID_NAME             CAN_MSG_FMT     DESCRIPTION
   // We send; Others receive
   uint32_t cid_unit_encoder; // CANID_UNIT_ENCODER1 83200000 | CANID_UNIT_ENCODER2 83400000 U8_VAR DiscoveryF4 encoder
   uint32_t cid_msg1_encoder; // CANID_MSG1_ENCODER1 83A00000 | CANID_MSG1_ENCODER2 83C00000 FF_FF lineout, drum speed
   uint32_t cid_msg2_encoder; // CANID_MSG2_ENCODER1 83E00000 | CANID_MSG1_ENCODER2 84000000 FF_FF accel, encoder speed
   uint32_t cid_msg3_encoder; // CANID_MSG3_ENCODER1 84200000 | CANID_MSG1_ENCODER2 84400000 FF_S32 drum speed, encoder counter

   /* Enable msg1,2,3 */
   uint8_t msg_enable[3]; // 1 = enable, 0 = do not send

// List of CAN ID's for setting up hw filter for incoming msgs
   	// We receive
   uint32_t cid_gps_sync;    // CANID_HB_TIMESYNC;  // 00400000 U8     GPS time sync distribution msg-GPS time sync msg
   uint32_t cid_mc_state;    // CANID_MC_STATE;     // 26000000 MC     MC Launch state msg
   uint32_t cid_cmd_encoder; // CANID_CMD_ENCODER1|CANID_CMD_ENCODER2', '83600000'|'83800000','UNIT_ENCODER', 1,1,'U8_VAR','DiscoveryF4 encoder: command');
   uint32_t cid_cmd_uni_bms_pc_i; // CANID_UNI_BMS_PC_I' AEC00000', UNIversal From PC, Used for reset');
};

/* *************************************************************************/
 void odometer_idx_v_struct_hardcode_params(struct ODOMETERLC* p);
/* @brief   : Init struct from hard-coded parameters (rather than database params in highflash)
 * @return  : 0
 * *************************************************************************/   


#endif

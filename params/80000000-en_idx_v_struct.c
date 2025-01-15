/******************************************************************************
* File Name          : 83400000_odometer_idx_v_struct.c
* Date First Issued  : 12/10/2024
* Description        : Odometer function parameters for CAN ID XXXXXXXX
* Usage              : Winch demo, encoder Discovery F4
*******************************************************************************/
#include "odometer_idx_v_struct.h"
#include "SerialTaskReceive.h"
#include "../../../GliderWinchCommons/embed/svn_common/trunk/db/gen_db.h"

/* *************************************************************************
 * void odometer_idx_v_struct_hardcode_params(truct ODOMETERLC* p);
 * @brief	: Init struct from hard-coded parameters (rather than database params in highflash)
 * @return	: 0
 * *************************************************************************/
void odometer_idx_v_struct_hardcode_params(struct ODOMETERLC* p)
{

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
   p->mydrum  =    1; // drum this node is assigned to     
   p->hb_t    = 4000; // Heartbeat: milliseconds between sending 
   p->zspd_t  =  500; // Zero drum speed detection duration (ms)

   p->rim_to_rope_default =   150.0f; // Distance (mm), drum rim -to- loaded rope
   p->rope_total_default  =  1000.0f; // Total rope loaded on drum (meters)
   p->hub_bare_dia    = 0.750f; // Drum without rope diameter (meters)
   p->rope_dia        = 4.5f;   // Effective dia for drum loading computations (mm)
   p->drum_outer_dia  = 0.975f; // Drum flange outer diameter(meters)
   p->drum_width      = 0.165f; // Drum inside width (meters)
   p->rim_to_cushion  = (p->drum_outer_dia - p->hub_bare_dia)*0.5f; // Temporary
   p->encoder_ratio   =  ((float)(53/32)*(float)(35/9)); // 6.4409722f; // Gear ratio--motor:drum
   p->scale_en_mtr    = ((float)1312500*60*64/(1440*4));// 875.0E3f; // encoder ct/time scale to motor rpm
   p->scale_mtr_drum  = 0.155279503f;// motor rpm scale to drum rpm
   p->scale_en_circum = 737.96964f; // encoder to drum circumference (meters)
   p->scale_speediff_accel = ((float)(3.141592654*2*1312500*64*64)/(1440*4)); // encoder channels ct/time diff sum to accel (rad/sec/sec)

// CAN ids encoder function
   //                         CANID_NAME      CANID_HEX  CAN_MSG_FMT     DESCRIPTION
   // We send; Others receive
   p->cid_unit_encoder  = CANID_UNIT_ENCODER1; // 83200000 U8_VAR DiscoveryF4 encoder demo winch:
   p->cid_msg1_encoder  = CANID_MSG1_ENCODER1; // 83A00000 FF_FF  DiscoveryF4 encoder demo winch: lineout, speed
   p->cid_msg2_encoder  = CANID_MSG2_ENCODER1; // 83E00000 FF_FF  DiscoveryF4 encoder demo winch: accel, encoder speed
   p->cid_msg3_encoder  = CANID_MSG3_ENCODER1; // 84200000 FF_S32 DiscoveryF4 encoder demo winch: drum speed, encoder counter

   /* Enable sending of these msgs (which may be at high rate). 1 = enable; 0 = disable. */
   p->msg_enable[0] = 1; // MSG1
   p->msg_enable[1] = 1; // MSG2
   p->msg_enable[2] = 1; // MSG3

   // We receive
   p->cid_gps_sync         = CANID_HB_TIMESYNC;  // 00400000 U8     GPS time sync distribution msg-GPS time sync msg
   p->cid_mc_state         = CANID_MC_STATE;     // 26000000 MC     MC Launch state msg
   p->cid_cmd_encoder      = CANID_CMD_ENCODER1; // 83600000 U8_VAR DiscoveryF4 encoder demo winch: command
   p->cid_cmd_uni_bms_pc_i = CANID_UNI_BMS_PC_I; // AEC00000 PC  UNIversal From PC, Used for CAN loading reset');


	return;
}


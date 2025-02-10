/******************************************************************************
* File Name          : OdometerTask.h
* Date First Issued  : 12/12/2024
* Description        : Odometer function w STM32CubeMX w FreeRTOS
*******************************************************************************/

#ifndef __ODOMETERTASK
#define __ODOMETERTASK

#include <stdint.h>
#include <math.h>
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "CanTask.h"
#include "controlpanel_items.h"
#include "mastercontroller_states.h"
#include "odometer_idx_v_struct.h"
#include "odometer_items.h"

/* TIM4 prescale = 13125; 84 MHz rate -> 1600 cts/sec */
#define ODOMETER_T4C1_DUR 100 // TIM2 ticks for 1/64 sec duration

// Task wait notification bits
#define ODOMETERNOTBITSWT1 (1<<0) // Software timer1
#define ODOMETERNOTBITSYNC (1<<1) // CAN msg: GPS time sync (0x004)
#define ODOMETERNOTBITMC   (1<<2) // CAN msg: MC state CAN  msg
#define ODOMETERNOTBITCMD  (1<<3) // CAN msg: Command requests
#define ODOMETERNOTBITTIM2 (1<<4) // TIM2 CH1 new readings
#define ODOMETERNOTBITRESET (1<<5) // CAN msg: Reset request from PC

#define NUMCANMSGSODOMETER 4 // Number of CAN msgs we send
#define ODOCANMSG_UNIT  0  // Unit 
#define ODOCANMSG_MSG1  1  // FF_FF: (float) line out & (float) drum speed)
#define ODOCANMSG_MSG2  2  // FF_FF: (float) Acceleration & (int32_t) encoder speed
#define ODOCANMSG_MSG3  3  // FF_S32: (float) Speed & (int32_t) encoder counter

#define SDEL 0.00001f // Zero speed detection window +/- (not scaled)

//  Input capture time and encoder count
struct ODOTIMCT
{
   uint32_t tim;
   uint32_t ct;
};

struct ODOTIMCT_INT
{
   int32_t tim;
   int32_t ct;
};

struct ODOMETERFUNCTION
{
   uint32_t hboct;   // Heartbeat duration count (swtim1 ticks)
   uint32_t hboctr;  // Heartbeat duration counter
   uint32_t zspdct;  // Zero speed detection (1/64th sec ticks)
   uint32_t zspdctr; // Zero speed detection counter
   uint8_t hbstate;  // Hearbeat<->64Hz sending 

   struct ODOMETERLC lc; // Parameters for odometer function

   /* Encoder counts and timer captures. */
   struct ODOTIMCT odotimct[4]; // Latest readings
   struct ODOTIMCT odotimct_buff[4]; // Buffer ISR readings
   struct ODOTIMCT_INT odotimct_int_diff[4]; // diff = (int)(new - previous) like edges
   struct ODOTIMCT odotimct_buff_prev[4]; // Latestreadings
   int32_t en_cnt;      // Latest encoder CNT at 1/64th sec TIM2 interrupt
   int32_t en_cnt_prev; // Previous
   int32_t en_cnt_diff; // Difference (buffering)
   uint8_t oe_A; // Odd/even (rising/falling edges) encoder chan A
   uint8_t oe_B; // Odd/even (rising/falling edges) encoder chan B

   /* Computation and scaling speed and acceleration. */
   float speed_sum;     // Sum speed: like edges, like channels
   float speed[4];      // Channel A & B, rising and falling edges
   float speed_prev[4]; // Previous for computing difference
   float accel_sum;     // Sum of speed differences
   float accel_ave_motor;
   float odo_speed[4];  // Raw speed (ct/tim)
   float odo_speed_ave_motor; // raw speed scaled (rpm)
   float odo_speed_ave_drum;  // motor speed scaled (rpm)
   float en_cnt_speed; // 
   float en_cnt_speed_prev; //
   float en_cnt_speed_diff; //
   float en_cnt_accel_motor; 

   /* Computation and scaling line-out. */
   float line_out;       // Line out (meters), estimated
   float initial_circum; // Circumference with loaded rope
   float working_circum; // Amount of line per drum rev (meters), varies
   float drum_rev_per_layer; // Number revs to lay one layer of rope
   float drum_dia_change_per_rev; // Diameter change per rev
   float drum_cir_change_per_rev; // Circumference change per rev
   float drum_rev_ctr;   // Drum revolutions, zero = all line in.
   float en_cnts_per_drum_rev; // Encoder counts per drum revolution
   float en_drum_ratio;  // encoder to drum rev ratio for line out
   float drum_cir_change_per_encoder_ct;
   int32_t line_out_ctr; // encoder counter
   int32_t line_out_ref; // encoder count for zero line out

   /* Incoming CAN msgs that notify this task. */
   struct MAILBOXCAN* pmbx_cid_gps_sync;    // CANID_HB_TIMESYNC:  U8 : GPS_1: U8 GPS time sync distribution msg-GPS time sync msg
   struct MAILBOXCAN* pmbx_cid_mc_state;    //'CANID_MC_STATE','26000000', 'MC', 'UNDEF','MC: Launch state msg');
   struct MAILBOXCAN* pmbx_cid_cmd_encoder; // CANID_CMD_ENCODER1|CANID_CMD_ENCODER2', '83600000'|'83800000','UNIT_ENCODER', 1,1,'U8_VAR','DiscoveryF4 encoder: command');
   struct MAILBOXCAN* pmbx_cid_cmd_uni_bms_pc_i; // CANID_UNI_BMS_PC_I' AEC00000', UNIversal From PC, Used for reset');

   /* CAN msgs this task sends. */
   struct CANTXQMSG canmsg[NUMCANMSGSODOMETER];

   uint32_t ledctr1;    // Counter for throttling green LED
   uint32_t ledctr2;    // Counter for throttling orangeLED
   uint32_t ledbit1;    // Bit for toggling green led
   uint32_t ledbit2;    // Bit for toggling orange led

   TimerHandle_t swtim1; // RTOS Timer #1 handle   

};

/* *************************************************************************/
 osThreadId xOdometerTaskCreate(uint32_t taskpriority);
/* @brief   : Create task; task handle created is global for all to enjoy!
 * @param   : taskpriority = Task priority (just as it says!)
 * @return  : OdometerTaskHandle
 * *************************************************************************/

 extern osThreadId OdometerTaskHandle;
 extern struct ODOMETERFUNCTION odometerfunction;

#endif


 

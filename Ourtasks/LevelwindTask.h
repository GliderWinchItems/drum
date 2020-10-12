/******************************************************************************
* File Name          : LevelwindTask.h
* Date First Issued  : 09/15/2020
* Description        : Levelwind function w STM32CubeMX w FreeRTOS
*******************************************************************************/

#ifndef __LEVELWINDTASK
#define __LEVELWINDTASK

#include <stdint.h>
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "CanTask.h"
#include "levelwind_switches.h"
#include "levelwind_idx_v_struct.h"
#include "levelwind_items.h"
#include "controlpanel_items.h"

/* Stepper switch bit positions */
#define LEVELWINDSWSNOTEBITLIMINDB   (1<<LIMITDBINSIDE)  
#define LEVELWINDSWSNOTEBITLIMOUTDB  (1<<LIMITDBOUTSIDE) 
#define LEVELWINDSWSNOTEBITLIMINNC   (LIMITINSIDENC) 
#define LEVELWINDSWSNOTEBITLIMINNO   (LIMITINSIDENO)
#define LEVELWINDSWSNOTEBITLIMOUTNC  (LIMITOUTSIDENC)
#define LEVELWINDSWSNOTEBITLIMOUTNO  (LIMITOUTSIDENO)
#define LEVELWINDSWSNOTEBITLIMINOVR  (OVERRUNSWINSIDE)
#define LEVELWINDSWSNOTEBITLIMOUTOVR (OVERRUNSWOUTSIDE)
#define LEVELWINDSWSNOTEBITISR       (1<<16)    // Stepper ISR
#define LEVELWINDSWSNOTEBITCAN1      (1<<17)    // CAN msg: Pushbuttons & CL position
#define LEVELWINDSWSNOTEBITSWT1      (1<<18)    // Software timer #1
#define LEVELWINDSWSNOTEBITCAN2      (1<<19)    // CAN msg: Launch state


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

// LW state machine  definitions. Lower nibble reserved for sub-states
#define LW_OFF       (0 << 4)
#define LW_OVERRUN   (1 << 4)
#define LW_MANUAL    (2 << 4)
#define LW_CENTER    (3 << 4)
#define LW_INDEX     (4 << 4)
#define LW_TRACK     (5 << 4)
#define LW_LOS       (6 << 4)

#define NUMCANMSGSLEVELWIND 2  // Number of CAN msgs levelwind sends
enum cididx
{
   CID_LEVELWIND_HB 
};

union PAYFLT
{
   float f;
   uint8_t u8[4];
   uint16_t u16[2];
   uint32_t u32;
   int32_t  s32;
   int16_t  s16[2];
}pf;

struct LEVELWINDFUNCTION
{
   struct   LEVELWINDLC lc; // Parameters for level-wind function
   union    PAYFLT   pf; // For extracting float from payload
   union    PAYFLT   posaccum;  // Stepper position accumulator
   union    PAYFLT   velaccum;  // Stepper velocity accumulator
   int16_t  pos_prev;   // Previous posaccum integral portion 
   int32_t  Lplus32;    // 32-bit extended Lplus
   int32_t  Lminus32;   // 32-bit extended Lminus
   int32_t  Ks;         // Sweep rate (Ks/65536) = levelwind pulses per encoder edge
   int32_t  rvrsldx;    // Reversal Distance
   float    speedcmdf;  // Speed command (float)
   float    focdur;     // Temp for computer inverse of CL position
   float    clpos;      // CL position extracted from CAN msg
   uint32_t cltimectr;  // Counter for loss of CL msgs
   uint32_t speedcmdi;  // Commanded speed (integer)
   uint32_t ocinc;      // OC register increment for indexing and sweeping
   uint32_t ocswp;      // OC register increment for sweep
   uint32_t hbctr;      // Count ticks for sending heartbeat CAN msg
   uint32_t drflag;     // BSRR pin set/reset bit position: direction
   uint32_t enflag;     // BSRR pin set/reset bit position: enable
   uint32_t iobits;     // Bits from CL CAN msg positioned for PB0
   uint8_t  drbit;      // Drum direction bit (0, forward|1, reverse)
   uint8_t  drbit_prev; // Previous Direction bit

   uint8_t  levelwindstatus;  // Reserved for levelwind status CAN msg
   uint8_t  lw_state;   // level-wind present state
   uint8_t  lw_mode;    // level-wind ISR mode
   uint8_t  lw_indexed; // indexed status 
   uint8_t  lw_overrun; // overrun status
   uint8_t  mc_state;   // master controller state    
   uint8_t  ocicbit;       
   uint8_t  ocicbit_prev;  

   struct   STEPPERSWCONTACT ctk[6];  // Measured switch contact open/close posaccum
   struct   EXTISWITCHSTATUS sw[6];   // Limit & overrun switches
   uint16_t swbits;     // Port E switch bits (10:15)

   // debug and characterization, potentially removable for operational code
   uint32_t dtwentry;   // DTW timer upon ISR entry
   int32_t  dtwdiff;    // DTW timer minus entry upon ISR exit
   int32_t  dtwmax;     // DTW difference max
   int32_t  dtwmin;     // DTW difference min
   uint32_t intcntr;    // interrupt counter
   uint32_t ocfauxinc;  // OC register increment for CL faux encoder  
   uint8_t  pay0;       // canmsg.cd.uc[0] saved

   uint32_t keepalive_k;  // keep-alive timeout (timeout delay timer ticks) 

   TimerHandle_t swtim1; // RTOS Timer #1 handle

/* Pointer into circular buffer for levelwind_items.c debugging. */
#if LEVELWINDDEBUG
   struct LEVELWINDDBGBUF*  pdbgbegin;
   struct LEVELWINDDBGBUF*  pdbgadd;
   struct LEVELWINDDBGBUF*  pdbgtake;
   struct LEVELWINDDBGBUF*  pdbgend;

   uint32_t ledctr1;    // Counter for throttling green LED
   uint32_t ledctr2;    // Counter for throttling orangeLED
   uint32_t ledbit1;    // Bit for toggling green led
   uint32_t ledbit2;    // Bit for toggling orange led

   uint32_t tim5cnt_offset;
#endif

   /* Pointers to incoming CAN msg mailboxes. */
   struct MAILBOXCAN* pmbx_cid_gps_sync;        // CANID_HB_TIMESYNC:  U8 : GPS_1: U8 GPS time sync distribution msg-GPS time sync msg
   struct MAILBOXCAN* pmbx_cid_drum_tst_stepcmd;// CANID_TST_STEPCMD: U8_FF DRUM1: U8: Enable,Direction, FF: CL position: E4600000
   struct MAILBOXCAN* pmbx_cid_mc_state; //'CANID_MC_STATE','26000000', 'MC', 'UNDEF','MC: Launch state msg');

   /* CAN msgs */
   struct CANTXQMSG canmsg[NUMCANMSGSLEVELWIND];
};

/* *************************************************************************/
 osThreadId xLevelwindTaskCreate(uint32_t taskpriority);
/* @brief   : Create task; task handle created is global for all to enjoy!
 * @param   : taskpriority = Task priority (just as it says!)
 * @return  : LevelwindTaskHandle
 * *************************************************************************/

 extern osThreadId LevelwindTaskHandle;
 extern struct LEVELWINDFUNCTION levelwindfunction;

#endif

/* *************************************************************************/
 void levelwind_task_cp_state_update();
/* @brief   :  update the CP state struct from CP state CAN message
 * @param   :  pcp = pointer to CP state struct
 * *************************************************************************/

 /* *************************************************************************/
 void levelwind_task_cp_state_init();
/* @brief   :  function to initialize the state struct before first CAN
 *             message
 * @param   :  pcp = pointer to CP state struct
 * *************************************************************************/
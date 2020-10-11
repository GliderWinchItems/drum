/******************************************************************************
* File Name          : ControlPanelState.h
* Date First Issued  : 10/10/2020
* Description        : Defines a structure for the control panel state
*******************************************************************************/

#ifndef __CONTROLPANELITEMS
#define __CONTROLPANELITEMS


#include <stdint.h>

/* Definition of payload values  */

// CL position payload location and scaling factor

// payload byte 0
#define CLPOS_BYTE            0  // u16 control lever position
#define CLPOS_SCL          0.5f  // each lsb equal 0.5%

// switch and pb inputs payload locations

// payload byte 1
#define SAFEACTIVE_BYTE       1
#define SAFEACTIVE_MASK       (0x01 << 7)

#define ARMED_BYTE            1
#define ARMED_MASK             (0x01 << 6)

#define RETRIEVEPREP_BYTE     1
#define RETRIEVEPREP_MASK     (0x01 << 5)

#define ZEROTEN_BYTE          1
#define ZEROTEN_MASK          (0x01 << 4)

#define ZEROODOM_BYTE         1
#define ZEROODOM_MASK         (0x01 << 3)

#define BRAKE_BYTE            1
#define BRAKE_MASK            (0x01 << 2)

#define GUILLOTINE_BYTE       1
#define GUILLOTINE_MASK       (0x01 << 1)

#define EMERGENCY_BYTE        1
#define EMERGENCY_MASK        (0x01 << 0)

// payload byte 2
#define LWMODE_BYTE           2
#define LWMODE_MASK           (0x03 << 6) // 2 bit field

#define LWINDEX_BYTE          2
#define LWINDEX_MASK          (0x01 << 5)

#define REVERSE_BYTE          2
#define REVERSE_MASK          (0x01 << 4)

#define LOCALREMOTE_BYTE      2
#define LOCALREMOTE_MASK      (0x01 << 3)

/* provisions for muliple retrieve_drums*/
#define ACTIVEDRUM_BYTE       2   
#define ACTIVEDRUM_MASK       (0x07 << 0) // 3 bit field


// payload byte 3
#define OPDRUMS_BYTE          3     
#define OPDRUMS_MASK          (0x7F << 1)   // 7 bit field


/* outputs  */
// work from right

// payload byte 7

// TBD


struct CONTROLPANELSTATE
{
   enum init {INITIALIZED, UPDATED} init;   

   /* control lever  */
   float    clpos;   // control lever position

   /* input signals  */
   // levels use enums, momentary push buttons are 1 for pushed, 0 not pushed
   // general assumption is all are debounced by control panel function if
   // needed
   enum     safe_active {SAFE, ACTIVE} safe_active;
   uint8_t  armed;
   uint8_t  retrieve_prep;
   uint8_t  zero_tension;
   uint8_t  zero_odometer;
   uint8_t  brake;
   uint8_t  guillotine;
   enum     emergency {NORMAL, EMERGENCY} emergency;
   uint8_t  lw_mode; // Off:0, Track:1, Center:2
   uint8_t  lw_index;   // index the level-wind   

   // provisions for future expansions
   enum     reverse {FORWARD, REVERSE} reverse;   // reverse control lever action
   enum     local_remote {LOCAL, REMOTE} local_remote; // local or remote control
   uint8_t  active_drum;  //  1-7
   uint8_t  operating_drums;      // active drums, bit-mapped
   

   /* output signals   */
   // state leds
   uint8_t  safe_led;
   uint8_t  prep_led;
   uint8_t  armed_led;
   uint8_t  grndrllrtn_led;
   uint8_t  ramp_led;
   uint8_t  climb_led;
   uint8_t  recovery_led;
   uint8_t  retrieve_led;
   uint8_t  abort_led;
   uint8_t  stop_led;

   // lighted push button leds
   uint8_t  arm_pb_led;
   uint8_t  prep_recovery_led;

   uint8_t  beeper;  
};

#endif



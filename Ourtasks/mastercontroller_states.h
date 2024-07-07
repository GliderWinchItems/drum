/******************************************************************************
* File Name          : mastercontroller_states.h
* Date First Issued  : 10/11/2020
* Description        : Defines (super-) states for Master Controller
*******************************************************************************/

#ifndef __MASTERCONTROLLERSTATES
#define __MASTERCONTROLLERSTATES

#include <stdint.h>

// Master Controller state machine  definitions 
// Lower nibble reserved for sub-states if needed
#define MC_SAFE      (0 << 4)
#define MC_PREP      (1 << 4)
#define MC_ARMED     (2 << 4)
#define MC_GRNDRTN   (3 << 4)
#define MC_RAMP      (4 << 4)
#define MC_CLIMB     (5 << 4)
#define MC_RECOVERY  (6 << 4)
#define MC_RETRIEVE  (7 << 4)
#define MC_ABORT     (8 << 4)
#define MC_STOP      (9 << 4)

#endif
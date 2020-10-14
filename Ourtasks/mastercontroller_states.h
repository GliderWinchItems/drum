/******************************************************************************
* File Name          : MasterControllerState.h
* Date First Issued  : 10/113/2020
* Description        : Defines (super-) states for Master Controller
*******************************************************************************/

#ifndef __MASTERCONTROLLERSTATES
#define __MASTERCONTROLLERSTATES

#include <stdint.h>

// Master Controller state machine  definitions 
// Lower nibble reserved for sub-states
#define MC_SAFE      (0 << 4)
#define MC_PREP      (1 << 4)
#define MC_ARMED     (2 << 4)
#define MC_GRNDRTN   (3 << 4)
#define MC_RAMP      (4 << 4)
#define MC_CLIMB     (5 << 4)
#define MC_RECOVERY  (6 << 4)
#define MC_RETRIEV   (7 << 4)
#define MC_ABORTED   (8 << 4)
#define MC_STOPPED   (9 << 4)

#endif
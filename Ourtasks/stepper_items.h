/******************************************************************************
* File Name          : stepper_items.h
* Date First Issued  : 07/31/2020
* Description        : Stepper motor associated items
*******************************************************************************/
/*
08/29/2020 fauxencoder
*/


#ifndef __STEPPER_ITEMS
#define __STEPPER_ITEMS

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "stepper_items.h"
#include "common_can.h"
#include "CanTask.h"

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

/* CAN msg: cid_drum_tst_stepcmd: payload[0] bit definitions. */
#define DRBIT 0x1  // (1) Bit mask Direction output pin: 0 = low; 1 = high
#define ENBIT 0x2  // (2) Bit mask Enable output pin: 0 = low; 1 = high
#define LMBIT 0x4  // Bit mask Limit switch simulation
#define IXBIT 0x8  // Bit mask Indexing command
/*
(1) CP pushbutton: Retrieve
(2) 
*/


#define NUMCANMSGSSTEPPER 1  // Number of CAN msgs stepper sends


/* Parameters stepper instance (LC = Local Copy) */
struct STEPPERLC
{
	float	 clfactor;	 // Constant to compute oc duration at CL = 100.0
	uint32_t cltimemax;  // Max timer count for shutdown
	int32_t  Lplus;      //
	int32_t  Lminus;     //
	uint32_t hbct;       // Number of ticks between hb msgs
	int32_t  Ka;
	int32_t  Ks;

	// stepper function sends: others receive the following CAN msgs
	uint32_t cid_hb_stepper;        // CANID_HB_STEPPER: U8_U32','STEPPER: U8: Status, U32: stepper position accum
};

union PAYFLT
{
	float f;
	uint8_t u8[4];
	uint16_t u16[4];
	uint32_t u32;
	int32_t  s32;
	int16_t  s16[2];
}pf;

struct STEPPERSTUFF
{
	struct STEPPERLC lc; // Parameters for stepper
	struct CANTXQMSG canmsg[NUMCANMSGSSTEPPER]; // CAN msgs sent
	union  PAYFLT    pf; // For extracting float from payload
	union  PAYFLT    posaccum;
	float    speedcmdf;  // Speed command (float)
	float    focdur;     // Temp for computer inverse of CL position
	float    clpos;      // CL position extracted from CAN msg
	uint32_t dtwentry;   // DTW timer upon ISR entry
	uint32_t dtwdiff;    // DTW timer minus entry upon ISR exit
	uint32_t dtwmax;     // DTW difference max
	uint32_t dtwmin;     // DTW difference min
	uint32_t ledctr1;    // Counter for throttling green LED
	uint32_t ledctr2;    // Counter for throttling orangeLED
	uint32_t ledbit1;    // Bit for toggling green led
	uint32_t ledbit2;    // Bit for toggling orange led
	uint32_t cltimectr;  // Counter for loss of CL msgs
	uint32_t speedcmdi;	 // Commanded speed (integer)
	uint32_t ocinc;      // oc register increment
	uint32_t hbctr;      // Count ticks for sending heartbeat CAN msg
	uint32_t drflag;     // BSRR pin set/reset bit position: direction
	uint32_t enflag;     // BSRR pin set/reset bit position: enable
	uint32_t iobits;     // Bits from CL CAN msg positioned for PB0
	int32_t  velaccum;   // Stepper velocity accumulator
	int32_t  velaccum_prev;  // Previous velaccum
	int16_t  posaccum_prev;  // Previous posaccum
	uint8_t  stepperstatus;  // Reserved for CAN msg
	uint8_t  pay0;       // canmsg.cd.uc[0] saved
	uint8_t  drbit;      // Direction bit (0|1)
	int8_t   drsign;     // Drum direction sign +/- 1

};

/* *************************************************************************/
 void stepper_items_init(void);
/* phtim = pointer to timer handle
 * @brief	: Initialization of channel increment
 * *************************************************************************/
  void stepper_items_clupdate(struct CANRCVBUF* pcan);
/* @param 	: pcan = pointer to CAN msg struct
 * @brief	: Initialization of channel increment
 * *************************************************************************/
  void stepper_items_timeout(void);
/* @brief	: Check for loss of CL CAN msgs
 * *************************************************************************/
  void stepper_items_CANsend(void);
/* @brief   : Send CAN msgs for stepper
 * *************************************************************************/

 extern struct STEPPERSTUFF stepperstuff;

#endif

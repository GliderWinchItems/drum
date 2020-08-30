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

#define DRBIT 0x1  // Bit mask for Direction output pin: 0 = low; 1 = high
#define ENBIT 0x2  // Bit mask for Enable output pin: 0 = low; 1 = high
#define LMBIT 0x4  // Bit mask for Limit switch simulation

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

	// drum sends: others receive
	uint32_t cid_hb_stepper;        // CANID_HB_STEPPER: U8_U32','STEPPER: U8: Status, U32: stepper position accum
};

	union PAYFLT
	{
		float f;
		uint8_t u8[4];
		uint32_t ui;
	}pf;

struct STEPPERSTUFF
{
	struct STEPPERLC lc; // Parameters for stepper
	struct CANTXQMSG canmsg[NUMCANMSGSSTEPPER]; // CAN msgs sent
	union   PAYFLT   pf; // For extracting float from payload
	float    speedcmdf;  // Speed command (float)
	float    focdur;     // Temp for computer inverse of CL position
	float    clpos;      // CL position extracted from CAN msg
	uint32_t ledctr1;    // Counter for throttling green LED
	uint32_t ledctr2;    // Counter for throttling orangeLED
	uint32_t ledbit1;    // Bit for toggling green led
	uint32_t ledbit2;    // Bit for toggling orange led
	uint32_t cltimectr;  // Counter for loss of CL msgs
	uint32_t speedcmdi;	 // Commanded speed (integer)
	uint32_t ocinc;      // Faux encoder: oc register increment
	uint32_t ocnxt;      // Faux encoder: next oc increment
	uint32_t ocrev;      // Increment for stepper reversal
	uint32_t speedinc;   // Low 16b of position accumulator
	uint32_t hbctr;      // Count ticks for sending heartbeat CAN msg
	uint32_t drflag;     // BSRR pin set/reset bit position
	uint32_t enflag;     // BSRR pin set/reset bit position
	uint32_t iobits;     // Bits from CL CAN msg positioned for PB0
	int32_t  accumpos;   // Position accumulator in upper 16b
	int32_t  accumpos_prev; // Previous accumpos (hi-ord 16b)
	uint8_t  zerohold;   // Special case of CL = 0.0;
	uint8_t  stepperstatus;
	uint8_t  pay0;       // canmsg.cd.uc[0] saved
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

/******************************************************************************
* File Name          : odometer_times.c
* Date First Issued  : 12/12/2024
* Description        : Odometer function w STM32CubeMX w FreeRTOS
*******************************************************************************/

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "malloc.h"

#include "main.h"
#include "morse.h"

#include "drum_items.h"
#include "OdometerTask.h"
#include "MailboxTask.h"
#include "controlpanel_items.h"
#include "OdometerTask.h"
#include "odometer_items.h"
#include "levelwind_items.h"
#include "tim2tim5common_init.h"

#include "DTW_counter.h"
/*
Encoder transitions step the encoder counter (TIM 5) + or -,
and also trigger Input Capture (TIM 2).

Motor encoder:
TIM5 32b encoder counter (no interrupt)
   CH3 PA0 encoder config: encoder A (TIM2 PA2)
   CH4 PA1 encoder config: encoder B (TIM2 PA3)

Encoder timer: TIM2CH1. Pullup resistors
PA0 - Encoder channel A connect to PA2
PA1 - Encoder channel B connect to PA3

TIM2 32b (84 MHz) capture mode (interrupt)
   CH3 PA2 input capture: encoder A (connects to TIM5 PA0)
   CH4 PA3 input capture: encoder B (connects to TIM5 PA1)
   CH2 PB3 input capture: encoder Z
   CH1 --- output capture: measurement duration
*/

/* From 'main.c' */
extern struct CAN_CTLBLOCK* pctl0;	// Pointer to CAN1 control block
extern CAN_HandleTypeDef hcan1; //

extern TIM_HandleTypeDef htim4; // Timer FreeRTOS handle

// Use these locally. levelwind and switches have their own
TIM_TypeDef  *pT4base; // Register base address 

/* *************************************************************************
 * void odometer_items_init(struct ODOMETERFUNCTION* p);
 * @brief	: Init for Odometer Tast
 * *************************************************************************/
void odometer_items_init(struct ODOMETERFUNCTION* p)
{
   float tmpf;

   /* Line out--### make conditional based on saved sram regs!!!! */
   // Zero line_out
   p->line_out = 0;
   p->line_out_ctr = 0; // encoder counter
   // Initial circumference of loaded rope (meters)
   tmpf = (p->lc.drum_outer_dia - (p->lc.rim_to_rope_default * 0.002));
   if (tmpf < 0) morse_trap(732);
   // Diameter to circumference
   p->initial_circum *= 3.14159265f;
   // Number of revs per layer
   p->drum_rev_per_layer = p->lc.drum_width / (p->lc.rope_dia * 0.001f);
   // Average layer thickness per rev
   p->drum_dia_change_per_rev = (p->lc.rope_dia * 0.001f) / p->drum_rev_per_layer;
   p->drum_cir_change_per_rev = p->drum_dia_change_per_rev * 3.14159265f;
   p->en_drum_ratio  = (1/(p->lc.encoder_ratio * 360.0f)); // Line out counting
   p->line_out       = 0;
   p->drum_rev_ctr   = 0;
   p->working_circum = 0;
 
   p->line_out       = 0;
   p->drum_rev_ctr   = 0;

    /* Heartbeat counter. */
    p->hboct = (p->lc.hb_t*64)/1000; // ms to hb ticks
    if (p->hboct == 0) morse_trap(731);
//p->hboct = 64*4;

    p->zspdct = (p->lc.zspd_t*64)/1000; // ms to 1/64s cts
    if (p->zspdct == 0) morse_trap(732);
//p->zspdct = 64/2;

    p->hboctr   = 1; // First HB soon
    p->zspdctr = 0; // Zero speed detection
    p->hbstate = 1;

  /* Pre-load fixed data in CAN msgs we send */
   int i;
   for (i = 0; i < NUMCANMSGSODOMETER; i++)
   {
      p->canmsg[i].pctl = pctl0;   // Control block for CAN module (CAN 1)
      p->canmsg[i].maxretryct = 8; //
      p->canmsg[i].bits = 0;       //
      p->canmsg[i].can.dlc = 8;    // Default payload size (might be modified when loaded and sent)
   }
   // Set CAN ids for msgs we send
   p->canmsg[ODOCANMSG_UNIT].can.id = p->lc.cid_unit_encoder;
   p->canmsg[ODOCANMSG_MSG1].can.id = p->lc.cid_msg1_encoder;    
   p->canmsg[ODOCANMSG_MSG2].can.id = p->lc.cid_msg2_encoder;    
   p->canmsg[ODOCANMSG_MSG3].can.id = p->lc.cid_msg3_encoder;   

   /* Intermediate ISR that is within FreeRTOS NVIC range.
      is triggered by TIM2 OC which is at top priority. */
    HAL_NVIC_SetPriority(I2C3_ER_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(I2C3_ER_IRQn);

   /* Initialize TIM2 & TIM5 for odometer, levelwind, levelwind_switches. */
 //$  tim2tim5common_init();

   /* Save base addresses of timers for faster use later. */
   pT4base  = htim4.Instance;

   /* Start 1/64th sec timer. */
   pT4base->CCR1  = pT4base->CNT + ODOMETER_T4C1_DUR; // 1/64 sec
   pT4base->DIER |= (1<<1); // CH1 interrupt enable
   pT4base->CR1  |= 1;  // Start counter

   return;
}
/* *************************************************************************
 * static void send_msg1(struct ODOMETERFUNCTION* p);
 * @brief   : Send CAN msg: drum speed, lineout (FF_FF)
 * *************************************************************************/
/*
   uint32_t cid_msg1_encoder; // CANID_MSG1_ENCODER1 83A00000 | CANID_MSG1_ENCODER2 83C00000 FF_FF lineout, drum speed
   uint32_t cid_msg2_encoder; // CANID_MSG2_ENCODER1 83E00000 | CANID_MSG1_ENCODER2 84000000 FF_FF accel, encoder speed
   uint32_t cid_msg3_encoder; // CANID_MSG3_ENCODER1 84200000 | CANID_MSG1_ENCODER2 84400000 FF_S32 drum speed, encoder counter
*/

static void send_msg1(struct ODOMETERFUNCTION* p)
{
   struct CANRCVBUF* pcan = &p->canmsg[ODOCANMSG_MSG1].can;
   union UF
   {
      float f;
      uint32_t ui;
   }uf;

   uf.f = p->line_out;
   pcan->cd.ui[0] = uf.ui;

// Revert drum speed (computed on edges) to encoder speed   
   uf.f = p->odo_speed_ave_drum * p->encoder_ratio;

//   uf.f = p->odo_speed_ave_drum;
//uf.f = p->working_circum; // Test/debugg
   pcan->cd.ui[1] = uf.ui;

   // Place CAN msg on CanTask queue
   xQueueSendToBack(CanTxQHandle,&p->canmsg[ODOCANMSG_MSG1],4);
   return;
}
/* *************************************************************************
 * static void send_msg2(struct ODOMETERFUNCTION* p);
 * @brief   : Send CAN msg: acceleration, speed ave drum (FF_FF)
 * *************************************************************************/
static void send_msg2(struct ODOMETERFUNCTION* p)
{
   struct CANRCVBUF* pcan = &p->canmsg[ODOCANMSG_MSG2].can;
   union UF
   {
      float f;
      uint32_t ui;
   }uf;

   uf.f = p->accel_ave_motor; 
   pcan->cd.ui[0] = uf.ui;

   uf.f = p->odo_speed_ave_drum;
   pcan->cd.ui[1] = uf.ui;   

   // Place CAN msg on CanTask queue
   xQueueSendToBack(CanTxQHandle,&p->canmsg[ODOCANMSG_MSG2],4);
   return;
}
/* *************************************************************************
 * static void send_msg3(struct ODOMETERFUNCTION* p);
 * @brief   : Send CAN msg: acceleration, encoder ctr (FF_S32)
 * *************************************************************************/
static void send_msg3(struct ODOMETERFUNCTION* p)
{
   struct CANRCVBUF* pcan = &p->canmsg[ODOCANMSG_MSG3].can;
   union UF
   {
      float f;
      uint32_t ui;
   }uf;

   uf.f = p->en_cnt_speed;   //p->accel_ave_motor; 
   pcan->cd.ui[0] = uf.ui;

   pcan->cd.ui[1] = p->en_cnt; // encoder counter +/-

   // Place CAN msg on CanTask queue
   xQueueSendToBack(CanTxQHandle,&p->canmsg[ODOCANMSG_MSG3],4);
   return;
}
/* *************************************************************************
 * void odometer_items_compute(void);
 * @brief   : Do speed and acceleration computations on new reading set
 * *************************************************************************/
void odometer_items_compute(void)
{
   struct ODOMETERFUNCTION* p = &odometerfunction; // Convenience pointer
   int i;
   /* Average speed and accel for each encoder reading pair. */
   p->speed_sum = 0;
   p->accel_sum = 0;

   /* Speed computed on like edges of like encoder channels. */
   for (i = 0; i < 4; i++)
   { // Check for divide by zero JIC!
      if (p->odotimct_int_diff[i].tim == 0)
      { // No encoder interrupts during measurement interval
         p->speed[i] = 0;
      }
      else
      {
         p->speed[i] = (float)p->odotimct_int_diff[i].ct/(float)p->odotimct_int_diff[i].tim;
      }
      p->accel_sum      += p->speed[i] - p->speed_prev[i];
      p->speed_sum      += p->speed[i];
      p->speed_prev[i]   = p->speed[i];
      p->accel_ave_motor = p->accel_sum * p->lc.scale_speediff_accel;
   }

   /* Speed and accel computed on encoder counts during 1/64th sec TIM2 OC. */
   p->en_cnt_speed = (float)p->en_cnt_diff * ((float)(60*64)/1440); // RPM
   p->en_cnt_speed_diff  = p->en_cnt_speed - p->en_cnt_speed_prev;
   // scale speed rpm differences to radians/sec^2 
   p->en_cnt_accel_motor = p->en_cnt_speed_diff* ((float)(3.141592654*2*64)/60);
   p->en_cnt_speed_prev  = p->en_cnt_speed; // Save for next cycle

 return;
}
/* *************************************************************************
 * void odometer_items_send_speed_lineout_msg(struct ODOMETERFUNCTION* p);
 * @brief   : Send speed & line-out CAN msg timing
 * *************************************************************************/
extern osThreadId defaultTaskHandle;
#define DEFAULTTSKBIT01 (1 << 1)  // Task notification bit for main

void odometer_items_send_speed_lineout_msg(struct ODOMETERFUNCTION* p)
{
   /* Compute and scale */
   p->odo_speed_ave_motor = p->speed_sum * p->lc.scale_en_mtr; // Scale raw->motor rpm
   p->odo_speed_ave_drum  = p->odo_speed_ave_motor * p->lc.scale_mtr_drum;  // Scale motor rpm->drum rpm

   float ftmp = (p->odotimct_int_diff[0].ct * p->en_drum_ratio); 
   // Accumulate line-out (+/-)
   p->line_out += (ftmp * p->working_circum);
   // Adjust working circumference
   p->working_circum -= ((float)p->odotimct_int_diff[0].ct * p->drum_cir_change_per_encoder_ct);
   /* Set up payload and send, if msg enabled. */
   if (p->lc.msg_enable[0] != 0)
      send_msg1(p);
   if (p->lc.msg_enable[1] != 0)
      send_msg2(p);
   if (p->lc.msg_enable[2] != 0)
      send_msg3(p);

   xTaskNotify(defaultTaskHandle, 
         DEFAULTTSKBIT01,  /* 'or' bit assigned to buffer to notification value. */
         eSetBits  
          ); 
   return;
}
/* *************************************************************************
 * void odometer_items_hearbeat(void);
 * @brief   : Send speed & line-out CAN msg timing
 * *************************************************************************/
void odometer_items_hearbeat(void)
{
   /* Whenever the speed is not zero the speed & line-out a CAN msg is sent every
duration measurement (determined by ODOMETER_T2C1_DUR).
      When the speed is essentially zero, the msgs are sent every duration
meausrement until the hboct duration expires, after which the msgs are sent
at the heatbeat duration.
   This avoids flooding the logs with speed & line-out CAN msgs when the 
drum is sitting idle.
   Since there could be some encoding jitter with an idle drum, SDEL
provides a +/- window around zero, for detecting idle drum and 
switching to the slow heartbeat rate.   
   */   
   struct ODOMETERFUNCTION* p = &odometerfunction; // Convenience pointer

   /* Check if sufficiently slow to switch to slow CAN msg sending. */
   if ((p->speed_sum < -SDEL) || (p->speed_sum > SDEL)) 
   { // Not zero speed detected. (Send msgs at 64/sec rate.)
      odometer_items_send_speed_lineout_msg(p);
      p->hbstate = 0;
      p->zspdctr  = p->zspdct; // Reset HB duration count
      return;
   }

    switch(p->hbstate)
    {
    case 0:
      p->zspdctr -= 1;
      if (p->zspdctr != 0)
      {
         odometer_items_send_speed_lineout_msg(p);
         return;
      }
      p->hbstate = 1;
      p->hboctr = p->hboct; // Reset HB duration counter
      break;

   case 1: 
      p->hboctr -= 1;
      if (p->hboctr == 0)
      {
         odometer_items_send_speed_lineout_msg(p);
         p->hboctr = p->hboct; // Reset HB duration counter 2
      }
      break;
    }
    return;
}
/*#######################################################################################
 * void odometer_items_TIM4_IRQHandler(void);
 * ISR routine for TIM4
 * CH1 - OC measurement duration for encoder measurement
 *####################################################################################### */
uint32_t dbgtim4ctr;
void odometer_items_TIM4_IRQHandler(void)
{   
   struct ODOMETERFUNCTION* p = &odometerfunction; // Convenience pointer

     // TIM4CH1 OC: End of measurement duration (1/64 sec)
   if ((pT4base->SR & (1 << 1)) != 0) // CH1 Interrupt flag
   { // Yes, OC drive 
      pT4base->SR = ~(1 << 1);  // Reset CH1 flag      

      // Duration increment for next interrupt
      pT4base->CCR1 += ODOMETER_T4C1_DUR; // Next measurement

      /* Save encoder counter at 1/64th sec time tick. */
      p->en_cnt = pT5base->CNT;

      // Struct copy readings to buffer for computation
      p->odotimct_buff[0] = p->odotimct[0];
      p->odotimct_buff[1] = p->odotimct[1];
      p->odotimct_buff[2] = p->odotimct[2];
      p->odotimct_buff[3] = p->odotimct[3];

// Cause interrupt for lower FreeRTOS ISR level handling
      HAL_NVIC_SetPendingIRQ(I2C3_ER_IRQn);

   }      
      
      return;
}
/*#######################################################################################
 * void odometer_items_TIM2_IRQHandler(void);
 * ISR routine for TIM2
 * CH1 - OC USED in LEVELWIND
 * CH2 - IC encoder channel Z reference (reserved: not implmented)
 * CH3 - IC encoder channel A 
 * CH4 - IC encoder channel B 
 *####################################################################################### */
void odometer_items_TIM2_IRQHandler(void)
{
   struct ODOMETERFUNCTION* p = &odometerfunction; // Convenience pointer

   /* TIM2CH3 = encodertimeA transition PA2 TIM5CH1 PA0  */
   if ((pT2base->SR & (1<<3)) != 0)  // CH3 Interrupt flag?
   { // Yes, encoder channel A
//      pT2base->SR = ~(1<<3);   // Reset CH3 flag

  	  p->oe_A ^= 0x1; // Odd/even toggle
     if (p->oe_A == 0)
     {         
        p->odotimct[0].tim  = pT2base->CCR3; // IC time edges
        p->odotimct[0].ct   = pT5base->CCR1; // Encoder counter chan A
     }
	  else
     {
        p->odotimct[1].tim  = pT2base->CCR3; // IC time edges
        p->odotimct[1].ct   = pT5base->CCR1; // Encoder counter chan A
     }
   }

   /* TIM2CH4 IC: encodertimeB transition PA3 TIM5CH1 PA1  */
   if ((pT2base->SR & (1<<4)) != 0)  // CH4 Interrupt flag?
   { // Yes, encoder channel B
      pT2base->SR = ~(1<<4);   // Reset CH4 flag

      p->oe_B ^= 0x1;
      if (p->oe_B == 0)
     {         
        p->odotimct[2].tim  = pT2base->CCR4; // IC time edges
        p->odotimct[2].ct   = pT5base->CCR2; // Encoder counter chan B
     }
     else
     {
        p->odotimct[3].tim  = pT2base->CCR4; // IC time edges
        p->odotimct[3].ct   = pT5base->CCR2; // Encoder counter chan B
     }
   }

   /* Levelwind will use TIM2 interrupts (and reset TIM2 flags). */
   levelwind_items_TIM2_IRQHandler();



   return;
}
/*#######################################################################################
 * void I2C3_ER_IRQHandler(void);
 * ISR routine for I2C3_ER
 * Intermediate ISR triggered by TIM2 ISR OC (1/64) sec interrupts, for notifying OdometerTask
 *####################################################################################### */
void I2C3_ER_IRQHandler(void)
{
   /* 
   This interrupt was triggered by TIM1CH1 OC, which marks the end of a measurement
interval. 
   This interrupt level is within the FreeRTOS range and can notify a FreeRTOS
task.
   Do the differences and save them in the 'buff. It is expected that the FreeRTOS 
task (OdometerTask) will use odotimct_buff before the next measurement interval (1/64 sec).
   */
   struct ODOMETERFUNCTION* p = &odometerfunction; // Convenience pointer

   /* Time duration and count differences from pervious interval. */

   // This could be a loop, but we will do it inline and save under-interrupt-cycles.
   p->odotimct_int_diff[0].tim = (int)(p->odotimct_buff[0].tim - p->odotimct_buff_prev[0].tim);
   p->odotimct_int_diff[0].ct  = (int)(p->odotimct_buff[0].ct  - p->odotimct_buff_prev[0].ct );
   p->odotimct_buff_prev[0]  = p->odotimct_buff[0]; // Update struct for next cycle

   p->odotimct_int_diff[1].tim = (int)(p->odotimct_buff[1].tim - p->odotimct_buff_prev[1].tim);
   p->odotimct_int_diff[1].ct  = (int)(p->odotimct_buff[1].ct  - p->odotimct_buff_prev[1].ct );
   p->odotimct_buff_prev[1]  = p->odotimct_buff[1];

   p->odotimct_int_diff[2].tim = (int)(p->odotimct_buff[2].tim - p->odotimct_buff_prev[2].tim);
   p->odotimct_int_diff[2].ct  = (int)(p->odotimct_buff[2].ct  - p->odotimct_buff_prev[2].ct );
   p->odotimct_buff_prev[2]  = p->odotimct_buff[2];

   p->odotimct_int_diff[3].tim = (int)(p->odotimct_buff[3].tim - p->odotimct_buff_prev[3].tim);
   p->odotimct_int_diff[3].ct  = (int)(p->odotimct_buff[3].ct  - p->odotimct_buff_prev[3].ct );
   p->odotimct_buff_prev[3]  = p->odotimct_buff[3];

   p->en_cnt_diff = p->en_cnt - p->en_cnt_prev;
   p->en_cnt_prev = p->en_cnt;

   /* Notify OdometerTask a new set of readings is ready. (End of 1/64 sec intervals.) */
   BaseType_t xHigherPriorityTaskWoken = pdFALSE;
   xTaskNotifyFromISR(OdometerTaskHandle,\
         ODOMETERNOTBITTIM2, eSetBits,\
         &xHigherPriorityTaskWoken );

// Debugging
#if 0
   p->ledctr1 += 1;
   if (p->ledctr1 >= 64)        
   {
      p->ledctr1 = 0;
      HAL_GPIO_TogglePin(GPIOD,LED_GREEN_Pin); // GREEN LED       
   }
#endif   
   return;
}
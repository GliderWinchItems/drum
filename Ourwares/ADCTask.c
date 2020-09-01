/******************************************************************************
* File Name          : ADCTask.c
* Date First Issued  : 02/01/2019
* Description        : Processing ADC readings after ADC/DMA issues interrupt
*******************************************************************************/

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "malloc.h"

#include "ADCTask.h"
#include "adctask.h"
#include "morse.h"
#include "adcfastsum16.h"
#include "adcextendsum.h"

#include "adcparams.h"
//#include "calib_control_lever.h"
#include "main.h"

extern ADC_HandleTypeDef hadc1;
extern osThreadId GevcuTaskHandle;

void StartADCTask(void const * argument);

uint32_t adcsumdb[6]; // debug
uint32_t adcdbctr = 0;// debug

osThreadId ADCTaskHandle;

float fclpos;

/* *************************************************************************
 * osThreadId xADCTaskCreate(uint32_t taskpriority);
 * @brief	: Create task; task handle created is global for all to enjoy!
 * @param	: taskpriority = Task priority (just as it says!)
 * @return	: ADCTaskHandle
 * *************************************************************************/
osThreadId xADCTaskCreate(uint32_t taskpriority)
{
 	osThreadDef(ADCTask, StartADCTask, osPriorityNormal, 0, 256+64);
	ADCTaskHandle = osThreadCreate(osThread(ADCTask), NULL);
	vTaskPrioritySet( ADCTaskHandle, taskpriority );
	return ADCTaskHandle;

}
/* *************************************************************************
 * void StartADCTask(void const * argument);
 *	@brief	: Task startup
 * *************************************************************************/
void StartADCTask(void const * argument)
{
	#define TSK02BIT02	(1 << 0)  // Task notification bit for ADC dma 1st 1/2 (adctask.c)
	#define TSK02BIT03	(1 << 1)  // Task notification bit for ADC dma end (adctask.c)

	uint16_t* pdma;

	/* A notification copies the internal notification word to this. */
	uint32_t noteval = 0;    // Receives notification word upon an API notify

	/* notification bits processed after a 'Wait. */
	uint32_t noteused = 0;

	/* Get buffers, "our" control block, and start ADC/DMA running. */
	struct ADCDMATSKBLK* pblk = adctask_init(&hadc1,TSK02BIT02,TSK02BIT03,&noteval);
	if (pblk == NULL) {morse_trap(15);}

  /* Infinite loop */
  for(;;)
  {
		/* Wait for DMA interrupt */
		xTaskNotifyWait(noteused, 0, &noteval, portMAX_DELAY);
		noteused = 0;	// Accumulate bits in 'noteval' processed.

		/* We handled one, or both, noteval bits */
		noteused |= (pblk->notebit1 | pblk->notebit2);

		if (noteval & TSK02BIT02)
		{
			pdma = adc1dmatskblk[0].pdma1;
		}
		else
		{
			pdma = adc1dmatskblk[0].pdma2;
		}

		xTaskNotify(GevcuTaskHandle, GEVCUBIT00, eSetBits);


		/* Sum the readings 1/2 of DMA buffer to an array. */
		adcfastsum16(&adc1.chan[0], pdma); // Fast in-line addition
		adc1.ctr += 1; // Update count

#define DEBUGGINGADCREADINGS
#ifdef DEBUGGINGADCREADINGS
		/* Save sum for defaultTask printout for debugging */
		adcsumdb[0] = adc1.chan[0].sum;
		adcsumdb[1] = adc1.chan[1].sum;
		adcsumdb[2] = adc1.chan[2].sum;
		adcsumdb[3] = adc1.chan[3].sum;
		adcsumdb[4] = adc1.chan[4].sum;
		adcsumdb[5] = adc1.chan[5].sum;
		adcdbctr += 1;
#endif

		/* Extended sum for smoothing and display. */
		adcextendsum(&adc1);

		/* Calibrate and filter ADC readings. */
		adcparams_cal();

		/* Notify GEVCUr Task that new readings are ready. */
		if( GevcuTaskHandle == NULL) morse_trap(51); // JIC task has not been created
	
// Do the work in this task and not GevcuTask.	
//		xTaskNotify(GevcuTaskHandle, GEVCUBIT00, eSetBits);

	/* Update Control Lever psosition with new ADC readings (or do initial calib). */
//	fclpos = calib_control_lever();
  }
}


/******************************************************************************
* File Name          : adcparamsinit.h
* Date First Issued  : 10/09/2019
* Board              : DiscoveryF4
* Description        : Initialization of parameters for ADC app configuration
*******************************************************************************/

#ifndef __ADCPARAMSINIT
#define __ADCPARAMSINIT

#include <stdint.h>
#include "adcparams.h"

#define SCALE1 (1 << 16)

/* Factory calibration pointers. */
#define PVREFINT_CAL ((uint16_t*)0x1FFF7A2A)  // Pointer to factory calibration: Vref
#define PTS_CAL1     ((uint16_t*)0x1FFF7A2C)  // Pointer to factory calibration: Vtemp
#define PTS_CAL2     ((uint16_t*)0x1FFF7A2E)  // Pointer to factory calibration: Vtemp

/* Factory Vdd for Vref calibration. */
#define VREFCALVOLT 3300  // Factory cal voltage (mv)
#define VREFCALVOLTF (VREFCALVOLT * 0.001)  // Factory cal voltage, float (volts)

/* *************************************************************************/
void adcparamsinit_init(struct ADCFUNCTION* p);
/*	@brief	: Load structs for compensation, calibration and filtering for ADC channels
 * @param	: p = Points to struct with "everything" for this ADC module
 * *************************************************************************/
void adcparamsinit_init(struct ADCFUNCTION* p);
/*	@brief	: Load structs for compensation, calibration and filtering for ADC channels
 * @param	: p = Points to struct with "everything" for this ADC module
 * *************************************************************************/
int16_t ratiometric_cal_zero(struct ADCFUNCTION* p, int16_t idx);
/*	@brief	: Adjust no-current ratio for a Hall-effect sensor
 * @param	: p = Pointer to struct "everything" for this ADC module
 * @param	: idx = ADC scan sequence index
 * @return	: 0 = no fault; -1 = out of tolerance
 * *************************************************************************/


#endif


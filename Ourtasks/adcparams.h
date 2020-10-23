/******************************************************************************
* File Name          : adcparams.h
* Date First Issued  : 10/09/2019
* Board              : DiscoveryF4
* Description        : Parameters for ADC app configuration
*******************************************************************************/
/* CALIBRATION NOTES:

5 volt supply calibration:
[Do this first: This calibration is applied to all 5v sensors]
- Measure 5v supply
- Measure Vdd
- Display ADCsum[5v supply]
Compute--
Ratio 'sensor5vcal' = (Vdd/V5volt) * (ADCsum[5v supply]/(4095*numdmaseq)
 where numdmaseq = number of ADC scans per 1/2 dma (i.e. number readings in sum)

Ratiometric 5v sensor calibration:
- With 5v supply calibration compiled-in:
- Measure Sensor voltage: Vx
- Display:
  . ADCsum[sensor]
  . Va = (V5 * ratio)/ADCsum[5v]
Compute
  Calibration ratio = ADCsum[sensor]/Va
*/

#ifndef __ADCPARAMS
#define __ADCPARAMS

#include "iir_filter_lx.h"
#include "iir_f1.h"
#include "adc_idx_v_struct.h"

#define ADC1DMANUMSEQ        16 // Number of DMA scan sequences in 1/2 DMA buffer
#define ADCEXTENDSUMCT     1024 // Sum of 1/2 DMA sums for additional averaging
#define ADC1IDX_ADCSCANSIZE   5 // Number ADC channels read
#define ZTOLERANCE         0.05 // +/- tolerance for re-adjustment of Hall_effect sensor zero
#define ADCSCALEbits         15 // 2^x scale large

/* ADC reading sequence/array indices                                           */
/* These indices -=>MUST<= match the hardware ADC scan sequence in STM32CubeMX. */
#define ADC1IDX_STEPPERV      0   // PA4 IN4  - Stepper controller power voltage
#define ADC1IDX_5VSUPPLY      1   // PA7 IN7  - 5V #also CAN driver RS output:pushpull
#define ADC1IDX_SPARE         2   // PC4 IN10? - 5V Spare ADC input
#define ADC1IDX_INTERNALTEMP  3   // IN17     - Internal temperature sensor
#define ADC1IDX_INTERNALVREF  4   // IN18     - Internal voltage reference

/* Calibrated ADC reading. */
union ADCCALREADING
{
	uint32_t ui;
	 int32_t  n;
	float     f;
};

/* This holds calibration values common to all ADC modules. 
     Some of these are not used.
*/
struct ADCCALCOMMON
{

	struct IIRFILTERL iiradcvref; // Intermediate filter params: vref 
	struct IIRFILTERL iiradctemp; // Intermediate filter params: temperature sensor

	// 5v supply
	float f5vsupply;       // 5v supply
	float fvddcomp;      // 5->Vdd adjusted factor
	float fvddrecip;
	float f5_Vddratio;   // (V5volt * Ratio)/ADCsum[5volt supply]
	float f5vsupplyprecal; // 5v supply precalc calibration ratio
	float f5vsupplyfilt;   // 5v supply, filtered
	float sensor5vcalVdd;// The 5v->Vdd divider ratio Vdd adjusted
	float f5vsupplyprecal_offset; // 5v supply precalc calibration ratio

	// Internal voltage reference
	float fvref;         // Vref: 1.18 min, 1.21 typ, 1.24 max
	float fadc;          // Float of struct ADC1CALINTERNAL adcvdd

	float tcoef;         // Vref: Temp coefficient (ppm/deg C: 30 typ; 50 max)
	float fvdd;          // Vdd: float (volts)
	float fvddfilt;      // Vdd: float (volts), filtered
	uint16_t ivdd;       // Vdd: fixed (mv)
	uint16_t ts_vref;
	uint32_t adccmpvref; // scaled vref compensated for temperature

	// Internal temperature sensor (floats)
	float ts_cal1;    // Vtemp: TS_CAL1 converted to float ( 30 deg C 3.3v)
	float ts_cal2;    // Vtemp: TS_CAL2 converted to float (110 deg C 3.3v)
	float ts_caldiff; // CAL2-CAL1
	float ts_80caldiff;
	float v25;           // Vtemp: 25 deg C (0.76v typ)
    float slope;         // Vtemp: mv/degC 
	float offset;        // Vtemp: offset
	float degC;          // Temperature: degrees C
	float degCfilt;      // Temperature: degrees C, filtered
 	uint32_t dmact;      // DMA interrupt running counter

	// For integer computation (much faster)
	uint32_t uicaldiff;
	int64_t ll_80caldiff;
	uint32_t ui_cal1;
	uint32_t ui_tmp;
};

struct ADCVTEMPVREF
{
	struct ADCCALCOMMON acc;
	float	vref;         // Latest value of Vref
	float vtemp;        // Latest value of Vtemp
	float degC;         // Temp sensor converted to degC       
	uint64_t u64_vref;  // Summation accumulator: Vref
	uint64_t u64_vtemp; // Summation accumulator: Vtemp
	uint32_t ct;        // Summation counter

};

/* Working values for absolute voltages adjusted using Vref. */
struct ADCABSOLUTE
{
	struct IIRFILTERL iir;// Intermediate filter params
	float fscale;        // Computed from measurements
	float k;             // divider ratio: (Vref/adcvref)*(adcvx/Vx)
	uint32_t adcfil;      // Filtered ADC reading
	uint32_t ival;        // scaled int computed value (not divider scaled)
};

/* Working values for ratiometric sensors using 5v supply. */
struct ADCRATIOMETRIC
{
	struct IIRFILTERL iir;    // Intermediate filter params
	float frko;      // Offset ratio: float (~0.5)
	float fscale;    // Scale factor
	uint32_t adcfil;  // Filtered ADC reading
	int32_t irko;     // Offset ratio: scale int (~32768)
	int32_t iI;       // integer result w offset, not final scaling
};

struct ADCCHANNEL	
{
	struct FILTERIIRF1 iir_f1;	// iir_f1 (float) filter
	float    fscale;  // Scale factor
	uint32_t xsum[2]; // Extended sum
	uint16_t sum;     // Sum of 1/2 DMA buffer
};

/* struct allows pointer to access raw and calibrated ADC1 data. */
struct ADC1DATA
{
  union ADCCALREADING adc1calreading[ADC1IDX_ADCSCANSIZE]; // Calibrated readings
  union ADCCALREADING adc1calreadingfilt[ADC1IDX_ADCSCANSIZE]; // Calibrated readings: filtered
  uint32_t ctr; // Running count of updates.
  uint16_t adcs1sum[ADC1IDX_ADCSCANSIZE]; // Sum of 1/2 DMA buffer for each channel
};

/* struct allows pointer to access raw and calibrated ADC1 data. */
struct ADCFUNCTION
{
	struct ADCGEVCULC lc;    // Local Copy of parameters
//	struct ADCINTERNAL    intern;// Vref & temperature
	struct ADCCALCOMMON common;
	struct ADCABSOLUTE    abs[ADCNUMABS];      // Absolute readings
	struct ADCRATIOMETRIC ratio[ADCNUMRATIO];  // Ratometric readings
	struct ADCCHANNEL	 chan[ADC1IDX_ADCSCANSIZE]; // ADC sums, calibrated endpt
	uint32_t ctr; // Running count of updates.
	uint32_t idx_xsum;
};

/* *************************************************************************/
void adcparams_init(void);
/*	@brief	: Copy parameters into structs
 * NOTE: => ASSUMES ADC1 ONLY <==
 * *************************************************************************/
void adcparams_internal(struct ADCCALCOMMON* pacom, struct ADCFUNCTION* padc1);
/*	@brief	: Update values used for compensation from Vref and Temperature
 * @param	: pacom = Pointer calibration parameters for Temperature and Vref
 * @param	: padc1 = Pointer to array of ADC reading sums plus other stuff
 * *************************************************************************/
void adcparams_chan(uint8_t adcidx);
/*	@brief	: calibration, compensation, filtering for channels
 * @param	: adcidx = index into ADC1 array
 * *************************************************************************/
void adcparams_cal(void);
/*	@brief	: calibrate and filter ADC readings (from ADCTask.c)
 * *************************************************************************/

/* Raw and calibrated ADC1 readings. */
extern struct ADCFUNCTION adc1data;

/* Calibration values common to all ADC modules. */
extern struct ADCCALCOMMON adcommon;

/* Everything for ADC1. */
extern struct ADCFUNCTION adc1;

/* Map ADC reading sequence index to Calibration type index 
   Given: ADC seq index, lookup calibration type array index. */
extern const int8_t adcmapabs[4];   // Absolute
extern const int8_t adcmapratio[4]; // Ratiometric


#endif

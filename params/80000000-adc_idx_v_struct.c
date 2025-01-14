/******************************************************************************
* File Name          : adc_idx_v_struct.c
* Date First Issued  : 10/09/2019
* Board              :
* Description        : Load sram local copy of parameters
*******************************************************************************/
/* 10/23/2020: Revised for Levelwind */
#include "adc_idx_v_struct.h"

/*
The radian frequency cutoff is Fs/K where Fs is the IIR filter rate. 
Ks is about 500 Hz so the radian bandwidth is 167 Sz or about 26.6 Hz. 
The time constant is the reciprocal of the radian bandwidth so about 6 ms. The 10-90 risetime would then be 13.2 ms. 
Since those outputs are at a 64 Hz rate (15.6 ms period), that seems like a pretty reasonable value for K.

*/

/* **************************************************************************************
 * int adc_idx_v_struct_hardcode_params(struct ADCGEVCULC* p);
 * @brief	: Hard-code load local copy with parameters
 * @return	: 0
 * ************************************************************************************** */
int adc_idx_v_struct_hardcode_params(struct ADCGEVCULC* p)
{
/* Copy for convenience--
 struct ADCGEVCULC
 {
	uint32_t size;			// Number of items in struct
 	uint32_t crc;			// crc-32 placed by loader
	uint32_t version;		// struct version number
	uint32_t hbct;       // heartbeat count (ms)
	struct ADC1CALINTERNAL calintern; // Vref and Temp internal sensors
	struct ADCCALABS cabs[ADCNUMABS];      // Absolute readings
	struct ADCCALHE  cratio[ADCNUMRATIO];  // Ratometric readings
 };
*/
	p->size     = 37; // Number of items in list (TODO update)
	p->crc      = 0;  // TODO
    p->version  = 1;
	p->hbct     = 1000;  // Time (ms) between HB msg

/* Reproduced for convenience 
  Internal sensor calibration. (Only applies to ADC1) 
struct ADC1CALINTERNAL
{
	struct IIR_L_PARAM iiradcvref; // Filter: adc readings: Vref 
	struct IIR_L_PARAM iiradctemp; // Filter: adc readings: temperature
	float frmtemp;    // (float) Room temp for reading (deg C)
	float fvtemp;     // (float) Voltage of temp sensor at rm temperature
	float fvdd;       // (float) measured Vdd (volts)
	float fslope;     // (float) mv/degC temperature sensor slope
	float fvreftmpco; // (float) Vref temperature coefficient (ppm/degC)
	uint32_t adcvdd;   // (ADC reading) for calibrating Vdd (3.3v)
	uint32_t adcrmtmp; // (ADC reading) room temperature temp sensor reading
};
*/
	p->calintern.iiradcvref.k     = 20;    // Filter time constant
	p->calintern.iiradcvref.scale = 64;

	p->calintern.iiradctemp.k     = 100;    // Filter time constant
	p->calintern.iiradctemp.scale = 4;

	// Internal voltage ref: ADC1IDX_INTERNALVREF  5   // IN18     - Internal voltage reference
	p->calintern.fvdd   = 2.936;	// Vdd for following Vref ADC reading
	p->calintern.adcvdd = 27093;  //(16*1495.5) ADC reading (DMA sum) for above Vdd

	// Internal temperature: ADC1IDX_INTERNALTEMP  4   // IN17     - Internal temperature sensor
	p->calintern.adcrmtmp  = 17838; // Room temp ADC (DMA sum) reading
	p->calintern.frmtemp   = 25.0;  // Room temp for ADC reading     
	p->calintern.fslope    = 4.3;   // mv/degC slope of temperature sensor
	p->calintern.fvreftmpco= 15;    // Vref temp coefficient (15 is based on similar parts)
	p->calintern.fvtemp    = 1.40;  // Vtemp voltage at 25 degC

/*  Reproduced for convenience 
struct ADCCALHE
{
	struct IIR_L_PARAM iir; // Filter: Time constant, integer scaling
	float   scale;     // 
	uint32_t zeroadcve; // connected, no current: HE adc reading
	uint32_t zeroadc5;  // connected, no current: 5v adc reading 
	uint32_t caladcve;  // connected, calibrate current: adc reading
	float   fcalcur;   // connected, calibrate current: current
};
*/
	/// PC5 IN15 - 5V ratiometric spare
	p->cratio[ADC1IDX_SPARE].iir.k     = 10;    // Filter time constant
	p->cratio[ADC1IDX_SPARE].iir.scale = 2;     // Filter integer scaling
	p->cratio[ADC1IDX_SPARE].zeroadcve = 27082; // connected, no current: HE adc reading
	p->cratio[ADC1IDX_SPARE].zeroadc5  = 63969; // connected, no current: 5v adc reading 
	p->cratio[ADC1IDX_SPARE].caladcve  = 29880; // connected, cal current: adc reading
	p->cratio[ADC1IDX_SPARE].fcalcur   = 16.03;  // connected, cal current: current * turns

/*  Reproduced for convenience 
struct ADCCALABS
{
	struct IIR_L_PARAM iir; // Filter: Time constant, integer scaling
	uint32_t adcvn;    // (ADC reading) vn 
    float    fvn;      // (float) measured vn (volts)
};
*/
	/// PA4 IN4  - Stepper controller power voltage
// Note: computed calibration: 110K|2.7K resistor divider	
	p->cabs[ADC1IDX_STEPPERV].iir.k     = 5;     // Filter time constant
	p->cabs[ADC1IDX_STEPPERV].iir.scale = 2;     // Filter integer scaling
	p->cabs[ADC1IDX_STEPPERV].adcvn     = 64668; // (ADC reading) at calibration volts
	p->cabs[ADC1IDX_STEPPERV].fvn       = 110.0;  // (float) calibration volts

	// PA7 IN7 - Regulated 5V 
	p->cabs[ADC1IDX_5VSUPPLY].iir.k      = 10;    // Filter time constant
	p->cabs[ADC1IDX_5VSUPPLY].iir.scale  = 2;     // Filter integer scaling
	p->cabs[ADC1IDX_5VSUPPLY].adcvn      = 64480; // (ADC reading) at calibration volts
	p->cabs[ADC1IDX_5VSUPPLY].fvn        = 4.99;  // (float) calibration volts

	// PC4 IN14 - Spare
	p->cabs[ADC1IDX_SPARE].iir.k     = 10;    // Filter time constant
	p->cabs[ADC1IDX_SPARE].iir.scale = 2;     // Filter integer scaling
	p->cabs[ADC1IDX_SPARE].adcvn     = 32000; // (ADC reading) at calibration volts
	p->cabs[ADC1IDX_SPARE].fvn       = 13.68; // (float) calibration volts

	return 0;	
}

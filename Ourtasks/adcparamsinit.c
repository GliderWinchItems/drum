/******************************************************************************
* File Name          : adcparamsinit.c
* Date First Issued  : 10/09/2019
* Board              : DiscoveryF4
* Description        : Initialization of parameters for ADC app configuration
*******************************************************************************/

/* 
This is where hard-coded parameters for the ADC are entered.

Later, this may be replaced with a "copy" of the flat file in high flash, generated
by the java program from the sql database.
*/

#include "adcparamsinit.h"
#include "adcparams.h"
#include "ADCTask.h"
#include "morse.h"

// Define limits for initialization check
#define VREFMIN (1.15)
#define VREFMAX (1.27)

/* *************************************************************************
 * void adcparamsinit_init_common(struct ADCFUNCTION* p); //struct ADCCALCOMMON* padccommon, struct ADCCHANNELSTUFF* pacsx);
 *	@brief	: Initialize struct with parameters common to all ADC for this =>board<=
 * @param	: padccommon = pointer to struct holding parameters
 * @param	: pacsx = Pointer to struct "everything" for this ADC module
 * *************************************************************************/
static void adcparamsinit_init_common(struct ADCFUNCTION* p)
{
	/* Convenience pointers. */
	struct ADCCALCOMMON* padccommon = &p->common;

	/* Pointers to fixed pt iir filter contstants. */
	padccommon->iiradcvref.pprm = &p->lc.calintern.iiradcvref;
	padccommon->iiradctemp.pprm = &p->lc.calintern.iiradctemp;

	/* Reassign float pt iir filter constants. */
	p->chan[ADC1IDX_INTERNALTEMP].iir_f1.skipctr  = 8;    // Initial readings skip count
	p->chan[ADC1IDX_INTERNALTEMP].iir_f1.coef     = 0.99;  // Filter coefficient (< 1.0)
	p->chan[ADC1IDX_INTERNALTEMP].iir_f1.onemcoef = (1 - p->chan[ADC1IDX_INTERNALTEMP].iir_f1.coef); // Pre-computed

	p->chan[ADC1IDX_INTERNALVREF].iir_f1.skipctr  = 8;    // Initial readings skip count
	p->chan[ADC1IDX_INTERNALVREF].iir_f1.coef     = 0.99;  // Filter coefficient (< 1.0)
	p->chan[ADC1IDX_INTERNALVREF].iir_f1.onemcoef = (1 - p->chan[ADC1IDX_INTERNALVREF].iir_f1.coef); // Pre-computed

	padccommon->ts_vref      = *PVREFINT_CAL; // Factory calibration
	padccommon->tcoef        = 30E-6; // 30 typ, 50 max, (ppm/deg C)

	padccommon->ts_cal1      = (float)(*PTS_CAL1) * (float)ADC1DMANUMSEQ; // Factory calibration
	padccommon->ts_cal2      = *PTS_CAL2; // Factory calibration
	padccommon->ts_caldiff   = *PTS_CAL2 - *PTS_CAL1; // Pre-compute
	padccommon->ts_80caldiff = (80.0 / (padccommon->ts_caldiff *(float)ADC1DMANUMSEQ)); // Pre-compute

	padccommon->uicaldiff    = *PTS_CAL2 - *PTS_CAL1; // Pre-compute
	padccommon->ll_80caldiff = (80 * SCALE1) /(padccommon->uicaldiff);
	padccommon->ui_cal1      =	(*PTS_CAL1) * ADC1DMANUMSEQ;

	/* Data sheet gave these values.  May not need them. */
	padccommon->v25     = 0.76; // Voltage at 25 °C, typ
	padccommon->slope   = 2.0;  // Average slope (mv/deg C), typ
	padccommon->offset  = 25.0;

	// Compute vref from measurements
	p->common.fadc  = p->lc.calintern.adcvdd; // ADC reading (~27360)
	p->common.fvref = p->lc.calintern.fvdd * (p->common.fadc / 65520.0);

	// Check for out-of-datasheet Vref spec 
	if ((p->common.fvref < (VREFMIN)) || (p->common.fvref > (VREFMAX))) 
	{
		morse_trap(81);
	}

	return;
}
/* *************************************************************************
 * static void abs_init(struct ADCFUNCTION* p, int8_t idx1);
 *	@brief	: Load structs for compensation, calibration and filtering for ADC channels
 * @param	: p = Points to struct with "everything" for this ADC module
 * @param	: idx1 = index in ADC sequence array
 * *************************************************************************/
static void abs_init(struct ADCFUNCTION* p, int8_t idx1)
{
/* Reproduced for convenience
struct ADCABSOLUTE
{
	struct IIRFILTERL iir;// Intermediate filter params
	float fscale;        // Computed from measurements
	float k;             // divider ratio: (Vref/adcvref)*(adcvx/Vx)
	uint32_t adcfil;      // Filtered ADC reading
	uint32_t ival;        // scaled int computed value (not divider scaled)
}; */	

	/* Lookup index in absolute array, given ADC sequence index. */
	int8_t idx2 = adcmapabs[idx1];
	if (idx2 < 0) morse_trap(60);	// Illegal index: Coding error

	struct ADCABSOLUTE*    pabs = &p->abs[idx2]; // Working param
	struct ADCCALABS* plc  = &p->lc.cabs[idx2]; // Calibration param
	
	pabs->iir.pprm = &plc->iir; // Filter param pointer
	pabs->k   = (plc->fvn / p->common.fvref) * (p->common.fadc / plc->adcvn);
	pabs->fscale = pabs->k * p->common.fvref;
	p->chan[idx1].fscale = pabs->k; // Save in channel array
	return;
}
/* *************************************************************************
static void ratiometric_cal(struct ADCRATIOMETRIC* p, struct ADCCALHE* plc);
 *	@brief	: Compute calibration constants for ratiometric sensor
 * @param	: p = points to struct with computed results
 * @param	: plc = points to parameter struct for this sensor
 * *************************************************************************/
//static void ratio_init(struct ADCFUNCTION* p, int16_t idx1)
void ratio_init(struct ADCFUNCTION* p, int16_t idx1)
{
/* Reproduced for convenience
struct ADCRATIOMETRIC
{
	struct IIRFILTERL iir;    // Intermediate filter params
	float frko;      // Offset ratio: float (~0.5)
	float fscale;    // Scale factor
	uint32_t adcfil;  // Filtered ADC reading
	int32_t irko;     // Offset ratio: scale int (~32768)
	int32_t iI;       // integer result w offset, not final scaling
};  */

	/* Lookup index in absolute array, given ADC sequence index. */
	int8_t idx2 = adcmapratio[idx1];
	if (idx2 < 0) morse_trap(61);	// Illegal index: Coding error

	struct ADCCALHE*      plc = &p->lc.cratio[idx2]; // Working param
	struct ADCRATIOMETRIC* pr = &p->ratio[idx2]; // Calibration param

	pr->iir.pprm = &plc->iir; // Filter param pointer
	
	// Sensor connected, no current -> offset ratio (~ 0.50)
	pr->frko  = ((float)plc->zeroadcve / (float)plc->zeroadc5) ;
	pr->irko  = (pr->frko * (1 << ADCSCALEbits) );

	// Sensor connected, test current applied with maybe more than one turn through sensor
	// fscale = amp-turns / ((calibration ADC ratio - offset ratio) * divider ratio);
	float ftmp = ( (float)plc->caladcve / (float)plc->zeroadc5 ) - pr->frko ;
	pr->fscale = plc->fcalcur / ftmp;
	p->chan[idx1].fscale = pr->fscale; // Save for convenient access

	return;
}
/* *************************************************************************
 * void adcparamsinit_init(struct ADCFUNCTION* p);
 *	@brief	: Load structs for compensation, calibration and filtering for ADC channels
 * @param	: p = Points to struct with "everything" for this ADC module
 * *************************************************************************/
void adcparamsinit_init(struct ADCFUNCTION* p)
{
	/* Initialize floating pt iir values for all. (JIC) */
	int i;
	for (i = 0; i < ADC1IDX_ADCSCANSIZE; i++)
	{ // Initialize all with default. ==> Others can change it later <==
		p->chan[i].iir_f1.skipctr  = 8;    // Initial readings skip count
		p->chan[i].iir_f1.coef     = 0.999;  // Filter coefficient (< 1.0)
		p->chan[i].iir_f1.onemcoef = (1 - p->chan[i].iir_f1.coef); // Pre-computed
	}

	/* Internal voltage reference and temperature sensor. */
	adcparamsinit_init_common(p);

	/* Absolute measurements. */
	abs_init(p, ADC1IDX_STEPPERV);
	abs_init(p, ADC1IDX_SPARE);
	abs_init(p, ADC1IDX_5VSUPPLY);

/* Ratiometric: if spare is ratiometric type. */
//	ratio_init(p, ADC1IDX_SPARE);

	return;
};
/* *************************************************************************
int16_t ratiometric_cal_zero(struct ADCFUNCTION* p, int16_t idx);
 *	@brief	: Adjust no-current ratio for a Hall-effect sensor
 * @param	: p = Pointer to struct "everything" for this ADC module
 * @param	: idx = ADC scan sequence index
 * @return	: 0 = no fault; -1 = out of tolerance
 * *************************************************************************/
int16_t ratiometric_cal_zero(struct ADCFUNCTION* p, int16_t idx)
{
	float ftmp;

	if (adcmapratio[idx] < 0) morse_trap(56); // Coding problem
	struct ADCRATIOMETRIC* pcur = &p->ratio[adcmapratio[idx]];

	// Check that re-zero'ing is not some crazy value
	ftmp  = ((float)p->chan[idx].sum / (float)p->chan[ADC1IDX_5VSUPPLY].sum) ;
	if ( (ftmp > (pcur->frko * (1+ZTOLERANCE))) || (ftmp < (pcur->frko * (1-ZTOLERANCE))) )
	{
		return -1;
	}
	else
	{ // Here adjustment is considered reasonable.
		pcur->frko = ftmp;
		pcur->irko  =ftmp * (1 << ADCSCALEbits);
	}
	return 0;
}




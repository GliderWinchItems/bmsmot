/******************************************************************************
* File Name          : A0000000-adc_idx_v_struct.c
* Date First Issued  : 06/27/2023
* Board              : bmsmot
* Description        : Load sram local copy of parameters
*******************************************************************************/
/* 10/23/2020: Revised for Levelwind */
/* 03/01/2022: Revised for bmsmax14921 */

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
int adc_idx_v_struct_hardcode_params(struct ADCLC* p)
{
	p->calintern.iiradcvref.k     = 20;    // Filter time constant
	p->calintern.iiradcvref.scale = 64;

	p->calintern.iiradctemp.k     = 100;    // Filter time constant
	p->calintern.iiradctemp.scale = 4;

	// Internal voltage ref: ADC1IDX_INTERNALVREF // IN18     - Internal voltage reference
	p->calintern.fvdd   = 3.302f;  // Vdd for following Vref ADC reading
	p->calintern.adcvdd = 24000;   //(16*1495.5) ADC reading (DMA sum) for above Vdd
	p->calintern.fvref  = 1.210f;  // reference voltage

	// Internal temperature: ADC1IDX_INTERNALTEMP // IN17     - Internal temperature sensor
	p->calintern.adcrmtmp  = 17838; // Room temp ADC (DMA sum) reading
	p->calintern.frmtemp   = 25.0f;  // Room temp for ADC reading     
	p->calintern.fslope    =  4.3f;   // mv/degC slope of temperature sensor
	p->calintern.fvreftmpco= 15.0f;    // Vref temp coefficient (15 is based on similar parts)
	p->calintern.fvtemp    =  1.40f;    // Vtemp voltage at 25 degC

	p->calintern.vcc = 3.3033; // 3.3v regulator measured voltage
	p->powergone = 9.0f; // Below this assume CAN power is gone

	p->offset = 1884; // Initial ADC2 zero input offset

/* ADC channels.*/
#define CELLTC 0.95f // Default filter time constant
#define SKIPCT 3    // Ignore initial readings to filter

/* Copied from adcparams.h 
#define ADC1IDX_THERMISTOR1   0	// PC0 IN10   JP9  Thermistor
#define ADC1IDX_THERMISTOR2	  1 // PC1 IN11   JP8  Thermistor
#define ADC1IDX_THERMISTOR3   2 // PC2 IN12   JP10 Thermistor
#define ADC1IDX_THERMISTOR4   3	// PC3 IN13   JP11 Thermistor
#define ADC1IDX_DIVIDEDSPARE  4	// PC4 IN14   JP17 Spare: 10k|10k divider
#define ADC1IDX_PRESS_SENSE   5	// PC5 IN15   JP24 Pressure sensor
#define ADC1IDX_12V_POWR      6	// PA7 IN7    12v Power supply
#define ADC1IDX_BATTLEAK_P    7	// PB0 IN8    Battery string plus leakage
#define ADC1IDX_BATTLEAK_M    8	// PB1 IN9    Battery string minus leakage
#define ADC1IDX_ACTRANSFRMR   9 // PA6 IN6    JP34 AC Transformer
#define ADC1IDX_INTERNALVBAT 10 // IN18       VBAT
#define ADC1IDX_INTERNALVREF 11 // IN17       VREF
*/
#define THRMOFF 86.0f
#define THRMSCALE -1.873E-3f;
#define THRMTC 0.990f
// ADC1IDX_THERMISTOR1   0	// PC0 IN10   JP9  Thermistor: JP9
	p->cabs[ADC1IDX_THERMISTOR1].iir_f1.coef     = THRMTC; // Filter time constant
	p->cabs[ADC1IDX_THERMISTOR1].iir_f1.skipctr  = SKIPCT;  // Initial skip
	p->cabs[ADC1IDX_THERMISTOR1].coef[1]     = THRMSCALE; // 
	p->cabs[ADC1IDX_THERMISTOR1].coef[0]    = THRMOFF;  // 

// ADC1IDX_THERMISTOR2	  1 // PC1 IN11   JP8  Thermistor: JP8
	p->cabs[ADC1IDX_THERMISTOR2].iir_f1.coef     = THRMTC; // Filter time constant
	p->cabs[ADC1IDX_THERMISTOR2].iir_f1.skipctr  = SKIPCT;  // Initial skip
	p->cabs[ADC1IDX_THERMISTOR2].coef[1]     = THRMSCALE; // 
	p->cabs[ADC1IDX_THERMISTOR2].coef[0]    = THRMOFF;  // 

// ADC1IDX_THERMISTOR3   2 // PC2 IN12   JP10 Thermistor: JP10
	p->cabs[ADC1IDX_THERMISTOR3].iir_f1.coef     = THRMTC; // Filter time constant
	p->cabs[ADC1IDX_THERMISTOR3].iir_f1.skipctr  = SKIPCT;  // Initial skip
	p->cabs[ADC1IDX_THERMISTOR3].coef[1]     = THRMSCALE; // 
	p->cabs[ADC1IDX_THERMISTOR3].coef[0]    = THRMOFF;  // 

// ADC1IDX_THERMISTOR4   3	// PC3 IN13   JP11 Thermistor: JP11
	p->cabs[ADC1IDX_THERMISTOR4].iir_f1.coef     = THRMTC; // Filter time constant
	p->cabs[ADC1IDX_THERMISTOR4].iir_f1.skipctr  = SKIPCT;  // Initial skip
	p->cabs[ADC1IDX_THERMISTOR4].coef[1]     = THRMSCALE;//5.424129E-04f; // Apply calibration below 
	p->cabs[ADC1IDX_THERMISTOR4].coef[0]    = THRMOFF;  // 

// ADC1IDX_DIVIDEDSPARE  4	// PC4 IN14   10k|10k divider
	p->cabs[ADC1IDX_DIVIDEDSPARE].iir_f1.coef     = CELLTC; // Filter time constant
	p->cabs[ADC1IDX_DIVIDEDSPARE].iir_f1.skipctr  = SKIPCT;  // Initial skip
	p->cabs[ADC1IDX_DIVIDEDSPARE].coef[1]     = 2.520833E-05f; // Apply calibration below 
	p->cabs[ADC1IDX_DIVIDEDSPARE].ioffset    = 0;  // Offset before scale

// ADC1IDX_PRESS_SENSE   5	// PC5 IN15   JP24 Pressure sensor JP24
	p->cabs[ADC1IDX_PRESS_SENSE].iir_f1.coef     = CELLTC; // Filter time constant
	p->cabs[ADC1IDX_PRESS_SENSE].iir_f1.skipctr  = SKIPCT;  // Initial skip
	p->cabs[ADC1IDX_PRESS_SENSE].coef[1]     = 2.520833E-05f; // Apply calibration below 
	p->cabs[ADC1IDX_PRESS_SENSE].ioffset    = 0;  // Offset before scale

// ADC1IDX_12V_POWR      6	// PA7 IN7    12v Power supply
	p->cabs[ADC1IDX_12V_POWR].iir_f1.coef     = 0.99f; // Filter time constant
	p->cabs[ADC1IDX_12V_POWR].iir_f1.skipctr  = SKIPCT;  // Initial skip
	p->cabs[ADC1IDX_12V_POWR].coef[1]     = 2.4363E-4f; // Apply calibration below 
	p->cabs[ADC1IDX_12V_POWR].ioffset    = 0;  // Offset before scale

// ADC1IDX_BATTLEAK_P    7	// PB0 IN8    Battery string minus leakage
	p->cabs[ADC1IDX_BATTLEAK_P].iir_f1.coef     = 0.99f; // Filter time constant
	p->cabs[ADC1IDX_BATTLEAK_P].iir_f1.skipctr  = SKIPCT;  // Initial skip
	p->cabs[ADC1IDX_BATTLEAK_P].coef[1]     = 0.03125; //  1/32 scale factor
	p->cabs[ADC1IDX_BATTLEAK_P].ioffset    = 0;  // Offset before scale

// ADC1IDX_BATTLEAK_M    8	// PB1 IN9    Battery string plus leakage
	p->cabs[ADC1IDX_BATTLEAK_M].iir_f1.coef     = CELLTC; // Filter time constant
	p->cabs[ADC1IDX_BATTLEAK_M].iir_f1.skipctr  = SKIPCT;  // Initial skip
	p->cabs[ADC1IDX_BATTLEAK_M].coef[1]     = 1.478422E-04f;
	p->cabs[ADC1IDX_BATTLEAK_M].ioffset    = 0;  // Offset before scale

// ADC1IDX_12V_POWR_DUP  9 // / PA7 IN7    12v Power supply (duplicate)
	p->cabs[ADC1IDX_12V_POWR_DUP].iir_f1.coef     = 0.99f; // Filter time constant
	p->cabs[ADC1IDX_12V_POWR_DUP].iir_f1.skipctr  = SKIPCT;  // Initial skip
	p->cabs[ADC1IDX_12V_POWR_DUP].coef[1]    = 2.4363E-4f; // Apply calibration below 
	p->cabs[ADC1IDX_12V_POWR_DUP].ioffset  = 0;  // Offset before scale

// ADC1IDX_INTERNALVBAT 10 // IN18   Internal temperature or VBAT
	p->cabs[ADC1IDX_INTERNALVBAT].iir_f1.coef     = CELLTC; // Filter time constant
	p->cabs[ADC1IDX_INTERNALVBAT].iir_f1.skipctr  = SKIPCT;  // Initial skip
	p->cabs[ADC1IDX_INTERNALVBAT].coef[1]     = (4*0.52176483E-04f); // VBAT is divided by 4
	p->cabs[ADC1IDX_INTERNALVBAT].ioffset    = 0;  // Offset before scale

// ADC1IDX_INTERNALVREF  11 // IN17   Internal voltage reference 
	p->cabs[ADC1IDX_INTERNALVREF].iir_f1.coef     = CELLTC; // Filter time constant
	p->cabs[ADC1IDX_INTERNALVREF].iir_f1.skipctr  = SKIPCT;  // Initial skip
	p->cabs[ADC1IDX_INTERNALVREF].coef[1]     = 0.5039682E-04f;
	p->cabs[ADC1IDX_INTERNALVREF].ioffset    = 0;  // Offset before scale	


	/* Initialize iir filter. */
	for (int i = 0; i < ADC1DIRECTMAX; i++)
	{
		p->cabs[i].iir_f1.onemcoef = 1 - p->cabs[i].iir_f1.coef;
	}

/* ############## ADC2 ########################## */
	// PA6 IN6    AC transformer w offset

	/* ADC2 offset calibration. */
	p->offset = 1885; // Initial ADC2 zero input offset
	/* IIR filtering of ongoing offset computation. */
	p->iir_adc2offset.coef     = 0.99;// Filter time constant
	p->iir_adc2offset.skipctr  = 4;	// Initial skip
	p->iir_adc2offset.onemcoef = 1 - p->iir_adc2offset.coef;
	p->adc2offset_scale    = .009810806f;	// Filter scale

	return 0;	
}
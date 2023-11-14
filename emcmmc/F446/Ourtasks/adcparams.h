/******************************************************************************
* File Name          : adcparams.h
* Date First Issued  : 06/27/2023
* Board              : bmsmot
* Description        : Parameters for ADC app configuration
*******************************************************************************/
/* 
03/03/2022 revised for bmsmax14921.c
06/02/2022 revised for bmabms1818.c
*/

#ifndef __ADCPARAMS
#define __ADCPARAMS

//#include "iir_filter_lx.h"
#include "iir_f1.h"
#include "adc_idx_v_struct.h"
#include "ADCTask.h"

/* Processor ADC reading sequence/array indices */
/* These indices -=>MUST<= match the hardware ADC scan sequence in STM32CubeMX. */
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

/* This holds calibration values common to all ADC modules. 
     Some of these are not used. */
struct ADCCALCOMMON
{
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
	float ts_cal1;    // Vtemp: TS_CAL1 converted to float
	float ts_cal2;    // Vtemp: TS_CAL2 converted to float
	float ts_caldiff; // (ts_cal2 - ts_cal1)
	float ts_calrate; // (calibration temp diff)/ts_calrate
	float v25;           // Vtemp: 25 deg C (0.76v typ)
    float slope;         // Vtemp: mv/degC 
	float offset;        // Vtemp: offset
	float degC;          // Temperature: degrees C
	float degCfilt;      // Temperature: degrees C, filtered
 	uint32_t dmact;      // DMA interrupt running counter
 	uint32_t dmact2;      // DMA interrupt running counter 2nd 1/2

 	float fvrefajd; // Vref temerature adjustment factor (nominally 1.0000000f)

 	uint32_t vref_sum;
 	uint32_t vtemp_sum;

	// For integer computation (much faster)
	uint32_t uicaldiff;
	int64_t ll_80caldiff;
	uint32_t ui_cal1;
	uint32_t ui_tmp;
};

/* Working values for absolute voltages adjusted using Vref. */
struct ADCABS
{
	float filt;      // Filtered ADC reading
	float f;         // Calibrated, not filtered
	int32_t sum;     // Working sum of multiple scans
	int32_t sumsave; // Saved sum for printf'ing
};
struct ADCCHANNEL	
{
	struct FILTERIIRF1 iir_f1;	// iir_f1 (float) filter
	float    fscale;  // Scale factor
	float    offset;  // Offset
	uint32_t sum;     // Fast Sum of ADC readings
};
/* Everything for the ADC in one struct. */
struct ADCFUNCTION
{
	struct ADCLC lc;    // Local Copy of parameters
	struct ADCCALCOMMON common; // Vref & temp stuff
	struct ADCABS abs[ADC1DIRECTMAX]; // Absolute readings calibrated
	struct ADCCHANNEL chan[ADC1DIRECTMAX]; //
	uint32_t ctr; // Running count of updates.
	uint8_t sumctr; // DMA summation counter
};


/* *************************************************************************/
void adcparams_init(void);
/*	@brief	: Copy parameters into structs
 * NOTE: => ASSUMES ADC1 ONLY <==
 * *************************************************************************/
 float adcparams_caltemp(void);
/*	@brief	: calibrate processor internal temperature reading
 *  @return : Internal temperature (deg C)
 * *************************************************************************/

/* Calibration values common to all ADC modules. */
extern struct ADCCALCOMMON adcommon;

/* Everything for ADC1. */
extern struct ADCFUNCTION adc1;
extern struct ADCFUNCTION adc2;


#endif

/******************************************************************************
* File Name          : adc_idx_v_struct.h
* Date First Issued  : 10/27/2021
* Board              : bmsbmsbq: STM32L431
* Description        : Load sram local copy of parameters
*******************************************************************************/
/*
                     Min  Typ  Max 
Internal reference: 1.16 1.20 1.24 V
Vref temperautre co.  --   --  100 ppm/DegC

Temperature sensor specs
                 Min  Typ  Max
Average slope    4.0  4.3  4.6 mV/°C
Voltage at 25 °C 1.34 1.43 1.52 

*/
#include <stdint.h>
#include "common_can.h"
#include "iir_filter_lx.h"
#include "ADCTask.h"

#ifndef __ADC_IDX_V_STRUCT
#define __ADC_IDX_V_STRUCT

#define ADC1DIRECTMAX  12 // 'F446 ADC1: Number of inputs read in one scan with DMA
#define ADC2DIRECTMAX   1 // 'F446 ADC2: Number of inputs read in one scan with DMA

/* Parameters for ADC reading */
// Flointing pt params are converted to scaled integers during initialization

/* Internal sensor calibration. (Only applies to ADC1) */
struct ADC1CALINTERNAL
{
	struct IIR_L_PARAM iiradcvref; // Filter: adc readings: Vref 
	struct IIR_L_PARAM iiradctemp; // Filter: adc readings: temperature
	float fvref;      // Internal reference voltage fixed
	float frmtemp;    // (float) Room temp for reading (deg C)
	float fvtemp;     // (float) Voltage of temp sensor at rm temperature
	float fvdd;       // (float) measured Vdd (volts)
	float fslope;     // (float) mv/degC temperature sensor slope
	float fvreftmpco; // (float) Vref temperature coefficient (ppm/degC)
	float vcc;        // (float) 3.3v regulator measured
	uint32_t adcvdd;   // (ADC reading) for calibrating Vdd (3.3v)
	uint32_t adcrmtmp; // (ADC reading) room temperature temp sensor reading
};

//* Absolute (non-ratiometric) sensor calibration. */
struct ADCCALABS
{
	struct FILTERIIRF1 iir_f1; // Filter: Time constant, integer scaling
	float coef[3]; // coefficients for: x^0 x^1 x^2
	float  f;
	int32_t  ioffset; // Offset before float conversion
};

/* Parameters for ADC. */
// LC = Local (sram) Copy of parameters
 struct ADCLC
 {
	uint32_t size;			// Number of items in struct
 	uint32_t crc;			// crc-32 placed by loader
	uint32_t version;		// struct version number
	struct ADC1CALINTERNAL calintern; // Vref and Temp internal sensors
	struct ADCCALABS cabs[ADC1DIRECTMAX]; // Processor ADC Absolute readings
	float powergone; // Begin ending sequence when DC-DC converter voltage drops below this

	uint16_t offset; // Initial ADC2 zero input offset
	struct FILTERIIRF1 iir_adc2offset;
	float adc2offset_scale;
 };

/* **************************************************************************************/
int adc_idx_v_struct_hardcode_params(struct ADCLC* p);
/* @brief	: Hard-code load local copy with parameters
 * @return	: 0
 * ************************************************************************************** */

#endif
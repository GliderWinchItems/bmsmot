/******************************************************************************
* File Name          : ADCTask.h
* Board              : bmsadbms1818: STM32L431
* Date First Issued  : 06/19/2022
* Description        : Processing ADC readings after ADC/DMA issues interrupt
*******************************************************************************/
#ifndef __ADCTASK
#define __ADCTASK

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "adcparams.h"
#include "main.h"
#include "adc_idx_v_struct.h"
#include "fast512summing.h"

#define ADC1SEQNUM 16 // Number of ADC1 scans in 1/2 of the DMA buffer
/* NOTE: fast512summing.h has one #define that defines the size
   of the ADC2 dma buffer (half). "fast512summing.S" uses this
   for a constant and repetition count for inline code. 
   C programs must use the same size.*/
#define ADC2SEQNUM FAST512SUMMING_BUFFSIZE // Number of ADC2 scans in 1/2 of the DMA buffer

/* Computation between EXTI interrupts. */
struct ADC2COMPUTED
{
	uint64_t sumsq; // Sum (reading - offset)^2
	uint32_t sum;   // Sum reading
	uint32_t n;     // Number of readings in above sums
	float fsum;     // sum/n
	float fsqr;     // sqrtf (sumsq/n)
};

/* ADC2 end of cycle grouping. */
// Sums and counters are "running" counts, so
//   (new - previous) is computed at end of AC cycle
struct ADC2NUM
{
	int64_t smq; // Sum square
	int32_t acc; // Sum
	int32_t ctr; // Count for above
int16_t idx;		
};
struct ADC2NUMALL
{
	struct ADC2NUM prev;
	struct ADC2NUM diff;
};

/* ADC2 fast processing collection. */
struct SUMSQBUNDLE
{
	uint64_t sumsq;     // Sum of (reading - offset)^2
	uint16_t* pdma;		// Ptr to dma buffer 
	uint16_t* pdma_end; // Ptr to dma buffer end+1
	uint32_t adcaccum;  // Sum of readings
	uint32_t adc2ctr;   // Running count of readings in both sums
	uint32_t offset;	// ADC2 offset with zero input 
	uint16_t n;			// Number of readings to sum

};

/* *************************************************************************/
osThreadId xADCTaskCreate(uint32_t taskpriority);
/* @brief	: Create task; task handle created is global for all to enjoy!
 * @param	: taskpriority = Task priority (just as it says!)
 * @return	: ADCTaskHandle
 * *************************************************************************/
 struct ADC2NUM* get_adc2num(void);
 /* @brief	: Circular buffer cycle-end differences
 * @return  : pointer to struct; NULL = buffer emtpy; 
 * NOTE: no buffer overrun protection.
 * *************************************************************************/

/* #######################################################################
   Pin 15 interrupt (AC opto-isolator) */
   void EXTI15_IRQHandler(void);
/*   ####################################################################### */

extern TaskHandle_t ADCTaskHandle;// replace: extern osThreadId ADCTaskHandle;
extern float    adcsumfilt[ADC1DIRECTMAX]; // Filtered
extern uint8_t  adcsumidx; // Index for currently being summed
extern struct ADC2NUMALL adc2numall;

// Debugging
#define DEBUGNUMSIZE 360
extern struct ADC2NUM debugnum[DEBUGNUMSIZE];
extern uint8_t  debugnum_flag;
extern struct SUMSQBUNDLE sumsqbundle;

#define BASE 512

#endif


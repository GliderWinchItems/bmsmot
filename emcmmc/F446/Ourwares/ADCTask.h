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

#define ADC1SEQNUM 16 // Number of ADC1 scans in 1/2 of the DMA buffer
#define ADC2SEQNUM (32*16) // Number of ADC2 scans in 1/2 of the DMA buffer

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
};
struct ADC2NUMALL
{
	struct ADC2NUM prev;
	struct ADC2NUM diff;
};

/* *************************************************************************/
osThreadId xADCTaskCreate(uint32_t taskpriority);
/* @brief	: Create task; task handle created is global for all to enjoy!
 * @param	: taskpriority = Task priority (just as it says!)
 * @return	: ADCTaskHandle
 * *************************************************************************/
/* #######################################################################
   Pin 15 interrupt (AC opto-isolator) */
   void EXTI15_IRQHandler(void);
/*   ####################################################################### */

extern TaskHandle_t ADCTaskHandle;// replace: extern osThreadId ADCTaskHandle;
extern float    adcsumfilt[ADC1DIRECTMAX]; // Filtered
extern uint8_t  adcsumidx; // Index for currently being summed
extern struct ADC2NUMALL adc2numall;

#define DEBUGNUMSIZE 360
extern struct ADC2NUM debugnum[DEBUGNUMSIZE];
extern uint8_t  debugnum_flag;

#endif


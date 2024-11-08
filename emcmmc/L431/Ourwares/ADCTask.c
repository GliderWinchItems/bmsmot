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
#include "adcparams.h"
#include "SensorTask.h"
#include "adcextendsum.h"

void StartADCTask(void const * argument);

uint32_t adcsumdb[6]; // debug
uint32_t adcdbctr = 0;// debug

osThreadId ADCTaskHandle;

extern ADC_HandleTypeDef hadc1;

extern osThreadId ContactorTaskHandle;

#include "DTW_counter.h"
uint32_t  debugADCt1;
uint32_t  debugADCt2;

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

osDelay(20); // Debugging

	/* Get buffers, "our" control block, and ==>START<== ADC/DMA running. */
	struct ADCDMATSKBLK* pblk = adctask_init(&hadc1,TSK02BIT02,TSK02BIT03,&noteval);
	if (pblk == NULL) {morse_trap(21);}

  /* Infinite loop */
  for(;;)
  {
		/* Wait for DMA interrupt notification from adctask.c */
		xTaskNotifyWait(noteused, 0, &noteval, portMAX_DELAY);
		noteused = 0;	// Accumulate bits in 'noteval' processed.

debugADCt2 = DTWTIME - debugADCt1;
debugADCt1 = DTWTIME;

		/* We handled one, or both, noteval bits */
		noteused |= (pblk->notebit1 | pblk->notebit2);

		if (noteval & TSK02BIT02)
		{
			pdma = adc1dmatskblk[0].pdma1; // [0] = adc1
		}
		else
		{
			pdma = adc1dmatskblk[0].pdma2;
		}

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
		adcsumdb[6] = adc1.chan[6].sum;
		adcdbctr += 1;
#endif

		/* Calibrate and filter ADC readings. */
		adcparams_cal();

		/* Extended sum for smoothing and display. */
		adcextendsum(&adc1);
#if 0
		/* Notify ContactorTask that new readings are ready. */
		if( SensorTaskHandle == NULL) morse_trap(22); // JIC task has not been created
		
		xTaskNotify(SensorTaskHandle, CNCTBIT00, eSetBits);
#endif
  }
}
/* *************************************************************************
 * osThreadId xADCTaskCreate(uint32_t taskpriority);
 * @brief	: Create task; task handle created is global for all to enjoy!
 * @param	: taskpriority = Task priority (just as it says!)
 * @return	: ADCTaskHandle
 * *************************************************************************/
osThreadId xADCTaskCreate(uint32_t taskpriority)
{
	
 	osThreadDef(ADCTask, StartADCTask, osPriorityNormal, 0, 96);
	ADCTaskHandle = osThreadCreate(osThread(ADCTask), NULL);
	vTaskPrioritySet( ADCTaskHandle, taskpriority );
	return ADCTaskHandle;

// CMISIS 2
#if 0	
	BaseType_t ret = xTaskCreate(StartADCTask, "ADCTask",\
     (96), NULL, taskpriority, &ADCTaskHandle);
	if (ret != pdPASS) return NULL;

	return ADCTaskHandle;
#endif
}


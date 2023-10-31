/******************************************************************************
* File Name          : ADCTask.c
* Board              : bmsadbms1818: STM32L431
* Date First Issued  : 06/19/2022
* Description        : Processing ADC readings after ADC/DMA issues interrupt
*******************************************************************************/
/* 10/23/2020: Revised for Levelwind */

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "malloc.h"

#include "ADCTask.h"
#include "adctask.h"
#include "morse.h"
//#include "adcfastsum16.h"
//#include "adcextendsum.h"
#include "rtcregs.h"

#include "main.h"
#include "DTW_counter.h"
#include "iir_f1.h"

extern ADC_HandleTypeDef hadc1;

uint32_t dbg_adcsum[ADCDIRECTMAX];

TaskHandle_t ADCTaskHandle;

extern osThreadId defaultTaskHandle;

float fclpos;


uint32_t debugadc1;
uint32_t debugadc2;
uint32_t debugadcctr;

/* *************************************************************************
 * void StartADCTask(void const * argument);
 *	@brief	: Task startup
 * *************************************************************************/
uint16_t* p16;

void StartADCTask(void *argument)
{
	#define TSK02BIT02	(1 << 0)  // Task notification bit for ADC dma 1st 1/2 (adctask.c)
	#define TSK02BIT03	(1 << 1)  // Task notification bit for ADC dma end (adctask.c)

	uint16_t* pdma;
	float ftmp;
	struct ADCCHANNEL* pz;
	struct ADCCHANNEL* pzend;
	
	/* A notification copies the internal notification word to this. */
	uint32_t noteval = 0;    // Receives notification word upon an API notify

	/* Get buffers, "our" control block, and start ADC/DMA running. */
	struct ADCDMATSKBLK* pblk = adctask_init(&hadc1,TSK02BIT02,TSK02BIT03,&noteval);
	if (pblk == NULL) {morse_trap(15);}	

	/* Initialize params for ADC. */
	adcparams_init();	

  	/* Infinite loop */
  	for(;;)
  	{
debugadc1 = DTWTIME;
		/* Wait for DMA interrupt */
		xTaskNotifyWait(0,0xffffffff, &noteval, portMAX_DELAY);
debugadc2 = DTWTIME - debugadc1;

		if (noteval & TSK02BIT02)
		{
			pdma = adc1dmatskblk[0].pdma1;
p16 = pdma;			
		}
		else
		{
			pdma = adc1dmatskblk[0].pdma2;
		}

		// Sum 12 scans in 1/2 DMA buffer. */
		pz = &adc1.chan[0];
		pzend = &adc1.chan[ADCDIRECTMAX];

		while (pz != pzend)
		{
			pz->sum = 
			   *(pdma + ADCDIRECTMAX     * 0)
			 + *(pdma + ADCDIRECTMAX     * 1)
			 + *(pdma + ADCDIRECTMAX     * 2)
			 + *(pdma + ADCDIRECTMAX     * 3)
			 + *(pdma + ADCDIRECTMAX     * 4)
			 + *(pdma + ADCDIRECTMAX     * 5)
			 + *(pdma + ADCDIRECTMAX     * 6)
			 + *(pdma + ADCDIRECTMAX     * 7)
			 + *(pdma + ADCDIRECTMAX     * 8)
			 + *(pdma + ADCDIRECTMAX     * 9)
			 + *(pdma + ADCDIRECTMAX     *10)
			 + *(pdma + ADCDIRECTMAX     *11)
			 + *(pdma + ADCDIRECTMAX     *12)
			 + *(pdma + ADCDIRECTMAX     *13)
			 + *(pdma + ADCDIRECTMAX     *14)
			 + *(pdma + ADCDIRECTMAX     *15);
			 pdma += 1;
			 pz += 1;
		}		 

		pz = &adc1.chan[0];

/* WOW. Optimizer does the following with one cycle per statement! */
dbg_adcsum[0]  = (pz +  0)->sum;
dbg_adcsum[1]  = (pz +  1)->sum;		
dbg_adcsum[2]  = (pz +  2)->sum;		
dbg_adcsum[3]  = (pz +  3)->sum;		
dbg_adcsum[4]  = (pz +  4)->sum;		
dbg_adcsum[5]  = (pz +  5)->sum;		
dbg_adcsum[6]  = (pz +  6)->sum;		
dbg_adcsum[7]  = (pz +  7)->sum;		
dbg_adcsum[8]  = (pz +  8)->sum;	
dbg_adcsum[9]  = (pz +  9)->sum;	
dbg_adcsum[10] = (pz + 10)->sum;	
dbg_adcsum[11] = (pz + 11)->sum;	

		adc1.ctr += 1; // Update count
debugadcctr = adc1.ctr;		
		// Calibrate and Pass sum through IIR filter
		for (int i = 0; i < ADCDIRECTMAX; i++)
		{ // Calibrate and filter sums
			ftmp = pz->sum; // Convert to float
			// y = a + b * x;
			adc1.abs[i].f = adc1.lc.cabs[i].offset + adc1.lc.cabs[i].scale * ftmp;
			adc1.abs[i].filt = iir_f1_f(&adc1.lc.cabs[i].iir_f1, adc1.abs[i].f);
			pz->sum = 0; // Zero sum in prep for next cycle.
			pz += 1;
		}
#if 0
		if (adc1.abs[6].f < adc1.lc.powergone)
		{ // Here, assume CAN power has been cut off
			/* Processor likely running on Cell #3 power,
			   which is connected when PC13 is ON. */
			rtcregs_update(); // Update RTC registers

HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);   // GRN OFF
HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET); // RED ON
osDelay(20);                 // Short delay for red led blink
			// Turn off Cell #3 power
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
			while(1==1);
		}
#endif		
  	}
  	return; // Never never
}
/* *************************************************************************
 * osThreadId xADCTaskCreate(uint32_t taskpriority);
 * @brief	: Create task; task handle created is global for all to enjoy!
 * @param	: taskpriority = Task priority (just as it says!)
 * @return	: ADCTaskHandle
 * *************************************************************************/
osThreadId xADCTaskCreate(uint32_t taskpriority)
{
	BaseType_t xRet;
	xRet = xTaskCreate(
		StartADCTask,     /* Function that implements the task. */
		"ADCTask",        /* Text name for the task. */
		256,              /* Stack size in words, not bytes. */
		NULL,             /* Parameter passed into the task. */
		taskpriority,     /* Priority at which the task is created. */
		&ADCTaskHandle ); /* Used to pass out the created task's handle. */ 

	if( xRet != pdPASS )return NULL;
   	return ADCTaskHandle;	
#if 0	

 	osThreadDef(ADCTask, StartADCTask, osPriorityNormal, 0, 256+64);
	ADCTaskHandle = osThreadCreate(osThread(ADCTask), NULL);
	vTaskPrioritySet( ADCTaskHandle, taskpriority );
	return ADCTaskHandle;
#endif

}


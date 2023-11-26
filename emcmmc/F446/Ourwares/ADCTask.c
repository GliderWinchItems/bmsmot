/******************************************************************************
* File Name          : ADCTask.c
* Board              : bmsadbms1818: STM32L431
* Date First Issued  : 06/19/2022
* Description        : Processing ADC readings after ADC/DMA issues interrupt
*******************************************************************************/
/* 10/23/2020: Revised for Levelwind */

#include <math.h>
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
extern ADC_HandleTypeDef hadc2;
extern DMA_HandleTypeDef hdma_adc2;
extern DMA_HandleTypeDef hdma_adc1;

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
static struct SUMSQBUNDLE sumsqbundle;

/* ADC2 end of cycle grouping. */
// Sums and counters are "running" counts, so
//   (new - previous) is computed at end of AC cycle
struct ADC2NUM
{
	uint64_t smq; // Sum square
	uint32_t acc; // Sum
	uint32_t ctr; // Count for above
};
struct ADC2NUMALL
{
	struct ADC2NUM prev;
	struct ADC2NUM diff;
};
static struct ADC2NUMALL adc2numall;
#define ADC2NUMSZ 8 // Circular buffer summary


void sumsquareswithif(struct SUMSQBUNDLE* psumsqbundle);
void sumsquaresfastest(void);
static void exti15_10_init(void);
void fastsumming(struct SUMSQBUNDLE* psumsqbundle, uint16_t idx);
void cycle_end(struct SUMSQBUNDLE* psmb, struct ADC2NUMALL* pall);	


uint32_t dbg_adcsum[ADC1DIRECTMAX];
uint32_t dbg_adcsum2;

TaskHandle_t ADCTaskHandle;

extern osThreadId defaultTaskHandle;

float fclpos;


uint32_t debugadc1;
uint32_t debugadc2;
uint32_t debugadcctr;
#define DEBUGSZ (513*8)
uint16_t debugadcsum[DEBUGSZ];
uint32_t debugadcsumidx;
uint16_t debugadcflag;
uint32_t debugadc2ctr;
uint32_t debugadc2dma_pdma2;
uint32_t debugadc2t;
uint32_t debugadc1t;

uint32_t adc2dma_flag;
uint32_t adc2dma_cnt;
uint32_t exti15dtw_reg;
uint32_t exti15dtw_reg1;



uint32_t exti15dtw;
uint32_t exti15dtw_prev;
uint32_t exti15dtw_diff;
uint32_t exti15dtw_flag;
uint32_t exti15dtw_flag_prev;
uint32_t exti15dtw_irqctr;

uint32_t sumsq_flag;  // Increments each time new sumsq saved


//I might want to make this a circular buffer
struct ADC2COMPUTED adc2computed; // Computed results of a cycle

#define OFFSET 1884
uint16_t offset = OFFSET; // Initial = calibration offset

uint32_t debugdma;
/* *************************************************************************
 * void StartADCTask(void const * argument);
 *	@brief	: Task startup
 * *************************************************************************/
void StartADCTask(void *argument)
{
	#define TSK02BIT02	(1 << 0)  // Task notification bit for ADC1 dma 1st 1/2 (adctask.c)
	#define TSK02BIT03	(1 << 1)  // Task notification bit for ADC1 dma end (adctask.c)
	#define TSK02BIT04	(1 << 2)  // Task notification bit for ADC2 dma 1st 1/2 (adctask.c)
	#define TSK02BIT05	(1 << 3)  // Task notification bit for ADC2 dma end (adctask.c)
	
	float ftmp;
	struct ADCCHANNEL* pz;
	struct ADCCHANNEL* pzend;
	uint16_t* pdma;
	uint16_t blkcnt;
	uint16_t blkidx;
	
	/* A notification copies the internal notification word to this. */
	uint32_t noteval = 0;    // Receives notification word upon an API notify

	/* Get buffers, "our" control block, and start ADC/DMA running. */
	// ADC1
	struct ADCDMATSKBLK* pblk1 = adctask_init(&hadc1,TSK02BIT02,TSK02BIT03,
		&noteval,ADC1SEQNUM, ADC1DIRECTMAX,0);
	if (pblk1 == NULL) {morse_trap(15);}	

	// ADC2
	struct ADCDMATSKBLK* pblk2 = adctask_init(&hadc2,TSK02BIT04,TSK02BIT05,
		&noteval,ADC2SEQNUM, ADC2DIRECTMAX,1);
	if (pblk2 == NULL) {morse_trap(15);}	

	/* Initialize params for ADC. */
	adcparams_init();	

	/* Initialize ADC2 sums. */
	sumsqbundle.sumsq    = 0; // Running sum squared
	sumsqbundle.adcaccum = 0; // Running sum of readings
	sumsqbundle.adc2ctr  = 0; // Running count of readings
	sumsqbundle.offset   = offset; // Params: default offset
	sumsqbundle.n        = 0xA; // For debugging

// Debugging
sumsqbundle.sumsq    = 0x100000002; // Running sum squared
sumsqbundle.adcaccum = 0x100; // Running sum of readings
sumsqbundle.adc2ctr  = 10; // Running count of readings
sumsqbundle.offset   = offset; // Params: default offset
sumsqbundle.n        = 0xA; // For debugging	

	/* Initialize EXTI15_10. */
	exti15_10_init();

  	/* Infinite loop */
  	for(;;)
  	{
debugadc1 = DTWTIME;
		/* Wait for DMA interrupt */
		xTaskNotifyWait(0,0xffffffff, &noteval, portMAX_DELAY);
debugadc2 = DTWTIME - debugadc1;
		if ((noteval & TSK02BIT04) || (noteval & TSK02BIT05))
		{ // ADC2 notification
			/* Set pointer to the half-buffer just completed.
			   Compute index (offset) into that half-buffer
			   where the EXTI interrupt took place. 

			   Note: sumsqbundle.adc2ctr is the DMA NDTR register that 
			   holds the number of "items" remaining to be stored. 
			   */
			if ((noteval & TSK02BIT04) != 0)
			{ // Here, dma half buffer complete notification
				sumsqbundle.pdma = adcdmatskblk[1].pdma1;
				adc2dma_cnt = (1024 - adc2dma_cnt);
			}
			else
			{ // Here, dma full buffer complete notification
				sumsqbundle.pdma = adcdmatskblk[1].pdma2;
				adc2dma_cnt = (512 - adc2dma_cnt);
			}
			// Possible for dma to have stored after EXTI saved sumsqbundle.adc2ctr
			if (adc2dma_cnt > (ADC2SEQNUM-1)) // (511)
				adc2dma_cnt = (ADC2SEQNUM-1);// morse_trap(7111);
			else if (adc2dma_cnt < 0)
					adc2dma_cnt = 0;// morse_trap(7222);

			sumsqbundle.pdma_end = sumsqbundle.pdma + ADC2SEQNUM;//(32*16) DMA 1/2 buffer end address
			pdma = sumsqbundle.pdma;	
#if 0
/* Save readings for output. */
debugadc2ctr += 1;
uint16_t* pdebug = &debugadcsum[debugadcsumidx];
//debugadc1t = DTWTIME;

if (debugadcsumidx < DEBUGSZ)			
{
	for(int j = 0; j < 512/16; j++)
	{
		*(pdebug + 0) = *(pdma + 0);
		*(pdebug + 1) = *(pdma + 1);
		*(pdebug + 2) = *(pdma + 2);
		*(pdebug + 3) = *(pdma + 3);
		*(pdebug + 4) = *(pdma + 4);
		*(pdebug + 5) = *(pdma + 5);
		*(pdebug + 6) = *(pdma + 6);
		*(pdebug + 7) = *(pdma + 7);
		*(pdebug + 8) = *(pdma + 8);
		*(pdebug + 9) = *(pdma + 9);
		*(pdebug +10) = *(pdma +10);
		*(pdebug +11) = *(pdma +11);
		*(pdebug +12) = *(pdma +12);
		*(pdebug +13) = *(pdma +13);
		*(pdebug +14) = *(pdma +14);
		*(pdebug +15) = *(pdma +15);
		pdebug += 16;
		pdma   += 16;
	}
debugadcsumidx += 512;			
debugadcsum[debugadcsumidx++] = 40000 + exti15dtw_irqctr;
}
#endif
//debugadc1t = DTWTIME;

			/* Subtract offset, sum readings, sum squares */
			// EXTI interrupt during the DMA loading of this buffer?
			if (adc2dma_flag != 0)
			{ // Zero crossing interrupt occured during dma of this buff half
				adc2dma_flag = 0; // Reset flag
	debugadc1t = DTWTIME;				

				/* Sum whole blocks up to block with EXTI interrupt */
				blkcnt = (adc2dma_cnt >> 5);
				blkidx  = (adc2dma_cnt & 0x1f);
				if (blkcnt != 0)
				{ // Here, EXTI was not in first block
					sumsqbundle.pdma_end = (sumsqbundle.pdma + blkcnt*32);
					fastestsumming(&sumsqbundle); // Sum whole blocks 
				}
				/* Sum block up to where EXTI occurred. */
				fastsumming(&sumsqbundle,blkidx); // Zero crossings in buffer
				cycle_end(&sumsqbundle, &adc2numall); // Save sums differences, etc.
				/* Sum remainder of block. */
				fastsumming(&sumsqbundle,(32 - blkidx)); 
				/* Sum remainder of buffer (whole blocks). */
				if (blkcnt != 15)
				{
					sumsqbundle.pdma_end = (sumsqbundle.pdma + (16 - blkcnt)*32);
					fastestsumming(&sumsqbundle);
				}
	debugadc2t = DTWTIME - debugadc1t;
			}
			else
			{ // Here, no EXTI interrupt occured during this buffer loading
//	debugadc1t = DTWTIME;
				// Sum the entire buffer
				fastestsumming(&sumsqbundle); // No zero crossings
//	debugadc2t = DTWTIME - debugadc1t;				
			}
		}
	/* ========= ADC1 notification handle here. ========= */
		if ((noteval & TSK02BIT02) || (noteval & TSK02BIT03))
		{ // ADC1 notification
			if (noteval & TSK02BIT02)
				pdma = adcdmatskblk[0].pdma1;
			else
				pdma = adcdmatskblk[0].pdma2;

			// Sum 16 twelve-input scans in 1/2 DMA buffer. */
			pz = &adc1.chan[0];
			pzend = &adc1.chan[ADC1DIRECTMAX];
			while (pz != pzend)
			{
				pz->sum = 
				   *(pdma + ADC1DIRECTMAX *  0)
				 + *(pdma + ADC1DIRECTMAX *  1)
				 + *(pdma + ADC1DIRECTMAX *  2)
				 + *(pdma + ADC1DIRECTMAX *  3)
				 + *(pdma + ADC1DIRECTMAX *  4)
				 + *(pdma + ADC1DIRECTMAX *  5)
				 + *(pdma + ADC1DIRECTMAX *  6)
				 + *(pdma + ADC1DIRECTMAX *  7)
				 + *(pdma + ADC1DIRECTMAX *  8)
				 + *(pdma + ADC1DIRECTMAX *  9)
				 + *(pdma + ADC1DIRECTMAX * 10)
				 + *(pdma + ADC1DIRECTMAX * 11)
				 + *(pdma + ADC1DIRECTMAX * 12)
				 + *(pdma + ADC1DIRECTMAX * 13)
				 + *(pdma + ADC1DIRECTMAX * 14)
				 + *(pdma + ADC1DIRECTMAX * 15);
				 pdma += 1;
				 pz += 1;
			}		 

// Save output for main, debugging and whatever...
	pz = &adc1.chan[0];
/* WOW. Optimizer does the following with one cycle per statement! */
// NB: could use a memory-memory dma here?
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
			for (int i = 0; i < ADC1DIRECTMAX; i++)
			{ // Calibrate and filter sums
				ftmp = pz->sum; // Convert to float
				// y = a + b * x;
				adc1.abs[i].f = adc1.lc.cabs[i].offset + adc1.lc.cabs[i].scale * ftmp;
				adc1.abs[i].filt = iir_f1_f(&adc1.lc.cabs[i].iir_f1, adc1.abs[i].f);
				pz->sum = 0; // Zero sum in prep for next cycle.
				pz += 1;
			}
	#if 0
	// this is from BMS1818--
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
  	}
  	morse_trap(7234);
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
		(256+96),              /* Stack size in words, not bytes. */
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
#define EXTI15REG 0x8000;
/* *************************************************************************
 * static void exti15_10_init(void);
 * @brief	: Init gpio input interrupt
 * *************************************************************************/
static void exti15_10_init(void)
{
	
	EXTI->RTSR |=  EXTI15REG;  // Trigger on rising edge
//	EXTI->FTSR |=  EXTI15REG;  // Trigger on falling edge
	EXTI->IMR  |=  EXTI15REG;  // Interrupt mask reg: 10:15
//	EXTI->EMR  |=  EXTI15REG;  // Event mask reg: enable 10:15
	EXTI->PR   |=  EXTI15REG;  // Clear any pending
	return;
}

/* #######################################################################
   Pin 15 interrupt (AC opto-isolator)
   ####################################################################### */
uint32_t exti15dtw_accum_flag;
uint32_t exti15dtw_accum_ctr;
int32_t  exti15dtw_accum_diff;
uint32_t exti15dtw_accum_prev;
uint32_t exti15dtw_accum;
void EXTI15_IRQHandler(void)
{
	exti15dtw = DTWTIME;
	EXTI->PR = EXTI15REG
	// JIC, as there is no hysteresis on opto-isolator
	if ((exti15dtw - exti15dtw_prev) > 10000)
	{
//Debug: exti15dtw_reg = *(uint32_t*)((uint8_t*)(0x40026400 + 0x14 + (0x18*2)));

		// Save DMA storing position: cnt is =>remaining<= count of dma items 
		adc2dma_cnt  = *(uint32_t*)(&hdma_adc2.Instance->NDTR);
		adc2dma_flag = 1; // DMA task notification processing sees this flag	

		exti15dtw_diff = exti15dtw - exti15dtw_prev;
		exti15dtw_prev = exti15dtw;
		exti15dtw_flag += 1;

// Debug: check system timer over a number of EXTI interrupts		
		exti15dtw_accum_ctr += 1;
		if (exti15dtw_accum_ctr >= 240)
		{
			exti15dtw_accum = exti15dtw;
			exti15dtw_accum_diff = (int32_t)(exti15dtw_accum - exti15dtw_accum_prev);
			exti15dtw_accum_prev = exti15dtw_accum;
			exti15dtw_accum_ctr = 0;
			exti15dtw_accum_flag = 1;
		}
	}
	// If this counter is larger than exti15dtw_accum_ctr there are hysteresis interrupts.
	exti15dtw_irqctr += 1;
	return;
}
/* *************************************************************************
 * void cycle_end((struct SUMSQBUNDLE* psumsqbundle);
 * @brief	: 
 * *************************************************************************/
/*
struct ADC2NUM
{
	uint64_t smq; // Sum square
	uint32_t acc; // Sum
	uint32_t ctr; // Count for above
};
struct ADC2NUMALL
{
	struct ADC2NUM prev;
	struct ADC2NUM diff;
};
static struct ADC2NUMALL adc2numall;
*/
void cycle_end(struct SUMSQBUNDLE* psmb, struct ADC2NUMALL* pall)

{
	// Compute differenes between EXTI interrupts
	// Sum of (reading - offset)^2
	pall->diff.smq = psmb->sumsq - pall->prev.smq;
	pall->prev.smq = psmb->sumsq;
	// Sum of (reading - offset)
	pall->diff.acc = psmb->adcaccum - pall->prev.acc;
	pall->prev.acc = psmb->adcaccum;
	// Count of readings between EXTI interrupts
	pall->diff.ctr = psmb->adc2ctr - pall->prev.ctr;
	pall->prev.ctr = psmb->adc2ctr;

// Might want to put pall->diff into a circular buffer
// Let a lower level task do things like the following--	
//	adc2computed.fsum  = (float)pall->diff.smb/pall->diff.ctr;
//	adc2computed.fsqr  = sqrtf((float)pall->diff.smq/pall->diff.ctr);

	sumsq_flag += 1; // Not needed if using queue
	return;
}

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

void sumsquareswithif(void);
void sumsquaresfastest(void);
static void exti15_10_init(void);

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
int32_t adc2dma_cnt;
uint32_t adc2dma_flag;
uint32_t exti15dtw_reg;
uint32_t exti15dtw_reg1;



uint32_t exti15dtw;
uint32_t exti15dtw_prev;
uint32_t exti15dtw_diff;
uint32_t exti15dtw_flag;
uint32_t exti15dtw_flag_prev;
uint32_t exti15dtw_irqctr;

static uint32_t adcaccum; // Running accumulation of ADC2 
static uint32_t adcaccum_prev; // Previous adcaccum_save
static uint32_t adcaccum_diff; // Difference: (save - prev)

static uint32_t adc2ctr;       // Running count of ADC2 readings
static uint32_t adc2ctr_prev;  // Previous adc2ctr saved
static uint32_t adc2ctr_diff;  // Difference: (save - prev)

static uint16_t* pdma_end;
static uint32_t isumsq;   // 32b sum of (reading - offset)^squared

uint64_t sumsq;      // Running accumulation of squares
uint64_t sumsq_prev; // Saved sumsq (i.e. "previous")
uint64_t sumsq_diff; // Difference: sumsqdiff = (new - previous)
uint32_t sumsq_flag;  // Increments each time new sumsq saved

uint64_t sumsq_ctr;      // Running count of readings
uint64_t sumsq_ctr_prev; // Saved sumq_ctr (i.e. "previous")
uint64_t sumsq_ctr_diff; // Difference: sumsqdiff = (new - previous)

//I might want to make this a circular buffer
struct ADC2COMPUTED adc2computed; // Computed results of a cycle

#define OFFSET 1884
uint16_t offset = OFFSET; // Initial = calibration offset

/* Location of in DMA buffer where zero crossing took place. */
uint16_t blkidx;    // Block (chunk) index (1 - 16)
uint16_t blksubidx; // Within block index (0 - 31)

static uint16_t* pdma;
static 	int32_t tmp;

struct SUMSQBUNDLE
{
	uint16_t* pdma;
	uint32_t adcaccum;
	uint32_t isumsq;
	uint16_t offset;
	uint16_t n;
};

struct SUMSQBUNDLE sumsqbundle;


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

			   Note: adc2dma_cnt is the DMA NDTR register that 
			   holds the number of "items" remaining to be stored. 
			   */
			if ((noteval & TSK02BIT04) != 0)
			{ // Here, dma half buffer complete notification
				pdma = adcdmatskblk[1].pdma1;
				adc2dma_cnt = (1024 - adc2dma_cnt);
			}
			else
			{ // Here, dma full buffer complete notification
				pdma = adcdmatskblk[1].pdma2;
				adc2dma_cnt = (512 - adc2dma_cnt);
			}

uint16_t* pdma_save = pdma;	// Debugging code might increment pdma

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
//debugadc2t = DTWTIME - debugadc1t;	
#endif

pdma = pdma_save; // Restore dma ptr

//debugadc1t = DTWTIME;

			/* Highly inlined: subtract offset, sum readings, sum squares */
			pdma_end = pdma + ADC2SEQNUM;//(32*16) DMA 1/2 buffer end address
			isumsq = 0; // 32b sum of squares over a 32 reading "chunk"

			if (adc2dma_flag != 0)
			{ // Zero crossing interrupt occured during dma of this buff half
				adc2dma_flag = 0; // Reset flag

				// Possible for dma to have stored after EXTI saved cnt
				if (adc2dma_cnt > (ADC2SEQNUM-1)) // (511)
					adc2dma_cnt = (ADC2SEQNUM-1);// morse_trap(7111);
				else if (adc2dma_cnt < 0)
					adc2dma_cnt = 0;// morse_trap(7222);

				// block/chunk where EXTI interrupt occured
				blkidx = (adc2dma_cnt >> 5); // 32 reading chunks
				// Index within block/chunk
				blksubidx = (adc2dma_cnt & 0x1F);
	debugadc1t = DTWTIME;				
				sumsquareswithif(); // Zero crossings check
	debugadc2t = DTWTIME - debugadc1t;				
			}
			else
			{ // Here, no EXTI interrupt occured during this buffer loading
//	debugadc1t = DTWTIME;				
				sumsquaresfastest(); // No zero crossings in buffer
//	debugadc2t = DTWTIME - debugadc1t;				
			}

// Something to cause comiler to include the code
void sumsquaresgoto(void);			
if (blkidx == 65535)
	sumsquaresgoto();

void sumsquaresasm(uint16_t* pread, uint32_t nadcaccum, uint16_t noffset, uint32_t nsumsq, uint32_t n);
if (blkidx == 65534)
	sumsquaresasm(pdma,adcaccum,offset,isumsq,15);

//void fastsumming(uint16_t* pread, uint32_t nadcaccum, uint16_t noffset, uint32_t nsumsq, uint32_t n);

sumsqbundle.pdma = pdma;
sumsqbundle.adcaccum = adcaccum;
sumsqbundle.isumsq = isumsq;
sumsqbundle.offset = offset;
sumsqbundle.n = 0xA;
void fastsumming(struct SUMSQBUNDLE* psumsqbundle);
if (blkidx == 65533)
	fastsumming(&sumsqbundle);
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
		adc2dma_cnt = *(uint32_t*)(&hdma_adc2.Instance->NDTR);
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
 * void loop32(void);
 * @brief	: 
 * *************************************************************************/
void loop32(void)
{
	for (int j = 0; j < 32; j++)
	{
		adc2ctr += 1; // Running total of number of ADC2 readings
		// Sum (reading - offset)
		uint32_t tmp = *(pdma + j); 

		adcaccum += tmp; // Sum (reading)
		tmp -= offset;
		// Sum (reading - offset) squared
		isumsq += tmp * tmp; 
		// Check if EXTI interrupt occurred at this reading
		if (blksubidx == j)
		{ // Here EXTI interrupt occurred on this reading
			sumsq += isumsq; // Sum to 64b accum
			isumsq = 0;      // Reset 32b accum

/* Might want to change the to have parameter for number of EXTI's between saves. */
			// Number of readings 
			adc2ctr_diff = adc2ctr - adc2ctr_prev;
			adc2ctr_prev = adc2ctr;
			// Sum of (reading - offset)
			adcaccum_diff = adcaccum - adcaccum_prev;
			adcaccum_prev = adcaccum;
			// Sum of (reading - offset)^2
			sumsq_diff = sumsq - sumsq_prev;
			sumsq_prev = sumsq; // Duration between

		// Might want to put this into a circular buffer
			adc2computed.sumsq = sumsq_diff;
			adc2computed.sum   = adcaccum_diff;
			adc2computed.n     = adc2ctr_diff;
			adc2computed.fsum  = (float)adc2computed.sum/adc2computed.n;
			adc2computed.fsqr  = sqrtf((float)adc2computed.sumsq/adc2computed.n);

			sumsq_flag += 1;
		}
	}
	pdma += 32;
	return;
}
/* *************************************************************************
 * void sumsquareswithif(void);
 * @brief	: Inline: subtract offset, sum readings, sum squares with"if" between chunks
 * *************************************************************************/
void sumsquareswithif(void)
{
	int i = 0; // Block/chunk index
	while (pdma != pdma_end)
	{
		isumsq = 0; // 32b accumulation of sum squares
		if (blkidx == i)
			loop32(); // EXTI occurred during this "chunk"
		else
		{
		/*  1 */ tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/*  2 */ tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/*  3 */ tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/*  4 */ tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/*  5 */ tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/*  6 */ tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/*  7 */ tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/*  8 */ tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/*  9 */ tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/* 10 */ tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/* 11 */ tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/* 12 */ tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/* 13 */ tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/* 14 */ tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/* 15 */ tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/* 16 */ tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/* 17 */ tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/* 18 */ tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/* 19 */ tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/* 20 */ tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/* 21 */ tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/* 22 */ tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/* 23 */ tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/* 24 */ tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/* 25 */ tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/* 26 */ tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/* 27 */ tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/* 28 */ tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/* 29 */ tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/* 30 */ tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/* 31 */ tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/* 32 */ tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);	
			adc2ctr += 32; // Running total of number of ADC2 readings	
		}
		sumsq += isumsq; // 64b accumulation of squares	
		i += 1;
	}		
		return;
}

/* *************************************************************************
 * void sumsquaresfastest(void);
 * @brief	: Inline: subtract offset, sum readings, sum squares
 * *************************************************************************/
void sumsquaresfastest(void)
{
	while (pdma != pdma_end)
	{
		isumsq = 0; // 32b accumulation of sum squares
	/*  1 */  tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/*  2 */  tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/*  3 */  tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/*  4 */  tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/*  5 */  tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/*  6 */  tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/*  7 */  tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/*  8 */  tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/*  9 */  tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/* 10 */  tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/* 11 */  tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/* 12 */  tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/* 13 */  tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/* 14 */  tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/* 15 */  tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/* 16 */  tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/* 17 */  tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/* 18 */  tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/* 19 */  tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/* 20 */  tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/* 21 */  tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/* 22 */  tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/* 23 */  tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/* 24 */  tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/* 25 */  tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/* 26 */  tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/* 27 */  tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/* 28 */  tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/* 29 */  tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/* 30 */  tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/* 31 */  tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/* 32 */  tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		sumsq += isumsq; // 64b accumulation of squares	
	}	
	adc2ctr += 512; // Running total of number of ADC2 readings
	return;
}
void sumsquaresasm(uint16_t* pread, uint32_t nadcaccum, uint16_t noffset, uint32_t nsumsq, uint32_t n)
{
	int32_t ntmp;

 asm goto (
        "bx r7 "  // %l == lowercase L
        :
        :
        :
        : sumstart          // specify c label(s) here
    );	

//asm volatile(
//    "mov lr, %1\n\t"
//    "bx %0\n\t"
//    : : "r" (main), "r" (sumstart)); 

sumstart:	
	ntmp = *pread++; nadcaccum += ntmp; ntmp -= noffset; nsumsq += (ntmp * ntmp);
	ntmp = *pread++; nadcaccum += ntmp; ntmp -= noffset; nsumsq += (ntmp * ntmp);
	ntmp = *pread++; nadcaccum += ntmp; ntmp -= noffset; nsumsq += (ntmp * ntmp);
	ntmp = *pread++; nadcaccum += ntmp; ntmp -= noffset; nsumsq += (ntmp * ntmp);
	ntmp = *pread++; nadcaccum += ntmp; ntmp -= noffset; nsumsq += (ntmp * ntmp);
	ntmp = *pread++; nadcaccum += ntmp; ntmp -= noffset; nsumsq += (ntmp * ntmp);
	ntmp = *pread++; nadcaccum += ntmp; ntmp -= noffset; nsumsq += (ntmp * ntmp);
	ntmp = *pread++; nadcaccum += ntmp; ntmp -= noffset; nsumsq += (ntmp * ntmp);
	ntmp = *pread++; nadcaccum += ntmp; ntmp -= noffset; nsumsq += (ntmp * ntmp);
	ntmp = *pread++; nadcaccum += ntmp; ntmp -= noffset; nsumsq += (ntmp * ntmp);
	ntmp = *pread++; nadcaccum += ntmp; ntmp -= noffset; nsumsq += (ntmp * ntmp);
	ntmp = *pread++; nadcaccum += ntmp; ntmp -= noffset; nsumsq += (ntmp * ntmp);
	ntmp = *pread++; nadcaccum += ntmp; ntmp -= noffset; nsumsq += (ntmp * ntmp);
	ntmp = *pread++; nadcaccum += ntmp; ntmp -= noffset; nsumsq += (ntmp * ntmp);
	ntmp = *pread++; nadcaccum += ntmp; ntmp -= noffset; nsumsq += (ntmp * ntmp);
	ntmp = *pread++; nadcaccum += ntmp; ntmp -= noffset; nsumsq += (ntmp * ntmp);
	ntmp = *pread++; nadcaccum += ntmp; ntmp -= noffset; nsumsq += (ntmp * ntmp);
	ntmp = *pread++; nadcaccum += ntmp; ntmp -= noffset; nsumsq += (ntmp * ntmp);
	ntmp = *pread++; nadcaccum += ntmp; ntmp -= noffset; nsumsq += (ntmp * ntmp);
	ntmp = *pread++; nadcaccum += ntmp; ntmp -= noffset; nsumsq += (ntmp * ntmp);
	ntmp = *pread++; nadcaccum += ntmp; ntmp -= noffset; nsumsq += (ntmp * ntmp);
	ntmp = *pread++; nadcaccum += ntmp; ntmp -= noffset; nsumsq += (ntmp * ntmp);
	ntmp = *pread++; nadcaccum += ntmp; ntmp -= noffset; nsumsq += (ntmp * ntmp);
	ntmp = *pread++; nadcaccum += ntmp; ntmp -= noffset; nsumsq += (ntmp * ntmp);
	ntmp = *pread++; nadcaccum += ntmp; ntmp -= noffset; nsumsq += (ntmp * ntmp);
	ntmp = *pread++; nadcaccum += ntmp; ntmp -= noffset; nsumsq += (ntmp * ntmp);
	ntmp = *pread++; nadcaccum += ntmp; ntmp -= noffset; nsumsq += (ntmp * ntmp);
	ntmp = *pread++; nadcaccum += ntmp; ntmp -= noffset; nsumsq += (ntmp * ntmp);
	ntmp = *pread++; nadcaccum += ntmp; ntmp -= noffset; nsumsq += (ntmp * ntmp);
	ntmp = *pread++; nadcaccum += ntmp; ntmp -= noffset; nsumsq += (ntmp * ntmp);
	ntmp = *pread++; nadcaccum += ntmp; ntmp -= noffset; nsumsq += (ntmp * ntmp);
	ntmp = *pread++; nadcaccum += ntmp; ntmp -= noffset; nsumsq += (ntmp * ntmp);
	adcaccum = nadcaccum;
	isumsq += nsumsq;
	adc2ctr += 512;
	return;

}


void items(uint16_t* pdma, uint32_t adcaccum, uint16_t offset, uint32_t isumsq, uint32_t idx);

void sumsquaresgoto(void)
{
	while (pdma != pdma_end)
	{
		if (adc2dma_flag == 0)
		{
			items(pdma,adcaccum,offset,isumsq,0);
		}
		else
		{
			adc2dma_flag = 0;
			items(pdma,adcaccum,offset,isumsq,(32-blksubidx));
			loop32();
			items(pdma,adcaccum,offset,isumsq,(blksubidx - 1));
		}
		return;
	}
}

void items(uint16_t* pdma, uint32_t adcaccum, uint16_t offset, uint32_t isumsq, uint32_t idx)
{

	switch(idx)	{
case  1: tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
case  2: tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
case  3: tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
case  4: tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
case  5: tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
case  6: tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
case  7: tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
case  8: tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
case  9: tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
case  10: tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
case  11: tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
case  12: tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
case  13: tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
case  14: tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
case  15: tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
case  16: tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
case  17: tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
case  18: tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
case  19: tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
case  20: tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
case  21: tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
case  22: tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
case  23: tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
case  24: tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
case  25: tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
case  26: tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
case  27: tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
case  28: tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
case  29: tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
case  30: tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
case  31: tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
case  32: tmp = *pdma++; adcaccum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	}
	sumsq += isumsq; // 64b accumulation of squares	
	return;
}
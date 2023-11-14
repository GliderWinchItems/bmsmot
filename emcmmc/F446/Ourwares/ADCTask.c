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

static uint32_t adcsum;
static uint16_t* pdma_end;
static uint32_t isumsq;


uint32_t exti15dtw;
uint32_t exti15dtw_prev;
uint32_t exti15dtw_diff;
uint32_t exti15dtw_flag;
uint32_t exti15dtw_flag_prev;
uint32_t exti15dtw_irqctr;

uint64_t sumsq;      // Running accumulation of squares
uint64_t sumsq_prev; // Saved sumsq (i.e. "previous")
uint64_t sumsq_diff; // Difference: sumsqdiff = (new - previous)

uint64_t sumsq_ctr;      // Running count of readings
uint64_t sumsq_ctr_prev; // Saved sumq_ctr (i.e. "previous")
uint64_t sumsq_ctr_diff; // Difference: sumsqdiff = (new - previous)

#define OFFSET 1884
uint16_t offset = OFFSET; // Initial = calibration offset

/* Location of in DMA buffer where zero crossing took place. */
uint16_t blkidx;    // Block (chunk) index (1 - 16)
uint16_t blksubidx; // Within block index (0 - 31)

static uint16_t* pdma;
static 	int32_t tmp;

uint32_t debugdma;
/* *************************************************************************
 * void StartADCTask(void const * argument);
 *	@brief	: Task startup
 * *************************************************************************/
uint16_t* p16;

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
			if ((noteval & TSK02BIT04) != 0)
			{ // Here, dma half complete notification
				pdma = adcdmatskblk[1].pdma1;
				if (adc2dma_flag != 0)
				{
					adc2dma_flag = 0;
					adc2dma_cnt = (1024 - adc2dma_cnt);
					if (adc2dma_cnt >= 512) 
						adc2dma_cnt = 1023;// morse_trap(7111);
					else if (adc2dma_cnt <= 0)
						adc2dma_cnt = 1;
				}
			}
			else
			{ // Here, dma full complete notification
				pdma = adcdmatskblk[1].pdma2;
				if (adc2dma_flag != 0)
				{ // Zero crossing interrupt occured during dma of this buff half
					adc2dma_flag = 0;
					adc2dma_cnt = (512 - adc2dma_cnt);
					if (adc2dma_cnt < 1) 
						adc2dma_cnt = 512;// morse_trap(7222);
					if (adc2dma_cnt == 0) 
						adc2dma_cnt = 1;// morse_trap(7222);
				}
			}
		uint16_t* pdma_save = pdma;			

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

			pdma = pdma_save; // Restore dma ptr

//debugadc1t = DTWTIME;
			pdma_end = pdma + (32*16); // DMA 1/2 buffer end address
			isumsq = 0; // 32b sum of squares
			adcsum = 0; // 32b readings sum
			/* Sum squares for buffer (inline code) */
blkidx = 1000; // Skip EXTI marker for now
			if (blkidx == 0)
			{ // Here, no EXTI interrupt occured during this buffer loading
debugadc1t = DTWTIME;				
				sumsquaresfastest(); // No zero crossings in buffer
debugadc2t = DTWTIME - debugadc1t;				
			}
			else
			{ // Here, EXTI interrupt signals start/end of input AC
debugadc1t = DTWTIME;				
				sumsquareswithif(); // Zero crossings
debugadc2t = DTWTIME - debugadc1t;				
			}
			sumsq += isumsq; // Accum 64b sum squares
		}

		if ((noteval & TSK02BIT02) || (noteval & TSK02BIT03))
		{ // ADC1 notification
			if (noteval & TSK02BIT02)
			{
				pdma = adcdmatskblk[0].pdma1;
//	p16 = pdma;			
			}
			else
			{
				pdma = adcdmatskblk[0].pdma2;
			}

			// Sum 16 12 item scans from 1/2 DMA buffer. */
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

			pz = &adc1.chan[0];

	/* WOW. Optimizer does the following with one cycle per statement! */
// NB: use a memory-memory dma here?
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
void EXTI15_IRQHandler(void)
{

	EXTI->PR = EXTI15REG
	exti15dtw = DTWTIME;
	if ((exti15dtw - exti15dtw_prev) > 10000)
	{
//exti15dtw_reg = *(uint32_t*)((uint8_t*)(0x40026400 + 0x14 + (0x18*2)));//*preg;

	adc2dma_cnt = *(uint32_t*)(&hdma_adc2.Instance->NDTR);
	adc2dma_flag = 1;		

		exti15dtw_diff = exti15dtw - exti15dtw_prev;
		exti15dtw_prev = exti15dtw;
		exti15dtw_flag += 1;
	}
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
		sumsq += *(pdma + j) * *(pdma + j);
		if (blksubidx == j)
		{
			sumsq_diff = sumsq - sumsq_prev;
			sumsq_prev = sumsq;
		}
	}
	return;
}

/* *************************************************************************
 * void sumsquareswithif(void);
 * @brief	: Inline sum squares with"if" between chunks
 * *************************************************************************/
void sumsquareswithif(void)
{
	int i = 1;
	while (pdma != pdma_end)
	{
		if (blkidx == i)
			loop32();
		else
		{
		/*  1 */ tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/*  2 */ tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/*  3 */ tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/*  4 */ tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/*  5 */ tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/*  6 */ tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/*  7 */ tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/*  8 */ tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/*  9 */ tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/* 10 */ tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/* 11 */ tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/* 12 */ tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/* 13 */ tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/* 14 */ tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/* 15 */ tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/* 16 */ tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/* 17 */ tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/* 18 */ tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/* 19 */ tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/* 20 */ tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/* 21 */ tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/* 22 */ tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/* 23 */ tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/* 24 */ tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/* 25 */ tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/* 26 */ tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/* 27 */ tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/* 28 */ tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/* 29 */ tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/* 30 */ tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/* 31 */ tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		/* 32 */ tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
		}
		i += 1;
	}		
		return;
}

/* *************************************************************************
 * void sumsquaresfastest(void);
 * @brief	: Inline sum squares with"if" between chunks
 * *************************************************************************/
void sumsquaresfastest(void)
{
// ##########################################################################

	while (pdma != pdma_end)
	{
	/*  1 */  tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/*  2 */  tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/*  3 */  tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/*  4 */  tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/*  5 */  tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/*  6 */  tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/*  7 */  tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/*  8 */  tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/*  9 */  tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/* 10 */  tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/* 11 */  tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/* 12 */  tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/* 13 */  tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/* 14 */  tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/* 15 */  tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/* 16 */  tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/* 17 */  tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/* 18 */  tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/* 19 */  tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/* 20 */  tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/* 21 */  tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/* 22 */  tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/* 23 */  tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/* 24 */  tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/* 25 */  tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/* 26 */  tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/* 27 */  tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/* 28 */  tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/* 29 */  tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/* 30 */  tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/* 31 */  tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);
	/* 32 */  tmp = *pdma; pdma += 1; adcsum += tmp; tmp -= offset; isumsq += (tmp * tmp);

	}	
	return;
}

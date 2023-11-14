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

#endif


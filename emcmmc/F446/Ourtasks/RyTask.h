/******************************************************************************
* File Name          : RyTask.h
* Date First Issued  : 06/14/2023
* Description        : Relay Task
*******************************************************************************/


#ifndef __RYTASK
#define __RYTASK

#include <stdint.h>
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "EMCTask.h"

/* Keep-Alive timer (TIM5) OC interrupt rate */
// Prescalar = 9000; Sysclock = 180 MHz
#define KPUPDATEDUR 100 // 10 ms = 100 * 0.1ms

/* Relay request */
struct RYREQ_Q
{
	uint8_t idx;  // Relay table index (0 - 11)
	uint8_t pwm;  // Percent: 0 - 100; 255 = use parameter value
	uint8_t cancel; // 1 = cancel request
};

/* *************************************************************************/
 TaskHandle_t xRyTaskCreate(uint32_t taskpriority);
/* @brief	: Create task; task handle created is global for all to enjoy!
 * @param	: taskpriority = Task priority (just as it says!)
 * @return	: RyTaskHandle
 * *************************************************************************/
 void ry_init(void);
/*	@brief	: init 
 * *************************************************************************/

 /* ############################################################################# */
 void TIM5_IRQ_Handler(void);  // Keep-alive timer
 void TIM13_IRQ_Handler(void); // Pull-in delay timer
 void TIM9_IRQ_Handler(void);
 /* ############################################################################# */

extern TaskHandle_t RyTaskHandle;
extern osMessageQId RyTaskReadReqQHandle;

#endif
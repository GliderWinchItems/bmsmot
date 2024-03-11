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
#include "common_can.h"
//#include "EMCTask.h"

// Relay header versus RyTask.c coding
#define HDR_OA1 0  // Group A
#define HDR_OA2 1  // Group A
#define HDR_OA3 2  // Group A
#define HDR_OA4 3  // Group A
#define HDR_OB1 4  // Group B
#define HDR_OB2 5  // Group B
#define HDR_OB3 6  // Group B
#define HDR_OB4 7  // Group B
#define HDR_OC1 8  // Group C
#define HDR_OC2 9  // Group C
#define HDR_OC3 10 // Group C
#define HDR_OC4 11 // Group C

// Relay assignments for EMCMMC local
#define RYASGN_CNTCR_HVP    HDR_OB1 // HV plus contactor coil (no pwm)
#define RYASGN_CNTCR_HVM    HDR_OB2 // HV minus contactor coil (no pwm)
#define RYASGN_SEL_AC_SRC   HDR_OA2 // Select AC source relay
#define RYASGN_SEL_RSP_ACDC HDR_OA3 // Select RSP input DC(AC) relay
#define RYASGN_SEL_AUX_RS   HDR_OA4 // Select AUX R S (?) relay

#define NRELAYS 12 // Number of fet driver relay outputs

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
void RyTask_CANpayload(struct CANRCVBUF* pcan);
/*	@brief	: Fill CAN payload with relay status and info
 *  @param  : pointer to CAN msg
 * *************************************************************************/

 /* ############################################################################# */
 void TIM5_IRQ_Handler(void);  // Keep-alive timer
 void TIM13_IRQ_Handler(void); // Pull-in delay timer
 void TIM9_IRQ_Handler(void);
 /* ############################################################################# */

extern TaskHandle_t RyTaskHandle;
extern osMessageQId RyTaskReadReqQHandle;

#endif
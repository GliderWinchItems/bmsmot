/******************************************************************************
* File Name          : StringChgrTask.h
* Date First Issued  : 09/06/2024
* Description        : Battery string charging
*******************************************************************************/


#ifndef __STRINGCHGRTASK
#define __STRINGCHGRTASK

#include <stdint.h>
#include "FreeRTOS.h"
#include "cmsis_os.h"
//#include "stm32f4xx_hal.h"

/* Status1 byte: Reporting status. */
#define SCTSTATUS_EXP (1<<0) // Expected number reporting
#define SCTSTATUS_OVF (1<<1) // Received more than table size
#define SCTSTATUS_FWR (1<<2) // Fewer than expected reporting
#define SCTSTATUS_MOR (1<<3) // More than expected reporting
#define SCTSTATUS_NR  (1<<4) // Missing node readings, or not up-to-date

/*
pay[0] [4:0] Status bits (see #define above)
pay[1] 

pay[4]-[7] (float) String voltage (Volts)

*/



/* Notification bits */
#define STRINGCHRGBIT00 (1<<0) // GatewayTask
#define STRINGCHRGBIT01 (1<<1) // RTOS timer swtim1

/* Table with element for each BMS node on string. */
#define BMSTABLESIZE 7 // Max size of table

/* Working struct for EMC local function. */
struct STRINGCHGRFUNCTION
{
	uint32_t hbct_t;   // Duration between STRINGCHGR function heartbeats(ms)
	uint32_t hbct_tic; // Loop tick count between heartbeats
	int32_t  hbct_ctr; // Heartbeat time count-down

	uint8_t bmsnum_expected; // Number of BMS nodes expected to be reporting

};

/* *************************************************************************/
 TaskHandle_t xStringChgrTaskCreate(uint32_t taskpriority);
/* @brief	: Create task; task handle created is global for all to enjoy!
 * @param	: taskpriority = Task priority (just as it says!)
 * @return	: ChgrTaskHandle
 * *************************************************************************/

extern TaskHandle_t StringChgrTaskHandle;


#endif
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

/* Notification bits */
#define STRINGCHRGBIT00 (1<<0) //
#define STRINGCHRGBIT01 (1<<0) //

/* Working struct for EMC local function. */
struct STRINGCHGRFUNCTION
{
	uint32_t hbct_t;   // Duration between STRINGCHGR function heartbeats(ms)
	uint32_t hbct_tic; // Loop tick count between heartbeats
	int32_t  hbct_ctr; // Heartbeat time count-down
};

/* *************************************************************************/
 TaskHandle_t xStringChgrTaskCreate(uint32_t taskpriority);
/* @brief	: Create task; task handle created is global for all to enjoy!
 * @param	: taskpriority = Task priority (just as it says!)
 * @return	: ChgrTaskHandle
 * *************************************************************************/

extern TaskHandle_t StringChgrTaskHandle;

#endif
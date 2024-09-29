/******************************************************************************
* File Name          : ContactorTask.c
* Date First Issued  : 09/19/2024
* Description        : Contactor via bmsmot board
*******************************************************************************/

#ifndef __CONTACTORTASK
#define __CONTACTORTASK

#include <stdint.h>
#include "FreeRTOS.h"
#include "cmsis_os.h"
//#include "stm32l4xx_hal.h"
#include "emcl_idx_v_struct.h"
#include "CanTask.h"
#include "adc_idx_v_struct.h"

/* Working struct for EMC local function. */
struct CONTACTORFUNCTION
{
   // Parameter loaded either by high-flash copy, or hard-coded subroutine
	struct EMCLLC lc; // Fixed parameters, (lc = Local Copy)


	/* CAN msgs */
	struct CANTXQMSG canmsg;
};
/* *************************************************************************/
osThreadId xContactorTaskCreate(uint32_t taskpriority);
/* @brief	: Create task; task handle created is global for all to enjoy!
 * @param	: taskpriority = Task priority (just as it says!)
 * @return	: ContactorTaskHandle
 * *************************************************************************/

 extern TaskHandle_t ContactorTaskHandle;
 extern struct CONTACTORFUNCTION contactorfunction;



#endif


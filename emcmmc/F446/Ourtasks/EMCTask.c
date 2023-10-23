/******************************************************************************
* File Name          : EMCTask.c
* Date First Issued  : 06/14/2023
* Description        : EMC task for bmsmot pcb
*******************************************************************************/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "malloc.h"

#include "EMCTask.h"
#include "morse.h"

#include "main.h"
#include "DTW_counter.h"
#include "iir_f1.h"
//#include "emc_items.h"
#include "rtcregs.h"

/* Function struct */
struct EMCFUNCTION emcfunction;

/* Queue */
#define QUEUESIZE 16	// Total size of bcb's tasks can queue up
osMessageQId EMCTaskReadReqQHandle;

struct EMCREQ_Q* pssb; // Pointer to struct for request details

TaskHandle_t EMCTaskHandle;


/* *************************************************************************
 * void StartEMCTask(void const * argument);
 *	@brief	: Task startup
 * *************************************************************************/
uint8_t dbgka;
void StartEMCTask(void *argument)
{
	BaseType_t ret;

	TickType_t tickref = xTaskGetTickCount();
	TickType_t tickwait;

   	/* Infinite loop */
  	for(;;)
  	{
  		/* Once per second with no slippage due to loop delay. */
  		tickref += 500;//1000; //pdMS_TO_TICKS(1000);
  		tickwait = tickref - xTaskGetTickCount();

  		/* Check queue of loaded items. */
		ret = xQueueReceive(EMCTaskReadReqQHandle,&pssb,tickwait);
		if (ret == pdPASS)
		{ // Request arrived
		}
	}
}

/* *************************************************************************
 * osThreadId xEMCTaskCreate(uint32_t taskpriority);
 * @brief	: Create task; task handle created is global for all to enjoy!
 * @param	: taskpriority = Task priority (just as it says!)
 * @return	: EMCTaskHandle
 * *************************************************************************/
osThreadId xEMCTaskCreate(uint32_t taskpriority)
{
	BaseType_t xRet;

	xRet = xTaskCreate(
		StartEMCTask,     /* Function that implements the task. */
		"EMCTask",        /* Text name for the task. */
		256,              /* Stack size in words, not bytes. */
		NULL,             /* Parameter passed into the task. */
		taskpriority,     /* Priority at which the task is created. */
		&EMCTaskHandle ); /* Used to pass out the created task's handle. */ 

	if( xRet != pdPASS )return NULL;

	EMCTaskReadReqQHandle = xQueueCreate(QUEUESIZE, sizeof(struct EMCREQ_Q*) );
	if (EMCTaskReadReqQHandle == NULL) return NULL;

   	return EMCTaskHandle;	
}

/******************************************************************************
* File Name          : ContactorTask.c
* Date First Issued  : 09/19/2024
* Description        : Contactor via bmsmot board
*******************************************************************************/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "malloc.h"

#include "main.h"
#include "morse.h"
#include "ContactorTask.h"
#include "emcl_idx_v_struct.h"
#include "EMCLTask.h"
#include "DTW_counter.h"

TaskHandle_t ContactorTaskHandle = NULL;

struct CONTACTORFUNCTION contactorfunction;

/* *************************************************************************
 * void StartContactorTTask(void const * argument);
 *	@brief	: Task startup
 * *************************************************************************/
void StartContactorTTask(void* argument)
{
	emcl_idx_v_struct_hardcode_params(&emclfunction.lc);

	for (;;)
	{
#if 0
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // GRN ON
    	osDelay(30);
	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);   // GRN OFF		
#endif
		osDelay(1901-30);
	}

}
/* *************************************************************************
 * TaskHandle_t xContactorTaskCreate(uint32_t taskpriority);
 * @brief	: Create task; task handle created is global for all to enjoy!
 * @param	: taskpriority = Task priority (just as it says!)
 * @return	: ContactorTaskHandle
 * *************************************************************************/
TaskHandle_t xContactorTaskCreate(uint32_t taskpriority)
{
	BaseType_t ret = xTaskCreate(StartContactorTTask, "ContactorTask",\
     (128), NULL, taskpriority,\
     &ContactorTaskHandle);
	if (ret != pdPASS) return NULL;
	return ContactorTaskHandle;
}

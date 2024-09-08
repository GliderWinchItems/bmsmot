/******************************************************************************
* File Name          : EMCLTask.c
* Date First Issued  : 10/29/2023
* Description        : Energy Management Control Local task
*******************************************************************************/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "malloc.h"

#include "main.h"
#include "morse.h"
#include "EMCLTask.h"
#include "emcl_idx_v_struct.h"
#include "DTW_counter.h"

TaskHandle_t EMCLTaskHandle = NULL;

struct EMCLFUNCTION emclfunction;

/* *************************************************************************
 * void StartEmclTask(void const * argument);
 *	@brief	: Task startup
 * *************************************************************************/
void StartEmclTask(void* argument)
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
 * TaskHandle_t xEMCLTaskCreate(uint32_t taskpriority);
 * @brief	: Create task; task handle created is global for all to enjoy!
 * @param	: taskpriority = Task priority (just as it says!)
 * @return	: EMCLTaskHandle
 * *************************************************************************/
TaskHandle_t xEMCLTaskCreate(uint32_t taskpriority)
{
	BaseType_t ret = xTaskCreate(StartEmclTask, "EMCLTask",\
     (128), NULL, taskpriority,\
     &EMCLTaskHandle);
	if (ret != pdPASS) return NULL;
	return EMCLTaskHandle;
}

/******************************************************************************
* File Name          : CoolingTask.c
* Date First Issued  : 12/21/2023
* Description        : Coolant and fan control
*******************************************************************************/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "malloc.h"

#include "main.h"
#include "morse.h"
#include "CoolingTask.h"
#include "emcl_idx_v_struct.h"
#include "DTW_counter.h"
#include "EMCLTask.h"

TaskHandle_t CoolingTaskHandle = NULL;

/* *************************************************************************
 * void StartCoolingTask(void const * argument);
 *	@brief	: Task startup
 * *************************************************************************/
void StartCoolingTask(void* argument)
{
	emcl_idx_v_struct_hardcode_params(&emclfunction.lc);

	for (;;)
	{
#if 0
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // GRN ON
    	osDelay(30);
	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET); // GRN OFF		
#endif
		osDelay(1901-30);
	}

}
/* *************************************************************************
 * TaskHandle_t xCoolingTaskCreate(uint32_t taskpriority);
 * @brief	: Create task; task handle created is global for all to enjoy!
 * @param	: taskpriority = Task priority (just as it says!)
 * @return	: RyTaskHandle
 * *************************************************************************/
TaskHandle_t xCoolingTaskCreate(uint32_t taskpriority)
{
	BaseType_t ret = xTaskCreate(StartCoolingTask, "CoolingTask",\
     (128), NULL, taskpriority,\
     &CoolingTaskHandle);
	if (ret != pdPASS) return NULL;
	return CoolingTaskHandle;
}

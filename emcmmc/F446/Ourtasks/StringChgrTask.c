/******************************************************************************
* File Name          : StringChgrTask.c
* Date First Issued  : 09/06/2024
* Description        : Battery string charging
*******************************************************************************/

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "malloc.h"

#include "main.h"
#include "morse.h"
#include "getserialbuf.h"
#include "SerialTaskSend.h"
#include "yprintf.h"

#include "StringChgrTask.h"
#include "GatewayTask.h"
#include "stringchgr_items.h"


extern UART_HandleTypeDef huart3;



TaskHandle_t StringChgrTaskHandle = NULL;
/* *************************************************************************
 * void StartStringChgrTask(void const * argument);
 *	@brief	: Task startup
 * *************************************************************************/
uint32_t dbgS1;				
uint8_t dbgalt;

void StartStringChgrTask(void* argument)
{
	uint32_t noteval;
//	struct CANRCVBUF* pcan;
	struct CANRCVBUFS* pcans;

/* Setup serial output buffers for uarts. */
	struct SERIALSENDTASKBCB* pbuf1 = getserialbuf(&HUARTMON,  96); // PC monitor uart	
/*
#define LED5_GRN_Pin GPIO_PIN_12
#define LED5_GRN_GPIO_Port GPIOB
#define LED6_RED_Pin GPIO_PIN_13
#define LED6_RED_GPIO_Port GPIOB
*/
	for (;;)
	{
		xTaskNotifyWait(0,0xffffffff, &noteval, 750-15);
		{
			if ((noteval & STRINGCHRGBIT00) != 0)
			{
dbgS1 += 1;				
				do
				{
					pcans = GatewayTask_takecan1();
					if (pcans != NULL)
					{
//yprintf(&pbuf1,"\tS1 %08X %d: %d\n\r",pcans->can.id,pcans->can.dlc,pcans->can.cd.uc[0]);
yprintf(&pbuf1,"\tS1 %d %08X %d: %d\n\r",pcans->sel, pcans->can.id,pcans->can.dlc,pcans->can.cd.uc[0]);

						if (pcans->sel == 0)
						{ // Here, a BMS node CAN msg
						//	bmsmsg(pcans);
						}

					}

				} while (pcans != NULL);


				/* Wink green led */
				if ((dbgalt++ & 1) == 0)
				{
					HAL_GPIO_WritePin(LED5_GRN_GPIO_Port,LED5_GRN_Pin,GPIO_PIN_RESET); // GRN LED
				}
				else
				{
					HAL_GPIO_WritePin(LED5_GRN_GPIO_Port,LED5_GRN_Pin,GPIO_PIN_SET); // GRN LED					
				}
			}
		}
	}		
}
    
/* *************************************************************************
 * TaskHandle_t xStringChgrTaskCreate(uint32_t taskpriority);
 * @brief	: Create task; task handle created is global for all to enjoy!
 * @param	: taskpriority = Task priority (just as it says!)
 * @return	: StringChgrTaskHandle
 * *************************************************************************/
TaskHandle_t xStringChgrTaskCreate(uint32_t taskpriority)
{

/*
BaseType_t xTaskCreate( TaskFunction_t pvTaskCode,
const char * const pcName,
unsigned short usStackDepth,
void *pvParameters,
UBaseType_t uxPriority,
TaskHandle_t *pxCreatedTask );
*/
	BaseType_t ret = xTaskCreate(StartStringChgrTask, "StringChgrTask",\
     (128), NULL, taskpriority,\
     &StringChgrTaskHandle);
	if (ret != pdPASS) return NULL;

	return StringChgrTaskHandle;
}


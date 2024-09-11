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
//	struct SERIALSENDTASKBCB* pbuf1 = getserialbuf(&HUARTMON,  96); // PC monitor uart	
/*
#define LED5_GRN_Pin GPIO_PIN_12
#define LED5_GRN_GPIO_Port GPIOB
#define LED6_RED_Pin GPIO_PIN_13
#define LED6_RED_GPIO_Port GPIOB
*/
	/* Init some things. */
	stringchgr_items_init();

	for (;;)
	{
		xTaskNotifyWait(0,0xffffffff, &noteval, 500-15);
		{
			if ((noteval & STRINGCHRGBIT00) != 0)
			{ // Here, gateway placed a CAN msg on circular buffer for us
dbgS1 += 1;				
				do // Loop until buffer emptied
				{ /* Gateway passes only CAN msgs needed for StringChgrTask. */
					pcans = GatewayTask_takecan1();
					if (pcans != NULL)
					{ // Here, pcans points to CAN msg in gateway's circular buffer
						/* Gateway selection adds a code, so no need to do compares. */
						if (pcans->pcl->code == C1SELCODE_BMS)
						{ // Here: code = BMS node CAN msg group
//yprintf(&pbuf1,"S %08X %d\n\r",pcans->can.id, pcans->pcl->code);
							do_tableupdate(pcans); // Build or update table of BMS nodes
						}
					}
				} while (pcans != NULL);
			}
			if (noteval == 0)
			{ /* Blink green led */
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


/******************************************************************************
* File Name          : StringChgrTask.c
* Date First Issued  : 09/06/2024
* Description        : Battery string charging
*******************************************************************************/

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "malloc.h"
#include "timers.h"

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
TimerHandle_t StringChgrTimerHandle;

uint32_t dbgS1;				
uint8_t dbgalt;

uint32_t swtim1_ctr; // Running count of swtim1 callbacks
static uint32_t swtim1_ctr_prev;
/* *************************************************************************
 * void swtim1_callback(TimerHandle_t tm);
 * @brief	: Software timer 1 timeout callback
 * *************************************************************************/
static void swtim1_callback(TimerHandle_t tm)
{
	swtim1_ctr += 1;
	xTaskNotify(StringChgrTaskHandle, STRINGCHRGBIT01, eSetBits);
	return;
}
/* *************************************************************************
 * void StartStringChgrTask(void const * argument);
 *	@brief	: Task startup
 * *************************************************************************/
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
#define SWTIME1PERIOD 10 // 10 ms ticks
#if 1
/* Create timer swtimer1 Auto-reload/periodic (128 per sec) */
	StringChgrTimerHandle = xTimerCreate("swtim1",SWTIME1PERIOD,pdTRUE,\
		(void *) 0, swtim1_callback);
	if (StringChgrTimerHandle == NULL) {morse_trap(404);}

	/* Start command/keep-alive timer */
	BaseType_t bret = xTimerReset(StringChgrTimerHandle, 10);
	if (bret != pdPASS) {morse_trap(405);}
#endif

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
						switch(pcans->pcl->code)
						{ 
						case C1SELCODE_BMS: // BMS node CAN msg group
							do_tableupdate(pcans); // Build or update table of BMS nodes
							break;
					
						case C1SELCODE_ELCON: // ELCON msg
							// do_elcon(pcans);
							
yprintf(&pbuf1,"%d %08X %d\n\r",swtim1_ctr-swtim1_ctr_prev,pcans->can.id,pcans->can.dlc);
swtim1_ctr_prev = swtim1_ctr;
							break;
						}
					}
				} while (pcans != NULL);
			}
			if ((noteval & STRINGCHRGBIT01) != 0)
			{ // Here, FreeRTOS timer (swtim1) callback 
static uint32_t dbgt1;
				dbgt1 += 1;
				if (dbgt1 >= 100)
				{
					dbgt1 = 0;
//yprintf(&pbuf1,"swtim1 tic\n\r");					
				}

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


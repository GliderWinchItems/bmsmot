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
extern struct CAN_CTLBLOCK* pctl0; // Pointer to CAN1 control block

TaskHandle_t StringChgrTaskHandle = NULL;
TimerHandle_t StringChgrTimerHandle;

uint32_t dbgS1;				
uint8_t dbgalt;

uint32_t swtim1_ctr; // Running count of swtim1 callbacks

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
 * @brief	: Task startup
 * *************************************************************************/
void StartStringChgrTask(void* argument)
{
	uint32_t noteval;
	struct CANRCVBUFS* pcans;
	struct STRINGCHGRFUNCTION* p = &emclfunction.lc.lcstring;

uint16_t tmpv;
uint16_t tmpa;

/* Setup serial output buffers for debugging uart */
	struct SERIALSENDTASKBCB* pbuf1 = getserialbuf(&HUARTMON,  96); // PC monitor uart	
//	struct SERIALSENDTASKBCB* pbuf2 = getserialbuf(&HUARTMON,  96); // PC monitor uart	
/*

uint16_t tmpv
#define LED5_GRN_Pin GPIO_PIN_12
#define LED5_GRN_GPIO_Port GPIOB
#define LED6_RED_Pin GPIO_PIN_13
#define LED6_RED_GPIO_Port GPIOB
*/
	/* Init some things. */
	stringchgr_items_init();

	/* Pre-load fixed data id CAN msg to be sent. */
	// CAN msg for sending ELCON commands
	p->canelcon.pctl       = pctl0; // Control block for CAN module (CAN 1)
	p->canelcon.maxretryct = 4;
	p->canelcon.bits       = 0;
	p->canelcon.can.cd.ull = 0; // Clear playload [5-7] reserved
	p->canelcon.can.dlc    = 8; // ELCON always 8

	// CAN control block & msg for polling BMS
	p->canbms.pctl         = pctl0; // Control block for CAN module (CAN 1)
	p->canbms.maxretryct   = 4;
	p->canbms.bits         = 0;
	p->canbms.can.cd.ull   = 0; // Clear playload [5-7] reserved
	p->canbms.can.dlc      = 8; // default size
	p->canbms.can.cd.uc[0] = CMD_CMD_TYPE2;	
	p->canbms.can.cd.uc[1] = (0x3 << 6); // All nodes respond

	p->chgr_rate[0]  = 0; // JIC: Must be zero
	p->chgr_rate_idx = 0; // Charge current step: current = 0;
	p->bmsnum        = 0; // Discovered number of BMS nodes

	/* Create timer swtimer1 Auto-reload/periodic */
	StringChgrTimerHandle = xTimerCreate("swtim1",
		SWTIME1PERIOD,
		pdTRUE,(void *) 0, swtim1_callback);
	if (StringChgrTimerHandle == NULL) {morse_trap(404);}

	/* Start timer */
	BaseType_t bret = xTimerReset(StringChgrTimerHandle, 10);
	if (bret != pdPASS) {morse_trap(405);}	

	for (;;)
	{
		xTaskNotifyWait(0,0xffffffff, &noteval, portMAX_DELAY);
		{
			/* ============ incoming CAN msg on circular buffer ================== */			
			if ((noteval & STRINGCHRGBIT00) != 0)
			{ // Here, gateway placed a CAN msg for us on circular buffer
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
							do_bms_status_check();
							break;
					
						case C1SELCODE_ELCON: // ELCON msg
							do_elcon(pcans);
							
tmpv = __REVSH(pcans->can.cd.us[0]);
tmpa = __REVSH(pcans->can.cd.us[1]);
yprintf(&pbuf1,"%08X %d V.1 %d A.1  0x%02X\n\t",pcans->can.id,tmpv,tmpa,pcans->can.cd.uc[4]);

							break;

						case C1SELCODE_EMC_CMDS: // '98000000'PC or '98200000'EMC 
							do_emc_cmds(pcans);
							break;

						case C1SELCODE_POLLS: // PC, EMC1, EMC2 polling
							// todo();
							break;				
						}
					}
				} while (pcans != NULL);
			}
			/* ================== RTOS timer tick ================== */			
			if ((noteval & STRINGCHRGBIT01) != 0)
			{ // Here, FreeRTOS timer (swtim1) callback 
				/* Timeout checks: countdown counters. */
				do_timeoutcheck();
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
     (192), NULL, taskpriority,\
     &StringChgrTaskHandle);
	if (ret != pdPASS) return NULL;

	return StringChgrTaskHandle;
}


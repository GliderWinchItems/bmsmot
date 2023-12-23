/******************************************************************************
* File Name          : CoolingTask.c
* Date First Issued  : 12/21/2023
* Description        : Coolant and fan control
*******************************************************************************/
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "cmsis_os.h"
#include "malloc.h"

#include "main.h"
#include "morse.h"
#include "CoolingTask.h"
#include "emcl_idx_v_struct.h"
#include "DTW_counter.h"
#include "EMCLTask.h"
#include "../../../../GliderWinchCommons/embed/svn_common/trunk/db/gen_db.h"


#define COOLTIMERMS 100 // Number ms per timer callback

#define COOLCANBIT00 (1<<0) // RTOS timer 
#define COOLCANBIT01 (1<<2) // CAN msg: actual torq
#define COOLCANBIT02 (1<<3) // CAN msg: temperatures 
#define COOLCANBIT03 (1<<4) // CAN msg: launch state

extern struct CAN_CTLBLOCK* pctl0; // Pointer to CAN1 control block
extern CAN_HandleTypeDef hcan1;

uint32_t dbgcool1;

TaskHandle_t CoolingTaskHandle = NULL;

static TimerHandle_t xCoolTimerHandle;
/* *************************************************************************
 * void CoolTimerCallback( TimerHandle_t xTimer );
 *	@brief	: Timer callback
 * *************************************************************************/
void CoolTimerCallback( TimerHandle_t xTimer )
{
	xTaskNotify(CoolingTaskHandle, COOLCANBIT00, eSetBits ); 
	return;
}
/* *************************************************************************
 * void timer_do(struct COOLINGFUNCTION* p);
 *	@brief	: Timer tick doing
 * *************************************************************************/
void timer_do(struct COOLINGFUNCTION* p)
{
	// CAN msgs timeout
	if (p->timeout_CANdmoc_ctr > 0)
	{
		p->timeout_CANdmoc_ctr -= 1;
	}
	if (p->timeout_mcstate_ctr > 0)
	{
		p->timeout_mcstate_ctr -= 1;
	}
	return;
}
/* *************************************************************************
 * void StartCoolingTask(void const * argument);
 *	@brief	: Task startup
 * *************************************************************************/
void StartCoolingTask(void* argument)
{
	struct COOLINGFUNCTION* p = &emclfunction.lc.lccool; // Convenience pointer
	struct CANRCVBUF* pcan;
	uint32_t noteval; // Notification

	// Setup parameters
	emcl_idx_v_struct_hardcode_params(&emclfunction.lc); // JIC
	/* Add CAN Mailboxes                               CAN     CAN ID             TaskHandle,Notify bit,Skip, Paytype */
	//47400000','DMOC',1,1,'I16','DMOC: Actual Torque: payload-30000'
    p->pmbx_cid_dmoc_actualtorq      = MailboxTask_add(pctl0,p->cid_dmoc_actualtorq, NULL, COOLCANBIT01,0,U8); 
    if (p->pmbx_cid_dmoc_actualtorq == NULL) morse_trap(650);
	//'CA200000','DMOC',1,1,'U8_U8_U8''DMOC: Temperature:rotor,invert,stator'
    p->pmbx_cid_dmoc_hv_temps      = MailboxTask_add(pctl0,p->cid_dmoc_hv_temps,   NULL, COOLCANBIT02,0,U8);
    if (p->pmbx_cid_dmoc_hv_temps == NULL) morse_trap(651);
	//'26000000','MC',1,5,'U8_U8','MC: Launch state msg'    
    p->pmbx_cid_mc_state      = MailboxTask_add(pctl0,p->cid_mc_state,   NULL, COOLCANBIT03,0,U8);
    if (p->pmbx_cid_mc_state == NULL) morse_trap(652);	

    /* Create and start 100 ms timer notifications */
	xCoolTimerHandle = xTimerCreate("Cooltimer", 
		pdMS_TO_TICKS(COOLTIMERMS), /*const TickType_t xTimerPeriod, */
		pdTRUE, /*const UBaseType_t uxAutoReload, */
		( void * ) 0, /*void * const pvTimerID, */
		&CoolTimerCallback );/*TimerCallbackFunction_t pxCallbackFunction );*/

	BaseType_t ret = xTimerStart(xCoolTimerHandle,0);
	if (ret != pdPASS) morse_trap(653);

	for (;;)
	{
		xTaskNotifyWait(0,0xffffffff, &noteval, ~0L);
		if ((noteval & COOLCANBIT00) != 0) // RTOS 100 ms timer tick
		{
			timer_do(p);
		}
		if ((noteval & COOLCANBIT01) != 0) // CAN msg: Actual torq
		{ //     
			pcan = &p->pmbx_cid_dmoc_actualtorq->ncan.can;
		}
		if ((noteval & COOLCANBIT02) != 0) // CAN msg: DMOC temperatures
		{ //     
			p->timeout_CANdmoc_ctr = p->timeout_CANdmoc; // Reset timeout ctr
			pcan = &p->pmbx_cid_dmoc_hv_temps->ncan.can;
		}
		if ((noteval & COOLCANBIT03) != 0) // CAN msg: launch state
		{ //     
			p->timeout_mcstate_ctr = p->timeout_mcstate; // Reset timeout ctr
			pcan = &p->pmbx_cid_mc_state->ncan.can;
		}



		dbgcool1 += 1;
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

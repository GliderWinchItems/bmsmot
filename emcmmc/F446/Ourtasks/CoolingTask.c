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

#define COOLCANBIT00 (1<<0) // RTOS timer 
#define COOLCANBIT01 (1<<2) // CAN msg: actual torq
#define COOLCANBIT02 (1<<3) // CAN msg: temperatures 
#define COOLCANBIT03 (1<<4) // CAN msg: launch state

void extract_dmoc_hv_temps(struct COOLINGFUNCTION* p, struct CANRCVBUF* pcan);
static void motorcontrol_init(struct COOLINGFUNCTION* p);
void do_ramp(struct MOTORRAMP* p);

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

	motorcontrol_init(p);	

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
		if ((noteval & COOLCANBIT00) != 0) // RTOS 50 ms timer tick
		{
			timer_do(p);
		}

		/* CAN msgs */
		if ((noteval & COOLCANBIT01) != 0) // CAN msg: Actual torq
		{ //     
			pcan = &p->pmbx_cid_dmoc_actualtorq->ncan.can;
		}
		if ((noteval & COOLCANBIT02) != 0) // CAN msg: DMOC temperatures
		{ //     
			p->timeout_CANdmoc_ctr = p->timeout_CANdmoc; // Reset timeout ctr
			pcan = &p->pmbx_cid_dmoc_hv_temps->ncan.can;
			extract_dmoc_hv_temps(p,pcan);
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
#if 0
/* ***********************************************************************************************************
 * void dmoc_control_GEVCUBIT14(struct DMOCCTL* pdmocctl, struct CANRCVBUF* pcan);
 * @brief	: CAN msg received: cid_dmoc_actualtorq
 * @param	: pdmocctl = pointer to struct with "everything" for this DMOC unit
 * @param	: pcan = pointer to CAN msg struct
 ************************************************************************************************************* */
void dmoc_control_GEVCUBIT14(struct DMOCCTL* pdmocctl, struct CANRCVBUF* pcan)
{
/*cid_dmoc_hv_temps,  NULL,GEVCUBIT14,0,U8_U8_U8); */
/* 0x651 CANID_DMOC_HV_TEMPS:  U8_U8_U8,  'DMOC: Temperature:rotor,invert,stator */

/*       RotorTemp = frame->data.bytes[0];
        invTemp = frame->data.bytes[1];
        StatorTemp = frame->data.bytes[2];
        temperatureInverter = (invTemp-40) *10;
        //now pick highest of motor temps and report it
        if (RotorTemp > StatorTemp) {
            temperatureMotor = (RotorTemp - 40) * 10;
        }
        else {
            temperatureMotor = (StatorTemp - 40) * 10;
        }
        activityCount++; */

	pdmocctl->rotortemp   = pcan->cd.uc[0];
	pdmocctl->invtemp     = pcan->cd.uc[1];
	pdmocctl->statortemp  = pcan->cd.uc[2];
	pdmocctl->invtempcalc = (pdmocctl->invtemp - 40) * 10;
	if (pdmocctl->rotortemp > pdmocctl->statortemp)
	{
		pdmocctl->motortemp = (pdmocctl->rotortemp - 40) * 10;
	}
	else
	{
		pdmocctl->motortemp = (pdmocctl->statortemp - 40) * 10;
	}
	pdmocctl->activityctr += 1;
	return;
}
/* ***********************************************************************************************************
 * void dmoc_control_GEVCUBIT08(struct DMOCCTL* pdmocctl, struct CANRCVBUF* pcan);
 * @brief	: CAN msg received: cid_dmoc_actualtorq
 * @param	: pdmocctl = pointer to struct with "everything" for this DMOC unit
 * @param	: pcan = pointer to CAN msg struct
 ************************************************************************************************************* */
void dmoc_control_GEVCUBIT08(struct DMOCCTL* pdmocctl, struct CANRCVBUF* pcan)
{
/* 0x23A CANID_DMOC_ACTUALTORQ:I16,   DMOC: Actual Torque: payload-30000 */
	/* Extract reported torque and update latest reading. */
//				torqueActual = ((frame->data.bytes[0] * 256) + frame->data.bytes[1]) - 30000;
	pdmocctl->torqueact = ((pcan->cd.uc[0] << 8) + (pcan->cd.uc[1])) - pdmocctl->torqueoffset;
	return;
}
#endif
/* ***********************************************************************************************************
 * void extract_dmoc_hv_temps(struct COOLINGFUNCTION* p, struct CANRCVBUF* pcan);
 * @brief	: CAN msg received: cid_dmoc_hv_temps
 * @param	: pcan = pointer to CAN msg struct
 ************************************************************************************************************* */
void extract_dmoc_hv_temps(struct COOLINGFUNCTION* p, struct CANRCVBUF* pcan)
{
	/* Check that unit8_t doesn't try to go negative */
	if (pcan->cd.uc[0] < 40) pcan->cd.uc[0] = 0;
	if (pcan->cd.uc[1] < 40) pcan->cd.uc[1] = 0;
	if (pcan->cd.uc[2] < 40) pcan->cd.uc[2] = 0;
	/* Convert CAN payload to deg C */
	p->rotortemp   = (pcan->cd.uc[0] - 40);//*10;
	p->invtemp     = (pcan->cd.uc[1] - 40);//*10;
	p->statortemp  = (pcan->cd.uc[2] - 40);//*10;
	// Pick the higher of the two motor temp readings
	if (p->rotortemp > p->statortemp)
		p->wcmottemp = p->rotortemp;
	else
		p->wcmottemp = p->statortemp; // Worse-case motor temp
	return;	
}
/* ***********************************************************************************************************
 * static void motorcontrol_init(struct COOLINGFUNCTION* pc);
 * @brief	: Initialize with assumptions about motor
 * @param   : pc = pointer to struct that includes motor control structs
 ************************************************************************************************************* */
static void motorcontrol_init(struct COOLINGFUNCTION* pc)
{
	// Convenience pointers
	struct MOTORRAMP*      p;   
	struct MOTORRAMPPARAM* pprm;

	for (int i = 0; i < MOTORNUM; i++)
	{
		p    = &pc->motorramp[i]; // Working struct
		pprm = &pc->motorrampparam[i];// Parameter struct		
		// Convert from pct-per-sec to pct-per-timer tick
		p->ramppertickup = (pprm->rampuprate * 1000) / COOLTIMERMS; // Timer tick conversion
		p->ramppertickup = (pprm->rampdnrate * 1000) / COOLTIMERMS; // Timer tick conversion
		p->shutdowntic = pprm->shutoffwait / COOLTIMERMS; // Unknown. Could be coasting down

		p->spindownctr = p->shutdowntic; // Re-boot might find motor still spinning down
		p->ryreq.idx   = pprm->hdrnum; // Map motor number (0-3) to header index (0-11)
		p->pryreq      = &p->ryreq;
		p->state       = MOTSTATE_SPINDWN;
		p->updatectr   = 0; // 
		p->ryreq.pwm   = 0; // Queue to RyTask
		p->frampaccum  = 0; // In-progress of changing pwm
		p->irampaccum  = 0; // (Probably not necessary)
		p->target      = 0; // Last static pwm value
		xQueueSendToBack(RyTaskReadReqQHandle,&p->pryreq,10000);
	}
	return;
}
/* ***********************************************************************************************************
 * static void motorcontrol(struct MOTORRAMP* p, uint8_t pwm);
 * @brief	: Ramping control
 * @param   : p = pointer to struct for this motor
 * @param	: pwm = 0-100 for new target pwm 
 *          : 0 = off
 *          : 100 = full on
 ************************************************************************************************************* */
void motorcontrol(struct COOLINGFUNCTION* pc, uint8_t i, uint8_t pwm)
{
	// Convenience pointers
	struct MOTORRAMP*      p    = &pc->motorramp[i]; // Working struct
	struct MOTORRAMPPARAM* pprm = &pc->motorrampparam[i];// Parameter struct

	switch (p->state)
	{
	case MOTSTATE_SPINDWN: // spin-down still counting.
		p->spindownctr -= 1;
		if (&p->spindownctr > 0)
			break;		
		p->state = MOTSTATE_RAMPING;
	
	case MOTSTATE_RAMPING:
		// Check for a turn-off request
		if (pwm == 0)
		{ // Here, request is for the motor to be OFF
			if (p->target == 0)
			{ // Here, previous was also zero (OFF)
				break; // No need to update.
			}
			// Here motor was previously NOT-OFF.
			p->spindownctr = p->shutdowntic; // Set a new spin-down delay
			p->frampaccum  = 0;
			p->irampaccum  = 0;		
			p->target      = 0;      // Update target pwm
			p->ryreq.pwm   = 0;      // Update RyTask request and queue request
			xQueueSendToBack(RyTaskReadReqQHandle,&p->pryreq,10000);
			p->state = MOTSTATE_SPINDWN;
			break;
		}
		// Check for initial over-target required to get motor spinning
		if (pwm < pprm->minstart)
		{ // Here, pwm request requires the motor to be turning
			if (p->target >= pprm->idle)
			{ // Here, motor was turning. Normal ramp-up/ramp-down
				p->target = pwm;
				do_ramp(p);
				break;
			}
			// Here motor likely not turning. 
			p->target = pprm->minstart;
			do_ramp(p);
			p->state  = MOTSTATE_MINSTART;
			break;
		}
		// Here, pwm request is greater than minstart
		p->target = pwm;
		do_ramp(p);
		break;

	case MOTSTATE_MINSTART:
		if (p->frampaccum == (float)p->target)
		{
			p->target = pwm;
			p->state = MOTSTATE_RAMPING;
			break;
		}
		do_ramp(p);
		break;

		// Here, not a turn off request (and not a spindownwait delay)
		if (pwm != p->target)
		{ // New target
			p->target = pwm;
		}
		break;		
	}
}
/* ***********************************************************************************************************
 * static void do_ramp(struct MOTORRAMP* p);
 * @brief	: Ramping control
 * @param   : p = pointer to working value struct for this motor
 ************************************************************************************************************* */
void do_ramp(struct MOTORRAMP* p)
{
	if (p->frampaccum == (float)p->target)
		return;

	if (p->frampaccum < (float)p->target)
	{ // Here, currently lower than request: current level too low
		p->frampaccum += p->ramppertickup;
		p->irampaccum  = p->frampaccum; // Convert float
		// In case ramp up-tick goes exceeds pct limit.
		if (p->irampaccum > 100) // Yes, it could be for fast ramp rates
			p->irampaccum = 100;
		// Compare whole pct values
		if (p->irampaccum_prev == p->irampaccum)
			return;
		// Here a pwm update is needed
		p->irampaccum_prev = p->irampaccum;
		p->ryreq.pwm = p->irampaccum;
		xQueueSendToBack(RyTaskReadReqQHandle,&p->pryreq,10000);
		if (p->irampaccum >= p->target)
		{ // Done. Reached goal
			p->frampaccum = p->target; // Clean up float
		}
		return;
	}
	// Here, framaccum > target: Current level too high
		p->frampaccum -= p->ramppertickdn;
		p->irampaccum  = p->frampaccum; // Convert float
		// Compare whole pct values
		if (p->irampaccum_prev == p->irampaccum)
			return;
		// Here a pwm update is needed
		p->irampaccum_prev = p->irampaccum;
		p->ryreq.pwm = p->irampaccum;
		xQueueSendToBack(RyTaskReadReqQHandle,&p->pryreq,10000);		
		if (p->irampaccum >= p->target)
		{ // Done. Reached goal
			p->frampaccum = p->target; // Clean up float
		}
	return;	
}


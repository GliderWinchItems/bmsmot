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
#include "EMCLTaskCmd.h"
#include "CanTask.h"
#include "RyTask.h"
#include "mastercontroller_states.h"
//#include "../../../../contactor/Ourtasks/ContactorTask.h"

static void tempsetOFF(void);
static void tempsetRUN(void);
static void tempsetIDLE(void);

#define COOLCANBIT00 (1<<0) // RTOS timer 
//#define COOLCANBIT01 (1<<1) // CAN msg: actual torq
#define COOLCANBIT02 (1<<2) // CAN msg: temperatures 
//#define COOLCANBIT03 (1<<3) // CAN msg: launch state
#define COOLCANBIT04 (1<<4) // CAN msg: contactor status
#define COOLCANBIT05 (1<<5) // CAN msg: PC command
#define COOLCANBIT06 (1<<6) // CAN msg: EMChost commmand
#define COOLCANBIT07 (1<<7) // TEST


void extract_dmoc_hv_temps(struct COOLINGFUNCTION* p, struct CANRCVBUF* pcan);
static void motorcontrol_init(struct COOLINGFUNCTION* p);
void motorcontrol(struct COOLINGFUNCTION* pc, uint8_t i, uint8_t pwm);
uint8_t do_ramp(struct MOTORRAMP* p);
static void do_pcCAN(struct CANRCVBUF* pcan);
static void send_hbstatus1(struct COOLINGFUNCTION* p);
static void send_hbstatus2(struct COOLINGFUNCTION* p);
static void set_motor_ry(struct CANRCVBUF* pcan);
static void relay_req_init(void);
static void tempautoctl_init();
static uint8_t temrun(void);
static void tempautoctl(void);
static void do_contactorCAN(struct CANRCVBUF* pcan);

extern struct CAN_CTLBLOCK* pctl0; // Pointer to CAN1 control block
extern struct CAN_CTLBLOCK* pctl1; // Pointer to CAN2 control block
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

// Relay CAN msg request
static struct RYREQ_Q ryreq_q[NRELAYS]; // Queue requests to RyTask
static struct RYREQ_Q* pryreq_q[NRELAYS]; // Queue requests to RyTask

/* CAN msg command EMCL_MOTOR_RY_SET control of motor and relays requires keep alive. */
#define CANMOTRY_TIMEOUT 600 // (30 sec with 50 ms per count) KeepAlive for CAN msg requests

struct MOTRY
{
	uint8_t canpwm; // CAN msg pwm request
	uint8_t autpwm; // Automatic control pwm request
	uint8_t control;  // control: 0 = automatic; 1 = CAN msg
};

/* Relays and motor requests can come from either CAN msgs or Automatic control.
The CAN msgs have priority as long as keep-alive count is active.
   To spread the processing and queueing load, the relay/motor keep-alive updates are
done one for each 50 ms timer tick. A cycle of eleven takes 550 ms. 
*/   
struct MOTRYINFO
{
	uint32_t cankeepalive_ctr; // CAN timer tick countdown,
	struct MOTRY motry[NRELAYS]; 
	uint8_t canautidx; // Index for cyclic updating
};
static struct MOTRYINFO motryinfo;

uint32_t dbgcool1;
struct CANRCVBUF cooltest; 
uint8_t cooltest1;

TaskHandle_t CoolingTaskHandle = NULL;

/*{
	struct CANRCVBUF can;		// CAN msg
	struct CAN_CTLBLOCK* pctl;	// Pointer to control block for this CAN
	uint8_t maxretryct;
	uint8_t bits;
};*/
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
 *	@brief  : Timer tick doing
  * @param  : p = pointer to function struct
 * *************************************************************************/
void timer_do(struct COOLINGFUNCTION* p)
{
	/* Count down CAN msg timeouts. When '*_ctr == 0, time has expired. */
	if (p->timeout_CANdmoc_ctr > 0)	
		p->timeout_CANdmoc_ctr	-= 1;
//		if (p->timeout_mcstate_ctr > 0)	
//			p->timeout_mcstate_ctr	-= 1;
	if (p->timeout_cntctrkar_ctr > 0)	
		p->timeout_cntctrkar_ctr -= 1;
	if (p->timeout_cmd_emcmmcx_pc > 0)	
		p->timeout_cmd_emcmmcx_pc -= 1;
	if (p->timeout_cmd_emcmmcx_pc_ctr > 0)	
		p->timeout_cmd_emcmmcx_pc_ctr -= 1;

/* Check if contactor CAN msg keep-alive timed out. */
	if (p->timeout_cntctrkar_ctr == 0)
		p->contactor_state = 0; // Not reporting

/* Count down for heartbeats. */
	if (p->hbct_ctr > 0)
		p->hbct_ctr -= 1;
	if (p->hbct_ctr <= 0)
	{
		send_hbstatus1(p); // Temperatures
		send_hbstatus2(p); // Relays & motors
	}

	/* RyTask expects queue requests to keep alive. */
	/* CAN msg control needs a separate keep-alive, and
	   the timer_do() handles updating queued requests
	   to RYTask and timeout for CAN msgs. */

	// CAN msg control overrides auto controls.
	if (motryinfo.cankeepalive_ctr > 0)
	{ // CAN msg control is still active
		motryinfo.cankeepalive_ctr -= 1; // Countdown time (50ms steps)
		if (motryinfo.cankeepalive_ctr == 0)
		{ // Here, keep-alive timed out. Cancel overrides
			for (int i = 0; i < NRELAYS; i++)
			{
				if (motryinfo.motry[i].control != 0)
				{ // Here, this relay/motor port was CAN msg controlled
					motryinfo.motry[i].control = 0; // Set automatic control
					if (i > 7)
					{ // Here, motor, revert via ramping to automatic setting
						motorcontrol(&emclfunction.lc.lccool,i,motryinfo.motry[i].canpwm);
					}
					else
					{ // Here, relay, revert to automatic control
						ryreq_q[i].pwm = motryinfo.motry[i].autpwm; // Queue setup
						xQueueSendToBack(RyTaskReadReqQHandle,&pryreq_q[i],500);
					}
				}
			}
		}
	}

	/* Update to satisfy RyTask keep-alive. */
	for (int i = 0; i < 4; i++)
	{ // Here, Motors
		if (motryinfo.motry[8+i].control != 0)
			motorcontrol(&emclfunction.lc.lccool,i,motryinfo.motry[8+i].canpwm);
		else
			motorcontrol(&emclfunction.lc.lccool,i,motryinfo.motry[8+i].autpwm);
	}

	#define B  motryinfo.canautidx //; // Convenience index
	if (motryinfo.motry[B].control != 0)
		ryreq_q[B].pwm = motryinfo.motry[B].canpwm; // CAN control pwm
	else
		ryreq_q[B].pwm = motryinfo.motry[B].autpwm; // Automatic control pwm
	xQueueSendToBack(RyTaskReadReqQHandle,&pryreq_q[B],500);
	B += 1; // Cycle index through 0-7 indexed relay ports
	if (B > 7) B = 0;

	/* Temperture controlled automatic control */
	tempautoctl();

	return;
}
/* *************************************************************************
 * static void coolingfunction_init(struct COOLINGFUNCTION* p);
 * @brief	: Initialize working values
 * @param   : p = pointer to function struct
 * *************************************************************************/
static void coolingfunction_init(struct COOLINGFUNCTION* p)
{
	// Heartbeat for cooling status CAN msg
	p->hbct_tic = p->hbct_t / COOLTIMERMS;// 50 Number ms per timer callback
	if (p->hbct_tic == 0) morse_trap(761);
	p->hbct_ctr = 1;

	/* Fixed data for CAN msgs. */
	p->cancool1.pctl = pctl0; // CAN 1
	p->cancool2.pctl = pctl1; // CAN 2
	p->cancool1.maxretryct = 4;
	p->cancool2.maxretryct = 4;
	p->cancool1.bits = 0;
	p->cancool2.bits = 0;
	p->cancool1.can.id = emclfunction.lc.cid_unit_emcmmcx;
	p->cancool2.can.id = p->cancool1.can.id;
	p->cancool1.can.dlc = 8;
	p->cancool2.can.dlc = 8;

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

	coolingfunction_init(p);
	motorcontrol_init(p);	
	relay_req_init();
	tempautoctl_init();

	/* Add CAN Mailboxes                               CAN     CAN ID             TaskHandle,Notify bit,Skip, Paytype */
	//47400000','DMOC',1,1,'I16','DMOC: Actual Torque: payload-30000'
 //   p->pmbx_cid_dmoc_actualtorq      = MailboxTask_add(pctl0,p->cid_dmoc_actualtorq, NULL, COOLCANBIT01,0,U8); 
 //   if (p->pmbx_cid_dmoc_actualtorq == NULL) morse_trap(650);
	//'CA200000','DMOC',1,1,'U8_U8_U8''DMOC: Temperature:rotor,invert,stator'
    p->pmbx_cid_dmoc_hv_temps      = MailboxTask_add(pctl0,p->cid_dmoc_hv_temps,NULL,COOLCANBIT02,0,U8);
    if (p->pmbx_cid_dmoc_hv_temps == NULL) morse_trap(651);
	//'26000000','MC',1,5,'U8_U8','MC: Launch state msg'    
 //   p->pmbx_cid_mc_state      = MailboxTask_add(pctl0,p->cid_mc_state,   NULL, COOLCANBIT03,0,U8);
 //   if (p->pmbx_cid_mc_state == NULL) morse_trap(652);	
	//'E3C00000',,'CNTCTR',1,6,'U8_U8_U8','Contactor: R KeepAlive response'
    p->pmbx_cid_cntctrkar      = MailboxTask_add(pctl0,p->cid_cntctrkar,NULL,COOLCANBIT04,0,U8);
    if (p->pmbx_cid_cntctrkar == NULL) morse_trap(652);	

/* NOTE: These are CAN2 (pctl1) when EMClocal is on a separate CAN bus. */
    p->pmbx_cid_cmd_emcmmcx_pc      = MailboxTask_add(pctl0,p->cid_cmd_emcmmcx_pc,NULL,COOLCANBIT05,0,U8);
    if (p->pmbx_cid_cmd_emcmmcx_pc == NULL) morse_trap(653);	

    p->pmbx_cid_cmd_emcmmcx_emc      = MailboxTask_add(pctl0,p->cid_cmd_emcmmcx_emc,NULL,COOLCANBIT06,0,U8);
    if (p->pmbx_cid_cmd_emcmmcx_emc == NULL) morse_trap(654);	

    p->pmbx_cid_test      = MailboxTask_add(pctl0,p->cid_test,NULL,COOLCANBIT07,0,U8);
    if (p->pmbx_cid_test == NULL) morse_trap(655);	


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
//		if ((noteval & COOLCANBIT01) != 0) // CAN msg: Actual torq
//		{ //     
//			pcan = &p->pmbx_cid_dmoc_actualtorq->ncan.can;
//		}
		if ((noteval & COOLCANBIT02) != 0) // CAN msg: DMOC temperatures
		{ //     
			p->timeout_CANdmoc_ctr = p->timeout_CANdmoc; // Reset timeout ctr
			pcan = &p->pmbx_cid_dmoc_hv_temps->ncan.can;
			extract_dmoc_hv_temps(p,pcan);
		}
//		if ((noteval & COOLCANBIT03) != 0) // CAN msg: launch state
//		{ //     
//			p->timeout_mcstate_ctr = p->timeout_mcstate; // Reset timeout ctr
//			pcan = &p->pmbx_cid_mc_state->ncan.can;
//		}
		if ((noteval & COOLCANBIT04) != 0) // CAN msg: contactor status
		{ //     
			p->timeout_cntctrkar_ctr = p->timeout_cntctrkar; // Reset timeout ctr
			pcan = &p->pmbx_cid_cntctrkar->ncan.can;
			do_contactorCAN(pcan);
		}

		if ((noteval & COOLCANBIT05) != 0) // CAN msg: PC 
		{ //     
			p->timeout_cmd_emcmmcx_pc_ctr = p->timeout_cmd_emcmmcx_pc; // Reset timeout ctr
			pcan = &p->pmbx_cid_cmd_emcmmcx_pc->ncan.can;
			do_pcCAN(pcan);
		}		

		if ((noteval & COOLCANBIT06) != 0) // CAN msg: EMChost 
		{ //     
			p->timeout_cmd_emcmmcx_emc_ctr = p->timeout_cmd_emcmmcx_emc; // Reset timeout ctr
			pcan = &p->pmbx_cid_cmd_emcmmcx_emc->ncan.can;
		}
		
		if ((noteval & COOLCANBIT07) != 0) // TEST
		{ //     
			cooltest = p->pmbx_cid_test->ncan.can;
			cooltest1 = 1;
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
		#define SCALE (COOLTIMERMS * 0.001f)
		p->ramppertickup = pprm->rampuprate  * SCALE; // Timer tick conversion
		p->ramppertickdn = pprm->rampdnrate  * SCALE; // Timer tick conversion
		p->shutdowntic   = pprm->shutoffwait * SCALE; // Unknown. Could be coasting down

		p->spindownctr = p->shutdowntic; // Re-boot might find motor still spinning down
		p->ryreq.idx   = (8+i); //pprm->hdrnum; // Map motor number (0-3) to header index (0-11)
		p->pryreq      = &p->ryreq;
		p->state       = MOTSTATE_SPINDWN;
		p->updatectr   = 0; // 
		p->ryreq.pwm   = 0; // Queue to RyTask
		p->frampaccum  = 0; // In-progress of changing pwm
		p->irampaccum  = 0; // (Probably not necessary)
		p->target      = 0; // Last static pwm value
// Might not want this. Spindown might be taking place. Depends on type of sub-board.
//		xQueueSendToBack(RyTaskReadReqQHandle,&p->pryreq,10000);
	}
	return;
}
/* ***********************************************************************************************************
 * void motorcontrol(struct COOLINGFUNCTION* pc, uint8_t i, uint8_t pwm);
 * @brief	: Ramping control
 * @param   : p = pointer to struct for this motor
 * @param   : i = motor number (0 - 3 i.e. MOTORNUM)
 * @param	: pwm = 0-100 for new target pwm 
 *          : 0 = off
 *          : 100 = full on
 ************************************************************************************************************* */
void motorcontrol(struct COOLINGFUNCTION* pc, uint8_t i, uint8_t pwm)
{
	// Convenience pointers
	struct MOTORRAMP*      p    = &pc->motorramp[i]; // Working struct
	struct MOTORRAMPPARAM* pprm = &pc->motorrampparam[i];// Parameter struct
	uint8_t ret;
if(i > 3) morse_trap(6767);
if (pwm > 100) morse_trap(888);
	/* JIC algorithm gets carried away. */
	if (pwm > 100) pwm = 100;

	switch (p->state)
	{
	case MOTSTATE_SPINDWN: // spin-down still counting.
		p->spindownctr -= 1;
		if (p->spindownctr > 0)
			break;		
		p->state = MOTSTATE_STOPPED;

	case MOTSTATE_STOPPED:
		// Check for a turn-off request
		if (pwm == 0)
		{ // Here, request is for the motor to be OFF
			p->spindownctr = p->shutdowntic; // Set a new spin-down delay
			p->frampaccum  = 0;
			p->irampaccum  = 0;		
			p->target      = 0;      // Update target pwm
			p->ryreq.pwm   = 0;      // Update RyTask request and queue request
			break;
		}

		p->target = pprm->minstart;
		do_ramp(p);
		p->state = MOTSTATE_MINSTART2;
		break;

	case MOTSTATE_MINSTART2:
		ret = do_ramp(p);
		if (ret != 0)
			break;
		// Here, motor ramp reached minstart
		p->state = MOTSTATE_RUN;

	case MOTSTATE_RUN:
/*
If pwm request would drop back and be below IDLE, then the motor
is held at IDLE pwm. But, when pwm is request is zero the shutdown
is state starts. */		
		if (pwm < pprm->idle)
		{
			if (pwm == 0)
			{
/* Sub-boards with diode flyback spin free when the pwm is dropped
from a running level to zero.

Half-bridge sub-boards apply a short if the immediately drops
to zero. If the power fet can withstand shorting the spinning
motor, and the mechanism can withstand the negative torque,
this is OK. Otherwise a rampdown is needed. */
				p->target = 0;
				if (pprm->subbrdtype != 0)
				{ // Here, half-bridge, so ramp down
					ret =do_ramp(p);
					if (ret != 0)
						break; 
					p->state = MOTSTATE_STOPPED;
					break;
				}
				// Here, diode flyback, so let spin
				p->frampaccum  = 0;				
				p->state = MOTSTATE_SPINDWN;
				break;
			}
			p->target = pprm->idle;
			do_ramp(p);
			break;
		}
		// Here, between at idle and 100%.
		p->target = pwm;
		do_ramp(p);
		break;

	default:
		morse_trap(7339);
		break;
	}		
	/* Update RyTask pwm and maintain keep-alive. */
	p->irampaccum = p->frampaccum; // Convert float
	p->ryreq.pwm  = p->irampaccum;
	xQueueSendToBack(RyTaskReadReqQHandle,&p->pryreq,1000);
	return;
}
/* ***********************************************************************************************************
 * static uint8_t do_ramp(struct MOTORRAMP* p);
 * @brief	: Ramping control
 * @param   : p = pointer to working value struct for this motor
 * @param   : 0 = done, frampaccum == target; 1 = ramping upwards; 2 = ramping downwards
 ************************************************************************************************************* */
uint8_t do_ramp(struct MOTORRAMP* p)
{
	if (p->frampaccum == (float)p->target)
		return 0;

	if (p->frampaccum < (float)p->target)
	{ // Here, currently lower than request: current level too low
		p->frampaccum += p->ramppertickup;
		p->irampaccum  = p->frampaccum; // Convert float
		// In case ramp up-tick goes exceeds pct limit.
		if (p->irampaccum > 100) // Yes, it could be for fast ramp rates
			p->irampaccum = 100;
		if (p->irampaccum >= p->target)
		{ // Done. Reached goal
			p->frampaccum = p->target; // Clean up float
		}
		return 1;
	}
	// Here, framaccum > target: Current level too high
		p->frampaccum -= p->ramppertickdn;
		p->irampaccum  = p->frampaccum; // Convert float
		if (p->irampaccum <= p->target)
		{ // Done. Reached goal
			p->frampaccum = p->target; // Clean up float
		}
	return 2;	
}
/* ***********************************************************************************************************
 * static void do_pcCAN(struct CANRCVBUF* pcan);
 * @brief	: Deal with CAN msg from PC
 * @param   : pcan = pointer to CAN msg
 ************************************************************************************************************* */
/* Incoming CAN msg
pay[0] - CMD_CODES_INSERT.sql & "EMCLTaskCmd.h" for codes 36 and above
if pay[0]==EMCL_MOTORPWM_SETPWMX   37 // PWM PCT for one motor
  pay[1]
    bit[0]: for motor index[0] 0=ignore; 1=pwm request preset
    bit[1]: for motor index[1] 0=ignore; 1=pwm request preset
    bit[2]: for motor index[2] 0=ignore; 1=pwm request preset
    bit[3]: for motor index[3] 0=ignore; 1=pwm request preset
  pay[2:3] = reserved  
  pay[4]=pwm for motor index 0
  pay[5]=pwm for motor index 1
  pay[6]=pwm for motor index 2
  pay[7]=pwm for motor index 3

No response is sent. Status msg is sufficient
*/
/* Outgoing CAN msg
pay[0]==EMCL_COOLING_STATUS1     36 // 
pay[1] 
  bit[0]: 1=Temperature alert pump outlet
  bit[1]: 1=Temperature alert motor outlet
  bit[2]: 1=Temperature alert heat exchanger outlet
  bit[3]: 1=Temperature alert ambient
  bit[4]: 1=Temperature alert motor DMOC msg
  bit[5]: 1=motor alert(?)
pay[2]=temperature deg C: pump outlet
pay[3]=temperature deg C: motor outlet
pay[4]=temperature deg C: heat exchanger outlet
pay[5]=temperature deg C: ambient outlet
pay[6]=temperature deg C: motor DMOC
pay[7]=temperature deg C: jic


*/
static void do_pcCAN(struct CANRCVBUF* pcan)
{
	struct COOLINGFUNCTION* p = &emclfunction.lc.lccool; // Convenience pointer
	switch(pcan->cd.uc[0])
	{
	case EMCL_COOLING_STATUS1:  // 36 GET: Alert status & temperature report
		send_hbstatus1(p);
		break;
	case EMCL_MOTOR_RY_SET:     // SET: Relays and PWM PCT for motors
		set_motor_ry(pcan);
		break;
	case EMCL_MOTOR_RY_STATUS2: // GET: Relay status groups OA, OB, and PWM PCT for OC motors
		send_hbstatus2(p);
		break;
	default:
		break;
	}
	return;
}
/* ***********************************************************************************************************
 * static void send_hbstatus1(struct COOLINGFUNCTION* p);
 * @brief  : Send heartbeat status CAN msg on CAN1
 * @param  : p = pointer to function struct
 ************************************************************************************************************* */
/* Payload layout
[0] EMCL_COOLING_STATUS1; // Code for payload
[1] Alert status: 0 = NO ALERTS; otherwise bits set--
    7: Contactor CAN msgs missing
    6:
    5: DMOC motor greater than over-temperature parameter
    4: Greater than over-temperature parameter: pump outlet
    3: Greater than over-temperature parameter: winch motor
    2: Greater than over-temperature parameter: heat exchanger
    1: Greater than over-temperature parameter: ambient
    0: Greater than over-temperature parameter: jic
    // Readings: deg C; 255 = not installed; less than 0 deg C = 0;
[2] adc1.abs[p->tx_pmpo]: pump outlet
[3] adc1.abs[p->tx_moto]: winch motor
[4] adc1.abs[p->tx_hexo]: heat exchanger
[5] adc1.abs[p->tx_amb ]: ambient
[6] adc1.abs[p->tx_jic ]: JIC
[7] Reserve for DMOC report of motor temperature 
*/
static uint8_t set_alert_temperature(struct COOLINGFUNCTION* p)
{
	uint8_t ret = 0;
	if (adc1.abs[p->tx_pmpo].filt > p->temperatureparm[p->tx_pmpo].toohi)
		ret  = (1<<0);
	if (adc1.abs[p->tx_moto].filt > p->temperatureparm[p->tx_moto].toohi)
		ret |= (1<<1);
   	if (adc1.abs[p->tx_hexo].filt > p->temperatureparm[p->tx_hexo].toohi)
   		ret |= (1<<2);
   	if (adc1.abs[p->tx_amb ].filt > p->temperatureparm[p->tx_amb ].toohi)
   		ret |= (1<<3);
   	if (adc1.abs[p->tx_jic ].filt > p->temperatureparm[p->tx_jic ].toohi)
   		ret |= (1<<4);
   	if (adc1.abs[p->tx_dmoc].filt > p->temperatureparm[p->tx_dmoc].toohi)
   		ret |= (1<<5);
	return ret;
}
static void send_hbstatus1(struct COOLINGFUNCTION* p)
{
	struct CANRCVBUF* pcan = &p->cancool1.can;
	pcan->cd.uc[0] = EMCL_COOLING_STATUS1; // See: EMCLTaskCmd.h
	pcan->cd.uc[1] = set_alert_temperature(p);
	pcan->cd.uc[2] = adc1.abs[p->tx_pmpo].filt;
	pcan->cd.uc[3] = adc1.abs[p->tx_moto].filt;
	pcan->cd.uc[4] = adc1.abs[p->tx_hexo].filt;
	pcan->cd.uc[5] = adc1.abs[p->tx_amb ].filt;
	pcan->cd.uc[6] = adc1.abs[p->tx_jic ].filt;
	pcan->cd.uc[7] = 0xA5; // Dummy for now
	p->hbct_ctr = p->hbct_tic; // Reset heartbeat count
	// Place CAN msg on CanTask queue
	xQueueSendToBack(CanTxQHandle,&p->cancool1,4);
	return;
}
/* ***********************************************************************************************************
 * static void send_hbstatus2(struct COOLINGFUNCTION* p);
 * @brief  : Send heartbeat status CAN msg on CAN1
 * @param  : p = pointer to function struct
 ************************************************************************************************************* */
/* Payload layout
[0] EMCL_MOTORPWM_GETPWMX; // Code for payload
[1] 7: reserved
	6: Contactor CAN msg missing
	5-0: reserved
[2] Group A & B bits
[3] reserved
[4] Group C percent
[5] Group C percent
[6] Group C percent
[7] Group C percent
*/
static void send_hbstatus2(struct COOLINGFUNCTION* p)
{
	struct CANRCVBUF* pcan = &p->cancool1.can;
	pcan->cd.uc[0] = EMCL_MOTOR_RY_STATUS2; // See: EMCLTaskCmd.h
	pcan->cd.uc[1] = 0; // Reserved
	RyTask_CANpayload(pcan); // Fill payload with Relay & motor info
	if (p->contactor_state == 0) // Set missing contactor CAN msgs
			pcan->cd.uc[1] = (1<<7);

	p->hbct_ctr = p->hbct_tic; // Reset heartbeat count
	// Place CAN msg on CanTask queue
	xQueueSendToBack(CanTxQHandle,&p->cancool1,4);
	return;
}
/* ***********************************************************************************************************
 * static void relay_req_init(void);
 * @brief  : Initialize relay and motor queue arrays
 ************************************************************************************************************* */
static void relay_req_init(void)
{
	int i;
	for (i = 0; i < NRELAYS; i++) // All relays and motors
	{
		pryreq_q[i] = &ryreq_q[i]; // Pointer for queue
		ryreq_q[i].cancel = 0;     // jic
		ryreq_q[i].pwm = 0;      // Use parameter list value
		ryreq_q[i].idx = i;        // Relay identification

		motryinfo.motry[i].canpwm  = 0;
		motryinfo.motry[i].autpwm  = 0;
		motryinfo.motry[i].control = 0;
	}
	motryinfo.cankeepalive_ctr = 0; // CAN in control timer tick countdown
	motryinfo.canautidx = 0; // Index for cyclic updating
	return;
}

static struct RYREQ_Q ryreq_q[NRELAYS]; // Queue requests to RyTask
static struct RYREQ_Q* pryreq_q[NRELAYS]; // Queue requests to RyTask
/* ***********************************************************************************************************
 * static void set_motor_ry(struct COOLINGFUNCTION* p);
 * @brief  : CAN code: EMCL_MOTOR_RY_SET sets relays and motors
 * @param  : p = pointer to CAN msg
 ************************************************************************************************************* */
/* Payload layout: EMCL_MOTOR_RY_SET       37 // SET: Relays and PWM PCT for motors
[0] EMCL_MOTOR_RY_SET; // Set relays & motors
[1] Motors to be changed bits: 0 = no change
    7: 1 = revert to automatic control (following payload gets ignored)
    6: reserved
    5: reserved
    4: reserved
    3: 1 = OC4 motor percent change to payload [7] relay array [11]
    2: 1 = OC3 motor percent change to payload [6] relay array [10]
    1: 1 = OC2 motor percent change to payload [5] relay array [ 9]
    0: 1 = OC1 motor percent change to payload [4] relay array [ 8]
[2] Relays to be set: bits 0 = no change
    7: OCB4
    6: OCB3
    5: OCB2
    4: OCB1
    3: OCA4
    2: OCA3
    1: OCA2
    0: OCA1
[3] Relay on/off bits: 1 = ON
    7: OCB4
    6: OCB3
    5: OCB2
    4: OCB1
    3: OCA4
    2: OCA3
    1: OCA2
    0: OCA1
[4] OC1 motor percent
[5] OC2 motor percent
[6] OC3 motor percent
[7] OC4 motor percent
*/
static void set_motor_ry(struct CANRCVBUF* pcan)
{
	/* Update list of relays and motors receiving CAN msgs w control changes. */
	motryinfo.cankeepalive_ctr = CANMOTRY_TIMEOUT; // Reset keep-alive count

	int i;
	if ((pcan->cd.uc[1] & 0x80) != 0)
	{ // Here. Command to stop CAN msg control
		// Next timer_do() will look like a CAN msg timeout
		motryinfo.cankeepalive_ctr = 1;
		return;
	}

	if ((pcan->cd.uc[1] & 0x0F) != 0)
	{ // Here, one or more motors to be set
		for (i = 0; i < 4; i++) // OC1-OC4
		{
			if ((pcan->cd.uc[1] & (1<<i)) != 0)
			{ // Send pwm request to motor ramp control
				motryinfo.motry[8+i].canpwm  = pcan->cd.uc[4+i]; // Save pwm
				motryinfo.motry[8+i].control = 1; // Show CAN msg controls
				motorcontrol(&emclfunction.lc.lccool, i, pcan->cd.uc[4+i]);
			}
		}
	}
	if (pcan->cd.uc[2] != 0)
	{ // Here, one or more relays to be set
		for (i = 0; i < 8; i++) // OA1-OA4,OB1-OB4
		{
			if ((pcan->cd.uc[2] & (1<<i)) != 0)
			{ // Here, set this relay
				if ((pcan->cd.uc[3] & (1<<i)) != 0)
					motryinfo.motry[i].canpwm  = 255; // Use parameter PWM.
				else
					motryinfo.motry[i].canpwm  = 0; // OFF

				motryinfo.motry[i].control = 1;	// CAN msg controls
				ryreq_q[i].pwm = motryinfo.motry[i].canpwm; // Queue setup
				xQueueSendToBack(RyTaskReadReqQHandle,&pryreq_q[i],500);
			}
		}
	}
	return;
}
#define TMR_IDLE_END (30*COOLTIMERTR) // Timeout ending of IDLE: 30 secs
#define TMR_TEST     (15*COOLTIMERTR) // Timeout waiting for thermister stablize: 15 secs

#define TEM_OFF  0
#define TEM_IDLE 1
#define TEM_TEST 2
#define TEM_RUN  3
struct TEMRUNX
{
	int32_t timectr;
	float fpwm;
	float delta;
	uint8_t ipwm;
	uint8_t alert;   // Bits for too high
	uint8_t run;     // Bits for run
	uint8_t toohi; 
	uint8_t ambient; // Ambient temperature
	uint8_t ttmp;
	uint8_t pwm[6];
	uint8_t tstate;  // Temperature control state machine state
};
static struct TEMRUNX temrunx;
/* ***********************************************************************************************************
 * static void temx(struct TEMRUNX* p, uint8_t idx, uint8_t abit);
 * @brief  : Temperature Automatic Control of cooling motors
 * @param  : p = pointer to convenience struct
 * @param  : idx = index into array for thermistor
 * @param  : abit = shift count for alert status byte setup
 * @return : pwm for motor associated with this thermistor index
 ************************************************************************************************************* */
static uint8_t temx(struct TEMRUNX*p, uint8_t idx, uint8_t abit)
{
	struct COOLINGFUNCTION* pc = &emclfunction.lc.lccool; // Convenience pointer	
	if (pc->temperatureparm[idx].not_installed == 0)
	{ // Here, motor coolant thermistor is installed
		p->ttmp = adc1.abs[idx].filt; // Get temperature
		if (p->ttmp >= pc->temperatureparm[idx].toohi)
			p->alert |= (1<<abit); // Set status alert bit for too high

		// When absolute temperature too hi return full speed pwm
		if (p->ttmp >= pc->temperatureparm[idx].toohimax)
		{
			p->toohi |= (1<<abit); // Set status alert bit for too high
			p->run   |= (1<<idx); // Needs to run
			p->pwm[idx] = 100;
			return 100;
		}
	}

	p->delta = (p->ambient - (float)p->ttmp);
	p->fpwm = pc->temperatureparm[idx].tcoef[0] + 
			  pc->temperatureparm[idx].tcoef[1] * p->delta;
	if (p->fpwm < 0) p->fpwm = 0; // Avoid undefined behavior
	p->pwm[idx] = p->fpwm; // Convert float to uint
	if (p->fpwm == 0)
	{ // Colder than ambient plus a bit
		p->run &= ~(1<<idx); // No need to run
		return 0;
	} 
	if (p->fpwm >= 100)
	{ // Quite hot indeed.
		p->run |= (1<<abit); // Set status alert bit for too high
		{	
			p->pwm[idx] = 100;	
			p->run |= (1<<idx); // Needs to run
			return 100;
		}	
	}
	if (p->fpwm > 0)
	{ // Here anything from 1 - 99.
		p->run |= (1<<idx); // Needs to run
	return p->pwm[idx];
	}
	return 0;
}	
/* ***********************************************************************************************************
 * static uint8_t temrun(void);
 * @brief  : Temperature Automatic Control of cooling motors
 * @return : 0 = NO; 1 = YES
 ************************************************************************************************************* */
/*
[1] Alert status: 0 = NO ALERTS; otherwise bits set--
    7: Contactor CAN msg missing
    6:
    5: DMOC motor greater than over-temperature parameter
    4: Greater than over-temperature parameter: pump outlet
    3: Greater than over-temperature parameter: winch motor
    2: Greater than over-temperature parameter: heat exchanger
    1: Greater than over-temperature parameter: ambient
    0: Greater than over-temperature parameter: jic 
*/
static uint8_t temrun(void)
{
	uint8_t ret = 0;
	struct COOLINGFUNCTION* pc = &emclfunction.lc.lccool; // Convenience pointer	
	struct TEMRUNX* p = &temrunx;
	temrunx.alert  = 0;
	temrunx.run    = 0;
	pc->contactor_state = 0;
	temrunx.ambient = adc1.abs[pc->tx_amb].filt; // Ambient air

	// If ambient is too hi, bad news! 
	if (temrunx.ambient > pc->temperatureparm[pc->tx_amb].toohi)
		temrunx.alert |= (1<<1);

//	if (p->contactor_state == 1) // 0 = not reporting; 1 = connected; 2 = disconnected or other)
	{

	}

	/* Check each temperature measurement. Set alert and run bits */
	temx(p, pc->tx_moto, 4); // Save pwm
	// Winch motor coolant outlet temperature
	temx(p, pc->tx_moto, 3);
	// Heat exchanger outlet temperature
	temx(p, pc->tx_hexo, 2);
	// DMOC cooling fins
	temx(p, pc->tx_dmoc, 0);

	if(p->pwm[4] > 0) ret |= (1<<4);
	if(p->pwm[3] > 0) ret |= (1<<3);
	if(p->pwm[2] > 0) ret |= (1<<3);

	return ret;
}
/* ***********************************************************************************************************
 * static void tempautoctl_init(void);
 * @brief  : Temperature Automatic Control initializations
 ************************************************************************************************************* */
static void tempautoctl_init(void)
{
	struct TEMRUNX* p = &temrunx;
	p->tstate  = TEM_TEST;
	p->timectr = TMR_TEST;
 // ... more?...
	return;
}
/* ***********************************************************************************************************
 * static void tempsetRUN(void);
 * @brief  : Set pwm for motors for RUN state
 ************************************************************************************************************* */
static void tempsetRUN(void)
{
	/* Pump, Heat exchanger fan, DMOC blower|fan */
	for (int i = 0; i < 3; i++)
		motorcontrol(&emclfunction.lc.lccool, i, temrunx.pwm[0]);
	return;
}
/* ***********************************************************************************************************
 * static void tempsetOFF(void);
 * @brief  : Set motors OFF
 ************************************************************************************************************* */
static void tempsetOFF(void)
{
	/* Pump, Heat exchanger fan, DMOC blower|fan */
	for (int i = 0; i < 3; i++)
		motorcontrol(&emclfunction.lc.lccool, i, 0);
	return;
}
/* ***********************************************************************************************************
 * static void tempsetIDLE(void);
 * @brief  : Set pwm for motors for IDLE state
 ************************************************************************************************************* */
static void tempsetIDLE(void)
{
	struct COOLINGFUNCTION* pc = &emclfunction.lc.lccool; // Convenience pointer	

	motorcontrol(pc,0, pc->motorrampparam[COOLX_PUMP].idle);   // Pump
	motorcontrol(pc,1, pc->motorrampparam[COOLX_HEXFAN].idle); // Heat exchanger
	motorcontrol(pc,2, pc->motorrampparam[COOLX_DMOCFAN].idle);// DMOC blower|fan
	return;
}
/* ***********************************************************************************************************
 * static void tempautoctl(void);
 * @brief  : Temperature Automatic Control of cooling motors. 50 ms timer poll calls this.
 ************************************************************************************************************* */
static void tempautoctl(void)
{
	struct TEMRUNX* p = &temrunx;
	/* Check temperatures */
	temrun();

	/* Overide states when thermistors indicate run condition. */
	if(p->run != 0)
	{
		tempsetRUN();
		p->tstate = TEM_RUN;
		return;
	}

	// Time counter used by states
	if (p->timectr > 0)
		p->timectr -= 1;

	switch (p->tstate)
	{
	case TEM_OFF:
		tempsetOFF();
		break;

	case TEM_IDLE:
		tempsetIDLE();
		break;

	case TEM_RUN:
		// Here, p->run == 0
		tempsetOFF();
		p->tstate = TEM_OFF;
		break;

	case TEM_TEST: // Run to bring thermistors up to temperature
		if (p->timectr != 0)
		{ // Waiting
			tempsetIDLE();
			break;
		}		
		if(p->run == 0)
		{
			tempsetOFF();
			p->tstate = TEM_OFF;
			break;
		}
		break;

	default:
		morse_trap(793);
		break;
	}
}
/* ***********************************************************************************************************
 * static void do_contactorCAN(struct CANRCVBUF* pcan);
 * @brief  : Deal with CAN msg: eval bit COOLCANBIT04; CAN msg: contactor status
 * @param  : pcan = pointer to contactor CAN msg
 ************************************************************************************************************* */
#if 1
// copied from contactor/ContactorTask.h for convenience
enum CONTACTOR_STATE
{
	DISCONNECTED,   /*  0 */
	CONNECTING,     /*  1 */
	CONNECTED,      /*  2 */
	FAULTING,       /*  3 */
	FAULTED,        /*  4 */
	RESETTING,      /*  5 */
	DISCONNECTING,  /*  6 */
	OTOSETTLING,    /*  7 one time intializing. */
};
#endif
static void do_contactorCAN(struct CANRCVBUF* pcan)
{
//	enum CONTACTOR_STATE cstate = CONNECTED;
	struct COOLINGFUNCTION* p = &emclfunction.lc.lccool; // Convenience pointer	
	// Reset keep-alive timeout (50 ms) counter. 
	p->timeout_cntctrkar_ctr = p->timeout_cntctrkar; // Reset timeout ctr

	// If contactor is CONNECTED cooling motors should run at least minimum
	if ((pcan->cd.uc[0] & 0x0F) == CONNECTED)
		p->contactor_state = 1; // CONNECTED
	else
		p->contactor_state = 0; // NOT CONNECTED or something in progress.
	return;
}

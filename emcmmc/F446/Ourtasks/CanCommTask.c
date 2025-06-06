/******************************************************************************
* File Name          : CanCommTask.c
* Date First Issued  : 11/06/2021
* Description        : Can communications
*******************************************************************************/
/*
When xTaskNotifyWait exits a timeout signals the time to send a heartbeat.
Notifications from an incoming CAN msg as well as the heartbeat places a pointer
on a queue to BMSTask.c to perform an'1818 BMS operation, e.g. read the cell
voltages. When BMSTask completes the request the notification of the completion
sets the notificaion word/bit specified in the request. The exit of 
xTaskNotifyWait then supplies the notification to complete the request (which
would a heartbeat or the variety of CAN msg requests).
*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "malloc.h"
//#include "semphr.h"

#include "main.h"
#include "DTW_counter.h"
#include "morse.h"
//#include "bmsspi.h"
//#include "BMSTask.h"
#include "CanCommTask.h"
#include "MailboxTask.h"
#include "CanTask.h"
#include "can_iface.h"
#include "canfilter_setup.h"
//#include "bq_items.h"
#include "cancomm_items.h"
#include "../../../../GliderWinchCommons/embed/svn_common/trunk/db/gen_db.h"

extern struct CAN_CTLBLOCK* pctl0; // Pointer to CAN1 control block
extern CAN_HandleTypeDef hcan1;

static void do_req_codes(struct CANRCVBUF* pcan);
static uint8_t q_do(struct CANRCVBUF* pcan);

uint32_t dbgCanCommTask1;
uint32_t dbgCanCommTask1_noteval;
uint32_t dbgcancommloop;
uint32_t dbgcancommmsg;

//static void canfilt(uint16_t mm, struct MAILBOXCAN* p);

#ifdef TEST_WALK_DISCHARGE_FET_BITS // See main.h
uint8_t dischgfet; // Test fet bit (0-17)
uint8_t walkdelay; // noteval == 0 counter delay
#endif

TaskHandle_t CanCommHandle;

struct CANRCVBUF  can_hb; // Dummy heart-beat request CAN msg
uint8_t hbctr; // Cell group seq number for dummy CAN msg

uint8_t rdyflag_cancomm; // Initialization complete and ready = 1

/* Delayed queue requests. 
When an incoming CAN msg requests a response that requires a reading
from the '1818 a request is queued to BMSTask and the incoming CAN msg
is saved along with a notification bit. The CanCommTask loop
continues and other requests could take place. When BMSTask completes
a queued request it notifies CanCommTask and the notification bit is
used to lookup the CAN msg that initiated the BMSTask request. 
*/
/* xTaskNotifyWait Notification bits */
#define CANCOMMBIT00 (1 <<  0) // A1600000 EMC CAN msg
#define CANCOMMBIT01 (1 <<  1) // A1800000 PC CAN msg
#define CANCOMMBIT02 (1 <<  2) // AEC00000 CAN loading
#define CANCOMMBIT03 (1 <<  3) // B0000000
#define CANCOMMBIT04 (1 <<  4) // B0200000



// Index for checking timeouts notification failures
//uint8_t idxhb;

// Number of RTOS ticks for notification wait timeout
#define TIMEOUTPOLLRATE 4 // Four per second
#define TIMEOUTPOLL (1000/TIMEOUTPOLLRATE) // RTOS ticks between wait timeouts

/* *************************************************************************
 * static uint8_t for_us(struct CANRCVBUF* pcan, struct BQFUNCTION* p);
 *	@brief	: Check if CAN msg request is for this unit
 *  @param  : pcan = point to CAN msg struct
 *  @return : 0 = yes, not 0 = no
 * *************************************************************************/
static uint8_t for_us(struct CANRCVBUF* pcan, struct EMCLFUNCTION* p)
{
	uint32_t canid;
	uint8_t code;

	/* Check for targeted RESET */
	if ((pcan->dlc == 1) && (pcan->cd.uc[0] == LDR_RESET))
	{			
		do_req_codes(pcan); // Do RESET
		return 1; // Not really necessary
	}

	 /* Extract CAN id for unit to respond. */
    canid = (pcan->cd.uc[4] << 0)|(pcan->cd.uc[5] << 8)|
            (pcan->cd.uc[6] <<16)|(pcan->cd.uc[7] <<24);

	/* Extract code from CAN msg.  */ 
	// Code for which modules should respond: bits [7:6]
	// 11 = All modules respond
    // 10 = All modules on identified string respond
    // 01 = Only identified string and module responds
    // 00 = reserved        
	code = pcan->cd.uc[1] & 0xC0; // Extract identification code

    // Does this CAN node qualify for a response?
	if  ((((code == (3 << 6))) ||
		  ((code == (2 << 6)) && ((pcan->cd.uc[2] & (3 << 4)) == p->ident_string)) ||
	/*	  ((code == (1 << 6)) && ((pcan->cd.uc[2] & 0x0F) == p->ident_onlyus)) || */
		  ((canid == p->lc.cid_unit_emcmmcx)))) /* I_AM_CANID; e.g. A0000000 */
	{
		return 0; // Yes, respond.	
	}
	return 1; // Skip. This request is not for us.
}
/* *************************************************************************
 * void CanComm_init(struct EMCLFUNCTION* p );
 *	@brief	: Task startup
 * *************************************************************************/
void CanComm_init(struct EMCLFUNCTION* p )
{
	/* Add CAN Mailboxes                               CAN     CAN ID             TaskHandle,Notify bit,Skip, Paytype */
    p->pmbx_cid_cmd_emcmmcx_pc      = MailboxTask_add(pctl0,p->lc.cid_cmd_emcmmcx_pc, NULL, CANCOMMBIT00,0,U8); // A1600000
    if (p->pmbx_cid_cmd_emcmmcx_pc == NULL) morse_trap(622);
 
    p->pmbx_cid_cmd_emcmmcx_emc      = MailboxTask_add(pctl0,p->lc.cid_cmd_emcmmcx_emc,   NULL, CANCOMMBIT01,0,U8); // A1800000
    if (p->pmbx_cid_cmd_emcmmcx_emc == NULL) morse_trap(623);

    p->pmbx_cid_uni_bms_pc_i      = MailboxTask_add(pctl0,p->lc.cid_uni_bms_pc_i,   NULL, CANCOMMBIT02,0,U8); // AEC00000
    if (p->pmbx_cid_uni_bms_pc_i == NULL) morse_trap(624);

    p->pmbx_cid_uni_bms_emc1_i      = MailboxTask_add(pctl0,p->lc.cid_uni_bms_emc1_i,   NULL, CANCOMMBIT03,0,U8); // B0000000
    if (p->pmbx_cid_uni_bms_emc1_i == NULL) morse_trap(625);

    p->pmbx_cid_uni_bms_emc2_i      = MailboxTask_add(pctl0,p->lc.cid_uni_bms_emc2_i,   NULL, CANCOMMBIT04,0,U8); // B0200000
    if (p->pmbx_cid_uni_bms_emc2_i == NULL) morse_trap(626);


    /* Add CAN msgs to incoming CAN hw filter. (Skip to allow all incoming msgs. */
 //   canfilt(603, p->pmbx_cid_cmd_emcmmcx_emc);
 //   canfilt(603, p->pmbx_cid_cmd_emcmmcx_pc);

	/* Pre-load fixed data id CAN msg to be sent. */
	p->canmsg.pctl = pctl0;   // Control block for CAN module (CAN 1)
	p->canmsg.maxretryct = 4; //
	p->canmsg.bits       = 0; //
	p->canmsg.can.cd.ull = 0; // Clear playload jic
	p->canmsg.can.dlc    = 8; // Default payload size (might be modified when loaded and sent)
	p->canmsg.can.id     = p->lc.cid_unit_emcmmcx; // I_AM_CANID [A0000000]

	/* Pre-load a dummy CAN msg request for sending heartbeat CAN msg. 
	   This looks like an incoming CAN msg is polling this unit. */
	can_hb.id       = p->lc.cid_unit_emcmmcx;
	can_hb.dlc      = 8;
	can_hb.cd.ull   = 0; // Clear entire payload
	can_hb.cd.uc[0] = 0;  // request code (initial, changed later)
	can_hb.cd.ui[1] = p->lc.cid_unit_emcmmcx; // Our CAN id embedded in payload
	
	return;
}
/* *************************************************************************
 * void StartCanComm(void const * argument);
 *	@brief	: Task startup
 * *************************************************************************/
void StartCanComm(void* argument)
{
	struct EMCLFUNCTION* p = &emclfunction;
	struct CANRCVBUF* pcan;
	uint32_t noteval;
	uint32_t timeoutwait;

	/* CAN communications parameter init. */
	CanComm_init(p);
	cancomm_items_init();

#if 0 // Started by CanTask
	extern CAN_HandleTypeDef hcan1;
	HAL_CAN_Start(&hcan1); // Start CAN1
#endif	

osDelay(20); // Wait for ADCTask to get going.

	rdyflag_cancomm = 1; // Initialization complete and ready

/* In the following 'for (;;)' loop xTaskNotifyWait will time out if there are no 
   incoming CAN msg notifications. The timeout is set to poll the timeouts for the
   two hearbeat msgs. The heartbeat status msg will be sent even when CAN msgs are
   coming in. The timeout for the heartbeat that sends cell readings is reset each
   time a set (of six) CAN msgs with cell readings is sent. The TIMEOUTPOLL sets
   the RTOS ticks to wair for the polling. */ 
	p->HBstatus_ctr = xTaskGetTickCount(); // Count RTOS ticks for hearbeat timing: status msg
//	p->HBcellv_ctr  = p->HBstatus_ctr; // Count RTOS ticks for hearbeat timing: cellv msg
	timeoutwait     = TIMEOUTPOLL; // TaskNotifyWait timeout

/* ******************************************************************* */
	for (;;)
	{

/* PCB board discharge fet testing. */
#ifdef TEST_WALK_DISCHARGE_FET_BITS  // See main.h for #define
		timeoutwait = 789; // Avoid keep-alive calls disrupting fets
#endif		

/* Using noteval instead of 0xffffffff would take care of the possibility
of a BMSTask request completing before xTaskNotifyWait was entered. If BMSTask
completed before xTaskNotifyWait was entered using 0xfffffff would clear the 
notification and it would be lost. */

		xTaskNotifyWait(0,0xffffffff, &noteval, timeoutwait);
//		xTaskNotifyWait(0,noteval, &noteval, timeoutwait);//timeoutwait);
dbgCanCommTask1_noteval = noteval;
		
/* ******* CAN msg to all nodes. PC poll msg. */
		if ((noteval & CANCOMMBIT00) != 0) // cmd_emcmmcx_pc [A1600000] 
		{ // 
			pcan = &p->pmbx_cid_cmd_emcmmcx_pc->ncan.can;
			if (for_us(pcan,p) == 0)
			{ // This CAN msg includes us.
				do_req_codes(pcan);
			}
		}

/* ******* CAN msg to all nodes. EMC poll msg. */
		if ((noteval & CANCOMMBIT01) != 0) // cid_cmd_emcmmcx_emc [A1800000]
		{ //     
			pcan = &p->pmbx_cid_cmd_emcmmcx_emc->ncan.can;
			if (for_us(pcan,p) == 0)
			{ // This CAN msg includes us.
				do_req_codes(pcan);
			}
		}

/* ******* CAN msg to all nodes. PC poll msg. BMS codes. */
		if ((noteval & CANCOMMBIT02) != 0) // CAN id: cid_uni_bms_pc_i [AEC00000]
		{ //   
			pcan = &p->pmbx_cid_uni_bms_pc_i->ncan.can;
			if (for_us(pcan,p) == 0)
			{ // This CAN msg includes us.
				do_req_codes(pcan);
			}
		}	
		
		if ((noteval & CANCOMMBIT03) != 0) // CAN id: cid_uni_bms_emc1_i [B0000000]
		{
			pcan = &p->pmbx_cid_uni_bms_emc1_i->ncan.can;
			if (for_us(pcan,p) == 0)
			{ // This CAN msg includes us.
	//			do_req_codes(pcan);
			}
		}
		if ((noteval & CANCOMMBIT04) != 0) // CAN id: cid_uni_bms_emc2_i [B0200000]
		{ 
			pcan = &p->pmbx_cid_uni_bms_emc2_i->ncan.can;
			if (for_us(pcan,p) == 0)
			{ // This CAN msg includes us.
	//			do_req_codes(pcan);
			}			
		}
		
/* ******* Heartbeat timing: status */
		if 	((int)(xTaskGetTickCount() - p->HBstatus_ctr) > 0)
		{
			p->HBstatus_ctr += p->hbct_k;
			{
			/* Use dummy CAN msg, then it looks the same as a request CAN msg. */
				can_hb.cd.ull   = 0xffffffff; // Clear entire payooad
				can_hb.cd.uc[0] = CMD_CMD_MISCHB; // Misc subcommands code
				can_hb.cd.uc[1] = MISCQ_STATUS;   // status code
				cancomm_items_sendcmdr(&can_hb);  // Handles as if incoming CAN msg.
			}	
		}			
	}
}

/* *************************************************************************
 * TaskHandle_t xCanCommCreate(uint32_t taskpriority);
 * @brief	: Create task; task handle created is global for all to enjoy!
 * @param	: taskpriority = Task priority (just as it says!)
 * @return	: CanCommHandle
 * *************************************************************************/
TaskHandle_t xCanCommCreate(uint32_t taskpriority)
{
	BaseType_t ret = xTaskCreate(StartCanComm, "CanComm",\
     (128+32), NULL, taskpriority,\
     &CanCommHandle);
	if (ret != pdPASS) return NULL;

	return CanCommHandle;
}
/* *************************************************************************
 * static void canfilt(uint16_t mm, struct MAILBOXCAN* p);
 * @brief	: Setup CAN hardware filter with CAN addresses to receive
 * @param	: p    = pointer to ContactorTask
 * @param   : mm = morse_trap numeric number
 * *************************************************************************/
#if 0
static void canfilt(uint16_t mm, struct MAILBOXCAN* p)
{
/*	HAL_StatusTypeDef canfilter_setup_add32b_id(uint8_t cannum, CAN_HandleTypeDef *phcan, \
    uint32_t id,   \
    uint8_t  fifo );
 @brief	: Add a 32b id, advance bank number & odd/even
 * @param	: cannum = CAN module number 1, 2, or 3
 * @param	: phcan = Pointer to HAL CAN handle (control block)
 * @param	: id    = 32b CAN id
 * @param	: fifo  = fifo: 0 or 1
 * @return	: HAL_ERROR or HAL_OK
*/
	HAL_StatusTypeDef ret;	
	ret = canfilter_setup_add32b_id(1,&hcan1,p->ncan.can.id,0);
	if (ret == HAL_ERROR) morse_trap(mm);	
	return;
}
#endif
/* *************************************************************************
 * static void do_req_codes(struct CANRCVBUF* pcan);
 *	@brief	: Respond to the CAN msg request code
 *  @param  : pcan = point to CAN msg struct
 * *************************************************************************/
static void do_req_codes(struct CANRCVBUF* pcan)
{
	/* First payload byte holds root request code. */
	switch (pcan->cd.uc[0])
	{
	case LDR_RESET: // Execute a RESET ###############################
		#define SCB_AIRCR 0xE000ED0C
		*(volatile unsigned int*)SCB_AIRCR = (0x5FA << 16) | 0x4;// Cause a RESET
//		while (1==1);// Redundant. Reset means it is "gone"
		break; 

	case CMD_CMD_CELLPOLL: // (42) Queue BSMTask read, then send cells when read completes
		// If sufficient time between requests, queue a BMSTask read. If
		// the readings are not stale BMSTask will do an immediate notification.
//		if (toosoonchk(pcan) == 0)
//			CanComm_qreq(REQ_READBMS, 0, pcan);
		break;

	case CMD_CMD_TYPE2: // (43) Misc: Some may not need queueing BMSTask
		if (q_do(pcan) == 0) // Queue BMSTask request if applicable to command
		{ // Here, queueing not needed, so handle request now.
			cancomm_items_sendcmdr(pcan);
		}
		break;

	default:
//		bqfunction.warning = 551;
morse_trap(551);
		break;
	}
	return;
}
/* *************************************************************************
 * static uint8_t q_do(struct CANRCVBUF* pcan);
 *	@brief	: Queue a BMS task if necessary
 *  @param  : pcan = point to CAN msg struct
 * *************************************************************************/
static uint8_t q_do(struct CANRCVBUF* pcan)
{
	uint8_t qsw = 0;
#if 0	
	/* Command code. */
	switch(pcan->cd.uc[1])
	{
	/* Group not requiring a BMSTask reading */
	case MISCQ_STATUS:      // 1 status
 	case MISCQ_CELLV_ADC:   // 3 cell voltage: adc counts 		send_bms_array(po, &p->raw_filt[0], p->lc.ncell);
 	case MISCQ_DCDC_V:      // 6 isolated dc-dc converter output voltageloadfloat(puc, &adc1.abs[ADC1IDX_PA4_DC_DC].filt);
 	case MISCQ_CHGR_V:      // 7 charger hv voltage loadfloat(puc, &adc1.abs[ADC1IDX_PA7_HV_DIV].filt);

	case MISCQ_TOPOFSTACK: // BMS top-of-stack voltage		send_bms_array(po, &bqfunction.cal_filt[19], 1);
 	case MISCQ_PROC_CAL: // Processor ADC calibrated readings
 	/*
 		for (i = 0; i < ADC1DIRECTMAX; i++) // Copy struct items to float array
 			ftmp[i] = adc1.abs[i].filt;
 		ftmp[1] = adc1.common.degC; // Insert special internal temperature calibration 
 		send_bms_array(po, &ftmp[0], ADC1DIRECTMAX);
	*/
 	case MISCQ_PROC_ADC: // Processor ADC raw adc counts for making calibrations
	/*	for (i = 0; i < ADC1DIRECTMAX; i++) // Copy struct items to float array
 			ftmp[i] = adc1.abs[i].sumsave;
		send_bms_array(po, &ftmp[0], ADC1DIRECTMAX); 	 */
	case MISCQ_R_BITS:      // 21 Dump, dump2, heater, discharge bits
	case MISCQ_CURRENT_CAL: // 24 Below cell #1 minus, current resistor: calibrated
	case MISCQ_CURRENT_ADC: // 25 Below cell #1 minus, current resistor: adc counts	
		qsw = 0; // No need to queue a reading
		break;

	/* BMSTask: REQ_SETFETS */
 	/* ???? Does this require a read AUX registers? */
	case MISCQ_SETFETBITS: //  27 // Set FET on/off discharge bits
 		CanComm_qreq(REQ_SETFETS, pcan->cd.ui[1], pcan);
 		break;

	/* BMSTask: REQ_TEMPERATURE */
 	case MISCQ_TEMP_CAL:    // 4 temperature sensor: calibrated send_bms_array(po, &bqfunction.cal_filt[16], 3);
 	case MISCQ_TEMP_ADC:    // 5 temperature sensor: adc counts send_bms_array(po, &p->raw_filt[16], 3);
 		CanComm_qreq(REQ_TEMPERATURE, 0, pcan);
 		qsw = 1;
 		break;

 	/* Not implemented. uses AUX registers. */
 	case MISCQ_HALL_CAL:    // 8 Hall sensor: calibrated
 	case MISCQ_HALL_ADC:    // 9 Hall sensor: adc counts
 		break;

	/* BMSTask: REQ_READBMS */
 	case MISCQ_CELLV_CAL:   // 2 cell voltage: calibrated
 	case MISCQ_CELLV_HI:   // 10 Highest cell voltage 		loaduint32(puc, p->cellv_high);
 	case MISCQ_CELLV_LO:   // 11 Lowest cell voltage 		loaduint32(puc, p->cellv_low);
 		CanComm_qreq(REQ_READBMS, 0, pcan);
 		qsw = 1;
 		break;
	}
#endif	
	return qsw;
}
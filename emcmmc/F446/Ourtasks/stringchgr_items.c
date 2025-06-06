
/******************************************************************************
* File Name          : stringchgr_items.c
* Date First Issued  : 09/08/2024
* Description        : Battery string charging
*******************************************************************************/
#include "stringchgr_items.h"
#include "cancomm_items.h"
#include "bubblesort_uint32_t.h"
#include "EMCLTaskCmd.h"
//#include "../../../../BMS/bmsadbms1818/Ourtasks/cancomm_items.h"
#include "morse.h"

// Debug w main yprintf
uint8_t do_bms_status_flag;

uint8_t discovery_end_flag; // Signal main for printf'ing
uint8_t polltim_flag;

extern uint32_t swtim1_ctr; // Running count of swtim1 callbacks

// Make these static after debugging. 
void do_elcon_poll(void);
void do_mode(void);
void do_idle(struct ELCONSTUFF* pe);
void do_chrg(struct ELCONSTUFF* pe);
void do_relax(struct ELCONSTUFF* pe);
void do_dbg(struct ELCONSTUFF* pe);
int8_t do_bms_status_check(void);
void sendcan_type2(struct STRINGCHGRFUNCTION* ps, uint8_t code, uint8_t uc3);
void do_bms_status_poll(void);
void do_bms_chg_limits_poll(void);
void do_cmdpending(struct STRINGCHGRFUNCTION* ps);

/* *************************************************************************
 * void stringchgr_items_init(void);
 * @brief	: Init stuff
 * *************************************************************************/
void stringchgr_items_init(void)
{
	struct STRINGCHGRFUNCTION* ps = &emclfunction.lc.lcstring;
	struct ELCONSTUFF* pe = &ps->elconstuff;
//	struct ELCONSTUFF* pe = &emclfunction.lc.lcstring.elconstuff;

	int i;
	/* Init remap CAN ID array. */
	 // Show possible BMS node CAN ID positions have not been accessed
	for (i = 0; i < BMSNODEIDSZ; i++)
	{
		ps->remap[i] = -1;
	}
	ps->bmsnum         = 0;
	ps->status1        = 0;
	ps->vi_max_rpt     = 0; // Bits show module reported V & I max limits
	ps->cmdpendingidle = 0;
	ps->cmdpendingchg  = 0;
	ps->cmdpendingchg_prev = 0;
	ps->stsstate       = STSSTATE_DISCOVERY; // Begin with BMS discovery

	pe->pollflag       = 0;
	pe->discovery_ctr  = DISCOVERY_POLL_CT; // Count ELCON timed BMS polls for discovery

	pe->toct_poll_dur        = TIMCTR_ELCON_POLL; // Poll duration
	pe->toctr_elcon_poll     = pe->toct_poll_dur; // Set first delay
	pe->toctr_elcon_ready    = TIMCTR_ELCON_TIMOUT; // Receving ELCON mgs, but status shows not ready
	pe->toctr_elcon_pollflag = TIMCTR_ELCON_POLLFLAG; //

	return;
}
/* *************************************************************************
 * static void bms_response_init(void);
 * @brief	: Clear response flags for next poll (status)
 * *************************************************************************/
static void bms_response_init(void)
{
	int i;
	struct BMSTABLE* ptbl = &emclfunction.lc.lcstring.bmstable[0];
	for (i = 0; i < emclfunction.lc.lcstring.bmsnum; i++)
	{
		ptbl->idxbits = 0; // 18 cell index

// Are these needed?		
		ptbl->toctr_status = TIMCTR_TIMEOUT_BMS;
		ptbl->toctr_cell   = TIMCTR_TIMEOUT_BMS;

		ptbl += 1;
	}

	struct STRINGCHGRFUNCTION* ps = &emclfunction.lc.lcstring;
	ps->cellrcv   = 0; // Clear all notes received cell readings
	ps->statusrcv = 0;
	return;
}
/* *************************************************************************
 * static void bms_response_init37(void);
 * @brief	: Clear response flags for next poll (MISCQ_CHG_LIMITS)
 * *************************************************************************/
#if 0
static void bms_response_init37(void)
{
//	int i;
	struct BMSTABLE* ptbl = &emclfunction.lc.lcstring.bmstable[0];
	ptbl->idxbits = 0; // 18 cell index

// Are these needed?		
//		ptbl->toctr_status = TIMCTR_TIMEOUT_BMS;
//		ptbl->toctr_cell   = TIMCTR_TIMEOUT_BMS;

	return;
}
#endif
/* *************************************************************************
 * static uint16_t updatetable(struct CANRCVBUFS* pcans, uint8_t idx);
 * @brief	: Update table for BMS node, from status or BMS cell CAN msgs
 * @param	: pcans = pointer to CAN msg w selection code
 * @param   : idx = index in bmstbl array for this node (0 - BMSTABLESIZE)
 * *************************************************************************/
static uint16_t updatetable(struct CANRCVBUFS* pcans, uint8_t idx)
{
	struct STRINGCHGRFUNCTION* ps = &emclfunction.lc.lcstring;
	struct BMSTABLE* ptbl = &emclfunction.lc.lcstring.bmstable[idx];

	switch (pcans->can.cd.uc[0])
	{
	case CMD_CMD_MISCHB:   // 0x2D 45d Heartbeat 
	case CMD_CMD_TYPE2:    // 0x2B 43d format 2: request for misc
	case CMD_CMD_MISCEMC1: // 0x30 43d misc data: response to emc1 sent TYPE2		
		switch (pcans->can.cd.uc[1])
		{
		case MISCQ_STATUS: // Here, BMS node w status payload
			ptbl->batt = pcans->can.cd.uc[4]; // Save status bytes
			ptbl->fet  = pcans->can.cd.uc[5];
			ptbl->mode = pcans->can.cd.uc[6];
			ptbl->toctr_status = TIMCTR_TIMEOUT_BMS; // Reset timeout counter
			ps->statusrcv |= (1<<idx);  // Bit shows this node reported status
			if ((ptbl->batt & BSTATUS_CELLTOOHI) == 0)
			{ // No cells above max for this node
				ps->statusmax &= ~(1<<idx);  // Bit shows this node reported status
			}
			else
			{ // One or more cells above max for this node
				ps->statusmax |= (1<<idx);  // Bit shows this node reported status				
			}
			break;

		case MISCQ_CHG_LIMITS: // Here, BMS node w Voltage & Charge current max limits
			ptbl->v_max = pcans->can.cd.us[3]; // Module voltage max
			ptbl->i_max = pcans->can.cd.uc[4]; // Module charge current max
			ptbl->i_bal = pcans->can.cd.uc[5]; // Module balancing current max
			ps->vi_max_rpt |= (1<<idx);  // Bit shows this node reported v & i max
			break;
		}

	/* Various sources can send a command. */
	case CMD_CMD_CELLHB:   // 44 Heartbeat
	case CMD_CMD_CELLEMC1: // 46 EMC1 poll
	case CMD_CMD_CELLPC:   // 47 PC poll
	case CMD_CMD_CELLEMC2: // 51 EMC2 poll
		idx = pcans->can.cd.uc[1] >> 4;
		ptbl->cell[idx + 0] = pcans->can.cd.us[1];
		ptbl->cell[idx + 1] = pcans->can.cd.us[2];
		ptbl->cell[idx + 2] = pcans->can.cd.us[3];

		/* Build summation of cells. */
		ptbl->vsum_work += pcans->can.cd.us[1] +
		                   pcans->can.cd.us[2] + 
		                   pcans->can.cd.us[3];

		/* Set bit in word that shows each cell msg of group was received. */
		ptbl->idxbits |= (1<<idx);		                   

		/* Save time for checking for failure to respond timeout. */
		ptbl->toctr_cell = TIMCTR_TIMEOUT_BMS; // Reset timeout counter

		if (idx == 0xF)
		{ // Here, last CAN msg of group

			/* Check if all 6 msgs were received. */
/* The idx sequence in the cell reading msg is the beginning of the
three cell readings in the payload, and therefore,
0, 3, 6, 9, 12, 15
So, rather than suffer dividing by 3, and fitting it in a single byte,
a 16b half-word is used. Hence, 0x9249 for having received all 6 msgs.
*/ 
			if (ptbl->idxbits == 0x9249)
			{
				// Time of update for timeout checking cell msgs
				ptbl->vsum  = ptbl->vsum_work;
				// Set bit: received all the cell readings from this node.
				emclfunction.lc.lcstring.cellrcv |= (1<<idx); 
			}
			ptbl->vsum_work = 0;
		}
		break;
	}
	return ptbl->idxbits;	
}
/* *************************************************************************
 * void discovery_end(void);
 * @brief	: Summarize discovered BMS data
 * *************************************************************************/
uint8_t discovery_end_flag; // Signal main for printf'ing
void discovery_end(void)
{
	discovery_end_flag = 1;

	int i;
	// Convenience pointers
	struct STRINGCHGRFUNCTION* ps = &emclfunction.lc.lcstring;
	struct ELCONSTUFF* pe = &emclfunction.lc.lcstring.elconstuff;

	/* Find string voltage and minimum currents, i.e. the max for ELCON settings */
	uint8_t imax  = 255; // (25.5a) Max charging current
	uint8_t ibal  = 255; // (25.5a) Max balancing current
	uint32_t vtot = 0;   // Sum module max voltages
	for (i = 0; i < ps->bmsnum; i++)
	{
		if (ps->bmstable[i].i_max < imax) imax = ps->bmstable[i].i_max;
		if (ps->bmstable[i].i_bal < ibal) ibal = ps->bmstable[i].i_bal;
		vtot += ps->bmstable[i].v_max; 
	}
	// JIC the BMS module has these as zero.
	if (imax == 0) imax = DEFAULT_IMAX;
	if (ibal == 0) ibal = DEFULAT_IBAL;

	/* ELCON setting limits */
	ps->chgr_maxvolts = (float)vtot * 0.1f; // Set charger voltage limit (volts)
	ps->chgr_maxamps  = (float)imax * 0.1f; // Set charger current limit (amps)
	ps->chgr_balamps  = (float)ibal * 0.1f; // Balancing current limit (amps)

	/* Rescale parameter and convert to unit16_t. */
	pe->ichgr_maxvolts = ps->chgr_maxvolts*10.0f; // Set charger voltage limit (0.1 volts)
	pe->ichgr_setvolts = pe->ichgr_maxvolts;
	pe->ichgr_maxamps  = ps->chgr_maxamps*10.0f;  // Set charger current limit (0.1 amps)	

	return;
}
/* *************************************************************************
 * void do_timeoutcheck(void);
 * @brief	: Timer tick: Check time out counters and take action if necessary
 * *************************************************************************/
void do_timeoutcheck(void)
{
	int i;
	// Convenience pointers
	struct STRINGCHGRFUNCTION* ps = &emclfunction.lc.lcstring;
//	struct ELCONSTUFF* pe = &ps->elconstuff;
	struct ELCONSTUFF* pe = &emclfunction.lc.lcstring.elconstuff;

	struct BMSTABLE* ptbl = &emclfunction.lc.lcstring.bmstable[0];
	for (i = 0; i < ps->bmsnum; i++)
	{
		if (ptbl->toctr_cell > 0)
		{
			ptbl->toctr_cell -= 1;
			if (ptbl->toctr_cell <= 0)
			{ // Cell readings are now stale
			// do_statereset();
			// Set stale cell readings bit
			}
		}

		if (ptbl->toctr_status > 0)
		{
			ptbl->toctr_status -= 1;
			if (ptbl->toctr_status <= 0)
			{ // Status is not stale
			// do_statereset();
			// Set stale status msg bit	
			}
		}
		ptbl += 1;
	}

	/* Receiving ELCON msgs. */
	if (pe->toctr_elcon_rcv > 0)
	{
		pe->toctr_elcon_rcv -= 1;
		if (pe->toctr_elcon_rcv <= 0)
		{ // ELCON not responding
			pe->status_elcon |= ELCONSTATUS_TIMOUT;
		}
	}

	/* Polling cycle */
	if (pe->pollflag == 0)
	{ //Someone else is NOT polling ELCON
		pe->toctr_elcon_poll -= 1;
		if (pe->toctr_elcon_poll < 0)
		{ // Poll timeout ended.
polltim_flag = 1; // Trigger main for printf
			// Set duration for next ELCON poll msg
			pe->toctr_elcon_poll = pe->toct_poll_dur;
			switch (ps->stsstate)			
			{
			case STSSTATE_IDLE: // 0 Just poll ELCON to keep its COMM status alive
				do_cmdpending(ps); // Check for commands from PC
				break;
			case STSSTATE_CHRG: // 1 Charging in progress
				do_chrg(pe); 
				do_cmdpending(ps); // Check for commands from PC
				do_bms_status_poll(); // Poll all BMS for status
				do_elcon_poll(); // Poll ELCON				
				break;
			case STSSTATE_RELAX:// 2 Waiting for cells to relax after charge paused.
				do_relax(pe);
				do_cmdpending(ps); // Check for commands from PC
				do_bms_status_poll(); // Poll all BMS for status
				do_elcon_poll(); // Poll ELCON
				break;
			case STSSTATE_DBG:  // 3 PC or EMC direct setting of volts, amps, rate idx			
				do_dbg(pe);
				break;
			case STSSTATE_DISCOVERY: // 4 Discovery phase
				// Here, poll for voltage and current limits
				pe->discovery_ctr -= 1;
				if (pe->discovery_ctr <= 0)
				{ // End BMS discovery polling
					// Summarize, analyze, present the prizes garnered
					discovery_end();
					ps->stsstate = STSSTATE_IDLE;
				}
				else
				{ // Discovery phase still active
					do_bms_chg_limits_poll(); // BMS poll for charger limits
				}
				do_bms_status_poll(); // Poll all BMS for status
				do_elcon_poll(); // Poll ELCON
				break;
			default: // JIC the goober programmer screwed up.
				morse_trap(474);
				break;
			}
		}
	}		
	else
	{ // Here, someone else sent an ELCON poll msg. Timeout if no more received.
		pe->toctr_elcon_pollflag -= 1;
		if (pe->toctr_elcon_pollflag <= 0)
		{ // Here, it looks like that "someone" is no longer sending ELCON poll msgs
			pe->pollflag = 0; // Reset the wait flag
			pe->toctr_elcon_poll = TIMCTR_ELCON_POLL; // Regular poll next time thru routine
			stringchgr_items_init();  // Restart, as if a reboot
		}		
	}	

	/* Wink GREEN LED. */
static int32_t toctr_led;
	HAL_GPIO_WritePin(LED5_GRN_GPIO_Port,LED5_GRN_Pin,GPIO_PIN_SET); // GRN LED	off				
	toctr_led += 1;
	if (toctr_led >= (2000/SWTIME1PERIOD))
	{
		toctr_led = 0;
		HAL_GPIO_WritePin(LED5_GRN_GPIO_Port,LED5_GRN_Pin,GPIO_PIN_RESET); // GRN LED on
	}
	return;
}
/* *************************************************************************
 * void do_tableupdate(struct CANRCVBUFS* pcans);
 * @brief	: BMS msg received. 
 * @param	: pcans = pointer to CAN msg w selection code
 * *************************************************************************/
void do_tableupdate(struct CANRCVBUFS* pcans)
{
	struct STRINGCHGRFUNCTION* ps = &emclfunction.lc.lcstring;
	int8_t rmaptmp = pcans->pcl->rmap;

	if (ps->remap[rmaptmp] < 0) 
	{ // This CAN id has not been added to table
		if (ps->bmsnum < BMSTABLESIZE)
		{ // Space available: Add CAN id to table

			// pcans->pcl->rmap will get index into table
			ps->remap[rmaptmp] = ps->bmsnum; // Set remap ID to table

			// Copy CAN ID into table
			ps->bmstable[ps->bmsnum].id = pcans->can.id;

			// Array of pointers for sorting.
			ps->pbmstbl[ps->bmsnum] = &ps->bmstable[ps->bmsnum];

			// Update table items depending CAN msg payload
			updatetable(pcans, ps->bmsnum);

			// Advance size of BMS nodes discovered
			ps->bmsnum += 1;

			/* Sort array for accessing in CAN id sorted order. */
			if (ps->bmsnum > 1)
			{
				bubble_sort_uint32t(&ps->pbmstbl[0], ps->bmsnum);
			}

			if (ps->bmsnum < ps->bmsnum_expected)
				ps->status1 |= SCTSTATUS_FWR; // Fewer than expected reporting
			else if ((ps->bmsnum == ps->bmsnum_expected))
			{
				ps->status1 &= ~SCTSTATUS_FWR; // Fewer than expected reporting
				ps->status1 |=  SCTSTATUS_EXP; // Expected number reporting
			}
			return;
		}		
		else
		{ // Here, BMS CAN ID not in table, but array is full.
			ps->status1 &= ~SCTSTATUS_EXP; // Expected number reporting
			ps->status1 |=  SCTSTATUS_OVF; // Received more than table size
			return; // Keep-on-trucking jic
		}
	}
	/* remap holds index into BMS table. */
	updatetable(pcans, ps->remap[rmaptmp]);
	return;
}
/* *************************************************************************
 * void do_elcon(struct CANRCVBUFS* pcans);
 * @brief	: Handle ELCON CAN msg received. 
 * @param	: pcans = pointer to CAN msg w selection code
 * *************************************************************************/
void do_elcon(struct CANRCVBUFS* pcans)
{
	struct STRINGCHGRFUNCTION* ps = &emclfunction.lc.lcstring;
	struct ELCONSTUFF* pe = &emclfunction.lc.lcstring.elconstuff;

	/* Save usable bits in ELCON status byte. */
	pe->status_elcon &= ~0x1F; // 
	pe->status_elcon |= (pcans->can.cd.uc[4] & 0x1F); // Save ELCON status byte

	/* Show ELCON is sending its status msgs. */
	pe->status_elcon &= ~ELCONSTATUS_TIMOUT;

	/* Set next timeout period. */
	pe->toctr_elcon_ready = TIMCTR_ELCON_TIMOUT;
	pe->toctr_elcon_rcv   = TIMCTR_ELCON_RCV;

	/* If first 4 bits of elcon status are not zero, it is not ready to charge. */
	if ((pe->status_elcon & ((1<<5)-1)) != 0)
	{ // ELCON status may have a reason to not charge
		if (pe->toctr_elcon_ready <= 0)
		{
//$			pe->ichgr_setamps  = 0; // Don't try to charge (redundant)
//$			ps->chgr_rate_idx  = 0; // Charge current rate index (zero current)
		}
	}
	else
	{ /* Extract payload, convert to little endian; float; scale float. */
		uint32_t itmpvolts = __REVSH(pcans->can.cd.us[0]);//(pcans->can.cd.uc[0] << 8) | (pcans->can.cd.uc[1]);
		pe->fmsgvolts  = itmpvolts;
		pe->fmsgvolts *= 0.1f; // Scale to volts
		uint32_t itmpamps  = __REVSH(pcans->can.cd.us[1]);//(pcans->can.cd.uc[2] << 8) | (pcans->can.cd.uc[3]);
		pe->fmsgamps   = itmpamps;	
		pe->fmsgamps  *= 0.1f; // Scale to amps
	}
	return;
}
/* *************************************************************************
 * void sendcan_type2(void);
 * @brief	: Send a TYPE2 CAN msg request to all BMS
 * @param   : ps   = pointer to struct with "everything"
 * @param   : code = command code
 * @param   : uc3  = sub-code (e.g. 0|1 for on|off command)
 * *************************************************************************/
void sendcan_type2(struct STRINGCHGRFUNCTION* ps, uint8_t code, uint8_t uc3)
{
	ps->canbms.can.cd.uc[0] = CMD_CMD_TYPE2;
	ps->canbms.can.cd.uc[1] = (0x3 << 6); // All nodes respond
	ps->canbms.can.cd.uc[2] = code; // Request code
	ps->canbms.can.cd.uc[3] = uc3;  // Sub-code
	ps->canbms.can.cd.ui[1] = 0; // Reminder; no specific CAN ID
	xQueueSendToBack(CanTxQHandle,&ps->canbms,4);
	return;
}
/* *************************************************************************
 * void do_bms_status_poll(void);
 * @brief	: Poll all BMS for status
 * *************************************************************************/
void do_bms_status_poll(void)
{
	struct STRINGCHGRFUNCTION* ps = &emclfunction.lc.lcstring;
	sendcan_type2(ps, MISCQ_STATUS,0); // 1 status

	/* Reset bits that show a response to this poll was received. */
	bms_response_init();	
	return;
}
/* *************************************************************************
 * void do_bms_chg_limits_poll(void);
 * @brief	: Poll all BMS for voltage and current limits
 * *************************************************************************/
void do_bms_chg_limits_poll(void)
{
	struct STRINGCHGRFUNCTION* ps = &emclfunction.lc.lcstring;
	sendcan_type2(ps, MISCQ_CHG_LIMITS,0); // 37d: Req volt & current limits
	return;
}
/* *************************************************************************
 * void do_elcon_poll(void);
 * @brief	: Send CAN msg to ELCON
 * *************************************************************************/
uint8_t dbg_do_elcon_poll_flag;
uint8_t dbg_do_elcon_poll_status_elcon;
int32_t dbg_do_elcon_poll_toctr_elcon_rcv;
int32_t dbg_amps;

void do_elcon_poll(void)
{
	struct ELCONSTUFF* pe = &emclfunction.lc.lcstring.elconstuff;
	struct CANTXQMSG*  px = &emclfunction.lc.lcstring.canelcon;

dbg_do_elcon_poll_status_elcon = pe->status_elcon;
dbg_do_elcon_poll_toctr_elcon_rcv = pe->toctr_elcon_rcv;

	if ((pe->status_elcon != 0) || (pe->toctr_elcon_rcv <= 0))
	{ // ELCON not ready or ELCON not responding
		px->can.cd.ui[0] = 0; // Zero volts AND amps
		px->can.cd.uc[4] = 1; // Charging stop

dbg_do_elcon_poll_flag = 1;		
	}
	else
	{
dbg_do_elcon_poll_flag = 2;

	// Make 16b payload big-endian with __REVSH
		px->can.cd.us[0] = __REVSH(pe->ichgr_setvolts); //(ichgr_setvolts >> 8) || (ichgr_setvolts << 8);
		px->can.cd.us[1] = __REVSH(pe->ichgr_setamps);  //(ichgr_setamps  >> 8) || (ichgr_setamps  << 8);	
		px->can.cd.uc[4] = 0; // Charging start
	}
	
//px->can.cd.us[0] = __REVSH((uint16_t)2160); //(ichgr_setvolts >> 8) || (ichgr_setvolts << 8);
//px->can.cd.us[1] = __REVSH((uint16_t)6);  //(ichgr_setamps  >> 8) || (ichgr_setamps  << 8);		
//px->can.cd.uc[4] = 0;
dbg_amps = pe->ichgr_setamps;
	xQueueSendToBack(CanTxQHandle,px,4);

	return;
}
/* *************************************************************************
 * void do_emc_cmds(struct CANRCVBUFS* pcans);
 * @brief	: Commands from either PC, EMC, or deus ex machina
 * @param	: pcans = pointer to CAN msg w selection code
 * *************************************************************************/
uint16_t pccmd; // Debug
void do_emc_cmds(struct CANRCVBUFS* pcans)
{
	struct STRINGCHGRFUNCTION* ps= &emclfunction.lc.lcstring;
	struct ELCONSTUFF* pe = &emclfunction.lc.lcstring.elconstuff;

// Save for main & debugging
pccmd = pcans->can.cd.us[0];

	if (pcans->can.cd.uc[0] == CMD_EMC_SETMODE)
	{ // Set EMC mode
		switch (pcans->can.cd.uc[1])
		{
		case EMCCMD_ELIDLE: // 0 // ELCON: Self discharging/stop
			ps->cmdpendingidle = 1; // Set a pending command
			pe->ichgr_setamps  = 0;
			ps->chgr_rate_idx  = 0; // Charge current rate index
			break;

		case EMCCMD_ELCHG:  // 1 // ELCON: Start charging
			ps->cmdpendingchg = 1; // Set a pending command
			break;

		case EMCCMD_LCCHG: //  2 // Node Low Current: set charging mode
			sendcan_type2(ps, MISCQ_SET_SELFDCHG, 0); // 31 Set ON|OFF self-discharge mode
			break;

		case EMCCMD_LCIDLE: // 3 // Node Low Current: self discharging/stop charging		
			sendcan_type2(ps, MISCQ_SET_SELFDCHG, 1); // 31 Set ON|OFF self-discharge mode
			break;	
		}
	}
	return;
}
/* *************************************************************************
 * int8_t do_bms_status_check(void);
 * @brief	: 
 * @return  : 0 = yes; -1 = no
 * *************************************************************************/
int8_t do_bms_status_check(void)
{
do_bms_status_flag = 1; // Signal main to print status

	struct STRINGCHGRFUNCTION* ps= &emclfunction.lc.lcstring;
	
	// Check if poll resulted with all nodes reported status 
	if (ps->statusrcv == ((1 << ps->bmsnum) - 1))
	{
		ps->status2 |= STS2_NODES_RPT; // All discovered nodes reported
		return 0;
	}
	ps->status2 &= ~STS2_NODES_RPT; // One or more nodes missing
	return -1;
}
/* *************************************************************************
 * void do_chrg(struct ELCONSTUFF* pe);
 * @briefichgr_setvolts	: Poll dur end:start--STSSTATE_CHRG   1  // Charging in progress
 * @param	: pe = pointer struct with all this mess
 * *************************************************************************/
void do_chrg(struct ELCONSTUFF* pe)
{
	struct STRINGCHGRFUNCTION* ps= &emclfunction.lc.lcstring;

	/* Any cell in any node above max? */
	if (ps->statusmax == 0)
	{ // No. Keep-on-truck'n
		return; 
	}
	// Yes. Reduce charge current after a relax duration
	if (ps->chgr_rate_idx > 0)
	{
		ps->chgr_rate_idx -= 1;
		if (ps->chgr_rate_idx == 0)
		{ // End of stepping down charge current rate
			ps->stsstate = STSSTATE_IDLE; // DONE!
			ps->chgr_rate_idx  = 0; // Charge current rate index
			sendcan_type2(ps, MISCQ_SET_SELFDCHG, 0); // 31 Set node chargers ON
		}
		else
		{ // Do relax duration before setting new, lower current.
			ps->stsstate = STSSTATE_RELAX;
			ps->toctr_relax = TIMCTR_RELAX; // Relax duration timer ticks
		}
		pe->ichgr_setamps  = 0;
	}
}
/* *************************************************************************
 * void do_relax(struct ELCONSTUFF* pe);
 * @brief	: Poll dur end:start--STSSTATE_RELAX  2  // Waiting for cells to relax after charge stopped.
 * @param	: pe = pointer struct with all this mess
 * *************************************************************************/
void do_relax(struct ELCONSTUFF* pe)
{
	struct STRINGCHGRFUNCTION* ps= &emclfunction.lc.lcstring;
	if (ps->toctr_relax > 0)
	{
		ps->toctr_relax -= 1;
		if (ps->toctr_relax > 0)
			return;

	 /* Relax time-out */
		/* Any cell in any node still above max? */
		if (ps->statusmax == 0)
		{ // No. Set new (reduced) charge current
			ps->chgr_rate_idx -= 1;
			if (ps->chgr_rate_idx <= 0)
			{ // End of stepping down charge current rate
				ps->stsstate = STSSTATE_IDLE; // ===> DONE! <===
				pe->ichgr_setamps  = 0;
				sendcan_type2(ps, MISCQ_SET_SELFDCHG, 0); // (31d) Set node chargers ON
			}
			else
			{ // Resume charging at previously computed reduced rate
				ps->stsstate = STSSTATE_CHRG; // Back to charging state
			}
		}
		else
		{ // Yes. Do another relax cycle and step the current down
			ps->chgr_rate_idx -= 1;
			if (ps->chgr_rate_idx <= 0)
			{
			 // End of stepping down charge current rate
				ps->stsstate = STSSTATE_IDLE; // ===> DONE! <===
				pe->ichgr_setamps  = 0;
				sendcan_type2(ps, MISCQ_SET_SELFDCHG, 0); // (31d) Set node chargers ON
			}
			else
			{ // Do another relax duration
				ps->toctr_relax = TIMCTR_RELAX; // Relax duration timer ticks
			}
		}
	}
	return;
}
/* *************************************************************************
 * void do_idle(struct ELCONSTUFF* pe);
 * @brief	: Poll dur end:start--STSSTATE_IDLE   0  // Just poll ELCON to keep its COMM status alive
 * @param	: pe = pointer struct with all this mess
 * *************************************************************************/
void do_idle(struct ELCONSTUFF* pe)
{
	pe->ichgr_setamps  = 0;
	return;
}
/* *************************************************************************
 * void do_dbg(struct ELCONSTUFF* pe);
 * @brief	: Poll dur end:start--STSSTATE_DBG    3  // PC or EMC direct setting of volts, amps, rate idx
 * @param	: pe = pointer to struct with alldo_bms_status_poll this mess
 * *************************************************************************/
void do_dbg(struct ELCONSTUFF* pe)
{
	// TODO
}
/* *************************************************************************
 * void do_cmdpending(struct STRINGCHGRFUNCTION* ps);
 * @brief	: Handle pending commands from PC
 * @param	: ps = pointer struct with string function stuff
 * *************************************************************************/
void do_cmdpending(struct STRINGCHGRFUNCTION* ps)
{
	struct ELCONSTUFF* pe = &emclfunction.lc.lcstring.elconstuff;
	/* Avoid problem of consecutive charge commands. */
	if (ps->cmdpendingchg != 0)
	{ // Command pending to set charge mode
		if (ps->cmdpendingchg_prev == 0)
		{
			ps->cmdpendingchg_prev = 1;
			ps->stsstate = STSSTATE_CHRG;
			ps->chgr_rate_idx = CHGRATENUM;
			pe->ichgr_setamps = pe->ichgr_maxamps;
			pe->ichgr_setvolts = pe->ichgr_maxvolts;
		}
	}

	/* PC wants return to IDLE */
	if (ps->cmdpendingidle != 0)
	{ // Command pending to set back to idle mode
//		stringchgr_items_init();
		// Override init
		ps->stsstate = STSSTATE_IDLE;
		ps->cmdpendingchg_prev = 0;
	}
	return;
}
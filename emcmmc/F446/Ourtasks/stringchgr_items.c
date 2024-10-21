
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

extern uint32_t swtim1_ctr; // Running count of swtim1 callbacks

// Make these static after debugging. 
void do_elcon_poll(void);
void do_mode(void);
void do_idle(struct ELCONSTUFF* pe);
void do_chrg(struct ELCONSTUFF* pe);
void do_relax(struct ELCONSTUFF* pe);
void do_dbg(struct ELCONSTUFF* pe);
int8_t do_bms_status_check(void);
void do_bms_send(struct STRINGCHGRFUNCTION* ps, uint8_t code, uint8_t uc3);

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
	ps->status1 = 0;
	ps->mode    = 0;

	/* Rescale parameter and convert to unit16_t. */
	pe->ichgr_maxvolts = ps->chgr_maxvolts*10.0f; // Set charger voltage limit (0.1 volts)
	pe->ichgr_setvolts = pe->ichgr_maxvolts;
	pe->ichgr_maxamps  = ps->chgr_maxamps*10.0f;  // Set charger current limit (0.1 amps)

	pe->toct_poll_dur     = TIMCTR_ELCON_POLL; // Poll duration
	pe->toctr_elcon_poll  = pe->toct_poll_dur; // Set first delay
	pe->toctr_elcon_ready = TIMCTR_ELCON_TIMOUT; // Receving ELCON mgs, but status shows not ready


	return;
}
/* *************************************************************************
 * static void bms_response_init(void);
 * @brief	: Clear response flags for next poll
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
 * tatic uint16_t updatetable(struct CANRCVBUFS* pcans, uint8_t idx);
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
	case CMD_CMD_MISCHB: // 45 Heartbeat 
		if (pcans->can.cd.uc[1] == MISCQ_STATUS)
		{ // Here, BMS node w status payload
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
		}
		break;

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

	/* Transmit ELCON w charinging logic/states. */
	if (pe->toctr_elcon_poll > 0)
	{ 
		pe->toctr_elcon_poll -= 1;
		if (pe->toctr_elcon_poll <= 0)
		{
			if (do_bms_status_check() == 0)
			{ // Poll cycle ends:start next cycle
				switch (ps->mode)			
				{
				case STSSTATE_IDLE: // 0 Just poll ELCON to keep its COMM status alive
					break;
				case STSSTATE_CHRG: // 1 Charging in progress
					do_chrg(pe); 
					break;
				case STSSTATE_RELAX:// 2 Waiting for cells to relax after charge stopped.
					do_relax(pe);
					break;
				case STSSTATE_DBG:  // 3 PC or EMC direct setting of volts, amps, rate idx			
					do_dbg(pe);
					break;
				}
				do_bms_status_poll(); // Start next BMS poll
			}
			else
			{ // Error: All nodes in table did not return a status in this interval

			}			
			// Set duration for next ELCON poll msg
			pe->toctr_elcon_poll = pe->toct_poll_dur; 
			do_elcon_poll();
		}
	}	

	if (pe->toctr_elcon_ready > 0)
	{
		pe->toctr_elcon_ready -= 1;
		if (pe->toctr_elcon_ready <= 0)
		{ // Not receiving ELCON status msgs
			pe->status_elcon |= ELCONSTATUS_TIMOUT;
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

			// Update table items depending CAN msg payloard
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

	/* If first 4 bits of elcon status are not zero, it is not ready to charge. */
	if ((pe->status_elcon & ((1<<5)-1)) != 0)
	{ // ELCON status may have a reason to not charge
		if (pe->toctr_elcon_ready <= 0)
		{
			pe->ichgr_setamps  = 0; // Don't try to charge (redundant)
			ps->chgr_rate_idx  = 0; // Charge current rate index (zero current)
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
 * void do_bms_send(void);
 * @brief	: Poll BMS for status
 * *************************************************************************/
void do_bms_send(struct STRINGCHGRFUNCTION* ps, uint8_t code, uint8_t uc3)
{
	ps->canbms.can.cd.uc[0] = CMD_CMD_TYPE2;
	ps->canbms.can.cd.uc[1] = (0x3 << 6); // All nodes respond
	ps->canbms.can.cd.uc[2] = code; // Request code
	ps->canbms.can.cd.uc[3] = uc3;  // Sub-code
	ps->canbms.can.cd.ui[1] = 0; // Reminder; no specific CAN ID

	/* Reset bits that show a response to this poll was received. */
	bms_response_init();	

	xQueueSendToBack(CanTxQHandle,&ps->canbms,4);
	return;
}
/* *************************************************************************
 * void do_bms_status_poll(void);
 * @brief	: Poll BMS for status
 * *************************************************************************/
void do_bms_status_poll(void)
{
	struct STRINGCHGRFUNCTION* ps = &emclfunction.lc.lcstring;
	do_bms_send(ps, MISCQ_STATUS,0); // 1 status

	/* Reset bits that show a response to this poll was received. */
	bms_response_init();	
	return;
}
/* *************************************************************************
 * void do_elcon_poll(void);
 * @brief	: Send CAN msg to ELCON
 * *************************************************************************/
void do_elcon_poll(void)
{
	struct ELCONSTUFF* pe = &emclfunction.lc.lcstring.elconstuff;
	struct CANTXQMSG*  px = &emclfunction.lc.lcstring.canelcon;

	if ((pe->status_elcon != 0) || (pe->toctr_elcon_rcv <= 0))
	{ // ELCON not ready or ELCON not responding
		px->can.cd.ui[0] = 0; // Zero volts and amps
	}
	else
	{
	// Make 16b payload big-endian with __REVSH
		px->can.cd.us[0] = __REVSH(pe->ichgr_setvolts); //(ichgr_setvolts >> 8) || (ichgr_setvolts << 8);
		px->can.cd.us[1] = __REVSH(pe->ichgr_setamps);  //(ichgr_setamps  >> 8) || (ichgr_setamps  << 8);	
	}
	px->can.cd.uc[4] = 1;
	xQueueSendToBack(CanTxQHandle,px,4);
	return;
}
/* *************************************************************************
 * void do_emc_cmds(struct CANRCVBUFS* pcans);
 * @brief	: Commands from either PC, EMC, or deus ex machina
 * @param	: pcans = pointer to CAN msg w selection code
 * *************************************************************************/
void do_emc_cmds(struct CANRCVBUFS* pcans)
{
	struct STRINGCHGRFUNCTION* ps= &emclfunction.lc.lcstring;
	struct ELCONSTUFF* pe = &emclfunction.lc.lcstring.elconstuff;

	if (pcans->can.cd.uc[0] == CMD_EMC_SETMODE)
	{ // Set EMC mode
		switch (pcans->can.cd.uc[1])
		{
		case EMCCMD_ELIDLE: // 0 // ELCON: Self discharging/stop
			// Stop charging now.
			ps->mode = EMCCMD_ELIDLE;
			pe->ichgr_setamps  = 0;
			ps->chgr_rate_idx  = 0; // Charge current rate index
			break;

		case EMCCMD_ELCHG:  // 1 // ELCON: Start charging
			// Ignore multiple requests
			if (ps->mode == EMCCMD_ELIDLE)
			{ // Presently not charging mode
				ps->mode = EMCCMD_ELCHG; // Set charging mode
				ps->chgr_rate_idx  = (CHGRATENUM-1); // Start at max rate
				ps->stsstate = STSSTATE_CHRG;
				pe->ichgr_setamps = ps->chgr_rate[ps->chgr_rate_idx] * 10.0f; 
				pe->toctr_elcon_poll = TIMCTR_ELCON_POLL;
				do_elcon_poll();
			}
			break;

		case EMCCMD_LCCHG: //  2 // Node Low Current: set charging mode
			do_bms_send(ps, MISCQ_SET_SELFDCHG, 0); // 31 Set ON|OFF self-discharge mode
			break;

		case EMCCMD_LCIDLE: // 3 // Node Low Current: self discharging/stop charging		
			do_bms_send(ps, MISCQ_SET_SELFDCHG, 1); // 31 Set ON|OFF self-discharge mode
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
 * @brief	: Poll dur end:start--STSSTATE_CHRG   1  // Charging in progress
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
			ps->mode = STSSTATE_IDLE; // DONE!
			pe->ichgr_setamps  = 0;
			ps->chgr_rate_idx  = 0; // Charge current rate index
			do_bms_send(ps, MISCQ_SET_SELFDCHG, 0); // 31 Set node chargers ON
		}
		else
		{ // Do relax duration before setting new, lower current.
			ps->mode = STSSTATE_RELAX;
			ps->toctr_relax = TIMCTR_RELAX; // Relax duration timer ticks
			pe->ichgr_setamps  = 0;
		}
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
		{ // Waiting for cell volts to settle 
		}
		else
		{ // Relax time-out: check next step
			/* Any cell in any node above max? */
			if (ps->statusmax == 0)
			{ // No. Set new (reduced) charge current
				ps->mode = STSSTATE_CHRG; // Back to charging state
				pe->ichgr_setamps = ps->chgr_rate[ps->chgr_rate_idx] * 10.0f;
			}
			else
			{ // Yes. relax some more
				if (ps->chgr_rate_idx > 0)
				{
					ps->chgr_rate_idx -= 1;
					if (ps->chgr_rate_idx == 0)
					{ // End of stepping down charge current rate
						ps->mode = STSSTATE_IDLE; // ===> DONE! <===
						ps->chgr_rate_idx  = 0; // Charge current rate index
						do_bms_send(ps, MISCQ_SET_SELFDCHG, 0); // 31 Set node chargers ON
						pe->ichgr_setamps  = 0;

					}
				}
				else
				{ // Do another relax duration
					ps->toctr_relax = TIMCTR_RELAX; // Relax duration timer ticks
				}
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
 * @param	: pe = pointer struct with all this mess
 * *************************************************************************/
void do_dbg(struct ELCONSTUFF* pe)
{
	// TODO
}
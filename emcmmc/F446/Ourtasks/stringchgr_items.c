
/******************************************************************************
* File Name          : stringchgr_items.c
* Date First Issued  : 09/08/2024
* Description        : Battery string charging
*******************************************************************************/
#include "stringchgr_items.h"
#include "cancomm_items.h"
#include "bubblesort_uint32_t.h"
#include "EMCLTaskCmd.h"

extern uint32_t swtim1_ctr; // Running count of swtim1 callbacks

void do_elcon_poll(void);

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
	pe->ichgr_maxvolts = ps->chgr_maxvolts*10; // Set charger voltage limit (0.1 volts)
	pe->ichgr_maxamps  = ps->chgr_maxamps*10;  // Set charger current limit (0.1 amps)

	pe->toct_poll_dur    = TIMCTR_ELCON_POLL; // Poll duration
	pe->toctr_elcon_poll = pe->toct_poll_dur; // Set first delay

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
	struct BMSTABLE* ptbl = &emclfunction.lc.lcstring.bmstable[idx];

	switch (pcans->can.cd.uc[0])
	{
	case CMD_CMD_MISCHB: // 45 Heartbeat 
		if (pcans->can.cd.uc[1] == MISCQ_STATUS)
		{ // Here, BMS node w status payload
			ptbl->batt = pcans->can.cd.uc[4]; // Save status bytes
			ptbl->fet  = pcans->can.cd.uc[5];
			ptbl->mode = pcans->can.cd.uc[6];
			emclfunction.lc.lcstring.statusrcv |= (1<<idx);  // Bit shows this node reported status
			ptbl->toctr_status = TIMCTR_TIMEOUT_BMS; // Reset timeout counter
		}
		break;

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
 * @brief	: Check time out counters and take action if necessary
 * *************************************************************************/
void do_timeoutcheck(void)
{
	int i;
	// Convenience pointers
	struct STRINGCHGRFUNCTION* ps = &emclfunction.lc.lcstring;
	struct ELCONSTUFF* pe = &ps->elconstuff;

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
		{
		//	do_state_reset();
		//  set ELON error status bit
		}
	}

	if (pe->toctr_wait1 > 0)
	{
		pe->toctr_wait1 -= 1;
		if (pe->toctr_wait1 <= 0)
		{ // Delay timed out
			// do_statereset();
		}
	}		

	if (pe->toctr_wait2 > 0)
	{
		pe->toctr_wait2 -= 1;
		if (pe->toctr_wait2 <= 0)
		{ // Delay timed out

		}
	}

	if (pe->toctr_elcon_poll > 0)
	{ 
		pe->toctr_elcon_poll -= 1;
		if (pe->toctr_elcon_poll <= 0)
		{ // Poll (send) ELCON
			// Set wait for sending next ELCON poll msg
			pe->toctr_elcon_poll = pe->toct_poll_dur; 
			do_elcon_poll();
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
 * void do_state_reset(void);
 * @brief	: 
 * *************************************************************************/
void do_state_reset(void)
{
	struct STRINGCHGRFUNCTION* ps = &emclfunction.lc.lcstring;
	struct ELCONSTUFF* pe = &emclfunction.lc.lcstring.elconstuff;
	pe->toctr_wait1    = 0; // Clear timeout waits
	pe->toctr_wait2    = 0;
	pe->ichgr_setvolts = 0;
	pe->ichgr_setamps  = 0;
	ps->chgr_rate_idx  = 0; // Charge current rate index
	ps->mode &= ~MODE_CHRGING; // Set mode to not charging
	return;
}
/* *************************************************************************
 * void do_elcon(struct CANRCVBUFS* pcans);
 * @brief	: Handle ELCON CAN msg received. 
 * @param	: pcans = pointer to CAN msg w selection code
 * *************************************************************************/
void do_elcon(struct CANRCVBUFS* pcans)
{
//	struct STRINGCHGRFUNCTION* ps = &emclfunction.lc.lcstring;
	struct ELCONSTUFF* pe = &emclfunction.lc.lcstring.elconstuff;
	pe->status_elcon = pcans->can.cd.uc[4]; // Save ELCON status byte
	if (pe->status_elcon != 0)
	{ // ELCON has a reason to not charge
		do_state_reset();
	}

	uint32_t itmpvolts = (pcans->can.cd.uc[0] << 8) | (pcans->can.cd.uc[1]);
	uint32_t itmpamps  = (pcans->can.cd.uc[2] << 8) | (pcans->can.cd.uc[3]);
	pe->fmsgvolts = itmpvolts;
	pe->fmsgamps  = itmpamps;	
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
		px->can.cd.uc[4] = 1;
	}
	else
	{
	// Make 16b payload big-endian with __REVSH
		px->can.cd.us[0] = __REVSH(pe->ichgr_setvolts); //(ichgr_setvolts >> 8) || (ichgr_setvolts << 8);
		px->can.cd.us[1] = __REVSH(pe->ichgr_setamps);  //(ichgr_setamps  >> 8) || (ichgr_setamps  << 8);
		px->can.cd.uc[4] = 1;
	}
	/* Reset bits that show a response to this poll was received. */
	bms_response_init();

	xQueueSendToBack(CanTxQHandle,px,4);
	return;
}
/* *************************************************************************
 * void do_emc_cmds(struct CANRCVBUFS* pcans);
 * @brief	: Commands from either PC, or EMC 
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
		case STRMODE_CHGR: // Request charging mode
			// Ignore multiple requests
			if (ps->mode == STRMODE_SELF)
			{
				do_elcon_poll();
				ps->mode = STRMODE_CHGR;
				ps->chgr_rate_idx  = (CHGRATENUM-1); // Set max rate
				pe->ichgr_setamps = ps->chgr_rate[ps->chgr_rate_idx];
			}
			break;

		case STRMODE_SELF:
			// Stop charging now.
			ps->mode = STRMODE_SELF;
			do_state_reset();
			pe->toctr_elcon_poll = TIMCTR_ELCON_POLL;
			break;
		}
	}
	return;
}
/* *************************************************************************
 * int8_t do_bms_status_check(struct CANRCVBUFS* pcans);
 * @brief	: Check statusrcv to see if all nodes sent a status msg
 * @param	: pcans = pointer to CAN msg w selection code 
 * @return  : 0 = yes; -1 = no
 * *************************************************************************/
int8_t do_bms_status_check(struct CANRCVBUFS* pcans)
{
	struct STRINGCHGRFUNCTION* ps= &emclfunction.lc.lcstring;
	// Are node bits for the nodes in the table ON?
	if (ps->statusrcv == ((1 << ps->bmsnum) - 1))
		return 0; // Yes
	return -1; // No
}
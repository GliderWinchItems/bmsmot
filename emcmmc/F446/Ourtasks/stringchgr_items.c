
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

/* Table with data for each BMS node on the string. */
struct BMSTABLE bmstable[BMSTABLESIZE];
uint8_t bmsnum; // Number of reported BMS nodes

/* Pointers for accessing table in sorted CAN id order. */
struct BMSTABLE* pbmstbl[BMSTABLESIZE];

/* Indices for accessing table element, given a lookup mapped index. */
// Entry every possible BMS CAN id. -1 is not in table; >= 0 is index of table
static int8_t remap[BMSNODEIDSZ]; 

static uint8_t status1;  // Bits on BMS number reporting
static uint16_t ichgr_maxvolts; // Parameter float(volts) to uint32_t ui(0.1v)
static uint16_t ichgr_maxamps;  // Parameter float(amps) to uint32_t ui(0.1a)
static uint16_t ichgr_setvolts;
static uint16_t ichgr_setamps;


/* *************************************************************************
 * void stringchgr_items_init(void);
 * @brief	: Init stuff
 * *************************************************************************/
void stringchgr_items_init(void)
{
	struct EMCLLC* p = &emclfunction.lc;
	int i;
	/* Remap CAN ID array. */
	for (i = 0; i < BMSNODEIDSZ; i++)
	{
		remap[i] = -1; // Show BMS node CAN ID position not accessed
	}
	status1 = 0;

	ichgr_maxvolts = p->lcstring.chgr_maxvolts*10; // Set charger voltage limit (0.1 volts)
	ichgr_maxamps  = p->lcstring.chgr_maxamps*10;  // Set charger current limit (0.1 amps)

	return;
}

/* *************************************************************************
 * static void updatetable(struct BMSTABLE* ptbl, struct CANRCVBUFS* pcans);
 * @brief	: Update table for BMS node
 * @param   : ptbl = pointer to table entry for this BMS node
 * @param	: pcans = pointer to CAN msg w selection code
 * *************************************************************************/
static void updatetable(struct BMSTABLE* ptbl, struct CANRCVBUFS* pcans)
{
	uint8_t idx;
	switch (pcans->can.cd.uc[0])
	{
	case CMD_CMD_MISCHB: // 45 Heartbeat 
		if (pcans->can.cd.uc[1] == MISCQ_STATUS)
		{ // Here, BMS node w status payload
			ptbl->batt = pcans->can.cd.uc[4];
			ptbl->fet  = pcans->can.cd.uc[5];
			ptbl->mode = pcans->can.cd.uc[6];
			// Time of update for timeout checking status msgs
			ptbl->toctr_status = swtim1_ctr; // Running count of swtim1 callbacks
			ptbl->stale_status = 0;
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

		/* Set bit to show each cell msg of group received. */
		ptbl->idxbits |= (1<<idx);		                   

		/* Save time for checking for failure to respond timeout. */
		ptbl->toctr_cell = swtim1_ctr;

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
				ptbl->toctr_cell = swtim1_ctr; // Running count of swtim1 callbacks	
				ptbl->vsum       = ptbl->vsum_work;
				ptbl->stale_cell = 0; 
			}
			ptbl->vsum_work = 0;	
			ptbl->idxbits   = 0;			
		}
		break;
	}
	return;	
}
/* *************************************************************************
 * void do_timeoutcheck(void);
 * @brief	: Scan BMS table and mark those that have timed out
 * @return	: stale_status and stale_cell set if timed out
 * *************************************************************************/
void do_timeoutcheck(void)
{
	int i;
	struct BMSTABLE* ptbl = &bmstable[0];
	for (i = 0; i < bmsnum; i++)
	{
		if ((int32_t)(swtim1_ctr - ptbl->toctr_cell) > BMSTIMEOUT)
		{ // The data in this BMS table entry are stale.
			ptbl->stale_cell = 1;
		}
		if ((int32_t)(swtim1_ctr - ptbl->toctr_status) > BMSTIMEOUT)
		{ // The data in this BMS table entry are stale.
			ptbl->stale_status = 1;
		}
		ptbl += 1;
	}
	return;
}
/* *************************************************************************
 * int8_t do_tableupdate(struct CANRCVBUFS* pcans);
 * @brief	: BMS msg received. 
 * @param	: pcans = pointer to CAN msg w selection code
 * @return	: 0 = updated; 1 = added entry and updated
 * *************************************************************************/
int8_t do_tableupdate(struct CANRCVBUFS* pcans)
{
	int8_t rmaptmp = pcans->pcl->rmap;
	if (remap[rmaptmp] < 0) 
	{ // This CAN id has not been added to table
		if (bmsnum < BMSTABLESIZE)
		{ // Space available: Add CAN id to table

			// pcans->pcl->rmap will get index into table
			remap[rmaptmp] = bmsnum; // Set remap ID to table

			// Copy CAN ID into table
			bmstable[bmsnum].id = pcans->can.id;

			// Array of pointers for sorting.
			pbmstbl[bmsnum] = &bmstable[bmsnum];

			// Update table items depending CAN msg payloard
			updatetable(&bmstable[bmsnum], pcans);

			// Advance size of BMS nodes discovered
			bmsnum += 1;

			/* Sort array for accessing in CAN id sorted order. */
			if (bmsnum > 1)
			{
				bubble_sort_uint32t(&pbmstbl[0], bmsnum);
			}

			if (bmsnum < emclfunction.lc.lcstring.bmsnum_expected)
				status1 |= SCTSTATUS_FWR; // Fewer than expected reporting
			else if ((bmsnum == emclfunction.lc.lcstring.bmsnum_expected))
			{
				status1 &= ~SCTSTATUS_FWR; // Fewer than expected reporting
				status1 |=  SCTSTATUS_EXP; // Expected number reporting
			}
			return 0;
		}		
		else
		{ // Here, BMS CAN ID not in table, but array is full.
			status1 &= ~SCTSTATUS_EXP; // Expected number reporting
			status1 |=  SCTSTATUS_OVF; // Received more than table size
			return -1; // Keep-on-trucking jic
		}
	}
	/* remap holds index into BMS table. */
	uint8_t idx = remap[rmaptmp];
	updatetable(&bmstable[idx], pcans);
	return 0;
}
/* *************************************************************************
 * void do_elcon(struct CANRCVBUFS* pcans);
 * @brief	: Handle ELCON CAN msg received. 
 * @param	: pcans = pointer to CAN msg w selection code
 * *************************************************************************/
float fmsgvolts;
float fmsgamps;
void do_elcon(struct CANRCVBUFS* pcans)
{
	uint32_t itmpvolts = (pcans->can.cd.uc[0] << 8) | (pcans->can.cd.uc[1]);
	uint32_t itmpamps  = (pcans->can.cd.uc[2] << 8) | (pcans->can.cd.uc[3]);
	fmsgvolts = itmpvolts;
	fmsgamps  = itmpamps;	

	return;
}
/* *************************************************************************
 * void do_elcon_poll(void);
 * @brief	: Send CAN msg to ELCON
 * *************************************************************************/
void do_elcon_poll(void)
{
	struct CANTXQMSG* p = &emclfunction.lc.lcstring.canelcon;
	p->can.cd.us[0] = __REVSH(ichgr_setvolts); //(ichgr_setvolts >> 8) || (ichgr_setvolts << 8);
	p->can.cd.us[1] = __REVSH(ichgr_setamps);  //(ichgr_setamps  >> 8) || (ichgr_setamps  << 8);
	p->can.cd.uc[4] = 1;
	xQueueSendToBack(CanTxQHandle,p,4);
	return;
}

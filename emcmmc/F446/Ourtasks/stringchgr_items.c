
/******************************************************************************
* File Name          : stringchgr_items.c
* Date First Issued  : 09/08/2024
* Description        : Battery string charging
*******************************************************************************/
#include "stringchgr_items.h"
#include "cancomm_items.h"
#include "bubblesort_uint32_t.h"

/* Status bits (see BQTask.h)
Battery--
#define BSTATUS_NOREADING (1 << 0)	// Exactly zero = no reading
#define BSTATUS_OPENWIRE  (1 << 1)  // Negative or over 5v indicative of open wire
#define BSTATUS_CELLTOOHI (1 << 2)  // One or more cells above max limit
#define BSTATUS_CELLTOOLO (1 << 3)  // One or more cells below min limit
#define BSTATUS_CELLBAL   (1 << 4)  // Cell balancing in progress
#define BSTATUS_CHARGING  (1 << 5)  // Charging in progress
#define BSTATUS_DUMPTOV   (1 << 6)  // Dump to a voltage in progress

FETS--
#define FET_DUMP     (1 << 0) // 1 = DUMP FET ON
#define FET_HEATER   (1 << 1) // 1 = HEATER FET ON
#define FET_DUMP2    (1 << 2) // 1 = DUMP2 FET ON (external charger)
#define FET_CHGR     (1 << 3) // 1 = Charger FET enabled: Normal charge rate
#define FET_CHGR_VLC (1 << 4) // 1 = Charger FET enabled: Very Low Charge rate

Mode status bits 'mode_status' --
#define MODE_SELFDCHG  (1 << 0) // 1 = Self discharge; 0 = charging
#define MODE_CELLTRIP  (1 << 1) // 1 = One or more cells tripped max

	struct BQFUNCTION* p = &bqfunction;
	po->cd.uc[1] = MISCQ_STATUS; // 
	po->cd.us[1] = 0; // uc[2]-[3] cleared
	// Data payload bytes [4]-[7]
	po->cd.ui[1] = 0; // Clear
	po->cd.uc[4] = p->battery_status;
	po->cd.uc[5] = p->fet_status;
	po->cd.uc[6] = p->mode_status;
*/

/* Table with data for each BMS node on the string. */
struct BMSTABLE bmstable[BMSTABLESIZE];
uint8_t bmsnum; // Number of reported BMS nodes

/* Pointers for accessing table in sorted CAN id order. */
struct BMSTABLE* pbmstbl[BMSTABLESIZE];

/* Indices for accessing table element, given lookup mapped index. */
// Entry every possible BMS CAN id. -1 = not in table, >= 0 index of table
static int8_t remap[BMSNODEIDSZ]; 

static uint8_t status1;  // Bits on BMS number reporting

/* *************************************************************************
 * void stringchgr_items_init(void);
 * @brief	: Init stuff
 * *************************************************************************/
void stringchgr_items_init(void)
{
	int i;
	/* Remap CAN ID array. */
	for (i = 0; i < BMSNODEIDSZ; i++)
	{
		remap[i] = -1; // Show BMS node CAN ID position not accessed
	}
	status1 = 0;

	return;
}

/* *************************************************************************
 * int8_t do_tableupdate(struct CANRCVBUFS* pcans);
 * @brief	: BMS msg received. 
 * @param	: pcans = pointer to CAN msg w selection code
 * @return	: 0 = updated; 1 = added entry and updated
 * *************************************************************************/
static void updatetable(struct BMSTABLE* ptbl, struct CANRCVBUFS* pcans)
{
	uint8_t idx;

	ptbl->rpid_now = 1; // Reported ID now

	switch (pcans->can.cd.uc[0])
	{
	case CMD_CMD_MISCHB: // 45 Heartbeat 
		if (pcans->can.cd.uc[1] == MISCQ_STATUS)
		{ // Here, BMS node w status payload
			ptbl->rpsts_ever = 1;
			ptbl->rpsts_now  = 1;
			ptbl->batt = pcans->can.cd.uc[4];
			ptbl->fet  = pcans->can.cd.uc[5];
			ptbl->mode = pcans->can.cd.uc[6];
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

		if (idx == 0xF)
		{ // Here, last CAN msg of group
			ptbl->vsum      = ptbl->vsum_work;
			ptbl->vsum_work = 0;
		}

		break;
	}

	return;	
}

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



							
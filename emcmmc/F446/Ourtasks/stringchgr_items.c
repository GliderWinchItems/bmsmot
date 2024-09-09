
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

struct BMSTABLE bmstable[BMSTABLESIZE];
uint8_t bmsnum; // Number of reported BMS nodes
/* Pointers for accessing table in sorted CAN id order. */
struct BMSTABLE* pbmstbl[BMSTABLESIZE];

/* *************************************************************************
 * int8_t do_tableupdate(struct CANRCVBUFS* pcans);
 * @brief	: BMS msg received. 
 * @param	: pcans = pointer to CAN msg w selection code
 * @return	: 0 = updated; 1 = added entry and updated
 * *************************************************************************/
static void updatetable(struct BMSTABLE* ptbl, struct CANRCVBUFS* pcans)
{
	uint8_t idx;

	ptbl->rpid_now = 1; // Current report

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
		break;
	}

	return;	
}

int8_t do_tableupdate(struct CANRCVBUFS* pcans)
{
	int i;
	for (i = 0; i < bmsnum; i++)
	{
		if (pcans->can.id == bmstable[i].id)
		{ // CAN id is in the list
			bmstable[i].rpid_now = 1; // Current report
			updatetable(&bmstable[i], pcans);
			return 0;
		}
	}
	/* Not in table. Add to table. */
	if (i >= bmsnum)
	{ // Not in table. Add to table
		bmsnum += 1;
		bmstable[i].id = pcans->can.id;
		pbmstbl[i] = &bmstable[i];
		updatetable(&bmstable[i], pcans);

		if (bmsnum > 1)
		{
			/* Sort array for accessing in CAN id sorted order. */
			bubble_sort_uint32t(&pbmstbl[0], bmsnum);
		}
	}

	return 1;
}
							
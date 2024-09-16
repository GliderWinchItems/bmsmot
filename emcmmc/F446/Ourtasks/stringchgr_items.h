/******************************************************************************
* File Name          : stringchgr_items.h
* Date First Issued  : 09/08/2024
* Description        : Battery string charging
*******************************************************************************/


#ifndef __STRINGCHGRITEMS
#define __STRINGCHGRITEMS
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "malloc.h"
#include "StringChgrTask.h"
#include "GatewayTask.h"
#include "gateway_table.h"
#include "main.h"
#include "../../../../GliderWinchCommons/embed/svn_common/trunk/db/gen_db.h"
#include "EMCLTask.h"

/* Status bits (see BQTask.h) */
//Battery--
#define BSTATUS_NOREADING (1 << 0)	// Exactly zero = no reading
#define BSTATUS_OPENWIRE  (1 << 1)  // Negative or over 5v indicative of open wire
#define BSTATUS_CELLTOOHI (1 << 2)  // One or more cells above max limit
#define BSTATUS_CELLTOOLO (1 << 3)  // One or more cells below min limit
#define BSTATUS_CELLBAL   (1 << 4)  // Cell balancing in progress
#define BSTATUS_CHARGING  (1 << 5)  // Charging in progress
#define BSTATUS_DUMPTOV   (1 << 6)  // Dump to a voltage in progress

//FETS--
#define FET_DUMP     (1 << 0) // 1 = DUMP FET ON
#define FET_HEATER   (1 << 1) // 1 = HEATER FET ON
#define FET_DUMP2    (1 << 2) // 1 = DUMP2 FET ON (external charger)
#define FET_CHGR     (1 << 3) // 1 = Charger FET enabled: Normal charge rate
#define FET_CHGR_VLC (1 << 4) // 1 = Charger FET enabled: Very Low Charge rate

//Mode status bits 'mode_status' --
#define MODE_SELFDCHG  (1 << 0) // 1 = Self discharge; 0 = charging
#define MODE_CELLTRIP  (1 << 1) // 1 = One or more cells tripped max
/* Copy of some code from somewhere
	struct BQFUNCTION* p = &bqfunction;
	po->cd.uc[1] = MISCQ_STATUS; // 
	po->cd.us[1] = 0; // uc[2]-[3] cleared
	// Data payload bytes [4]-[7]
	po->cd.ui[1] = 0; // Clear
	po->cd.uc[4] = p->battery_status;
	po->cd.uc[5] = p->fet_status;
	po->cd.uc[6] = p->mode_status;
*/


/* BMS TABLE (not Bowel Movements STable) */
struct BMSTABLE
{
	uint32_t id;       // Node CAN id
	uint32_t vsum;     // last complete voltage sum of cells (100uv)
	uint32_t vsum_work;// summation in progress
	uint32_t toctr_cell;   // Last update, RTOS time: cell readings
	uint32_t toctr_status; // Last update, RTOS time: status
	uint16_t cell[18]; // Cell readings (100uv)
	uint16_t idxbits;  // Bit for 'idx' of six cell msgs
	uint8_t batt;      // Battery status
	uint8_t fet;       // FET status
	uint8_t mode;      // Mode status
	uint8_t groupnum;  // Six cell readings group number
	uint8_t stale_cell;   // Timeout for good 6 cell group
	uint8_t stale_status; // Timeout for status msg
};
#define BMSTABLESIZE 7 // Max size of table

/* *************************************************************************/
 int8_t do_tableupdate(struct CANRCVBUFS* pcans);
/* @brief	: BMS msg received. 
 * @param	: pcans = pointer to CAN msg w selection code
 * @return	: 0 = updated; 1 = added entry and updated
 * *************************************************************************/
 void stringchgr_items_init(void);
/* @brief	: Init stuff
 * *************************************************************************/
 void do_elcon_poll(void);
/* @brief	: Send CAN msg to ELCON
 * *************************************************************************/
 void do_timeoutcheck(void);
/* @brief	: Scan BMS table and mark those that have timed out
 * @return	: stale_status and stale_cell set if timed out
 * *************************************************************************/

extern struct BMSTABLE bmstable[BMSTABLESIZE];
extern uint8_t bmsnum; // Number of reported BMS nodes
extern struct BMSTABLE* pbmstbl[BMSTABLESIZE];

#endif
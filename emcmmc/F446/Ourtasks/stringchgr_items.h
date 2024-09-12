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

/* BMS TABLE (not Bowel Movements STable) */
struct BMSTABLE
{
	uint32_t id;       // Node CAN id
	uint32_t vsum;     // last complete voltage sum of cells (100uv)
	uint32_t vsum_work;// summation in progress
	uint16_t cell[18]; // Cell readings (100uv)
	uint8_t batt;      // Battery status
	uint8_t fet;       // FET status
	uint8_t mode;      // Mode status
	uint8_t rpid_ever; // CAN idReported: at least once
	uint8_t rpid_now;  // CAN id Reported: poll response
	uint8_t rpsts_ever;// Status reported: at least once
	uint8_t rpsts_now; // Status reported: poll response
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

extern struct BMSTABLE bmstable[BMSTABLESIZE];
extern uint8_t bmsnum; // Number of reported BMS nodes
extern struct BMSTABLE* pbmstbl[BMSTABLESIZE];

#endif
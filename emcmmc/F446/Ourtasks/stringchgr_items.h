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

/* *************************************************************************/
  void do_tableupdate(struct CANRCVBUFS* pcans);
/* @brief	: BMS msg received. 
 * @param	: pcans = pointer to CAN msg w selection code
 * *************************************************************************/
 void stringchgr_items_init(void);
/* @brief	: Init stuff
 * *************************************************************************/
 void do_elcon(struct CANRCVBUFS* pcans);
/* @brief	: Handle ELCON CAN msg received. 
 * @param	: pcans = pointer to CAN msg w selection code
 * *************************************************************************/
 void do_elcon_poll(void);
/* @brief	: Send CAN msg to ELCON
 * *************************************************************************/
 void do_timeoutcheck(void);
/* @brief	: Scan BMS table and mark those that have timed out
 * @return	: stale_status and stale_cell set if timed out
 * *************************************************************************/
  void do_emc_cmds(struct CANRCVBUFS* pcans);
/* @brief	: Commands from either PC, EMC, or deus ex machina
 * @param	: pcans = pointer to CAN msg w selection code
 * *************************************************************************/
  int8_t do_bms_status_check(void);
/* @brief	: Check statusrcv to see if all nodes sent a status msg
 * @return  : 0 = yes; -1 = no
 * *************************************************************************/

#endif
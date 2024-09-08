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
#include "main.h"

/* *************************************************************************/
 void do_tablecheck(struct CANRCVBUFS* pcans);
/* @brief	: BMS msg received. 
 * @param	: pcans = pointer to CAN msg w selection code
 * @return	: ?
 * *************************************************************************/

#endif
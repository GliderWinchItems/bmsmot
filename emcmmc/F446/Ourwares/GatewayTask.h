/******************************************************************************
* File Name          : GatewayTask.h
* Date First Issued  : 02/25/2019
* Description        : Bulk transfer to StringChgrTask, notified by MailboxTask
*******************************************************************************/
/*
09/07/2024 revised for bmsmot/emcmmc 
*/
#ifndef __GATEWAYTASK
#define __GATEWAYTASK

//#include "stm32f4xx_hal_def.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "malloc.h"
#include "common_can.h"
#include "../../../../GliderWinchCommons/embed/svn_common/trunk/db/gen_db.h"

/* *************************************************************************/
osThreadId xGatewayTaskCreate(uint32_t taskpriority);
/* @brief	: Create task; task handle created is global for all to enjoy!
 * @param	: taskpriority = Task priority (just as it says!)
 * @return	: GatewayHandle
 * *************************************************************************/
void StartGatewayTask(void const * argument);
/*	@brief	: Task startup
 * *************************************************************************/
osThreadId xGatewayTaskCreate(uint32_t taskpriority);
/* @brief	: Create task; task handle created is global for all to enjoy!
 * @param	: taskpriority = Task priority (just as it says!)
 * @return	: GatewayHandle
 * *************************************************************************/
 struct CANRCVBUF* GatewayTask_takecan1(void);
 struct CANRCVBUF* GatewayTask_takecan2(void);
/*	@brief	: Get CAN1/2 msg if buffer not empty
 *  @return : NULL = no entries, else pointer to CAN msg
 * *************************************************************************/


/* A notification copies the internal notification word to this. */
extern uint32_t GatewayTask_noteval;    // Receives notification word upon an API notify

#endif


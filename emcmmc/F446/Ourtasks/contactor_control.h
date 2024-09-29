/******************************************************************************
* File Name          : contactor_control.h
* Date First Issued  : 12/01/2019
* Board              : DiscoveryF4
* Description        : Control of contactor unit
*******************************************************************************/
#ifndef __CONTACTOR_CONTROL
#define __CONTACTOR_CONTROL

#include <stdio.h>
#include <string.h>
#include "ContactorTask.h"
//#include "gevcu_idx_v_struct.h"
#include "main.h"
#include "morse.h"
#include "common_can.h"

#define CNCTR_KATICKS (128/3)
#define CNCTR_KAQUICKTIC (128/5)	

/* Command request bits assignments. 
Sourced location: ../contactor/OurTasks/ContactorTask.h
*/
#define CMDCONNECT (1 << 7) // 1 = Connect requested; 0 = Disconnect requested
#define CMDRESET   (1 << 6) // 1 = Reset fault requested; 0 = no command

enum CONTACTOR_CONTROL_STATE
{
	CTL_INITTIM,
	CTL_CLEARFAULT,
	CTL_CONNECTING,
	CTL_CONNECTED1,
	CTL_CONNECTED
};

enum CONTACTOR_STATE
{
	DISCONNECTED,   /*  0 */
	CONNECTING,     /*  1 */
	CONNECTED,      /*  2 */
	FAULTING,       /*  3 */
	FAULTED,        /*  4 */
	RESETTING,      /*  5 */
	DISCONNECTING,  /*  6 */
	OTOSETTLING,    /*  7 one time intializing. */
};

/*
     payload[0]
       bit 7 - faulted (code in payload[2])
       bit 6 - warning: minimum pre-chg immediate connect.
              (warning bit only resets with power cycle)
		 bit[0]-[3]: Current main state code
*/

/* Contactor Control */
struct CNTCTRCTL
{
	struct CANTXQMSG canka; // CAN keepalive msg
	uint32_t nextctr;
	uint16_t ctr;     // Repetition counter
	uint8_t state;    // State machine
	uint8_t cmdrcv;   // Latest payload[0] with command
	uint8_t cmdsend;  // Command we send
	uint8_t sendflag; // 1 = send CAN msg, 0 = skip
	uint8_t req;      // Request code: 0 = connect, 1 = disconnect
};

/* ***********************************************************************************************************/
void contactor_control_init(void);
/* @brief	: Prep for contactor handling
 ************************************************************************************************************* */
void contactor_control_time(uint32_t ctr);
/* @brief	: Timer input to state machine
 * @param	: ctr = sw1ctr time ticks
 ************************************************************************************************************* */
void contactor_control_CANrcv(struct CANRCVBUF* pcan);
/* @brief	: Handle contactor command CAN msgs being received
 * @param	: pcan = pointer to CAN msg struct
 ************************************************************************************************************* */
void contactor_control_CANsend(void);
/* @brief	: Send CAN msg
 ************************************************************************************************************* */

extern struct CNTCTRCTL cntctrctl; // Contactor Control

#endif


/******************************************************************************
* File Name          : dmoc_control.c
* Date First Issued  : 12/04/2019
* Board              : DiscoveryF4
* Description        : Control of dmoc unit [state machine version]
*******************************************************************************/
/* More formal state machine version. */

#include "dmoc_control.h"
#include "main.h"
#include "morse.h"
#include "DMOCchecksum.h"
//#include "paycnvt.h"
#include "can_iface.h"
//#include "spiserialparallel.h"
//#include "shiftregbits.h"
//#include "calib_control_lever.h"

/* Name the indices to correspond to the GEVCU DMOC documentation. */
#define CMD1 0 
#define CMD2 1 
#define CMD3 2 

#define FALSE 1
#define TRUE  0

/* Command request bits assignments. 
Sourced location: ../dmoc/OurTasks/ContactorTask.h
*/

extern struct CAN_CTLBLOCK* pctl0;	// Pointer to CAN1 control block

/* struct holds "everything" for each DMOC. */
struct DMOCCTL dmocctl[NUMDMOC]; // Array allows for multiple DMOCs

/* ***********************************************************************************************************
 * void dmoc_control_init(struct DMOCCTL* pdmocctl);
 * @param	: pdmocctl = pointer to struct with "everything" for this DMOC unit
 * @brief	: Prep for dmoc handling
 ************************************************************************************************************* */
void dmoc_control_init(struct DMOCCTL* pdmocctl)
{
	int i,j;

	/* If values are semi-permanent in high-flash, then the following would be
      accomplished by copying from the high-flash areas. */

	/* NOTE--When there is more than one DMOC and the following parameters
      are applicable to both, then something like the following would be
      needed to be added, e.g.
      if (pdmocctl == &dmocctl[0])
      { // Initial DMOC #1
        ...
      }
		else
		{ // Here, initial "other" DMOC
        ... 
		}
		NOTE: for dynamometer testing one DMOC will be in torque mode
      and the other in speed mode. If the following initialization is used
      for both, the difference in modes will need to be set by "someone".
	*/

	pdmocctl->state        = DMOCINIT1; // Initial state
	pdmocctl->sendflag     = 0;
	pdmocctl->alive        = 0; // DMOC count increments by 2 & truncated
	pdmocctl->activityctr  = 0; // Count DMOC 0x476 (0x23B) incoming msgs
	pdmocctl->dmocstatefaulted = 0; // 1 = faulted
	pdmocctl->dmocnotsending   = 0; // 1 = dmoc CAN msgs not being received

	pdmocctl->activityctr_prev =   0; // Previous count (for computing difference)
	pdmocctl->activityctr      = 128; // Count CAN msgs from dmoc
	pdmocctl->activitytimctr   =  50; // Number of sw timer ticks between activity check
	pdmocctl->activitylimit    =   4; // Number dmoc CAN msgs received during interval

	pdmocctl->dmocstateact = DMOC_INIT; //DMOC_DISABLED;   // Assume initial state
	pdmocctl->dmocopstate  = DMOC_DISABLED;   // Requested startup state
	pdmocctl->dmocgear     = DMOC_NEUTRAL;    // Gear selection
	pdmocctl->mode         = DMOC_MODETORQUE; // Speed or Torque mode selection

	pdmocctl->maxregenwatts = 60000; // ?
	pdmocctl->maxaccelwatts = 60000; // ?
	pdmocctl->currentoffset =  5000; // Current offset (0.1 amps) (reported)

	pdmocctl->speedreq      =     0; // Requested speed
	pdmocctl->maxspeed      =  2500; // Max speed (signed)  ### SMALL FOR 03/11/20 TEST ###
	pdmocctl->speedoffset   = 20000; // Speed command offset

	pdmocctl->torqueact     =     0; // Torque actual (reported)
	pdmocctl->ftorquereq    =   0.0; // Requested torque (Nm)
	pdmocctl->fmaxtorque_pbopen   =  30.0; // Max torque (Nm) (Pushbutton open/released)
	pdmocctl->fmaxtorque_pbclosed = -30.0; // Max torque (Nm) (Pushbutton closed/pressed)
	pdmocctl->torqueoffset  = 30000; // Torque command offset 

//	pdmocctl->regencalc = 65000 - (pdmocctl->maxregenwatts / 4); // Computed in CMD3
//	pdmocctl->accelcalc = (pdmocctl->maxaccelwatts / 4);         // Computed in CMD3

	/* Load fixed data into three DMOC command CAN msgs. */
	for (i = 0; i < 3; i++)
	{
		pdmocctl->cmd[i].txqcan.pctl       = pctl0; // CAN1 control block ptr (from main.c)
		pdmocctl->cmd[i].txqcan.maxretryct = 8;
		pdmocctl->cmd[i].txqcan.bits       = CANMSGLOOPBACKBIT; // Route tx copy as if received
		pdmocctl->cmd[i].txqcan.can.dlc    = 8; // All command msgs have 8 payload bytes

		for (j = 0; j < 8; j++) // Clear out payload (later, some bytes are bytes are overwritten)
		{
			pdmocctl->cmd[i].txqcan.can.cd.uc[j] = 0;
		}
	}
	
/* Load CAN id into DMOC command CAN msgs. */

	//Commanded RPM plus state of key and gear selector
	// CANID_DMOC_CMD_SPEED', '46400000','DMOC','I16_X6',         'DMOC: cmd: speed, key state'
	pdmocctl->cmd[CMD1].txqcan.can.id = dmocfunction.lc.cid_dmoc_cmd_speed;

	//Torque limits
	// CANID_DMOC_CMD_TORQ',  '46600000','DMOC','I16_I16_I16_X6', 'DMOC: cmd: torq,copy,standby,status
	pdmocctl->cmd[CMD2].txqcan.can.id = dmocfunction.lc.cid_dmoc_cmd_torq;

	//Power limits plus setting ambient temp and whether to cool power train or go into limp mode
	//CANID_DMOC_CMD_REGEN', '46800000','DMOC','I16_I16_X_U8_U8','DMOC: cmd: watt,accel,degC,alive
	pdmocctl->cmd[CMD3].txqcan.can.id = dmocfunction.lc.cid_dmoc_cmd_regen;

/* Preset some payload bytes that do not change. */
	pdmocctl->cmd[CMD2].txqcan.can.cd.uc[4] = 0x75; // msb standby torque. -3000 offset, 0.1 scale. These bytes give a standby of 0Nm
	pdmocctl->cmd[CMD2].txqcan.can.cd.uc[5] = 0x30; // lsb

	pdmocctl->cmd[CMD3].txqcan.can.cd.uc[5] = 60;   // 20 degrees celsius ambient temp

	return;
}
/* ***********************************************************************************************************
 * void dmoc_control_time(struct DMOCCTL* pdmocctl, uint32_t ctr);
 * @brief	: Timer input to state machine
 * @param	: pdmocctl = pointer to struct with "everything" for this DMOC unit
 * @param	: ctr = sw1ctr time ticks
 ************************************************************************************************************* */
void dmoc_control_time(struct DMOCCTL* pdmocctl, uint32_t ctr)
{
	int32_t diff;

	if (pdmocctl->state == DMOCINIT1)
	{ // OTO 
		pdmocctl->nextctr = ctr + DMOC_KATICKS;
		pdmocctl->sendflag = 1;       // Trigger sending now
		pdmocctl->state = DMOCINIT2;  // 
		return;
	}

	/* Activity check =>from<= dmoc. */
	diff = pdmocctl->activityctr  - pdmocctl->activityctr_prev;
	if (diff > 0) // Reset whenever we have one or more msgs received between time ticks
	{
		pdmocctl->activitytimctr = 0; // Reset time count
		pdmocctl->dmocnotsending = 0; // 1 = dmoc CAN msgs not being received
		pdmocctl->activityctr_prev = pdmocctl->activityctr;
	}
	else
	{
		pdmocctl->activitytimctr += 1;// Number of sw timer ticks between activity check
		if (pdmocctl->activitytimctr >= pdmocctl->activitylimit)
		{ // Time to see if dmoc has gone away
			pdmocctl->activitytimctr = 0; // Reset timout counter
			pdmocctl->dmocnotsending = 1; // 1 = dmoc CAN msgs not being received
		}
	}

	/* Keepalive timing, based on sw1tim time ticks (e.g. (1/128) ms). */
	if ((int)(pdmocctl->nextctr - ctr)  > 0) return;

	/* Set next ctr for next KA msg. */
	pdmocctl->nextctr = ctr + DMOC_KATICKS; // Send next KA msg
	pdmocctl->sendflag = 1; // Send first msg

	return;
}
/* ***********************************************************************************************************
 * void dmoc_control_GEVCUBIT08(struct DMOCCTL* pdmocctl, struct CANRCVBUF* pcan);
 * @brief	: CAN msg received: cid_dmoc_actualtorq
 * @param	: pdmocctl = pointer to struct with "everything" for this DMOC unit
 * @param	: pcan = pointer to CAN msg struct
 ************************************************************************************************************* */
void dmoc_control_GEVCUBIT08(struct DMOCCTL* pdmocctl, struct CANRCVBUF* pcan)
{
/* 0x23A CANID_DMOC_ACTUALTORQ:I16,   DMOC: Actual Torque: payload-30000 */
	/* Extract reported torque and update latest reading. */
//				torqueActual = ((frame->data.bytes[0] * 256) + frame->data.bytes[1]) - 30000;
	pdmocctl->torqueact = ((pcan->cd.uc[0] << 8) + (pcan->cd.uc[1])) - pdmocctl->torqueoffset;
	return;
}
/* ***********************************************************************************************************
 * void dmoc_control_GEVCUBIT09(struct DMOCCTL* pdmocctl, struct CANRCVBUF* pcan);
 * @brief	: CAN msg received: cid_dmoc_speed
 * @param	: pdmocctl = pointer to struct with "everything" for this DMOC unit
 * @param	: pcan = pointer to CAN msg struct
 ************************************************************************************************************* */
void dmoc_control_GEVCUBIT09(struct DMOCCTL* pdmocctl, struct CANRCVBUF* pcan)
{
/* cid_dmoc_speed,     NULL,GEVCUBIT09,0,I16_X6); */
/* 0x23B CANID_DMOC_SPEED:     I16_X6,DMOC: Actual Speed (rpm?) */

	pdmocctl->activityctr += 1;

	// Speed (signed)
	pdmocctl->speedact = ( (pcan->cd.uc[0] << 8) | pcan->cd.uc[1]) - pdmocctl->speedoffset;

	// DMOC status
	pdmocctl->dmocstaterep = (pcan->cd.uc[6] >> 4);

        //actually, the above is an operation status report which doesn't correspond
        //to the state enum so translate here.
	switch (pdmocctl->dmocstaterep) 
	{
	case 0: //Initializing
	/* When DMOC sends '0' it is initializing. We send DISABLED until
	   DMOC responds with DISABLED. If dmocopstate is STANDBY or ENABLE
	   then the state sequence in 'dmoc_control_CANsend' will step up
   the DMOC to the requested state, e.g. STANDBY or ENABLE. */
            pdmocctl->dmocstateact = DMOC_INIT; //DMOC_DISABLED;
            pdmocctl->dmocstatefaulted = FALSE;
            break;

	case 1: //disabled
            pdmocctl->dmocstateact =  DMOC_DISABLED;
            pdmocctl->dmocstatefaulted = FALSE;
            break;

	case 2: //ready (standby)
            pdmocctl->dmocstateact =  DMOC_STANDBY;
            pdmocctl->dmocstatefaulted = FALSE;
            break;

	case 3: //enabled
            pdmocctl->dmocstateact =  DMOC_ENABLE;
            pdmocctl->dmocstatefaulted = FALSE;
            break;

	case 4: //Power Down
            pdmocctl->dmocstateact =  DMOC_POWERDOWN;
            pdmocctl->dmocstatefaulted = FALSE;
			pdmocctl->dmocopstate = DMOC_DISABLED;
			pdmocctl->dmocstateact =  DMOC_INIT;//DMOC_DISABLED;
            break;

	case 5: //Fault
            pdmocctl->dmocstateact =  DMOC_INIT;//DMOC_DISABLED;
            pdmocctl->dmocstatefaulted = TRUE;
	/* Attempt a restart? */
			pdmocctl->dmocopstate = DMOC_DISABLED;
            break;

	case 6: //Critical Fault
            pdmocctl->dmocstatefaulted = TRUE;
	/* Attempt a restart? */
			pdmocctl->dmocopstate  = DMOC_POWERDOWN;
         pdmocctl->dmocstateact = DMOC_POWERDOWN;
				break;

	case 7: //LOS
            pdmocctl->dmocstateact =  DMOC_DISABLED;
            pdmocctl->dmocstatefaulted = TRUE;
            break;
	}
	return;
}
/* ***********************************************************************************************************
 * void dmoc_control_GEVCUBIT13(struct DMOCCTL* pdmocctl, struct CANRCVBUF* pcan);
 * @brief	: CAN msg received: cid_dmoc_actualtorq
 * @param	: pdmocctl = pointer to struct with "everything" for this DMOC unit
 * @param	: pcan = pointer to CAN msg struct
 ************************************************************************************************************* */
void dmoc_control_GEVCUBIT13(struct DMOCCTL* pdmocctl, struct CANRCVBUF* pcan)
{
/* cid_dmoc_hv_status, NULL,GEVCUBIT13,0,I16_I16_X6); */
/* 0x650 CANID_DMOC_HV_STATUS: I16_I16_X6,'DMOC: HV volts:amps, status */

/*        dcVoltage = ((frame->data.bytes[0] * 256) + frame->data.bytes[1]);
        dcCurrent = ((frame->data.bytes[2] * 256) + frame->data.bytes[3]) - 5000; //offset is 500A, unit = .1A
        activityCount++; */

	pdmocctl->voltageact = (pcan->cd.uc[0] << 8 | pcan->cd.uc[1]);
	pdmocctl->currentact = (pcan->cd.uc[2] << 8 | pcan->cd.uc[3]) - pdmocctl->currentoffset;
	pdmocctl->activityctr += 1;
	return;
}
/* ***********************************************************************************************************
 * void dmoc_control_GEVCUBIT14(struct DMOCCTL* pdmocctl, struct CANRCVBUF* pcan);
 * @brief	: CAN msg received: cid_dmoc_actualtorq
 * @param	: pdmocctl = pointer to struct with "everything" for this DMOC unit
 * @param	: pcan = pointer to CAN msg struct
 ************************************************************************************************************* */
void dmoc_control_GEVCUBIT14(struct DMOCCTL* pdmocctl, struct CANRCVBUF* pcan)
{
/*cid_dmoc_hv_temps,  NULL,GEVCUBIT14,0,U8_U8_U8); */
/* 0x651 CANID_DMOC_HV_TEMPS:  U8_U8_U8,  'DMOC: Temperature:rotor,invert,stator */

/*       RotorTemp = frame->data.bytes[0];
        invTemp = frame->data.bytes[1];
        StatorTemp = frame->data.bytes[2];
        temperatureInverter = (invTemp-40) *10;
        //now pick highest of motor temps and report it
        if (RotorTemp > StatorTemp) {
            temperatureMotor = (RotorTemp - 40) * 10;
        }
        else {
            temperatureMotor = (StatorTemp - 40) * 10;
        }
        activityCount++; */

	pdmocctl->rotortemp   = pcan->cd.uc[0];
	pdmocctl->invtemp     = pcan->cd.uc[1];
	pdmocctl->statortemp  = pcan->cd.uc[2];
	pdmocctl->invtempcalc = (pdmocctl->invtemp - 40) * 10;
	if (pdmocctl->rotortemp > pdmocctl->statortemp)
	{
		pdmocctl->motortemp = (pdmocctl->rotortemp - 40) * 10;
	}
	else
	{
		pdmocctl->motortemp = (pdmocctl->statortemp - 40) * 10;
	}
	pdmocctl->activityctr += 1;
	return;
}

/* ***********************************************************************************************************
 * void dmoc_control_CANsend(struct DMOCCTL* pdmocctl);
 * @brief	: Send group of three CAN msgs to DMOC
 * @param	: pdmocctl = pointer to struct with "everything" for this DMOC unit
 ************************************************************************************************************* */
/*
	This is called from GevcuUpdates.c. If the 'sendflag' is set during GevcuEvents, or GevcuStates,
   the command CAN messages are sent.
*/
void dmoc_control_CANsend(struct DMOCCTL* pdmocctl)
{
	int32_t ntmp;
	int32_t ntmp1;

	if (pdmocctl->sendflag == 0) return; // Return when not flagged to send.
	pdmocctl->sendflag = 0; // Reset flag


	// Convert float in Nm to Nm tenths as an integer.
	pdmocctl->itorquereq = (pdmocctl->ftorquereq * 10.0f);

	/* Increment 'alive' counter by two each time group of three is sent */
	pdmocctl->alive = (pdmocctl->alive + 2) & 0x0F;

	/* CMD1: RPM plus state of key and gear selector ***************  */

/*  --DmocMotorController.cpp --<cmd1 snip>--  
    if (throttleRequested > 0 && operationState == ENABLE && selectedGear != NEUTRAL && powerMode == modeSpeed)
        speedRequested = 20000 + (((long) throttleRequested * (long) config->speedMax) / 1000);
    else
        speedRequested = 20000;
    output.data.bytes[0] = (speedRequested & 0xFF00) >> 8;
    output.data.bytes[1] = (speedRequested & 0x00FF);
    output.data.bytes[2] = 0; //not used
    output.data.bytes[3] = 0; //not used
    output.data.bytes[4] = 0; //not used
    output.data.bytes[5] = ON; //key state
 */

	/* Translate above DmocMotorController.cpp */
	ntmp = pdmocctl->speedreq; // Requested speed (RPM?)

	// (Bogus speed request check: -20000 < speedreq < 20000)
	if ((ntmp > pdmocctl->speedoffset) || (ntmp < -pdmocctl->speedoffset))
	         ntmp = 0;

	// Send non-zero speed command only when everything is ready
	if ((ntmp != 0) && 
	    (pdmocctl->dmocopstate == DMOC_ENABLE ) && 
	    (pdmocctl->dmocgear    != DMOC_NEUTRAL) && 
       (pdmocctl->mode        == DMOC_MODESPEED) )
            ntmp += pdmocctl->speedoffset; // Command requested speed
    else
            ntmp = pdmocctl->speedoffset; // Command zero speed

	// Update payload
	pdmocctl->cmd[CMD1].txqcan.can.cd.uc[0] = (ntmp & 0xFF00) >> 8;
	pdmocctl->cmd[CMD1].txqcan.can.cd.uc[1] = (ntmp & 0x00FF);
	pdmocctl->cmd[CMD1].txqcan.can.cd.uc[5] = 1; // Key state = ON (this could be in 'init')

/* --DmocMotorController.cpp -- <cmd1 snip>--
   //handle proper state transitions
    newstate = DISABLED;
    if (actualState == DISABLED && (operationState == STANDBY || operationState == ENABLE))
        newstate = STANDBY;
    if ((actualState == STANDBY || actualState == ENABLE) && operationState == ENABLE)
        newstate = ENABLE;
    if (operationState == POWERDOWN)
        newstate = POWERDOWN;

    if (actualState == ENABLE) {
        output.data.bytes[6] = alive + ((byte) selectedGear << 4) + ((byte) newstate << 6); //use new automatic state system.
    }
    else { //force neutral gear until the system is enabled.
        output.data.bytes[6] = alive + ((byte) NEUTRAL << 4) + ((byte) newstate << 6); //use new automatic state system.
    }
*/
//#define USEOLDCODE
#ifdef USEOLDCODE
	/* When in DMOC_INIT state always send disabled. */
	if (pdmocctl->dmocstateact == DMOC_INIT)
		 pdmocctl->dmocopstate = DMOC_DISABLED; // Overwrite anything else.

/* Translate above DmocMotorController.cpp */
	pdmocctl->dmocstatenew = DMOC_DISABLED;

	/* When DMOC shows DISABLED, send our request for STANDBY, providing we want ENABLE. */
	if (pdmocctl->dmocstateact == DMOC_DISABLED && (pdmocctl->dmocopstate == DMOC_ENABLE || pdmocctl->dmocopstate == DMOC_STANDBY))
		pdmocctl->dmocstatenew = DMOC_STANDBY;

	/* Send ENABLE when DMOC shows either STANDBY or ENABLE. */
	if ((pdmocctl->dmocstateact == DMOC_STANDBY || pdmocctl->dmocstateact == DMOC_ENABLE) && (pdmocctl->dmocopstate == DMOC_ENABLE))
		pdmocctl->dmocstatenew = DMOC_ENABLE;

	/* In case we want to initiate powerdown the DMOC via operational state. */
	if (pdmocctl->dmocopstate == DMOC_POWERDOWN)
		pdmocctl->dmocstatenew = DMOC_POWERDOWN;

	if (pdmocctl->dmocstateact == DMOC_ENABLE)
	{ // DMOC is showing that it is enabled.
		pdmocctl->cmd[CMD1].txqcan.can.cd.uc[6] = pdmocctl->alive | (DMOC_DRIVE << 4) | (pdmocctl->dmocstatenew << 6) ;
	}
	else
	{ // Show DMOC neutral gear until the DMOC shows that it is enabled.
		pdmocctl->cmd[CMD1].txqcan.can.cd.uc[6] = pdmocctl->alive | (DMOC_NEUTRAL << 4) | (pdmocctl->dmocstatenew << 6) ;
	}
#else
	switch (pdmocctl->dmocstateact)
	{
	case DMOC_INIT:
		pdmocctl->dmocgear = DMOC_NEUTRAL;
		pdmocctl->dmocstatenew = DMOC_DISABLED;
		break;

	case DMOC_DISABLED:
		pdmocctl->dmocgear = DMOC_NEUTRAL;
		if ((pdmocctl->dmocopstate == DMOC_ENABLE) ||
		    (pdmocctl->dmocopstate == DMOC_STANDBY) )
			pdmocctl->dmocstatenew = DMOC_STANDBY;
		else
			pdmocctl->dmocstatenew = DMOC_DISABLED;
		break;

	case DMOC_STANDBY:
		pdmocctl->dmocgear = DMOC_NEUTRAL;
		if (pdmocctl->dmocopstate == DMOC_ENABLE)
			pdmocctl->dmocstatenew = DMOC_ENABLE;
		break;

	case DMOC_ENABLE:
		if (pdmocctl->dmocopstate == DMOC_ENABLE)
		{
			pdmocctl->dmocstatenew = DMOC_ENABLE;
			if (pdmocctl->itorquereq >= 0)
				pdmocctl->dmocgear = DMOC_DRIVE;
			else
				pdmocctl->dmocgear = DMOC_REVERSE;
		}
		else
		{
			pdmocctl->dmocstatenew = pdmocctl->dmocopstate;
			pdmocctl->dmocgear = DMOC_NEUTRAL;
		}
		break;

	case DMOC_POWERDOWN:
		pdmocctl->dmocstatenew = DMOC_POWERDOWN;
		break;
	}

	pdmocctl->cmd[CMD1].txqcan.can.cd.uc[6] = pdmocctl->alive | (pdmocctl->dmocgear << 4) | (pdmocctl->dmocstatenew << 6) ;
#endif

	/* Add the weird dmoc checksum. */
	pdmocctl->cmd[CMD1].txqcan.can.cd.uc[7] = DMOCchecksum(&pdmocctl->cmd[CMD1].txqcan.can); 

	// Queue CAN msg
	xQueueSendToBack(CanTxQHandle,&pdmocctl->cmd[CMD1].txqcan,4);

	/* CMD2: Torque limits ****************************************** */

	/* Don't load payload if dmoc is not in ENABLE state. */
	if (pdmocctl->dmocstateact != DMOC_ENABLE)
     	pdmocctl->itorquereq = 0;

	/* Speed or Torque mode. */
	if (pdmocctl->mode == DMOC_MODETORQUE)
	{ // Torque
	/* If max speed (positive) over max, and requested torque is positive, set
		requested to torque to zero. Otherwise, allow requested torque, whether 
		positive or negative, to remain as requested. */
      if ((pdmocctl->speedact > pdmocctl->maxspeed) && (pdmocctl->itorquereq >= 0))
      	pdmocctl->itorquereq = 0;				

	/* Opposite of above. Max speed in reverse, with negative torque requested sets
      torque to zero. Otherwise, allow whatever torque is requested.*/
      if ((pdmocctl->speedact < -pdmocctl->maxspeed) && (pdmocctl->itorquereq < 0))
      	pdmocctl->itorquereq = 0;				

		/* Convert Nm to Nm tenths, and thence to signed integer with offset applied. */
		ntmp = pdmocctl->itorquereq + pdmocctl->torqueoffset;

		pdmocctl->cmd[CMD2].txqcan.can.cd.uc[0] = (ntmp & 0xFF00) >> 8;
		pdmocctl->cmd[CMD2].txqcan.can.cd.uc[2] = (ntmp & 0xFF00) >> 8;
		pdmocctl->cmd[CMD2].txqcan.can.cd.uc[1] = (ntmp & 0x00FF);
		pdmocctl->cmd[CMD2].txqcan.can.cd.uc[3] = (ntmp & 0x00FF);
	}
	else
	{ // Speed mode
		ntmp1 = pdmocctl->itorquereq; // integer of Nm in tenths
		// Speed mode Max positive torque
		ntmp = pdmocctl->torqueoffset + ntmp1;
		pdmocctl->cmd[CMD2].txqcan.can.cd.uc[0] = (ntmp & 0xFF00) >> 8;
		pdmocctl->cmd[CMD2].txqcan.can.cd.uc[1] = (ntmp & 0x00FF);

		// Speed mode Max negative torque
		ntmp = pdmocctl->torqueoffset - ntmp1; 
		pdmocctl->cmd[CMD2].txqcan.can.cd.uc[2] = (ntmp & 0xFF00) >> 8;
		pdmocctl->cmd[CMD2].txqcan.can.cd.uc[3] = (ntmp & 0x00FF);
	}
	
	//what the hell is standby torque? Does it keep the transmission spinning for automatics? I don't know.
//   -3000 offset, 0.1 scale. These bytes give a standby of 0Nm
// These are set during init so the following two are commented out.
//	pdmocctl->cmd[CMD2].txqcan.can.cd.uc[4] = 0x75; //msb standby torque. [Set during init]
//	pdmocctl->cmd[CMD2].txqcan.can.cd.uc[5] = 0x30; //lsb [Set during init]

	pdmocctl->cmd[CMD2].txqcan.can.cd.uc[6] = pdmocctl->alive;
	pdmocctl->cmd[CMD2].txqcan.can.cd.uc[7] = DMOCchecksum(&pdmocctl->cmd[CMD2].txqcan.can); 

	// Queue CAN msg
	xQueueSendToBack(CanTxQHandle,&pdmocctl->cmd[CMD2].txqcan,4);

	/* CMD3: Power limits plus setting ambient temp *************** */
  	  // [Could these two be an OTO init, or are they updated as the battery sags?]
	pdmocctl->regencalc = 65000 - (pdmocctl->maxregenwatts / 4);
	pdmocctl->accelcalc = (pdmocctl->maxaccelwatts / 4);

	// Update payload
	pdmocctl->cmd[CMD3].txqcan.can.cd.uc[0] = ((pdmocctl->regencalc & 0xFF00) >> 8); //msb of regen watt limit
	pdmocctl->cmd[CMD3].txqcan.can.cd.uc[1] = (pdmocctl->regencalc & 0xFF); //lsb
	pdmocctl->cmd[CMD3].txqcan.can.cd.uc[2] = ((pdmocctl->accelcalc & 0xFF00) >> 8); //msb of acceleration limit
	pdmocctl->cmd[CMD3].txqcan.can.cd.uc[3] = (pdmocctl->accelcalc & 0xFF); //lsb
//	pdmocctl->cmd[CMD3].txqcan.can.cd.uc[4] = 0; //not used [Set during init]
//	pdmocctl->cmd[CMD3].txqcan.can.cd.uc[5] = 60; //20 degrees celsius [Set during init]
	pdmocctl->cmd[CMD3].txqcan.can.cd.uc[6] = pdmocctl->alive;
	pdmocctl->cmd[CMD3].txqcan.can.cd.uc[7] = DMOCchecksum(&pdmocctl->cmd[CMD3].txqcan.can); 

	// Queue CAN msg
	xQueueSendToBack(CanTxQHandle,&pdmocctl->cmd[CMD3].txqcan,4);

	return;
}


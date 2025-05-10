/******************************************************************************
* File Name          : StringChgrTask.h
* Date First Issued  : 09/06/2024
* Description        : Battery string charging
*******************************************************************************/


#ifndef __STRINGCHGRTASK
#define __STRINGCHGRTASK

#include <stdint.h>
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "can_iface.h"
//#include "stm32f4xx_hal.h"
#include "GatewayTask.h"

/* emcmmc general status. */
#define MMCSTATUS_BMSSTAT  (1 << 0) // 1 = ALL BMS nodes reporting status
#define MMCSTATUS_BMSCELL  (1 << 1) // 1 = ALL BMS nodes reporting cell readings
#define MMCSTATUS_CHRGR    (1 << 2) // 1 = Charger reporting OK
#define MMCSTATUS_BMSLMITS (1 << 3) // 1 = All BMS nodes reported v & i limits

/* Status: BMS node reporting status. */
#define SCTSTATUS_EXP (1<<0) // Expected number is reporting
#define SCTSTATUS_OVF (1<<1) // Received more than table size
#define SCTSTATUS_FWR (1<<2) // Fewer than expected reporting
#define SCTSTATUS_MOR (1<<3) // More  than expected reporting
#define SCTSTATUS_NR  (1<<4) // Missing node readings, or not up-to-date

/* Status: Reporting ELCON charger. */
#define ELCONSTATUS_HWFAIL (1<<0) //  1 Hardware failure
#define ELCONSTATUS_OVTEMP (1<<1) //  2 Over temperature
#define ELCONSTATUS_INVOLT (1<<2) //  4 Input voltage wrong
#define ELCONSTATUS_DISCNT (1<<3) //  8 Battery disconnected
#define ELCONSTATUS_COMMTO (1<<4) // 10 Communications timeout
/* Additional bits for ELCON. */
#define ELCONSTATUS_REPORT (1<<5) // 20 ELCON is reporting
#define ELCONSTATUS_TIMOUT (1<<6) // 40 Time out bits [0:4] not ready

/* Charging status. */
#define CHGSTATUS_MODE  (1<<0) // 0 = relaxation; 1 = charging

#define CHGRATENUM 6 // Number of charger current steps

/* If BMS does not report module volt and charge current maxs */
#define DEFAULT_VMAX ((3650*18)*0.1f) // (Voltage max 0.1v)
#define DEFAULT_IMAX (13) // Charging max (0.1a)
#define DEFULAT_IBAL (1)  // Balancing max (0.1a)

/*
Status1 CAN msg
pay[0] : Msg is status
pay[1] [4:0] Status bits: Module reporting (see #define above)
pay[2] [4:0] Status bits: ELCON 
pay[3] 
pay[4]-[7] (float) String voltage (Volts)
*/

/* Notification bits */
#define STRINGCHRGBIT00 (1<<0) // GatewayTask
#define STRINGCHRGBIT01 (1<<1) // RTOS timer swtim1

/* Timer durations. */
#define SWTIME1PERIOD 50 // 50 ms ticks
#define TIMCTR_ELCON_POLL  ( 900/SWTIME1PERIOD) // Time between ELCON polls (900 ms)
#define TIMCTR_TIMEOUT_BMS (1000/SWTIME1PERIOD) // Timecheck BMS responding. (1000 ms)
#define TIMCTR_ELCON_TIMOUT (15000/SWTIME1PERIOD) // ELCON fails ready status (15sec)
#define TIMCTR_RELAX (3000/SWTIME1PERIOD) // Charging turned off wait for cell volts to settle
#define TIMCTR_ELCON_POLLFLAG (6000/SWTIME1PERIOD) // Wait to reset ELCON poll flag
#define TIMCTR_ELCON_RCV (2000/SWTIME1PERIOD) // Wait for ELCON response to poll

#define DISCOVERY_POLL_LIMIT 5 // Allow 5 polls before ending discovery phase

/* Modes */
#define EMCCMD_ELIDLE   0 // ELCON: Self discharging/stop 
#define EMCCMD_ELCHG    1 // ELCON: Start charging
#define EMCCMD_LCCHG    2 // Node Low Current: set charging mode
#define EMCCMD_LCIDLE   3 // Node Low Current: self discharging/stop 
#define EMCCMD_DBGSET   4 // Debug: ELCON: Force volts & amps setting
#define EMCCMD_POLLRATE 5 // Set BMS poll rate

/* Table with element for each BMS node on string. */
#define BMSTABLESIZE 7 // Max size of table

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
#define MODE_CHRGING   (1 << 0) // 0 = Self discharge; 1 = charging
#define MODE_CHRGEND   (1 << 1) // 0 = Chrg ended; 1 = In progress
#define MODE_CELLTRIP  (1 << 2) // 1 = One or more cells tripped max

/* state switch on BMS status msg. */
#define STSSTATE_IDLE   0  // Just poll ELCON to keep its COMM status alive
#define STSSTATE_CHRG   1  // Charging in progress
#define STSSTATE_RELAX  2  // Waiting for cells to relax after charge stopped.
#define STSSTATE_DBG    3  // PC or EMC direct setting of volts, amps, rate idx
#define STSSTATE_DISCOVERY 4 // 

/* status2 */
#define STS2_NODES_RPT  (1 << 0) // 1 = All discovered nodes reported status

/* BMS TABLE (not Bowel Movements STable) */
struct BMSTABLE
{
	uint32_t id;        // Node CAN id
	uint32_t vsum;      // last complete (18 cells) sum of cells (100uv)
	uint32_t vsum_work; // summation from six CAN msgs in progress

	/* toctrs are time tick countdown and zero means timed out. */	
	int32_t toctr_cell;   // Timeout counter: cell readings (0 = timed out)
	int32_t toctr_status; // Timeout counter: status (0 = timed out)

	uint16_t cell[18]; // Cell readings (100uv)
	uint16_t idxbits;  // Bit for 'idx' of six cell msgs (all on = 0x9249)
	uint16_t v_max;    // Voltage max for this module
	uint8_t i_max;     // Max charging current (255 = 25.5a or greater) 
	uint8_t i_bal;     // Max balancing current (0.1 steps)

	uint8_t batt;       // Battery status
	uint8_t fet;        // FET status
	uint8_t mode;       // Mode status
	uint8_t temperature;// Temperature status (maybe not implemented)
	uint8_t groupnum;   // Six cell readings group number
};

/* Group vars having to do with ELCON together. */
struct ELCONSTUFF
{
	float fmsgvolts;         // ELCON reported: volts
	float fmsgamps;          // ELCON reported: amps
	int32_t toct_poll_dur;   // Timer tick ct for polling duration (might be variable)

	/* toctrs are time tick countdown and zero means timed out. */
	int32_t toctr_elcon_rcv; // Timeout counter: ELCON rcv msgs
	int32_t toctr_wait1;     // Timeout counter: delay 1
	int32_t toctr_wait2;     // Timeout counter: delay 2
	int32_t toctr_elcon_poll;// Timeout counter: ELCON poll
	int32_t toctr_elcon_ready; // Receving ELCON mgs, but status shows not ready
	int32_t toctr_elcon_pollflag; // Timeout counter: others sent a ELCON poll

	/* Note: ichgr 16b are big endian in ELCON CAN payloads. */
	uint16_t ichgr_maxvolts; // Parameter float(volts) to uint32_t ui(0.1v)
	uint16_t ichgr_maxamps;  // Parameter float(amps) to uint32_t ui(0.1a)
	uint16_t ichgr_setvolts; // Set ELCON: (0.1v)
	uint16_t ichgr_setamps;  // Set ELCON: (0.1a)

	/* ELCON status bits 0:4 HW FAIL:OVR TEMP:INPUT RVRSE:BATT DISC:COMM TO: */
	uint8_t status_elcon;    // uc[4] status byte rcvd from ELCON
	uint8_t pollflag;        // 0 = OK; 1 = someone else sent a poll CAN msg

	int8_t discovery_ctr; // Count ELCON timed BMS polls for discovery
};

/* Working struct. */
struct STRINGCHGRFUNCTION
{
	struct ELCONSTUFF elconstuff; // Group ELCON vars together
	struct BMSTABLE bmstable[BMSTABLESIZE]; // Entry for each BMS node
	struct BMSTABLE* pbmstbl[BMSTABLESIZE]; // Ptrs for sorted sequence

	/* CAN msgs */
	struct CANTXQMSG canelcon; // CAN msg for sending to ELCON
	struct CANTXQMSG canbms;   // CAN msg for sending to BMS nodes

	uint32_t hbct_t;   // Duration between STRINGCHGR function heartbeats(ms)
	uint32_t hbct_tic; // Loop tick count between heartbeats
	int32_t  hbct_ctr; // Heartbeat time count-down

	int32_t toctr_relax; // Wait after turning off ELCON 

	float chgr_maxvolts; // Set charger voltage limit (volts)
	float chgr_maxamps;  // Set charger current limit (amps)
	float chgr_balamps;  // Balancing current limit (amps)

	uint16_t vi_max_rpt; // Bits that module reported V & I max limits

	/* Charging current steps. */
	float chgr_rate[CHGRATENUM]; // Amps

	uint8_t bmsnum_expected; // Number of BMS nodes expected to be reporting
	uint8_t bmsnum;    // Discovered number of BMS nodes.

	// chgr_rate_idx = (CHGRATENUM-1) is MAX rate
	uint8_t chgr_rate_idx; // Index of rate in effect. 

	uint8_t nodeok; // Bits for each node that all readings received from poll

/* Indices for accessing table element, given a lookup mapped index. */
// Entry every possible BMS CAN id. -1 is not in table; >= 0 is index of table
	int8_t remap[BMSNODEIDSZ]; 

	uint8_t status1;   // Bits on BMS number reporting
	uint8_t mode;      // Charging or off
//	uint8_t state;     // Switch on status msg
	uint8_t cellrcv;   // Node bits: all cell readings received 
	uint8_t statusrcv; // Node bits: status msg received
	uint8_t statusmax; // Node bits: Cell above max in status report
	uint8_t status2;   // ?
	uint8_t stsstate;  // Charging management state
};

/* *************************************************************************/
 TaskHandle_t xStringChgrTaskCreate(uint32_t taskpriority);
/* @brief	: Create task; task handle created is global for all to enjoy!
 * @param	: taskpriority = Task priority (just as it says!)
 * @return	: ChgrTaskHandle
 * *************************************************************************/

extern TaskHandle_t StringChgrTaskHandle;

#endif
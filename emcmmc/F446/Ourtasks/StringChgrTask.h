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

/* emcmmc general status. */
#define MMCSTATUS_BMSSTAT (1 << 0) // 1 = ALL BMS nodes reporting status
#define MMCSTATUS_BMSCELL (1 << 1) // 1 = ALL BMS nodes reporting cell readings
#define MMCSTATUS_CHRGR   (1 << 2) // 1 = Charger reporting OK


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
#define ELCONSTATUS_COMMTO (1<<4) // 16 Communications timeout
#define ELCONSTATUS_REPORT (1<<5) // 32 ELCON is reporting

/* Charging status. */
#define CHGSTATUS_MODE  (1<<0) // 0 = relaxation; 1 = charging

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

#define SWTIME1PERIOD 50 // 50 ms ticks
#define TIMCTR_ELCON_POLL (900/SWTIME1PERIOD) // Time between ELCON polls (900 ms)
#define TIMCTR_BMSTIMEOUT (1000/SWTIME1PERIOD) // Timecheck BMS responding. (1000 ms)

#define BMSTIMEOUT (3000/SWTIME1PERIOD) // Timeout: BMS msgs missing. (3000 ms)

/* Table with element for each BMS node on string. */
#define BMSTABLESIZE 7 // Max size of table



/* Working struct for EMC local function. */
struct STRINGCHGRFUNCTION
{
	uint32_t hbct_t;   // Duration between STRINGCHGR function heartbeats(ms)
	uint32_t hbct_tic; // Loop tick count between heartbeats
	int32_t  hbct_ctr; // Heartbeat time count-down

	uint8_t bmsnum_expected; // Number of BMS nodes expected to be reporting

	float chgr_maxvolts; // Set charger voltage limit (volts)
	float chgr_maxamps;  // Set charger current limit (amps)

	/* CAN msgs */
	struct CANTXQMSG canelcon; // CAN msg for sending to ELCON


};

/* *************************************************************************/
 TaskHandle_t xStringChgrTaskCreate(uint32_t taskpriority);
/* @brief	: Create task; task handle created is global for all to enjoy!
 * @param	: taskpriority = Task priority (just as it says!)
 * @return	: ChgrTaskHandle
 * *************************************************************************/

extern TaskHandle_t StringChgrTaskHandle;


#endif
/******************************************************************************
* File Name          : EMCLTask.h
* Date First Issued  : 10/29/2023
* Description        : Energy Management Control Local task
*******************************************************************************/

#ifndef __EMCLTASK
#define __EMCLTASK

#include <stdint.h>
#include "FreeRTOS.h"
#include "cmsis_os.h"
//#include "stm32l4xx_hal.h"
#include "emcl_idx_v_struct.h"
#include "CanTask.h"
#include "adc_idx_v_struct.h"

struct EMCREQ_Q
{
	int placeholder;
};


/* Working struct for EMC local function. */
struct EMCLFUNCTION
{
   // Parameter loaded either by high-flash copy, or hard-coded subroutine
	struct EMCLLC lc; // Fixed parameters, (lc = Local Copy)

//	struct ADCFUNCTION* padc; // Pointer to ADC working struct

	/* Timings in milliseconds. Converted later to timer ticks. */
	uint32_t hbct_k;      // Heartbeat ct: ticks between sending

	uint8_t ident_string; // Packed: string
	uint8_t ident_onlyus; // Packed: string and module numbers
	/*  payload [0-1] U16 – Payload Identification
  [15:14] Winch (0 - 3)(winch #1 - #4)
  [13:12] Battery string (0 – 3) (string #1 - #4)
  [11:8] Module (0 – 15) (module #1 - #16)
  [7:3] Cell (0 - 31) (cell #1 - #32)
  [2:0] Group sequence number (0 - 7) */
	uint32_t morse_err; // Error code retrieved from backup SRAM registers
	uint8_t err;
	uint16_t warning;   // Error code that is a warning

	uint8_t hbseq; // heartbeat CAN msg sequence number
	uint32_t HBstatus_ctr; // Count RTOS ticks for hearbeat timing: status msg

	/* Pointers to incoming CAN msg mailboxes. */
   struct MAILBOXCAN* pmbx_cid_cmd_emcmmcx_pc;  //'CANID_CMD_EMCMMC1_PC' , A1600000 PC SENDS');
   struct MAILBOXCAN* pmbx_cid_cmd_emcmmcx_emc; //'CANID_CMD_EMCMMC1_EMC', A1800000 EMC SENDS'); 
   struct MAILBOXCAN* pmbx_cid_uni_bms_emc1_i;  // CANID_UNI_BMS_EMC1_I;    B0000000 UNIversal From EMC
   struct MAILBOXCAN* pmbx_cid_uni_bms_emc2_i;  // CANID_UNI_BMS_EMC2_I;    B0200000 UNIversal From EMC
   struct MAILBOXCAN* pmbx_cid_uni_bms_pc_i;    // CANID_UNI_BMS_PC_I;      AEC00000 UNIversal From PC

	uint8_t state;      // main state
	uint8_t substateA;  // 
	uint8_t substateB;  // spare substate 

	/* CAN msgs */
	struct CANTXQMSG canmsg;
};
/* *************************************************************************/
osThreadId xEMCLTaskCreate(uint32_t taskpriority);
/* @brief	: Create task; task handle created is global for all to enjoy!
 * @param	: taskpriority = Task priority (just as it says!)
 * @return	: EMCLTaskHandle
 * *************************************************************************/

 extern TaskHandle_t EMCLTaskHandle;
 extern struct EMCLFUNCTION emclfunction;



#endif


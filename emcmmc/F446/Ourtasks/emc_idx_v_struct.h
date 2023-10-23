/******************************************************************************
* File Name          : emc_idx_v_struct.h
* Date First Issued  : 06/14/2023
* Board              :
* Description        : Fixed parameter struct
*******************************************************************************/

#include <stdint.h>
#include "common_can.h"
//#include "iir_filter_lx.h"
#include "iir_f1.h"
#include "RyTask.h"

#ifndef __EMC_IDX_V_STRUCT
#define __EMC_IDX_V_STRUCT


#define NRELAY 12 // Number of relay outputs

#define RAWTC 0.975f  // Filter time constant
#define RAWSKIPCT 2  // Ignore initial readings to filter

struct RELAY
{
   uint32_t kp;  // Maximum time duration (ms) between keep-alive requests
                   //  ~0 = no timeout (i.e. always alive)
   uint16_t pulldelay; // Delay time duration (ms) before pwm for relay pull-in
                          //  0 = no delay (e.g. external fet drive for fans)
   uint8_t pwm;   // PWM percent when energized: (0 - 100)
                     //   100 = full ON; 20-40 = normal range, relay dependent
};



/* Parameters levelwind instance (LC = Local Copy) */
struct EMCLC
 {
/* NOTE: all suffix _t parameters are times in milliseconds */
	uint32_t size;
	uint32_t crc;   // TBD
   uint32_t version;   //

   /* Identification of this module node. */
   uint8_t  winchnum;  // Winch number (1-4)
   uint8_t  stringnum; // Battery string number (1 - 4)


	/* Timings in milliseconds. Converted later to 'swtim1' ticks. */
	uint32_t hbct_t;     // Heartbeat ct: ms between sending 
   uint32_t hbct;       // Number of ticks between hb msgs

   uint32_t CanComm_hb; // CanCommTask 'wait' RTOS ticks per heartbeat sending

// Relay/Controls
   struct RELAY relay[NRELAY];


 // CAN ids ...........................................................................
   //                                  CANID_NAME            CANID       PAYLOAD FORMAT
    // BMS sends; EMC, PC or other receives
 //  uint32_t cid_msg_bms_cellvsmr; // CAN id for cell voltage burst format 
//   uint32_t cid_cmd_bms_miscr;    // CAN id for all other responses

 // List of CAN ID's for setting up hw filter for incoming msgs
//   uint32_t cid_uni_bms_emc_i;     // CANID_UNI_BMS_EMC_I      B0000000 multi-purpose universal BMS
//   uint32_t cid_uni_bms_pc_i;      // CANID_UNI_BMS_PC_I       B0200000 multi-purpose universal BMS

   uint8_t hbseq; // heartbeat CAN msg sequence number
 };

/* *************************************************************************/
 void emc_idx_v_struct_hardcode_params(struct EMCLC* p);
/* @brief : Init struct from hard-coded parameters (rather than database params in highflash)
 * @return  : 0
 * *************************************************************************/

 
#endif


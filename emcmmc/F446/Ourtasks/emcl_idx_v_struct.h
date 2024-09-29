/******************************************************************************
* File Name          : emcl_idx_v_struct.h
* Date First Issued  : 06/14/2023
* Board              :
* Description        : Fixed parameter struct
*******************************************************************************/

#include <stdint.h>
#include "common_can.h"
//#include "iir_filter_lx.h"
#include "iir_f1.h"
#include "RyTask.h"
#include "CoolingTask.h"
#include "StringChgrTask.h"

#ifndef __EMCL_IDX_V_STRUCT
#define __EMCL_IDX_V_STRUCT

#define EMCL_STATUS   60 // Report status
#define EMCL_PWM_REP  61 // Report relay pwm
#define EMCL_PWM_CMD  62 // Set/command relay pwm

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
// The following is used to prevent pwm requests of 100% locking out the
// sub-board over-current logic tripping the latch and turning the FET continous
// OFF. The substituted pwm, i.e. pwmx, might be 95 (95 percent) more or less.
   uint8_t limit; // 1 = this relay/motor has a pwm limit (e.g. 96%)
   uint8_t pwmx;  // If trans != 0, then pwm > than pwmx, use pmx.   
};

/* Parameters levelwind instance (LC = Local Copy) */
struct EMCLLC
 {
/* RelayTask: Relay/Controls */
   struct RELAY relay[NRELAY]; // Relay status & control

/* CoolingTask: */
  struct COOLINGFUNCTION lccool;

/* StringChgrTask: */
  struct STRINGCHGRFUNCTION lcstring;

/* NOTE: all suffix _t parameters are times in milliseconds */
	uint32_t size;
	uint32_t crc;   // TBD
   uint32_t version;   //

/* Identification of this module node. */
   uint8_t  winchnum;  // Winch number (1-4)
   uint8_t  stringnum; // Battery string number (1 - 4)

	/* Timings in milliseconds. Converted later to 'swtim1' ticks. */
	uint32_t hbct_t;     // Heartbeat ct: ms between sending 
   uint32_t hbct;      // Number of ticks between hb msgs

 // CAN ids ...........................................................................
   //                                  CANID_NAME            CANID       PAYLOAD FORMAT
   // EMCMMC1,2 sends; EMC, PC or other receives
  uint32_t cid_unit_emcmmcx; // e.g.'CANID_UNIT_EMCMMC1','A0000000'

 // List of CAN ID's for setting up hw filter for incoming msgs
  uint32_t cid_cmd_emcmmcx_pc;  //'CANID_CMD_EMCMMC1_PC' ,'A1600000','PC SENDS');
  uint32_t cid_cmd_emcmmcx_emc; //'CANID_CMD_EMCMMC1_EMC','A1800000', EMC SENDS'); 

  uint32_t cid_uni_bms_emc1_i; // CANID_UNI_BMS_EMC1_I;   // B0000000 UNIversal From EMC,  Incoming msg to BMS: X4=target CANID');   
  uint32_t cid_uni_bms_emc2_i; // CANID_UNI_BMS_EMC2_I;   // B0200000 UNIversal From EMC,  Incoming msg to BMS: X4=target CANID');   
  uint32_t cid_uni_bms_pc_i;   // CANID_UNI_BMS_PC_I;     // AEC00000 UNIversal From PC,   Incoming msg to BMS: X4=target CANID');       

  uint8_t hbseq; // heartbeat CAN msg sequence number
 };

/* *************************************************************************/
 void emcl_idx_v_struct_hardcode_params(struct EMCLLC* p);
/* @brief : Init struct from hard-coded parameters (rather than database params in highflash)
 * @return  : 0
 * *************************************************************************/

#endif


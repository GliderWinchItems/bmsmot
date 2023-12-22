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
};

/* Parameters levelwind instance (LC = Local Copy) */
struct EMCLLC
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

/* RelayTask: Relay/Controls */
   struct RELAY relay[NRELAY]; // Relay status & control


/* CoolingTask: */
   // Temperature sensing thermistors
   uint8_t tx_pmpo; // ADC index: coolant pump outlet thermistor
   uint8_t tx_moto; // ADC index: motor outlet thermistor
   uint8_t tx_hexo; // ADC index: heat exchange outlet thermistor
   uint8_t tx_amb;  // ADC index: ambient air temperature thermistor
   uint8_t tx_jic;  // ADC index: provision if 5th thermistor added to ADC sequence
   // PWM header mapping:
   uint8_t pwm_mot; // Ry index: Pump motor 
   uint8_t pwm_blo; // Ry index: Heat exchanger blower motor
   uint8_t pwm_dmc; // Ry index: DMOC fans
   uint8_t pwm_jic; // Ry index: just in case spare

 // CAN ids ...........................................................................
   //                                  CANID_NAME            CANID       PAYLOAD FORMAT
    // EMCMMC1,2 sends; EMC, PC or other receives
  uint32_t cid_unit_emcmmcx; // e.g.'CANID_UNIT_EMCMMC1','A0000000'

 // List of CAN ID's for setting up hw filter for incoming msgs
  uint32_t cid_cmd_emcmmcx_pc;  //'CANID_CMD_EMCMMC1_PC' ,'A1600000','PC SENDS');
  uint32_t cid_cmd_emcmmcx_emc; //'CANID_CMD_EMCMMC1_EMC','A1800000', EMC SENDS'); 

  /* Cooling task */
  uint32_t cid_dmoc_actualtorq;// CANID_DMOC_ACTUALTORQ','47400000','DMOC',1,1,'I16','DMOC: Actual Torque: payload-30000'
  uint32_t cid_dmoc_hv_temps;// CANID_DMOC_HV_TEMPS',  'CA200000','DMOC',1,1,'U8_U8_U8''DMOC: Temperature:rotor,invert,stator'
  uint32_t cid_mc_state;// CANID_MC_STATE','26000000','MC',1,5,'U8_U8','MC: Launch state msg'

  uint8_t hbseq; // heartbeat CAN msg sequence number
 };

/* *************************************************************************/
 void emcl_idx_v_struct_hardcode_params(struct EMCLLC* p);
/* @brief : Init struct from hard-coded parameters (rather than database params in highflash)
 * @return  : 0
 * *************************************************************************/

 
#endif


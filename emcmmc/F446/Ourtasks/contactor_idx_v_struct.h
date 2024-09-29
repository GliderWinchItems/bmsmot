/******************************************************************************
* File Name          : contactor_idx_v_struct.h
* Date First Issued  : 09/19/2024
* Board              :
* Description        : Load parameter struct
*******************************************************************************/

#include <stdint.h>
#include "common_can.h"
#include "iir_filter_lx.h"
#include "ContactorTask.h"

#ifndef __CONTACTOR_IDX_V_STRUCT
#define __CONTACTOR_IDX_V_STRUCT

/* GevcuTask counts 'sw1timer' ticks for various timeouts.
 SWTIM1TICKDURATION
 We want the duration long, but good enough resolution(!)
 With systick at 512/sec, specifying 8 ms yields a 4 tick duration
 count = 4 -> 64/sec (if we want to approximate the logging rate)
 count = 64 -> 1/sec 
*/ 
#define SWTIM1TICKDURATION 8
#define SWTIM1TICKPERSEC (1000/SWTIM1TICKDURATION)

#define SWTIM1_64PERSEC (configTICK_RATE_HZ/64) // swtim1 ticks 

/* Parameters gevcu instance */
struct CONTACTORLC
 {

/* NOTE: all suffix _t parameters are times in milliseconds */

	uint32_t size;
	uint32_t crc;   // TBD
  uint32_t version;   // 

	uint32_t ka_t;        // Gevcu polling timer

	/* Timings in milliseconds. Converted later to 'swtim1' ticks. */
	uint32_t keepalive_t;
	uint32_t ka_cntct_t;  // Contactor sending keepalive/command (ms)
	uint32_t ka_gevcur_t; // EMCMMC keepalive/commmand from PC (ms) 
	uint32_t hbct_t;      // Heartbeat ct: ticks between sending 

 // CAN ids ...........................................................................
   //                                  CANID_NAME             CAN_MSG_FMT     DESCRIPTION
    // EMCMMC sends; Contactor receives
	uint32_t cid_cntctr_keepalive_i; // CANID_CMD_CNTCTRKAI:U8',    Contactor1: I KeepAlive and connect command
    // EMCMMC sends; PC receives
	uint32_t cid_gevcur_keepalive_r;// CANID_CMD_GEVCURKAR: U8_U8 : EMCMMC: R KeepAlive response

 // List of CAN ID's for setting up hw filter for incoming msgs
     // Contactor sends; we receive
	uint32_t cid_cntctr_keepalive_r; // CANID_CMD_CNTCTRKAR: U8_U8_U8: Contactor1: R KeepAlive response to poll
     // PC sends;  we receive 
	uint32_t cid_gevcur_keepalive_i; // CANID_CMD_GEVCURKAI:U8 : EMCMMC: I KeepAlive and connect command
   // DMOC sends; we receive
   // GPS/Logger sends; we receive
	uint32_t cid_gps_sync; // CANID_HB_TIMESYNC:  U8 : GPS_1: U8 GPS time sync distribution msg-GPS time sync msg

 };

/* *************************************************************************/
void contactor_idx_v_struct_hardcode_params(struct CONTACTORLC* p);
/* @brief	: Init struct from hard-coded parameters (rather than database params in highflash)
 * @return	: 0
 * *************************************************************************/
 
#endif


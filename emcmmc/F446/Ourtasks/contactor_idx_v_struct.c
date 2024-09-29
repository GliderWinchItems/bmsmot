/******************************************************************************
* File Name          : contactor_idx_v_struct.c
* Date First Issued  : 09/19/2024
* Board              :
* Description        : Load parameter struct
*******************************************************************************/

#include "contactor_idx_v_struct.h"
#include "SerialTaskReceive.h"

/* *************************************************************************
 * void dmoc_idx_v_struct_hardcode_params(struct struct DMOCLC* p);
 * @brief	: Init struct from hard-coded parameters (rather than database params in highflash)
 * @return	: 0
 * *************************************************************************/
void dmoc_idx_v_struct_hardcode_params(struct DMOCLC* p)
{
	p->size       = 47;
	p->crc        = 0;   // TBD
  p->version    = 1;   // 

	/* Timings in milliseconds. Converted later to timer ticks. */

/* GevcuTask counts 'sw1timer' ticks for various timeouts.
 We want the duration long, but good enough resolution(!)
 With systick at 512/sec, specifying 8 ms yields a 4 tick duration
 count = 4 -> 64/sec (if we want to approximate the logging rate)
 count = 64 -> 1/sec 
*/ 
	p->ka_t       = 4; // Gevcu polling timer: 4 * (1/512) 

	p->keepalive_t= 2555; // keep-alive timeout (timeout delay ms)
	p->hbct_t    = 1000; // Heartbeat ct: ticks between sending 
	p->ka_dmoc_r_t = 2;  // DMOC keepalive/torque command (sw1tim ticks)

 // CAN ids we send
   //                      CANID_HEX      CANID_NAME             CAN_MSG_FMT     DESCRIPTION
	p->cid_cntctr_keepalive_i  = 0xE3800000; // CANID_CMD_CNTCTRKAI:U8': Contactor1: I KeepAlive and connect command
    // GEVCUr sends to PC response to keepalive msg PC sends to GEVCUr
   p->cid_gevcur_keepalive_r = 0xE4200000; // CANID_CMD_GEVCURKAR: U8_U8 : GEVCUr: R KeepAlive response


 // List of CAN ID's for setting up hw filter for incoming msgs
     // Contactor sends
	p->cid_cntctr_keepalive_r  = 0xE3C00000; // CANID_CMD_CNTCTRKAR: U8_U8_U8: Contactor1: R KeepAlive response to poll
     // PC sends to GEVCUr (i.e. "us")
	p->cid_gevcur_keepalive_i = 0xE3E00000; // CANID_CMD_GEVCURKAI:U8 : GEVCUr: I KeepAlive and connect command

   // Others send
	p->cid_gps_sync     = 0x00400000; // CANID_HB_TIMESYNC:  U8 : GPS_1: U8 GPS time sync distribution msg-GPS time sync msg



	return;
}

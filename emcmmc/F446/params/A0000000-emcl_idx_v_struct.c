/******************************************************************************
* File Name          : A0000000-emcl_idx_v_struct.c
* Date First Issued  : 06/14/2023
* Board              : bmsmot
* Description        : Load EMCL parameters
*******************************************************************************/
/*
10/29/23 update for emc local
*/
#include "emcl_idx_v_struct.h"
#include "SerialTaskReceive.h"
#include "morse.h"
#include "../../../../GliderWinchCommons/embed/svn_common/trunk/db/gen_db.h"

static uint8_t initsw = 0;
/* *************************************************************************
 * void emcl_idx_v_struct_hardcode_params(struct EMCLC* p);
 * @brief	: Init struct from hard-coded parameters (rather than database params in highflash)
 * @return	: 0
 * *************************************************************************/
void emcl_idx_v_struct_hardcode_params(struct EMCLLC* p)
{
   if (initsw != 0)
      return;
   initsw = 1;
	p->size    = 32; //

	p->crc     = 0;   // TBD
   p->version = 1;   // 

	/* Timings in milliseconds. Converted later to timer ticks. */
   p->hbct_t       = 4000; // Heartbeat ct: milliseconds between sending 
   p->hbct         =   64; // Number of swctr ticks between heartbeats
//   p->adc_hb       = 64;     // Number of ticks for heartbeat ADC readout

   p->CanComm_hb = 1000; // CanCommTask 'wait' RTOS ticks per heartbeat sending

   int i;
   /* Default for all relays. */
   #define KYTODEFAULT ((8 * 100)/12) // 5 sec: Keep alive timer: (1 count = 10ms per OC, 12 relays per cycle)
   #define PULLINDEFAULT (1000) // 1 sec: (delay timer: 1 count = 0.1ms)
   #define HOLDPWM  25 // Holding percent pwm (when commanded with 255 pct pwm)
   for (i = 0; i < NRELAY; i++)
   {
      // TIM5 interrupt 100/sec, 1 out of 12 relays each interrupt
      p->relay[i].kp        = KYTODEFAULT; //((KYTODEFAULT*10) /(12*KPUPDATEDUR)); // Timeout duration between requests
      p->relay[i].pulldelay = PULLINDEFAULT; // Pull-in delay (TIM9: 0.1 ms)
      p->relay[i].pwm       = HOLDPWM; // After pull-in pwm (0 - 100%)
   }
   /* Override defaults. */
   p->relay[ 0].pulldelay =  500; //
   p->relay[ 1].pulldelay =  400; //
   p->relay[ 2].pulldelay =  600; //   
   p->relay[ 3].pulldelay =  100; //
   p->relay[ 4].pulldelay =  100; //
   p->relay[ 5].pulldelay =  100; //
   p->relay[ 6].pulldelay =  100; //
   p->relay[ 7].pulldelay =  100; //

   /* Override default for group C. */
   for (i = 8; i < 12; i++)
   {
      p->relay[i].pulldelay =    0; // These (normally) don't have a delay
      p->relay[i].pwm       =  100; // These (normally) are full on/off
   }

// List of CAN ID's for suscribing to incoming msgs
   p->cid_cmd_emcmmcx_pc  = CANID_CMD_EMCMMC1_PC; // 'A1600000','PC SENDS');
   p->cid_cmd_emcmmcx_emc = CANID_CMD_EMCMMC1_EMC;// 'A1800000', EMC SENDS'); 

// CAN ids EMCMMC sends, others receive
   p->cid_unit_emcmmcx = I_AM_CANID; // A0000000
	return;
}

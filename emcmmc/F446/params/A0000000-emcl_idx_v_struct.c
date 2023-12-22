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
#include "adcparams.h"
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

/* CoolingTask: */
   // Temperature sensing thermistors: map function to header
   p->tx_pmpo = ADC1IDX_THERMISTOR1; // ADC index: JP9  coolant pump outlet thermistor
   p->tx_moto = ADC1IDX_THERMISTOR2; // ADC index: JP8  motor outlet thermistor
   p->tx_hexo = ADC1IDX_THERMISTOR3; // ADC index: JP10 heat exchange outlet thermistor
   p->tx_amb  = ADC1IDX_THERMISTOR4; // ADC index: JP11 ambient air temperature thermistor
   p->tx_jic  = ADC1IDX_DIVIDEDSPARE;// ADC index: JP17 provision if 5th thermistor added to ADC sequence

   // Motor drive: PWM header mapping: map function to header
      // C group: no delay, 74VHCT125 drive buffer to sub-board
   p->pwm_mot =  8; // Ry index: OC1: Pump motor 
   p->pwm_blo =  9; // Ry index: OC2: Heat exchanger blower motor
   p->pwm_dmc = 10; // Ry index: OC3: DMOC fans
   p->pwm_jic = 11; // Ry index: OC4: just in case spare

// List of CAN ID's for suscribing to incoming msgs

   p->cid_cmd_emcmmcx_pc  = CANID_CMD_EMCMMC1_PC; // 'A1600000','PC SENDS');
   p->cid_cmd_emcmmcx_emc = CANID_CMD_EMCMMC1_EMC;// 'A1800000', EMC SENDS'); 

// Cooling task
  p->cid_dmoc_actualtorq = CANID_DMOC_ACTUALTORQ; //47400000','DMOC',1,1,'I16','DMOC: Actual Torque: payload-30000'
  p->cid_dmoc_hv_temps = CANID_DMOC_HV_TEMPS; //'CA200000','DMOC',1,1,'U8_U8_U8''DMOC: Temperature:rotor,invert,stator'
  p->cid_mc_state = CANID_MC_STATE; //'26000000','MC',1,5,'U8_U8','MC: Launch state msg'

// CAN ids EMCMMC sends, others receive
   p->cid_unit_emcmmcx = I_AM_CANID; // A0000000
	return;
}

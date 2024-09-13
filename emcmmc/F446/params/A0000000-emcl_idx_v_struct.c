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
#include "RyTask.h"
#include "../../../../GliderWinchCommons/embed/svn_common/trunk/db/gen_db.h"

#define DMOC_CANMSG_TEMP_IDX 5 // Index for temperature reading array

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

//  p->CanComm_hb = 1000; // CanCommTask 'wait' RTOS ticks per heartbeat sending

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
      p->relay[i].limit     =    0; // Default: 0 = no limiting
      p->relay[i].pwmx      =   96; // Default pwm limit
   }
   /* Override defaults. */
   // Relay pullin delays
   p->relay[ 0].pulldelay =  500; //
   p->relay[ 1].pulldelay =  400; //
   p->relay[ 2].pulldelay =  600; //   
   p->relay[ 3].pulldelay =  100; //
   p->relay[ 4].pulldelay =  100; //
   p->relay[ 5].pulldelay =  100; //
   p->relay[ 6].pulldelay =  100; //
   p->relay[ 7].pulldelay =  100; //

   // Motors with current limiter comparatr sub-board limit at less than 100%
   p->relay[ 8].limit = 0; // OC1 motor 1 
   p->relay[ 9].limit = 1; // OC2 motor 2
   p->relay[10].limit = 1; // OC3 motor 3
   p->relay[11].limit = 0; // OC4 jic

   /* Override default for group C. */
   for (i = 8; i < 12; i++)
   {
      p->relay[i].pulldelay =    0; // These (normally) don't have a delay
      p->relay[i].pwm       =  100; // These (normally) are full on/off
   }
   // NOTE: sub-board pwm for 'v6 must not be 100%
   // Set the appropiate index and trans = 1;
   // Uncomment, e/g/--
//     p->relay[8].limit     =    0; // Default: no translation 
//     p->relay[8].PWMX      =   90; // Max pwm at 90%.
 

/* ============== CoolingTask: =============== */
   // Temperature sensing thermistors: map function to pcb header
   p->lccool.tx_pmpo = ADC1IDX_THERMISTOR1; // ADC index: JP9  coolant pump outlet thermistor
   p->lccool.tx_moto = ADC1IDX_THERMISTOR2; // ADC index: JP8  motor outlet thermistor
   p->lccool.tx_hexo = ADC1IDX_THERMISTOR3; // ADC index: JP10 heat exchange outlet thermistor
   p->lccool.tx_amb  = ADC1IDX_THERMISTOR4; // ADC index: JP11 ambient air temperature thermistor
   p->lccool.tx_jic  = ADC1IDX_DIVIDEDSPARE;// ADC index: JP17 provision if 5th thermistor added to ADC sequence
   p->lccool.tx_dmoc = DMOC_CANMSG_TEMP_IDX;// DMOC motor temperature CAN msg report

   for (i = 0; i < COOLARRAYSZ; i++)
      p->lccool.temperatureparm[i].not_installed = 0;
   p->lccool.temperatureparm[p->lccool.tx_dmoc].not_installed = 1;

   // Temperature is too high threshold: set alert
   p->lccool.temperatureparm[p->lccool.tx_pmpo].toohi = 60;  // Pump outlet
   p->lccool.temperatureparm[p->lccool.tx_moto].toohi = 60;  // Winch motor outlet
   p->lccool.temperatureparm[p->lccool.tx_hexo].toohi = 60;  // Heat exchanger outlet
   p->lccool.temperatureparm[p->lccool.tx_amb ].toohi = 40;  // Ambient
   p->lccool.temperatureparm[p->lccool.tx_jic ].toohi = 255; // Not installed
   p->lccool.temperatureparm[p->lccool.tx_dmoc].toohi = 50;  // DMOC cooling fins

   // PWM versus temperature difference from ambient: offset (coef[0])
   p->lccool.temperatureparm[p->lccool.tx_pmpo].tcoef[0] =  5.0f;
   p->lccool.temperatureparm[p->lccool.tx_moto].tcoef[0] =  5.0f;
   p->lccool.temperatureparm[p->lccool.tx_hexo].tcoef[0] =  5.0f;
   p->lccool.temperatureparm[p->lccool.tx_amb ].tcoef[0] =  5.0f;
   p->lccool.temperatureparm[p->lccool.tx_jic ].tcoef[0] =  5.0f; // Not installed
   p->lccool.temperatureparm[p->lccool.tx_dmoc].tcoef[0] =  5.0f;
   // PWM versus temperature difference from ambient: scale (coef[1])
   p->lccool.temperatureparm[p->lccool.tx_pmpo].tcoef[1] = 6.0f;
   p->lccool.temperatureparm[p->lccool.tx_moto].tcoef[1] = 6.0f;
   p->lccool.temperatureparm[p->lccool.tx_hexo].tcoef[1] = 6.0f;
   p->lccool.temperatureparm[p->lccool.tx_amb ].tcoef[1] = 6.0f;
   p->lccool.temperatureparm[p->lccool.tx_jic ].tcoef[1] = 6.0f; // Not installed
   p->lccool.temperatureparm[p->lccool.tx_dmoc].tcoef[1] = 6.0f;

// PWM = full speed when temperature is equal/greater than this temperature (degC)
   p->lccool.temperatureparm[p->lccool.tx_pmpo].toohimax = 60;
   p->lccool.temperatureparm[p->lccool.tx_moto].toohimax = 60;
   p->lccool.temperatureparm[p->lccool.tx_hexo].toohimax = 60;
   p->lccool.temperatureparm[p->lccool.tx_amb ].toohimax =  0; // Not applicable
   p->lccool.temperatureparm[p->lccool.tx_jic ].toohimax = 60; // Not installed
   p->lccool.temperatureparm[p->lccool.tx_dmoc].toohimax = 60;


   // Timeout (50 ms ticks) for missing CAN msgs
   p->lccool.timeout_CANdmoc     = (100*20); // 10 secs
 //  p->lccool.timeout_mcstate     = (100*20); // 10 secs
   p->lccool.timeout_CANdmoc_ctr = (100*20); // 10 secs
 //  p->lccool.timeout_mcstate_ctr = (100*20); // 10 secs
   p->lccool.timeout_cntctrkar_ctr = (100*20); // 10 secs

// Contactor status
//   p->contactor_state = 0; // 0 = not reporting; 1 = connected; 2 = disconnected or other

   p->lccool.status_cool = 0;

   p->lccool.cid_test = 0xB2200000; // Cooling function test of CAN


/* Cooling task function */

  p->lccool.hbct_t = 4000; // Duration between cooling function status heartbeats (ms)

//  p->lccool.cid_dmoc_actualtorq = CANID_DMOC_ACTUALTORQ; //47400000','DMOC',1,1,'I16','DMOC: Actual Torque: payload-30000'
  p->lccool.cid_dmoc_hv_temps = CANID_DMOC_HV_TEMPS; //'CA200000','DMOC',1,1,'U8_U8_U8''DMOC: Temperature:rotor,invert,stator'
//  p->lccool.cid_mc_state = CANID_MC_STATE; //'26000000','MC',1,5,'U8_U8','MC: Launch state msg'
  p->lccool.cid_cntctrkar = CANID_CMD_CNTCTRKAR;//'E3C00000','CNTCTR',1,6,'U8_U8_U8','Contactor1: R KeepAlive response'
  p->lccool.cid_cmd_emcmmcx_pc  = CANID_CMD_EMCMMC1_PC; // 'A1600000','PC SENDS');
  p->lccool.cid_cmd_emcmmcx_emc = CANID_CMD_EMCMMC1_EMC;// 'A1800000', EMC SENDS'); 

// Cooling motor control
  // DMOC fan|blower IDX 0: Header OC1
  p->lccool.motorrampparam[COOLX_DMOCFAN].idle        = 15;    // Pct for idle speed
  p->lccool.motorrampparam[COOLX_DMOCFAN].minstart    = 30;    // Pct for minimum start
  p->lccool.motorrampparam[COOLX_DMOCFAN].initstart   = 12;    // Pct for initial startup
  p->lccool.motorrampparam[COOLX_DMOCFAN].shutoffwait = 3000;  // Wait after turn off for turn on (ms)
  p->lccool.motorrampparam[COOLX_DMOCFAN].rampuprate  = 5.0f;  // Pct per sec ramping up 
  p->lccool.motorrampparam[COOLX_DMOCFAN].rampdnrate  = 3.0f;  // Pct per sec ramping down
  p->lccool.motorrampparam[COOLX_DMOCFAN].subbrdtype  = 1;     // Sub-board type (1 = half-bridge)
  p->lccool.motorrampparam[COOLX_DMOCFAN].hdrnum      = HDR_OC3;// Header (Relay index) to this motor
  // Pump IDX 1: Header OC2
  p->lccool.motorrampparam[COOLX_PUMP].idle        = 15;    // Pct for idle speed
  p->lccool.motorrampparam[COOLX_PUMP].minstart    = 30;    // Pct for minimum start
  p->lccool.motorrampparam[COOLX_PUMP].initstart   = 15;    // Pct for initial startup
  p->lccool.motorrampparam[COOLX_PUMP].shutoffwait = 3000;  // Wait after turn off for turn on (ms)
  p->lccool.motorrampparam[COOLX_PUMP].rampuprate  = 4.0f;  // Pct per sec ramping up 
  p->lccool.motorrampparam[COOLX_PUMP].rampdnrate  = 3.0f;  // Pct per sec ramping down
  p->lccool.motorrampparam[COOLX_PUMP].subbrdtype  = 1;     // Sub-board type
  p->lccool.motorrampparam[COOLX_PUMP].hdrnum      = HDR_OC1;// Header (Relay index) to this motor
  // Heat Exchanger fan|blower IDX 2: Header OC3
  p->lccool.motorrampparam[COOLX_HEXFAN].idle        = 15;    // Pct for idle speed
  p->lccool.motorrampparam[COOLX_HEXFAN].minstart    = 30;    // Pct for minimum start
  p->lccool.motorrampparam[COOLX_HEXFAN].initstart   = 30;    // Pct for initial startup
  p->lccool.motorrampparam[COOLX_HEXFAN].shutoffwait = 3000;  // Wait after turn off for turn on (ms)
  p->lccool.motorrampparam[COOLX_HEXFAN].rampuprate  = 2.5f;  // Pct per sec ramping up 
  p->lccool.motorrampparam[COOLX_HEXFAN].rampdnrate  = 2.0f;  // Pct per sec ramping down
  p->lccool.motorrampparam[COOLX_HEXFAN].subbrdtype  = 1;     // Sub-board type
  p->lccool.motorrampparam[COOLX_HEXFAN].hdrnum      = HDR_OC2;// Header (Relay index) to this motor
  // IDX3 OC4
  p->lccool.motorrampparam[COOLX_JIC].idle        = 15;    // Pct for idle speed
  p->lccool.motorrampparam[COOLX_JIC].minstart    = 30;    // Pct for minimum start
  p->lccool.motorrampparam[COOLX_JIC].initstart   =  1;    // Pct for initial startup
  p->lccool.motorrampparam[COOLX_JIC].shutoffwait = 3000;  // Wait after turn off for turn on (ms)
  p->lccool.motorrampparam[COOLX_JIC].rampuprate  = 4.0f;  // Pct per sec ramping up 
  p->lccool.motorrampparam[COOLX_JIC].rampdnrate  = 2.5f;  // Pct per sec ramping down
  p->lccool.motorrampparam[COOLX_JIC].subbrdtype  = 0;     // Sub-board type (0 = diode flyback)
  p->lccool.motorrampparam[COOLX_JIC].hdrnum      = HDR_OC4;// Header (Relay index) to this motor

  /* ================ StringChgrfunction ============================ */
  p->lcstring.bmsnum_expected = 3; // Number of BMS nodes expected to be reporting

  p->lcstring.chgr_maxvolts = (72.0*3); // Set charger voltage limit (volts)
  p->lcstring.chgr_maxamps  = 1.0;      // Set charger current limit (amps)

  p->lcstring.canelcon.can.id = CANID_ELCON_RX; //C0372FA4 'ELCON Charger receive: ');  

/* ==================== ========================================= */
// CAN ids EMCMMC sends, others receive
   p->cid_unit_emcmmcx = I_AM_CANID; // A0000000

// List of CAN ID's for suscribing to incoming msgs
  p->cid_cmd_emcmmcx_pc  = CANID_CMD_EMCMMC1_PC; // 'A1600000','PC SENDS');
  p->cid_cmd_emcmmcx_emc = CANID_CMD_EMCMMC1_EMC;// 'A1800000', EMC SENDS'); 

  p->cid_uni_bms_emc1_i = CANID_UNI_BMS_EMC1_I;   // B0000000 UNIversal From EMC1,  Incoming msg to BMS: X4=target CANID');   
  p->cid_uni_bms_emc2_i = CANID_UNI_BMS_EMC2_I;   // B0200000 UNIversal From EMC2,  Incoming msg to BMS: X4=target CANID');   
  p->cid_uni_bms_pc_i   = CANID_UNI_BMS_PC_I;     // AEC00000 UNIversal From PC,    Incoming msg to BMS: X4=target CANID');   



	return;
}

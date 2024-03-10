/******************************************************************************
* File Name          : CoolingTask.h
* Date First Issued  : 12/21/2023
* Description        : Coolant and fan control
*******************************************************************************/

#ifndef __COOLINGTASK
#define __COOLINGTASK

#include <stdint.h>
#include "FreeRTOS.h"
#include "cmsis_os.h"
//#include "stm32l4xx_hal.h"
#include "CanTask.h"
#include "MailboxTask.h"
#include "adc_idx_v_struct.h"
#include "RyTask.h"

#define COOLX_TEMPS_NUM 6 // Number of temperatures managed
// 0 = cabs[ADC1IDX_THERMISTOR1]
// 1 = cabs[ADC1IDX_THERMISTOR2]
// 2 = cabs[ADC1IDX_THERMISTOR3]
// 3 = cabs[ADC1IDX_THERMISTOR4]
// 4 = cabs[ADC1IDX_DIVIDEDSPARE]
// 5 = DMOC CAN msg

#define COOLXNUM 4 // Number of cooling sub-board/headers
#define COOLX_DMOCFAN 0 // OC1 Header 
#define COOLX_PUMP    1 // OC2 Header 
#define COOLX_BLOWER  2 // OC3 Header 
#define COOLX_JIC     3 //OC4 Header

#define COOLTIMERMS 50 // Number ms per timer callback
#define MOTORNUM 4	//Number of motors controlled
#define COOLARRAYSZ 6 // Number of temperatures 

#define MOTSTATE_SPINDWN   0
#define MOTSTATE_MINSTAT   1
#define MOTSTATE_RAMPING   2
#define MOTSTATE_MINSTART1 3
#define MOTSTATE_MINSTART2 4
#define MOTSTATE_RUN       5
#define MOTSTATE_STOPPED   6
#define MOTSTATE_IDLE      7
#define MOTSTATE_SHUTDOWN  8


/* Command codes. */
#define MISCQ_COOL_TEMPS 1  // Send temperatures



struct COOLX
{
	uint8_t idx_ry;    // Map PWM device to board header
	int8_t status;     // 0 = idle; -1 = ramping up; 1 = run
	uint8_t pwm_idle;  // Low limit pwm;
	uint8_t pwm_ramp;  // % pwm per 100 ms tick
	uint32_t to_ctr;   // Timeout
	// The following are computed
	uint8_t pwm;       // Target pwm;
	uint32_t to_wkctr; // Timeout working counter
};

struct MOTORRAMPPARAM // Parameters
{
	uint32_t thermdelay;  // Wait for flow to give thermister reading (ms)
	uint32_t shutoffwait; // Spin-down duration (ms)
 	float rampuprate;     // Pct per sec ramping up 
	float rampdnrate;     // Pct per sec ramping down
	uint8_t idle;         // PWM for running at "idle" speed (0-100)
	uint8_t minstart;     // PWM required to assure motor runs (0-100)
	uint8_t initstart;    // Start from stopped initial pwm (0-100)
	uint8_t subbrdtype;   // Sub-board type (affects shutdown strategy)
	uint8_t hdrnum;       // Header number mapping (Relay number) to this motor
};

	struct MOTORRAMP // Working values
{
	struct RYREQ_Q ryreq;// Queue item for sending to RyTask 
	struct RYREQ_Q* pryreq;// Pointer to queue item
	int32_t shutdowntic; // Number of timer ticks to spin down
	int32_t spindownctr; // Working counter
	float ramppertickup; // Ramp rate: pct-per-timertick up
	float ramppertickdn; // Ramp rate: pct-per-timertick down
	float frampaccum;    // Current ramping 
	int16_t updatectr;   // Tick counter
	int16_t irampaccum;  // frampaccum converted for RyTask use
	int16_t irampaccum_prev; // Check for change
	uint8_t state;       // State machine
	uint8_t target;      // Striving to reach this pwm
};

struct MOTORCONTROL
{
	uint8_t X;
};

struct TEMPERATUREPARAM
{
	float tdel;     // Run Above Ambient
	float coef[2];  // pwm = coef[0]+coef[1]*Temperature
	uint8_t toohi;    // Temperature too high to run--Alert
	uint8_t not_installed;	
};
struct TEMPERATUREWORK
{
	float fpwm;   // Computed pwm for a given temperature
	uint8_t ipwm; // converted from fpwm
	uint8_t pcpwm;  // pwm commanded from PC
	uint8_t emcpwm; // pwm commanded from EMC
	uint8_t contactor; // 0 = off; not 0 = engaged
	uint32_t contactor_timeout; // CAN msg timeout counter
};

/* Working struct for EMC local function. */
struct COOLINGFUNCTION
{
	struct COOLX coolx[COOLXNUM];	

	uint32_t hbct_t;   // Duration between coolingfunction heartbeats(ms)
	uint32_t hbct_tic; // Loop tick count between coolingfunction heartbeats
	int32_t hbct_ctr;  // Heartbeat time count-down

	
	// Temperature sensing thermistors
	uint8_t tx_pmpo; // ADC index: coolant pump outlet thermistor
	uint8_t tx_moto; // ADC index: motor outlet thermistor
	uint8_t tx_hexo; // ADC index: heat exchange outlet thermistor
	uint8_t tx_amb;  // ADC index: ambient air temperature thermistor
	uint8_t tx_jic;  // ADC index: provision if 5th thermistor added to ADC sequence
	uint8_t tx_dmoc; // index: provision for DMOC CAN msg temperature report

	uint8_t status_cool; // 0 = no cooling; 1 = cooling in progress

	// Incoming CAN msg missing timeout
	int32_t timeout_CANdmoc;   // Time out limit (100 ms ticks)
//	int32_t timeout_mcstate;   // Time out limit (100 ms ticks)
	int32_t timeout_cntctrkar; // Time out limit (100 ms ticks)
	int32_t timeout_cmd_emcmmcx_pc; // Time out limit (100 ms ticks)
	int32_t timeout_cmd_emcmmcx_emc; // Time out limit (100 ms ticks)

	int32_t timeout_CANdmoc_ctr;   // Timeout counter: count down
//	int32_t timeout_mcstate_ctr;   // Timeout counter: count down
	int32_t timeout_cntctrkar_ctr; // Timeout counter: count down
	int32_t timeout_cmd_emcmmcx_pc_ctr; // Timeout counter: count down
	int32_t timeout_cmd_emcmmcx_emc_ctr; // Timeout counter: count down

	struct CANTXQMSG cancool1; // CAN1 msg for tx
	struct CANTXQMSG cancool2; // CAN2 msg for tx

	uint32_t cid_dmoc_actualtorq;// CANID_DMOC_ACTUALTORQ','47400000','DMOC',1,1,'I16','DMOC: Actual Torque: payload-30000'
	uint32_t cid_dmoc_hv_temps;  // CANID_DMOC_HV_TEMPS',  'CA200000','DMOC',1,1,'U8_U8_U8''DMOC: Temperature:rotor,invert,stator'
	uint32_t cid_mc_state;       // CANID_MC_STATE','26000000','MC',1,5,'U8_U8','MC: Launch state msg'
	uint32_t cid_cntctrkar;      // CANID_CMD_CNTCTRKAR'E3C00000','CNTCTR',1,6,'U8_U8_U8','Contactor1: R KeepAlive response'
	uint32_t cid_cmd_emcmmcx_pc; // CANID_CMD_EMCMMC1_PC' ,'A1600000','UNIT_ECM1PC' , 1,1,'U8_U8_U8_X4','bmsmot 1 PC SENDS'
	uint32_t cid_cmd_emcmmcx_emc;// CANID_CMD_EMCMMC1_EMC','A1800000','UNIT_ECM1PC' , 1,1,'U8_U8_U8_X4','bmsmot 1 PC SENDS'

	uint32_t cid_test;// TEST 


	// Extracted readings from DMOC CAN msg: cid_dmoc_hv_temps
	uint8_t rotortemp;  // (pcan->cd.uc[0] - 40);//*10;
	uint8_t invtemp;    // (pcan->cd.uc[1] - 40);//*10;
	uint8_t statortemp; // (pcan->cd.uc[2] - 40);//*10;
	uint8_t wcmottemp;  // worse-case motor temp (max of rotor & stator)

	// Motor ramp up/down 
	struct MOTORRAMPPARAM motorrampparam[MOTORNUM]; // Fixed parameters
	struct MOTORRAMP motorramp[MOTORNUM]; // Working values

	/* Temperature parameters: motors plus DMOC motor */
	struct TEMPERATUREPARAM	temperatureparm[COOLARRAYSZ];

	/* Pointers to incoming CAN msg mailboxes. */
	//struct MAILBOXCAN* pmbx_cid_dmoc_actualtorq; //'47400000','DMOC',1,1,'I16','DMOC: Actual Torque: payload-30000'
	struct MAILBOXCAN* pmbx_cid_dmoc_hv_temps;   //'CA200000','DMOC',1,1,'U8_U8_U8''DMOC: Temperature:rotor,invert,stator'
	//struct MAILBOXCAN* pmbx_cid_mc_state;        //'26000000','MC',1,5,'U8_U8','MC: Launch state msg'
	struct MAILBOXCAN* pmbx_cid_cntctrkar;       //'E3C00000','CNTCTR',1,6,'U8_U8_U8','Contactor1: R KeepAlive response'

	struct MAILBOXCAN* pmbx_cid_cmd_emcmmcx_pc;   //'A1600000','UNIT_ECM1PC' , 1,1,'U8_U8_U8_X4','bmsmot 1 PC SENDS');
	struct MAILBOXCAN* pmbx_cid_cmd_emcmmcx_emc; //'A1800000','UNIT_ECM1EMC', 1,1,'U8_U8_U8_X4','bmsmot 1 EMC SENDS');

	struct MAILBOXCAN* pmbx_cid_test; // TEST


};
/* *************************************************************************/
osThreadId xCoolingTaskCreate(uint32_t taskpriority);
/* @brief	: Create task; task handle created is global for all to enjoy!
 * @param	: taskpriority = Task priority (just as it says!)
 * @return	: CoolingTaskHandle
 * *************************************************************************/

 extern TaskHandle_t CoolingTaskHandle;


#endif


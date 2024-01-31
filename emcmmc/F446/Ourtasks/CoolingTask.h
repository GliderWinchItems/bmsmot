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

#define COOLXNUM 4 // Number of cooling sub-board/headers
#define COOLX_PUMP    0 //OC1 Header
#define COOLX_BLOWER  1 //OC2 Header
#define COOLX_DMOCFAN 2 //OC3 Header
#define COOLX_JIC     3 //OC4 Header

#define COOLTIMERMS 50 // Number ms per timer callback
#define MOTORNUM 4	//Number of motors controlled

#define MOTSTATE_SPINDWN  0
#define MOTSTATE_MINSTAT  1
#define MOTSTATE_RAMPING  2
#define MOTSTATE_MINSTART 3


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
	float runaa;    // Run Above Ambient
	float toohi;    // Temperature too high to run--Alert
	float coef[2];  // pwm = coef[0]+coef[1]*Temperature
};

/* Working struct for EMC local function. */
struct COOLINGFUNCTION
{
	struct COOLX coolx[COOLXNUM];	
	
	// Temperature sensing thermistors
	uint8_t tx_pmpo; // ADC index: coolant pump outlet thermistor
	uint8_t tx_moto; // ADC index: motor outlet thermistor
	uint8_t tx_hexo; // ADC index: heat exchange outlet thermistor
	uint8_t tx_amb;  // ADC index: ambient air temperature thermistor
	uint8_t tx_jic;  // ADC index: provision if 5th thermistor added to ADC sequence

	uint8_t status_cool; // 0 = no cooling; 1 = cooling in progress

	int32_t timeout_CANdmoc; // Time out limit (100 ms ticks)
	int32_t timeout_mcstate; // Time out limit (100 ms ticks)
	int32_t timeout_CANdmoc_ctr; // Timeout counter: count down
	int32_t timeout_mcstate_ctr; // Timeout counter: count down

	uint32_t cid_dmoc_actualtorq;// CANID_DMOC_ACTUALTORQ','47400000','DMOC',1,1,'I16','DMOC: Actual Torque: payload-30000'
	uint32_t cid_dmoc_hv_temps;  // CANID_DMOC_HV_TEMPS',  'CA200000','DMOC',1,1,'U8_U8_U8''DMOC: Temperature:rotor,invert,stator'
	uint32_t cid_mc_state;       // CANID_MC_STATE','26000000','MC',1,5,'U8_U8','MC: Launch state msg'

	// Extracted readings from DMOC CAN msg: cid_dmoc_hv_temps
	uint8_t rotortemp;  // (pcan->cd.uc[0] - 40);//*10;
	uint8_t invtemp;    // (pcan->cd.uc[1] - 40);//*10;
	uint8_t statortemp; // (pcan->cd.uc[2] - 40);//*10;
	uint8_t wcmottemp;  // worse-case motor temp (max of rotor & stator)

	// Motor ramp up/down 
	struct MOTORRAMPPARAM motorrampparam[MOTORNUM]; // Fixed parameters
	struct MOTORRAMP motorramp[MOTORNUM]; // Working values

	/* Pointers to incoming CAN msg mailboxes. */
	struct MAILBOXCAN* pmbx_cid_dmoc_actualtorq; //47400000','DMOC',1,1,'I16','DMOC: Actual Torque: payload-30000'
	struct MAILBOXCAN* pmbx_cid_dmoc_hv_temps;   //'CA200000','DMOC',1,1,'U8_U8_U8''DMOC: Temperature:rotor,invert,stator'
	struct MAILBOXCAN* pmbx_cid_mc_state;        //'26000000','MC',1,5,'U8_U8','MC: Launch state msg'

};
/* *************************************************************************/
osThreadId xCoolingTaskCreate(uint32_t taskpriority);
/* @brief	: Create task; task handle created is global for all to enjoy!
 * @param	: taskpriority = Task priority (just as it says!)
 * @return	: CoolingTaskHandle
 * *************************************************************************/

 extern TaskHandle_t CoolingTaskHandle;


#endif


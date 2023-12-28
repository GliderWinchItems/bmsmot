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

#define COOLXNUM 4 // Number of cooling sub-board/headers
#define COOLX_PUMP    0
#define COOLX_BLOWER  1
#define COOLX_DMOCFAN 2
#define COOLX_JIC     3


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


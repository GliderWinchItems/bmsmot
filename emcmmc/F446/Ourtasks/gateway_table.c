/******************************************************************************
* File Name          : gateway_table.c
* Date First Issued  : 09/09/2024
* Description        : CAN id table for passing CAN msgs 
*******************************************************************************/
#include "gateway_table.h"
#include "common_can.h"
#include "../../../../GliderWinchCommons/embed/svn_common/trunk/db/gen_db.h"

/* Table lookup to classify CAN ID */
/* ##### CANID *MUST* BE IN SORTED ORDER for bsearch ##### */
const struct CANIDCLASS canidclass[] =
{
	{CANID_MSG_CNTCTR1V,   C1SELCODE_CONTACTOR0 }, /* '50400000' Contactor1: poll response: HV1:Current sensor1 */
	{CANID_MSG_CNTCTR1A,   C1SELCODE_CONTACTOR0 }, /* '50600000' Contactor1: poll response: HV2 battery gnd to: DMOC+, HV3 DMOC- */
	{CANID_UNI_BMS_PC_I,   C1SELCODE_POLLS },      /* 'AEC00000' */
	{CANID_UNI_BMS_EMC2_I, C1SELCODE_POLLS },      /* 'AEE00000' */
	{CANID_UNI_BMS_PC_I,   C1SELCODE_POLLS },      /* 'B0000000' */
	{CANID_UNI_BMS_EMC2_I, C1SELCODE_POLLS },      /* 'B0200000' */ 
	{CANID_UNIT_BMS01,     C1SELCODE_BMS },        /* 'B0600000','UNIT_BMS01' */
	{CANID_UNIT_BMS02,     C1SELCODE_BMS },        /* 'B0800000','UNIT_BMS02' */
	{CANID_UNIT_BMS03,     C1SELCODE_BMS },        /* 'B0A00000','UNIT_BMS03' */
	{CANID_UNIT_BMS04,     C1SELCODE_BMS },        /* 'B0C00000','UNIT_BMS04' */
	{CANID_UNIT_BMS05,     C1SELCODE_BMS },        /* 'B0E00000','UNIT_BMS05' */
	{CANID_UNIT_BMS06,     C1SELCODE_BMS },        /* 'B1000000','UNIT_BMS06' */
	{CANID_UNIT_BMS07,     C1SELCODE_BMS },        /* 'B1200000','UNIT_BMS07' */
	{CANID_UNIT_BMS08,     C1SELCODE_BMS },        /* 'B1400000','UNIT_BMS08' */
	{CANID_UNIT_BMS09,     C1SELCODE_BMS },        /* 'B1600000','UNIT_BMS09' */
	{CANID_UNIT_BMS10,     C1SELCODE_BMS },        /* 'B1800000','UNIT_BMS10' */
	{CANID_UNIT_BMS11,     C1SELCODE_BMS },        /* 'B1A00000','UNIT_BMS11' */
	{CANID_UNIT_BMS12,     C1SELCODE_BMS },        /* 'B1E00000','UNIT_BMS12' */
	{CANID_UNIT_BMS13,     C1SELCODE_BMS },        /* 'B2000000','UNIT_BMS13' */
	{CANID_UNIT_BMS14,     C1SELCODE_BMS },        /* 'B2200000','UNIT_BMS14' */
	{CANID_UNIT_BMS15,     C1SELCODE_BMS },        /* 'B2400000','UNIT_BMS15' */
	{CANID_UNIT_BMS16,     C1SELCODE_BMS },        /* 'B2600000','UNIT_BMS16' */
	{CANID_UNIT_BMS17,     C1SELCODE_BMS },        /* 'B2800000','UNIT_BMS17' */
	{CANID_UNIT_BMS18,     C1SELCODE_BMS },        /* 'B2A00000','UNIT_BMS18' */		
	{CANID_ELCON_TX,       C1SELCODE_ELCON },      /* 'C7FA872C' */
	{CANID_CMD_CNTCTR1R,   C1SELCODE_CONTACTOR0 }, /* 'E3600000' Contactor1: R Command response */
	{CANID_CMD_CNTCTR1I,   C1SELCODE_CONTACTOR0 }, /* 'E360000C' Contactor1: I Command incoming */
	{CANID_CMD_CNTCTRKAR,  C1SELCODE_CONTACTOR0 }, /* 'E3C00000' Contactor1: R KeepAlive response */
	{CANID_CMD_GEVCURKAR,  C1SELCODE_CONTACTOR0 }, /* 'E3E00000' GEVCUr: R KeepAlive response */
	{CANID_CMD_GEVCURKAI,  C1SELCODE_CONTACTOR0 }, /* 'E4200000' GEVCUr: I KeepAlive and connect command */
	{CANID_CMD_CNTCTRKAI,  C1SELCODE_CONTACTOR0 }, /* 'E3800000' Contactor1: I KeepAlive and connect command */
	{CANID_HB_CNTCTR1A,    C1SELCODE_CONTACTOR0 }, /* 'FF000000' Contactor1: Heartbeat: High voltage2:Current sensor2 */
	{CANID_HB_CNTCTR1V,    C1SELCODE_CONTACTOR0 }, /* 'FF800000' Contactor1: Heartbeat: High voltage1:Current sensor1 */
};
/*
-- DMOC sends. Unit #1 untranslated
INSERT INTO CANID VALUES ('CANID_DMOC_ACTUALTORQ','47400000','DMOC',1,1,'I16',       'DMOC: Actual Torque: payload-30000');
INSERT INTO CANID VALUES ('CANID_DMOC_SPEED',     '47600000','DMOC',1,1,'I16_X6',    'DMOC: Actual Speed (rpm?)');
INSERT INTO CANID VALUES ('CANID_DMOC_DQVOLTAMP', '47C00000','DMOC',1,1,'I16_I16_I16_I16','DMOC: D volt:amp, Q volt:amp');
INSERT INTO CANID VALUES ('CANID_DMOC_TORQUE',    '05683004','DMOC',1,1,'I16_I16',   'DMOC: Torque,-(Torque-30000)');
INSERT INTO CANID VALUES ('CANID_DMOC_CRITICAL_F','056837fc','DMOC',1,1,'NONE',      'DMOC: Critical Fault: payload = DEADB0FF');
INSERT INTO CANID VALUES ('CANID_DMOC_HV_STATUS', 'CA000000','DMOC',1,1,'I16_I16_X6','DMOC: HV volts:amps, status');
INSERT INTO CANID VALUES ('CANID_DMOC_HV_TEMPS',  'CA200000','DMOC',1,1,'U8_U8_U8',  'DMOC: Temperature:rotor,invert,stator');
-- DMOC sends: translated
-- Unit #2
INSERT INTO CANID VALUES ('CANID_DMOC2_ACTUALTORQ','4740001C','DMOC',1,1,'I16',       'DMOC2: Actual Torque: payload-30000');
INSERT INTO CANID VALUES ('CANID_DMOC2_SPEED',     '4760001C','DMOC',1,1,'I16_X6',    'DMOC2: Actual Speed (rpm?)');
INSERT INTO CANID VALUES ('CANID_DMOC2_DQVOLTAMP', '47C0001C','DMOC',1,1,'I16_I16_I16_I16','DMOC1: D volt:amp, Q volt:amp');
INSERT INTO CANID VALUES ('CANID_DMOC2_TORQUE',    '05683014','DMOC',1,1,'I16_I16',   'DMOC2: Torque,-(Torque-30000)');
INSERT INTO CANID VALUES ('CANID_DMOC2_CRITICAL_F','056838fc','DMOC',1,1,'NONE',      'DMOC2: Critical Fault: payload = DEADB0FF');
INSERT INTO CANID VALUES ('CANID_DMOC2_HV_STATUS', 'CA00001C','DMOC',1,1,'I16_I16_X6','DMOC2: HV volts:amps, status');
INSERT INTO CANID VALUES ('CANID_DMOC2_HV_TEMPS',  'CA20001C','DMOC',1,1,'U8_U8_U8',  'DMOC2: Temperature:rotor,invert,stator');
*/

/* Init code sets this to the number of elements in the above array. */
uint16_t cidclsz; 
/* *************************************************************************
 * uint16_t canidclass_init(void);
 *	@brief	: Setup size of table
 * *************************************************************************/
uint16_t canidclass_init(void)
{
	/* Number of elements in CAN1 id (const) lookup table. */
	cidclsz = sizeof(canidclass)/sizeof(canidclass[0]);
	return cidclsz;
}
/******************************************************************************
* File Name          : gateway_table.c
* Date First Issued  : 09/09/2024
* Description        : CAN id table for passing CAN msgs 
*******************************************************************************/
#include "gateway_table.h"
#include "common_can.h"
#include "../../../../GliderWinchCommons/embed/svn_common/trunk/db/gen_db.h"

/* Table lookup to classify CAN ID for StringChgrTask. */
/* ##### CANID *MUST* BE IN SORTED ORDER for bsearch ##### */
const struct CANIDCLASS canidclass[] =
{
	{CANID_MSG_CNTCTR1V,   C1SELCODE_CONTACTOR0, 0 }, /* '50400000' Contactor1: poll response: HV1:Current sensor1 */
	{CANID_MSG_CNTCTR1A,   C1SELCODE_CONTACTOR0, 1},  /* '50600000' Contactor1: poll response: HV2 battery gnd to: DMOC+, HV3 DMOC- */
	{CANID_CMD_PC_EMC,       C1SELCODE_EMC_CMDS, 0},  /* '98000000',PC sends commands to EMC */
	{CANID_CMD_EMC_EMC,      C1SELCODE_EMC_CMDS, 1},  /* '98200000',EMC sends commands to EMC1 or 2) */
	{CANID_UNI_BMS_PC_I,   C1SELCODE_POLLS, 0 },      /* 'AEC00000' */
	{CANID_UNI_BMS_EMC2_I, C1SELCODE_POLLS, 1 },      /* 'AEE00000' */
	{CANID_UNI_BMS_PC_I,   C1SELCODE_POLLS, 2 },      /* 'B0000000' */
	{CANID_UNI_BMS_EMC2_I, C1SELCODE_POLLS, 3 },      /* 'B0200000' */ 
	{CANID_UNIT_BMS01,      C1SELCODE_BMS,  0 },      /* 'B0600000','UNIT_BMS01' */
	{CANID_UNIT_BMS02,      C1SELCODE_BMS,  1 },      /* 'B0800000','UNIT_BMS02' */
	{CANID_UNIT_BMS03,      C1SELCODE_BMS,  2 },      /* 'B0A00000','UNIT_BMS03' */
	{CANID_UNIT_BMS04,      C1SELCODE_BMS,  3 },      /* 'B0C00000','UNIT_BMS04' */
	{CANID_UNIT_BMS05,      C1SELCODE_BMS,  4 },      /* 'B0E00000','UNIT_BMS05' */
	{CANID_UNIT_BMS06,      C1SELCODE_BMS,  5 },      /* 'B1000000','UNIT_BMS06' */
	{CANID_UNIT_BMS07,      C1SELCODE_BMS,  6 },      /* 'B1200000','UNIT_BMS07' */
	{CANID_UNIT_BMS08,      C1SELCODE_BMS,  7 },      /* 'B1400000','UNIT_BMS08' */
	{CANID_UNIT_BMS09,      C1SELCODE_BMS,  8 },      /* 'B1600000','UNIT_BMS09' */
	{CANID_UNIT_BMS10,      C1SELCODE_BMS,  9 },      /* 'B1800000','UNIT_BMS10' */
	{CANID_UNIT_BMS11,      C1SELCODE_BMS, 10 },      /* 'B1A00000','UNIT_BMS11' */
	{CANID_UNIT_BMS12,      C1SELCODE_BMS, 11 },      /* 'B1E00000','UNIT_BMS12' */
	{CANID_UNIT_BMS13,      C1SELCODE_BMS, 12 },      /* 'B2000000','UNIT_BMS13' */
	{CANID_UNIT_BMS14,      C1SELCODE_BMS, 13 },      /* 'B2200000','UNIT_BMS14' */
	{CANID_UNIT_BMS15,      C1SELCODE_BMS, 14 },      /* 'B2400000','UNIT_BMS15' */
	{CANID_UNIT_BMS16,      C1SELCODE_BMS, 15 },      /* 'B2600000','UNIT_BMS16' */
	{CANID_UNIT_BMS17,      C1SELCODE_BMS, 16 },      /* 'B2800000','UNIT_BMS17' */
	{CANID_UNIT_BMS18,      C1SELCODE_BMS, 17 },      /* 'B2A00000','UNIT_BMS18' */		
	{CANID_ELCON_RX,      C1SELCODE_ELCON,  1 },      /* 'C0372FA4' Someone else is polling ELCON */
	{CANID_ELCON_TX,      C1SELCODE_ELCON,  0 },      /* 'C7FA872C' ELCON charger TX */
	{CANID_CMD_CNTCTR1R,   C1SELCODE_CONTACTOR0, 2 }, /* 'E3600000' Contactor1: R Command response */
	{CANID_CMD_CNTCTR1I,   C1SELCODE_CONTACTOR0, 3 }, /* 'E360000C' Contactor1: I Command incoming */
	{CANID_CMD_CNTCTRKAR,  C1SELCODE_CONTACTOR0, 4 }, /* 'E3C00000' Contactor1: R KeepAlive response */
	{CANID_CMD_GEVCURKAR,  C1SELCODE_CONTACTOR0, 5 }, /* 'E3E00000' GEVCUr: R KeepAlive response */
	{CANID_CMD_GEVCURKAI,  C1SELCODE_CONTACTOR0, 6 }, /* 'E4200000' GEVCUr: I KeepAlive and connect command */
	{CANID_CMD_CNTCTRKAI,  C1SELCODE_CONTACTOR0, 7 }, /* 'E3800000' Contactor1: I KeepAlive and connect command */
	{CANID_HB_CNTCTR1A,    C1SELCODE_CONTACTOR0, 8 }, /* 'FF000000' Contactor1: Heartbeat: High voltage2:Current sensor2 */
	{CANID_HB_CNTCTR1V,    C1SELCODE_CONTACTOR0, 9 }, /* 'FF800000' Contactor1: Heartbeat: High voltage1:Current sensor1 */
};

/*-- DMOC sends. Unit #1 untranslated */
/* Table lookup for DMOC task. */
const struct CANIDCLASS canidclass1[] =
{
	{CANID_DMOC_TORQUE,     C1SELCODE_DMOC, 0 }, /*'05683004': Torque,-(Torque-30000) */
	{CANID_DMOC_CRITICAL_F, C1SELCODE_DMOC, 1 }, /*'056837fc': Critical Fault: payload = DEADB0FF */
	{CANID_DMOC_ACTUALTORQ, C1SELCODE_DMOC, 2 }, /*'47400000': Actual Torque: payload-30000 */
	{CANID_DMOC_SPEED,      C1SELCODE_DMOC, 3 }, /*'47600000': Actual Speed (rpm?) */
	{CANID_DMOC_DQVOLTAMP,  C1SELCODE_DMOC, 4 }, /*'47C00000': D volt:amp, Q volt:amp */
	{CANID_DMOC_HV_STATUS,  C1SELCODE_DMOC, 5 }, /*'CA000000': HV volts:amps, status */
	{CANID_DMOC_HV_TEMPS,   C1SELCODE_DMOC, 6 }, /*'CA200000': Temperature:rotor,invert,stator */
};


/* Init code sets this to the number of elements in the above array. */
uint16_t cidclsz; 
uint16_t cidcl1sz; 

/* *************************************************************************
 * uint16_t canidclass_init(void);
 *	@brief	: Setup size of table
 * *************************************************************************/
uint16_t canidclass_init(void)
{
	/* Number of elements in CAN1 id (const) lookup table. */
	cidclsz = sizeof(canidclass)/sizeof(canidclass[0]);
	return cidcl1sz;
}
uint16_t canidclass1_init(void)
{
	/* Number of elements in CAN1 id (const) lookup table. */
	cidcl1sz = sizeof(canidclass1)/sizeof(canidclass1[0]);
	return cidcl1sz;
}
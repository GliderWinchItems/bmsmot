/******************************************************************************
* File Name          : gateway_table.h
* Date First Issued  : 09/09/2024
* Description        : CAN id table for passing CAN msgs 
*******************************************************************************/

#ifndef __GATEWAYTABLE
#define __GATEWAYTABLE
#include <stdint.h>

/* CAN msg ID codes for CANRCVBUFS passed to StringChgrTask. */
#define C1SELCODE_NOT_FOUND  0  // Not in table
#define C1SELCODE_BMS        1 // BMS node CAN id 
#define C1SELCODE_ELCON      2 // ELCON (not translated) on string
#define C1SELCODE_POLLS      3 // PC, EMC1, EMC2 polling
#define C1SELCODE_CONTACTOR0 4 // CONTACTOR on string
#define C1SELCODE_EMC_CMDS   6 // PC->EMC, EMC->EMC1,2

#define C1SELCODE_DMOC   5 // Temp code for now

#define BMSNODEIDSZ 18 // Number of CAN IDs in table

/* Classification lookup table element. */
struct CANIDCLASS
{
	uint32_t id;   // CAN id
	uint16_t code; // code identifies "group"
	uint16_t rmap; // remapping (0 - n)
};
/* *************************************************************************/
 uint16_t canidclass_init(void);
 uint16_t canidclass1_init(void);
/*	@brief	: Setup size of table
 * *************************************************************************/

//extern const struct CANIDCLASS canidclass[];

/* Init code sets this to the number of elements in the above array. */
extern uint16_t cidclsz; 
extern uint16_t cidcl1sz; 


#endif

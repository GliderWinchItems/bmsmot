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
#define C1SELCODE_CONTACTOR1 4 // CONTACTOR on string
#define C1SELCODE_CONTACTOR2 4 // CONTACTOR on string
#define C1SELCODE_CONTACTOR3 4 // CONTACTOR on string


struct CANIDCLASS
{
	uint32_t id;   // CAN id
	uint16_t code; // code identifies "who" it is for
};

/* *************************************************************************/
 uint16_t canidclass_init(void);
/*	@brief	: Setup size of table
 * *************************************************************************/

//extern const struct CANIDCLASS canidclass[];

/* Init code sets this to the number of elements in the above array. */
extern uint16_t cidclsz; 

#endif

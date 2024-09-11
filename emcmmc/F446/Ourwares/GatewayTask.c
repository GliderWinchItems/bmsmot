/******************************************************************************
* File Name          : GatewayTask.c
* Date First Issued  : 02/25/2019
* Description        : Bulk transfer to StringChgrTask, notified by MailboxTask
*******************************************************************************/
/*
  09/07/2024 revised for bmsmot/emcmmc 
For managing battery string BMS nodes the CAN msgs are in numerical sequence which
allows selection based on range of CAN id. Mailbox is better suited for single
selections.

There is also a bridging between CAN1 and CAN2 that the emcmmc handles.
CAN1 - BMS modules; DMOC; ELCON charger; contactor; +12v power CAN1 bus control
CAN2 - system CAN bus

*/

/*
  02/26/2019
This task works in conjunction with 'MailboxTask'.  'MailboxTask' notifies this
task when a CAN module circular buffer has one or more CAN msgs.  This task directly
accesses the circular buffer via 'take' pointer.

The 'MailboxTask' is likely a high FreeRTOS priority task.  This task might run at
a lower priority since timing is not critical, however delays require that the 
circular buffer be large enough to avoid overrun since there is no protection for
buffer overflow (since CAN msgs are added to the circular buffer under interrupt).

  09/09/2024
This takes a CAN msg from the incoming the Mailbox circular buffer (see above) and
checks if it should be sent to an emcmmc task. If so, it places the msg on a 
circular buffer for the task and makes a notification. 

The table for determining the fate of the CAN msg stored in flash, and the struct
holds the CAN id (of course) and a code for the use, e.g. is it a BMS node msg. The
recipient of the msg, e.g. StringChgrTask, can use the code and avoid another CAN id
lookup.
*/

#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_can.h"
#include "GatewayTask.h"
#include "can_iface.h"
#include "MailboxTask.h"
#include "getserialbuf.h"
#include "SerialTaskSend.h"
#include "morse.h"
#include "yprintf.h"
//#include "SerialTaskReceive.h"
//#include "gateway_PCtoCAN.h"
//#include "gateway_CANtoPC.h"
#include "main.h"
#include "StringChgrTask.h"
#include "gateway_table.h"

uint32_t dbuggateway1;


/* Circular buffers with selected CAN msgs. */
#define CAN1CIRBUFSIZE 32
#define CAN2CIRBUFSIZE 16

struct CANRCVBUFS can1cirbuf[CAN1CIRBUFSIZE];
struct CANRCVBUFS can2cirbuf[CAN2CIRBUFSIZE];

struct CIRBUF
{
	struct CANRCVBUFS* pbegin;
	struct CANRCVBUFS* pend;
	struct CANRCVBUFS* padd;
	struct CANRCVBUFS* ptake;	
};
static struct CIRBUF circan1;
static struct CIRBUF circan2;

extern UART_HandleTypeDef huart3;

/* from 'main' */
extern struct CAN_CTLBLOCK* pctl0;	// Pointer to CAN1 control block
extern struct CAN_CTLBLOCK* pctl1;	// Pointer to CAN2 control block

void StartGatewayTask(void const * argument);
static struct CANIDCLASS* selectCAN1(struct CANRCVBUF* pcan);

/* Table (in flash) for CAN id lookup. */
extern const struct CANIDCLASS canidclass[];

osThreadId GatewayTaskHandle;

/* *************************************************************************
 * struct CANRCVBUFS* GatewayTask_takecan1(void);
 * struct CANRCVBUFS* GatewayTask_takecan2(void);
 *	@brief	: Get CAN1 msg if buffer not empty
 *  @return : NULL = no entries, else pointer to CAN msg on buffer with code
 * *************************************************************************/
static struct CANRCVBUFS* takecan(struct CIRBUF* pcircan)
{
	struct CANRCVBUFS* ptmp;
	if (pcircan->ptake == pcircan->padd)
		return NULL;
	ptmp = pcircan->ptake;
	pcircan->ptake += 1;
	if (pcircan->ptake >= pcircan->pend) 
		pcircan->ptake = pcircan->pbegin;
	return ptmp;
}
struct CANRCVBUFS* GatewayTask_takecan1(void)
{
	return takecan(&circan1);
}
struct CANRCVBUFS* GatewayTask_takecan2(void)
{
	return takecan(&circan2);
}
/* *************************************************************************
 * static int8_t addcan(struct CIRBUF* pcircan, struct CANIDCLASS* pcl);
 *	@brief	: Add CAN1/2 msg to circular buffer
 *  @param  : pcircan = pointer to pointers to circular buffer
 *  @param  : pcl = pointer to const table in flash
 *  @param  : pcan = pointer to CAN msg to be added to buffer
 * *************************************************************************/
static int8_t addcan(struct CIRBUF* pcircan, struct CANRCVBUF* pcan, struct CANIDCLASS* pcl)
{
	struct CANRCVBUFS* ptmp = pcircan->padd; // Current available position
	ptmp += 1; // Advance and check if an overflow
	if (ptmp >= pcircan->pend) ptmp = pcircan->pbegin;
	if (ptmp == pcircan->ptake)
		return -1; // Here: overflow
	pcircan->padd->can = *pcan; // Copy CAN msg to buffer
	pcircan->padd->pcl = pcl;   // Selection code
	pcircan->padd      = ptmp;  // Update add pointer to next position
	return 0;
}
/* *************************************************************************
 * osThreadId xGatewayTaskCreate(uint32_t taskpriority);
 * @brief	: Create task; task handle created is global for all to enjoy!
 * @param	: taskpriority = Task priority (just as it says!)
 * @return	: GatewayHandle
 * *************************************************************************/
osThreadId xGatewayTaskCreate(uint32_t taskpriority)
{
 /* definition and creation of CanTask */
   osThreadDef(GatewayTask, StartGatewayTask, osPriorityNormal, 0, (272));
   GatewayTaskHandle = osThreadCreate(osThread(GatewayTask), NULL);
	vTaskPrioritySet( GatewayTaskHandle, taskpriority );

	return GatewayTaskHandle;
}
/* *************************************************************************
 * void StartGatewayTask(void const * argument);
 *	@brief	: Task startup
 * *************************************************************************/
uint8_t gatercvflag = 0;

void StartGatewayTask(void const * argument)
{
	int i;
	struct CANRCVBUFN* pncan;

	// Pre-load fixed elements for queue to CAN 'put' 
	// CAN1
	struct CANTXQMSG canqtx1;
	canqtx1.pctl       = pctl0;
	canqtx1.maxretryct = 8;
	canqtx1.bits       = 0; // /NART

#ifdef CONFIGCAN2 // CAN2 implemented
   // CAN2
	struct CANTXQMSG canqtx2;
	canqtx2.pctl       = pctl1;
	canqtx2.maxretryct = 8;
	canqtx2.bits       = 0; // /NART
#endif

	/* Setup serial output buffers for uarts. */
	struct SERIALSENDTASKBCB* pbuf2 = getserialbuf(&HUARTMON,  96); // PC monitor uart
//	struct SERIALSENDTASKBCB* pbuf3 = getserialbuf(&HUARTMON,  96); // PC monitor uart

	/* Pointers into the CAN  msg circular buffer for each CAN module. */
	struct CANTAKEPTR* ptake[STM32MAXCANNUM] = {NULL};

	/* Init pointers for circular buffering. */
	circan1.pbegin = &can1cirbuf[0];
	circan1.padd   = &can1cirbuf[0];
	circan1.ptake  = &can1cirbuf[0];
	circan1.pend   = &can1cirbuf[CAN1CIRBUFSIZE];

	circan2.pbegin = &can2cirbuf[0];
	circan2.padd   = &can2cirbuf[0];
	circan2.ptake  = &can2cirbuf[0];
	circan2.pend   = &can2cirbuf[CAN2CIRBUFSIZE];

gatercvflag = 1;

	/* Get pointers to circular buffer pointers for each CAN module in list. */	
	for (i = 0; i < STM32MAXCANNUM; i++)
	{
		if ((mbxcannum[i].pmbxarray != NULL) && (mbxcannum[i].pctl != NULL))
		{
			ptake[i] = can_iface_add_take(mbxcannum[i].pctl);
			yprintf(&pbuf2,"\n\rStartGateway: mbxcannum[%i] setup OK. array: 0x%08X pctl: 0x%08X",i,mbxcannum[i].pmbxarray,mbxcannum[i].pctl);
		}
		else
		{
			yprintf(&pbuf2,"\n\rStartGateway: mbxcannum[%i] was not setup.",i);
		}
	}

	/* Init number of elements in CAN1 id (const) lookup table. */
	canidclass_init(); // sets: uint16_t cidclsz;

#if 0 // Started by CanTask
	/* Start CANs */
extern CAN_HandleTypeDef hcan1;
	HAL_CAN_Start(&hcan1); // CAN1

#ifdef CONFIGCAN2
	extern CAN_HandleTypeDef hcan2;	
		HAL_CAN_Start(&hcan2); // CAN2
#endif
#endif

	uint32_t noteval = 0;    // Receives notification word upon an API notify

  /* Infinite RTOS Task loop */
  for(;;)
  {
		/* Wait for 'MailboxTask' notifications. */
		xTaskNotifyWait(0,0xffffffff, &noteval, portMAX_DELAY);

		/* CAN1 incoming msg: Check notification bit */
		i = 0;	// CAN1 index
		{
			if ((noteval & (1 << i)) != 0)
			{
				do
				{
					/* Get pointer into CAN msg circular buffer */
					pncan = can_iface_get_CANmsg(ptake[i]);
					if (pncan != NULL)
					{			
					/* Convert binary to the ascii/hex format for PC. */
						canqtx2.can = pncan->can; // Save a local copy

						struct CANIDCLASS* pcl = selectCAN1(&pncan->can); // Selection for StringChgrTask
						if (pcl != NULL)
						{ // Here, CAN msg is in table for StringChgrTask
							if (pcl->code == C1SELCODE_BMS)
							{	/* Place CAN msg on circular buffer w selection code */
								if (addcan(&circan1, &pncan->can, pcl) == 0)
								{ // Here, success (no overflow)
									/* Notify StringChgrTask of an addition to buffer. */
									extern TaskHandle_t StringChgrTaskHandle;							
									xTaskNotify(StringChgrTaskHandle,STRINGCHRGBIT00,eSetBits);

						/* Bridging === CAN1 -> CAN2 === */
	#ifdef CONFIGCAN2 // CAN2 setup
						/* === CAN1 -> CAN2 === */
	//?						xQueueSendToBack(CanTxQHandle,&canqtx2,portMAX_DELAY);
	#endif
								}
							}
						}
					}
				} while (pncan != NULL);	// Drain the buffer
			}
		}
#ifdef CONFIGCAN2 // CAN2 implemented
		/* CAN2 incoming msg: Check notification bit */
		i = 1;	// CAN2 index
		{
			if ((noteval & (1 << i)) != 0)
			{
				do
				{
					/* Get pointer into CAN msg circular buffer */
					pncan = can_iface_get_CANmsg(ptake[i]);
					if (pncan != NULL)
					{			
					/* Convert binary to the ascii/hex format for PC. */
						canqtx1.can = pncan->can;	// Save a local copy

					/* Bridging === CAN2 -> CAN1 === */
//?						xQueueSendToBack(CanTxQHandle,&canqtx1,portMAX_DELAY);
					}
				} while (pncan != NULL);	// Drain the buffer
			}
		}
#endif
  }
}
/* *************************************************************************
 * static struct CANIDCLASS* selectCAN1(struct CANRCVBUF* pcan;
 *	@brief	: Find if CAN id is in table (in flash) for StringChgrTask
 *  @param  : pcan = pointer to msg of interest
 *  @return : NULL = not found, otherwise pointer to table entry
 * *************************************************************************/
//#define LINEAR_SEARCH 
#ifdef LINEAR_SEARCH
/* LINEAR SEARCH */
static struct CANIDCLASS* selectCAN1(struct CANRCVBUF* pcan)
{
	struct CANIDCLASS* pcl = &canidclass[0];
	uint32_t id = pcan->id;
	for (int i = 0; i < cidclsz; i++)
	{
		if (pcl->id == id)
		{
			return pcl;
		}
		pcl += 1;
	}
	return NULL; // No match
}

#else
/* BSEARCH */
static struct CANIDCLASS* selectCAN1(struct CANRCVBUF* pcan)
{
	struct CANIDCLASS* pcl = &canidclass[0];
	uint32_t id = pcan->id;
	int16_t i,j,k;
	  i = 0; j = (cidclsz - 1);
	  while (i <= j) 
	  {
	      k = i + ((j - i) / 2);
	      if ((pcl+k)->id == id) 
	      { // Here, found
	      	return (pcl+k);
	      }
	      else if ((pcl+k)->id < id) 
	          i = k+1;
	      else 
	          j = k-1;
	  }
	return NULL; // No match
}
#endif	


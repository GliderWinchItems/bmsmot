/******************************************************************************
* File Name          : GatewayTask.c
* Date First Issued  : 02/25/2019
* Description        : PC<->gateway using usart2, notified by MailboxTask
*******************************************************************************/
/*
  02/26/2019
This task works in conjunction with 'MailboxTask'.  'MailboxTask' notifies this
task when a CAN module circular buffer has one or more CAN msgs.  This task directly
accesses the circular buffer via 'take' pointer.

The 'MailboxTask' is likely a high FreeRTOS priority task.  This task might run at
a lower priority since timing is not critical, however delays require that the 
circular buffer be large enough to avoid overrun since there is no protection for
buffer overflow (since CAN msgs are added to the circular buffer under interrupt).

This version only handles PC->CAN bus msgs for CAN1 module.  To mix CAN1 and CAN2
requires implementing the scheme of commandeering the low order bit(s) from the
sequence number byte.

CAN->PC direction CAN1 and CAN2 msgs are mixed together, except for the cases where
the CANIDs are identical such as DMOC msgs, in which case the CAN2 msgs are tagged 
as 29b address. [04/06/2019--not implemented]
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
#include "SerialTaskReceive.h"
#include "gateway_PCtoCAN.h"
#include "gateway_CANtoPC.h"
#include "main.h"

uint32_t dbuggateway1;

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;

/* from 'main' */
extern struct CAN_CTLBLOCK* pctl0;	// Pointer to CAN1 control block
extern struct CAN_CTLBLOCK* pctl1;	// Pointer to CAN2 control block

void StartGatewayTask(void const * argument);

osThreadId GatewayTaskHandle;

/* A notification to Gateway copies the internal notification word to this. */
uint32_t GatewayTask_noteval = 0;    // Receives notification word upon an API notify

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
//osDelay(100); // Debugging

	int i;

	/* The lower order bits are reserved for incoming CAN module msg notifications. */
	#define TSKGATEWAYBITc1	 (1 << (STM32MAXCANNUM + 2))  // Task notification bit for huart2 incoming ascii CAN
	#define TSKGATEWAYBITCDC (1 << (STM32MAXCANNUM + 3))  // Task notification bit for CDC_OUT (read via usb)

	/* notification bits processed after a 'Wait. */
	uint32_t noteused = 0;

	struct SERIALRCVBCB* prbcb2;	// usart2 (PC->CAN msgs)
	struct CANRCVBUFPLUS* pcanp;  // Basic CAN msg Plus error and seq number
	struct CANRCVBUFN* pncan;

	/* PC, or other CAN, to CAN msg */
	// Pre-load fixed elements for queue to CAN 'put' 

#ifdef CONFIGCAN2 // CAN2 implemented
	// CAN1
	struct CANTXQMSG canqtx1;
	canqtx1.pctl       = pctl0;
	canqtx1.maxretryct = 8;
	canqtx1.bits       = 0; // /NART
#endif

   // CAN2
	struct CANTXQMSG canqtx2;
	canqtx2.pctl = pctl1;
	canqtx2.maxretryct = 8;
	canqtx2.bits       = 0; // /NART

	// PC -> CAN1 (no PC->CAN2)
	struct CANTXQMSG pccan1;
	pccan1.pctl       = pctl0;
	pccan1.maxretryct = 8;
	pccan1.bits       = 0; // /NART

	/* Setup serial output buffers for uarts. */
	struct SERIALSENDTASKBCB* pbuf2 = getserialbuf(&HUARTMON,  96); // PC monitor uart
	struct SERIALSENDTASKBCB* pbuf3 = getserialbuf(&HUARTGATE,128); // Gateway uart, CAN1

#ifdef CONFIGCAN2 // CAN2 implemented
	struct SERIALSENDTASKBCB* pbuf4 = getserialbuf(&HUARTGATE,128); // Gateway uart, CAN2
#endif


	/* Pointers into the CAN  msg circular buffer for each CAN module. */
	struct CANTAKEPTR* ptake[STM32MAXCANNUM] = {NULL};

	/* Setup serial input buffering and line-ready notification */
     //   (ptr uart handle, dma flag, notiification bit, 
     //   ptr notification word, number line buffers, size of lines, 
     //   dma buffer size);
	/* PC-to-CAN ascii/hex incoming "lines" directly converts to CAN msgs. */

	/* WARNING: For the CAN mode, the number of bytes for the line buffer
      must be result in an alignment for double word copying, or else a Hard Fault
      will be thrown. See struct 'CANRCVBUFPLUS' in 'SerialTaskReceive.h'.
      struct CANRCVBUF is four words, but the 'PLUS adds two bytes, so rounding
      up makes 20 the minimum size. */
	prbcb2 = xSerialTaskRxAdduart(&HUARTGATE,1,TSKGATEWAYBITc1,\
		&GatewayTask_noteval,16,20,128,1); // buff 12 CAN, of 20 bytes, 192 total dma, CAN mode
	if (prbcb2 == NULL) morse_trap(41);

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

	/* Start CANs */
extern CAN_HandleTypeDef hcan1;
	HAL_CAN_Start(&hcan1); // CAN1
#ifdef CONFIGCAN2
	HAL_CAN_Start(&hcan2); // CAN2
#endif

//taskENTER_CRITICAL();
//  MX_USB_DEVICE_Init();
//taskEXIT_CRITICAL();

  /* Infinite RTOS Task loop */
  for(;;)
  {
		/* Wait for 'MailboxTask' notifications. */
		xTaskNotifyWait(noteused, 0, &GatewayTask_noteval, portMAX_DELAY);
		noteused = 0;	// Accumulate bits in 'noteval' processed.

		/* CAN1 incoming msg: Check notification bit */
		i = 0;	// CAN1 index
		{
			if ((GatewayTask_noteval & (1 << i)) != 0)
			{
				noteused |= (GatewayTask_noteval & (1 << i)); // We handled the bit			
				do
				{
					/* Get pointer into CAN msg circular buffer */
					pncan = can_iface_get_CANmsg(ptake[i]);
					if (pncan != NULL)
					{			
					/* Convert binary to the ascii/hex format for PC. */
						canqtx2.can = pncan->can; // Save a local copy
						xSemaphoreTake(pbuf3->semaphore, 5000);
						gateway_CANtoPC(&pbuf3, &canqtx2.can);

					/* === CAN1 -> PC === */			
						vSerialTaskSendQueueBuf(&pbuf3); // Place on queue for usart sending


#ifdef CONFIGCAN2 // CAN2 setup
					/* === CAN1 -> CAN2 === */
						xQueueSendToBack(CanTxQHandle,&canqtx2,portMAX_DELAY);
#endif
					}
				} while (pncan != NULL);	// Drain the buffer
			}
		}
#ifdef CONFIGCAN2 // CAN2 implemented
		/* CAN2 incoming msg: Check notification bit */
		i = 1;	// CAN2 index
		{
			if ((GatewayTask_noteval & (1 << i)) != 0)
			{
				noteused |= (GatewayTask_noteval & (1 << i)); // We handled the bit			
				do
				{
					/* Get pointer into CAN msg circular buffer */
					pncan = can_iface_get_CANmsg(ptake[i]);
					if (pncan != NULL)
					{			
					/* Convert binary to the ascii/hex format for PC. */
						canqtx1.can = pncan->can;	// Save a local copy
						xSemaphoreTake(pbuf4->semaphore, 5000);
						gateway_CANtoPC(&pbuf4, &canqtx1.can);

					/* === CAN2 -> PC === */			
						vSerialTaskSendQueueBuf(&pbuf4); // Place on queue for usart2 sendingpctocanc



					/* === CAN1 -> CAN2 === */
						xQueueSendToBack(CanTxQHandle,&canqtx1,portMAX_DELAY);
					}
				} while (pncan != NULL);	// Drain the buffer
			}
		}
#endif
  }
}


/******************************************************************************
* File Name          : uart-uart-rcv.c
* Date First Issued  : 06/08/2023
* Description        : 
*******************************************************************************/

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "cmsis_os.h"
#include "malloc.h"

#include "SerialTaskReceive.h"
#include "stm32f4xx_hal_usart.h"
#include "stm32f4xx_hal_uart.h"
#include "morse.h"
#include "main.h"

/* *************************************************************************
 * void uart-uart-rcv_init(void);
 * @brief	: Initialize the U-U receive of CAN msgs
 * *************************************************************************/
void uart-uart-rcv_init(void)
{
	/* DMA1 CH5 (usart2 read) peripheral and memory addresses: */
	hdma_usart2_rx.Instance->CPAR = (uint32_t)hspi1.Instance + 0x0C???; // SPI DR address
	hdma_usart2_rx.Instance->CMAR = (uint32_t)&spirx12.u8[0]???; // DMA stores from this array

	/* MX may have these setup */
	hdma_usart2_rx.Instance->CCR |=  0x2;  // Enable DMA read channel interrupt

	return;
}


/* #######################################################################
   UART interrupt callbacks
   ####################################################################### */
/* *************************************************************************
 * void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *phuart);
 * @brief	: DMA callback at the halfway point in the circular buffer
 * *************************************************************************/
/* NOTE: under interrupt from callback. */

/* DMA Half buffer complete callback (dma only) */
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *phuart)
{
	HAL_UART_RxCpltCallback(phuart);
}
/* *************************************************************************
 * void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *phuart);
 * @brief	: DMA callback at the halfway point in the circular buffer
 *				: OR, char-by-char completion of sending
 * *************************************************************************/
/* DMA buffer complete .*/

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *phuart)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	/* Look up buffer control block, given uart handle */
	struct SERIALRCVBCB* prtmp = prbhd;
	while (prtmp->phuart != phuart) 
	{
		prtmp++;
		if (prtmp == prtmp->pnext)      morse_trap(553);
		if (prtmp > (struct SERIALRCVBCB*)0x2001ff00) morse_trap(554);
	}

	if (SerialTaskReceiveHandle == NULL) return;

	/* Trigger Recieve Task to poll dma uarts */
	xTaskNotifyFromISR(SerialTaskReceiveHandle, 
		0,	/* 'or' bit assigned to buffer to notification value. */
		eSetBits,      /* Set 'or' option */
		&xHigherPriorityTaskWoken ); 

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	return;
}
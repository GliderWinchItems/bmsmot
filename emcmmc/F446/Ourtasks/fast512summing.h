/**
****************************************************************************
* File Name          : fast512summing.h
* Board              : bmsmot
* Date First Issued  : 11/27/23
* Description        : This sets the buffer and inline code size for ADC2 DMA
******************************************************************************
*/   
#ifndef __FAST512SUMMING
#define __FAST512SUMMING
/* Number of half-words (i.e. 12b ADC readings) of 1/2 the full dma buffer. */
#define FAST512SUMMING_BUFFSIZE 256
#endif

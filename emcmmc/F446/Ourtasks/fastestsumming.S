/**
****************************************************************************
* File Name          : fastestsumming.S
* Board              : bmsmot
* Date First Issued  : 11/23/23
* Description        : ADC2: sum readings, (readings - offset)^2
******************************************************************************
*/    
  .syntax unified
  .cpu cortex-m4
  .fpu softvfp
  .thumb

.global  fastestsumming

  .section  .text
  .type  fastestsumming, %function

  .macro  sumw from=0, to=31
loop\@ :
  ldrh  r4, [r6], #2
  add   r3, r4
  subs  r4, r4, r5
  smlal   r1, r2, r4, r4
    .if     \to-\from
      sumw "(\from+1)",\to
    .endif
    .endm


/*
C call to this--
void fastestsumming(struct SUMSQBUNDLE* psumsqbundle);
   psumsqbundle = pointer (r0) to following struct--
struct SUMSQBUNDLE
{
  uint64_t sumsq;     @  +0 Sum of (reading - offset)^2
  uint16_t* pdma;     @  +8 Ptr to dma buffer 
  uint16_t* pdma_end; @  +12 Ptr to dma buffer end+1
  uint32_t adcaccum;  @  +16 Sum of readings
  uint32_t adc2ctr;   @  +20 Running count of readings in both sums
  uint32_t offset;    @  +24 ADC2 offset with zero input 
};
 return: updates sumsqbundle: .pdma .sumsq .adcaccum .adc2ctr */
@ ===================================================
@ Struct pointer offsets 
    #define SUMSQ     0   
    #define PDMA      8
    #define PDMA_END 12
    #define ADCACCUM 16
    #define ADC2CTR  20
    #define OFFSET   24
@ =================================================== /*  
  Register usage--
r0 = pointer to struct SUMSQBUNDLE (passed in function call)
r1 = sumsq lo-ord
r2 = sumsq hi-ord
r3 = sum readings (adcaccum)
r4 = ADC reading (*pdma) | pdma-pdma_end/2
r5 = ADC2 zero input offset (offset) | adc2ctr update
r6 = pointer to input buffer (pdma)   
r7 = pointer to input buffer end (pdma_end)        */
@ ===================================================
fastestsumming:  
@ ===================================================
  push  {r4, r5, r6, r7}

@ Load registers via pointer to struct
  ldr  r6,[r0, #PDMA] @ pdma 
  ldr  r7,[r0, #PDMA_END] @ pdma_end

  @ adc2ctr += (pdma - pdma_end)/2 
  sub  r4, r7, r6 @ 
  lsr  r4, r4, #1 
  ldr  r5,[r0, #ADC2CTR] @ adc2ctr += number readings
  add  r5, r4
  str  r5,[r0, #ADC2CTR]

  ldrd r1, r2, [r0, SUMSQ] @ 64b sumsq
  ldr  r3,[r0, #ADCACCUM]  @ adcaccum (sum readings)
  ldrh r5,[r0, #OFFSET] 
  
@ Inline repetition of sum and sumsquare  
loop:
  @ Process 32 readings
  sumw

@ End of pass
  cmp   r6, r7
  bne   loop

@ End of total buffer. Update struct
  strd  r1, r2, [r0, #SUMSQ]  @ Update 64b sum sq
  str   r3, [r0, #ADCACCUM] @ Update adcaccum
  str   r6, [r0, #PDMA]
  pop  {r4, r5, r6, r7}
  bx  lr    
 .size  fastestsumming, .-fastestsumming

.end


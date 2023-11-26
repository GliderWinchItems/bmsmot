/**
****************************************************************************
* File Name          : fastsumming.S
* Board              : bmsmot
* Date First Issued  : 11/21/23
* Description        : ADC2: sum readings, (readings - offset)^2
******************************************************************************
*/    
  .syntax unified
  .cpu cortex-m4
  .fpu softvfp
  .thumb

.global  fastsumming

  .section  .text
  .type  fastsumming, %function

  .macro  sumz from=0, to=31
loop\@ :
  ldrh  r4, [r6, #\@*2]
  add   r3, r4
  subs  r4, r4, r5
  smlal   r1, r2, r4, r4
    .if     \to-\from
      sumz "(\from+1)",\to
    .endif
    .endm

/* C call to this--
void fastsumming(struct SUMSQBUNDLE* psumsqbundle, uint16_t idx);
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
  r1 = idx: 0 - 31 (not checked. MUST BE IN THIS RANGE)

  return: updates sumsqbundle: .sumsq .adcaccum .adc2ctr */
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
r0 = pointer to sumsqbundle struct (passed in function call)
r1 = sumsq lo-ord | idx (count_to_do) (passed in function call)
r2 = sumsq hi-ord
r3 = sum readings (adcaccum)
r4 = ADC reading (*pdma)
r5 = ADC2 zero input offset (offset) | adc2ctr update
r6 = pointer to input buffer (pdma)           */
@ ===================================================
fastsumming:  
  push  {r4, r5, r6}
@ ===================================================

@ Load registers via pointer (r0) to struct
  @ Update adc2ctr in struct
  ldr  r5, [r0, #ADC2CTR] @ Get current count
  add  r5, #32  @ Add max count
  sub  r5, r1   @ Subtract index
  str  r5, [r0, #ADC2CTR] @ Update 

  @ Computer jump address
  movs r4, #(loop1-loop0) @ size one macro item
  mul  r4, r4, r1 @ jump distance
  ldr  r1, =loop  @ Get base address
  add  r4, r1  @ (base + jump distance)
  orr  r4, #1 @ bit(0) must be a 1 to be valid

  @ Load regs, compute entry and jump
  ldrd r1, r2, [r0, #SUMSQ] @ Get sum-squared
  ldr  r3, [r0, #ADCACCUM] @ Get sum
  ldr  r5, [r0, #OFFSET] @ Zero input ADC offset
  ldr  r6, [r0, #PDMA]  @ Pointer to dma buffer

  bx   r4    @ branch to entry point.
  
@ Inline repetition of sum and sumsquare  
loop:
  sumz

@ End of pass: Update struct and return 
  strd  r1, r2, [r0, #SUMSQ] @ Update struct 
  str   r3,[r0, #ADCACCUM] @ readings summed
  @ ADC2CTR was updated at beginning
  pop  {r4, r5, r6}
  bx  lr    
 .size  fastsumming, .-fastsumming

.end


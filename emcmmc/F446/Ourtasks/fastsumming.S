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
  ldrh  r1, [r6]
  add   r3, r1
  subs  r1, r1, r2
  mla   r4, r1, r1, r4
  add   r6, r7
     .if     \to-\from
        sumz     "(\from+1)",\to
        .endif
        .endm

fastsumming:  
  push  {r4, r5, r6, r7, r8}

/*
C call to this--
void fastsumming(struct SUMSQBUNDLE* psumsqbundle);
Where pointer is the following--
struct SUMSQBUNDLE
{
  uint16_t* pdma;
  uint32_t adcaccum;
  uint32_t isumsq;
  uint16_t offset;
  uint16_t n;
};

Upon completion this routine 
updates sumsqbundle except: offset, n

Register usage--
r0 = pointer to sumsqbundle struct
r1 = reading
r2 = offset
r3 = sum readings
r4 = sum square of (reading - offset)
r5 = number to do
r6 = pointer to input buffer (pdma)
r7 = increment readings pointer = 2
*/
@ Load registers via pointer to struct
  ldrh r2,[r0, #12] @ offset
  ldr  r3,[r0, #4] @ adcaccum (sum readings)
  ldr  r4,[r0, #8] @ isumsq 
  ldrh r5,[r0, #14] @ n
  ldr  r6,[r0] @ pdma 
  mov  r7,#2 @ increment

@ Compute entry and jump
  mov  r1, #12 @ size of one item
  mul  r5,r5,r1 @ "n" * size of item
  ldr  r1, #loop @ Get base address
  add  r1,r5 @ Add base and offset
  bx   r1 @ branch to location.
  
@ Inline repetition of sum and sumsquare  
loop:
  sumz
 
@ Update struct and return
  str   r6,[r0] @ pdma pointer updated
  str   r4,[r0, #8] @ 32b sum squares updated
  str   r3,[r0, #4] @ sum readings summed
  pop  {r4, r5, r6, r7, r8}
  bx  lr    
 .size  fastsumming, .-fastsumming

.end


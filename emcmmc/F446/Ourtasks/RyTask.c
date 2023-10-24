/******************************************************************************
* File Name          : RyTask.h
* Date First Issued  : 06/14/2023
* Description        : Relay Task
*******************************************************************************/
/* NOTES:
- Incoming requests carry a pwm 0 - 100, and the timer channel associated with
the relay is set to by this, appropriately scaled. 0 is OFF, and 100 full ON.
Inbetween provides proportional output.

- The pwm frame-rate is the same for all timers (TIM1,2,3,12) used.

- Any or all relays can have a pull-in delay where 100% pwm is applied for a
pull-in time duration, followed by a reduction in the pwm for a holding a
current. The delay and holding pwm are specified in the xx-emc-idx-v-struct.c,
where xx is the board CAN ID, e.g. A0000000.

- Only one relay that requires a pull-in delay (i.e. intially 100% pwm) can be
started at a time. Requests that arrive while a relay is in the pull-in phase
are buffered.

- Relays that don't require a pull-in delay such as external FET drivers that
drive motors skip the pull-in delay. The incoming requests set the pwm for the
FET directly. 

- A keep-alive timeout can be specified in the parameters. In coming requests
reset the keep-alive timeout. If a timeout occurs the relay is turned off. 
Some relays may not have a keep-alive. A ~0L timeout skips the keep-alive.

10/12/23 - Resume debugging.

*/

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "malloc.h"

#include "main.h"
#include "morse.h"
#include "RyTask.h"
#include "emc_idx_v_struct.h"
#include "DTW_counter.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim12;

uint32_t debugry1;
uint32_t debugry2;
uint32_t debugry3;
uint32_t debugry4;
uint32_t debugry5;
uint32_t debugry6;
uint32_t debugry7;
uint32_t debugry[16];
uint32_t debugryx;

static void ry_setpwm(uint32_t pwm, uint8_t idx);

TaskHandle_t RyTaskHandle = NULL;

/* MX sets timers: 1,2,3,12 to 36000 ARR with 90 MHz timer clocking */
#define PWMPCTtoTICK 360 // Number of timer ticks per percent PWM

/* Queue */
#define QUEUESIZE 16	// Total size of relay queue
osMessageQId RyTaskReadReqQHandle;


/* Relay status and working vars. */
#define NRELAYS 12 // Number of fet driver relay outputs
struct RELAYWV // Working variables
{
   uint32_t kp_wv; // Maximum time duration (ms) between keep-alive requests
                      //  ~0 = no timeout (i.e. always alive)
   uint16_t pulldelay_wv; // Delay time duration (ms) before pwm for relay pull-in
                             //  ~0 = no delay (e.g. external fet drive)
   uint8_t pwm_wv;     // PWM percent when energized: (0 - 100)
                             //   100 = full ON; 20-40 = normal range, relay dependent
   int8_t on;         // minus = pulling in; zero = OFF; plus = pulled & pwm'ing

   uint8_t cancel;   // 1 = cancel request for delay; 0 = not cancel

   uint8_t kp_flag;   // Upon interrupt: 1 = reset kp time; 0 = let kp_wv countdown or remain zero
};
struct RELAYWV relaywv[NRELAYS];

struct EMCLC emclc;


struct RELAYDBUF // Buffer: relay requests pending turn-on delay
{
	uint8_t idx;
	uint8_t pwm;
	uint8_t on;
	uint8_t cancel;
};

#define RYBUFSIZE 16 // Max size of linked list
static struct RELAYDBUF rybuf[RYBUFSIZE];
static struct RELAYDBUF* prybuf_add; // 
static struct RELAYDBUF* prybuf_take; // 
//static struct RELAYDBUF* prybuf_isr; // 

static uint8_t idx_kp;  // Keep-alive interrupt timer cycles through relays
static  int8_t idx_delay; // Index for relay in process of pull-in; minus = none
static  int8_t idx_abort; // 1 = TIM9 abort current pullin
static uint8_t delay_flag; // 1 = TIM9 delay is in-progress; 0 = idle

/* *************************************************************************
 * static void rydbuf_startdelay(uint8_t idx);
 *	@brief	: Start delay timer
 *  @param  : idx = index of relay
 * *************************************************************************/
static void rydbuf_startdelay(uint8_t idx)
{ // Here, a time delay is not in progress
	if (delay_flag != 0) morse_trap(714);

	// Pull-in at 100%
	ry_setpwm(100, idx); // Set 100% pwm for this ry

	// Set delay countdown
	relaywv[idx].on = -1; // Set status: delay in progress

	// Tell TIM9 ISR which relay is being delayed
	idx_delay = idx;

	relaywv[idx].cancel = 0; // Reset any prior cancellations

	delay_flag = 1; // Delay timing is active
	
	// Set delay: TIM9 (does not have a count-down direction mode)
	TIM9->CNT  = 0;
	TIM9->ARR  = (10 * emcfunction.lc.relay[idx].pulldelay);
	TIM9->SR   = 0x0; // Clear interrupt flags jic
	TIM9->CR1  = 0x9; // Start counting (upwards), one-shot mode
debugry6 += 1;
	return;
}
/* *************************************************************************
 * static struct RELAYDBUF* rydbuf_search(uint8_t idx);
 *	@brief	: Search list for this relay
 *  @param  : idx = index of relay to be searched for
 *  @return : Not NULL: pointer to list entry that matches this index.
 *          : NULL: This relay index is not in the list
 * *************************************************************************/
 static struct RELAYDBUF* rydbuf_search(uint8_t idx)
 {
 	struct RELAYDBUF* padd;
 	struct RELAYDBUF* ptake;

 	if (prybuf_add == prybuf_take)
 		return NULL;

 	padd  = prybuf_add;
 	ptake = prybuf_take;
 	while (padd != ptake)
 	{
 		if (ptake->idx == idx)
 			return ptake;
 		ptake += 1;
 		if (ptake >= &rybuf[RYBUFSIZE]) 
 			ptake = &rybuf[0];
 	}
	return NULL;
}
/* *************************************************************************
 * static void rybuf_add(uint8_t idx);
 *	@brief	: Add relay request that involves a delay.
 * *************************************************************************/
static void rybuf_add(uint8_t pwm, uint8_t idx)
{
	prybuf_add->idx  = idx;
	prybuf_add->pwm  = pwm;
	prybuf_add->cancel = 0;
	/* Advance to next. */
	prybuf_add += 1;
	if (prybuf_add >= &rybuf[RYBUFSIZE]) 
		prybuf_add  = &rybuf[0];
	return;
}
/* *************************************************************************
 * static struct RELAYDBUF* rybuf_next(void);
 *	@brief	: Get buffer ptr to next request (that is not cancelled)
 *  @return : pointer to buffer position; NULL = no more in buffer
 * *************************************************************************/
struct RELAYDBUF* rybuf_next(void)
{
	while (prybuf_add != prybuf_take)
	{ // Here, buffer is not empty
		prybuf_take += 1; // Next entry
		if (prybuf_take >= &rybuf[RYBUFSIZE]) 
	 			prybuf_take = &rybuf[0];

	 	if (prybuf_take->cancel == 0)
			return prybuf_take;	 		
	} 
	return NULL;	
}
/* *************************************************************************
 * static void struct RELAYDBUF* rybuf_cancel(uint8_t idx);
 *	@brief	: Scan rybuf and cancel all entries for this ry.
 *  @param  : idx = relay index
 * *************************************************************************/
static void rybuf_cancel(uint8_t idx)
{
 	struct RELAYDBUF* padd;
 	struct RELAYDBUF* ptake;

 	if (prybuf_add == prybuf_take)
 		return; // Here, list is empty

 	padd  = prybuf_add;
 	ptake = prybuf_take;
 	while (padd != ptake)
 	{
 		if (ptake->idx == idx)
 		{
 			relaywv[ptake->idx].cancel = 1;
 		}
 		ptake += 1;
 		if (ptake >= &rybuf[RYBUFSIZE]) 
 			ptake = &rybuf[0];
 	}
	return;
}
/* *************************************************************************
 * void ry_init(void);
 *	@brief	: init 
 * *************************************************************************/
void ry_init(void)
{
	/* capture/compare enable register makes output active high. */
	/*                        CH1   CH2  CH3  CH4 */
	TIM1->CCER  = 0x1111; //  oc3   oc1  oc4  oc2
	TIM2->CCER  = 0x1110; // acdet jp21  oa1  oa3
	TIM3->CCER  = 0x1111; //  ob3   ob1  ob2  ob4
	TIM12->CCER = 0x0011; //  oa4   oa2

	/* Timers: configuration should have been done with 'MX initialization. */
	TIM1->CR1  |= 0x81; // Start PWM timer
	TIM2->CR1  |= 0x81; // Start PWM timer
	TIM3->CR1  |= 0x81; // Start PWM timer
	TIM12->CR1 |= 0x81; // Start PWM timer

	TIM5->CR1   = 0;
	TIM5->DIER  = 0x2; // Bit 1CC1IF: Capture/compare 1 interrupt flag
//	TIM5->PSC   = 4500;
//	TIM5->EGR = TIM_EGR_UG; // Trigger update of Prescaler
	TIM5->CCR1  = TIM5->CNT + KPUPDATEDUR;
	TIM5->SR    = 0; // Reset all flags jic
	TIM5->CR1   = 0x1; // Start Keep-alive timer 

	TIM9->SR    = 0;
	TIM9->DIER  = 0;

	/* Circular buffer hold requests needing a delay. */
	prybuf_add  = &rybuf[0]; // Ptr for additions
	prybuf_take = &rybuf[0]; // Ptr for removals
	return;
}
/* *************************************************************************
 * static void ry_setpwm(uint32_t pwm,uint8_t idx);
 *	@brief	: Set timer|channel with pwm setting
 *  @kparam : pwm = timer value to be set in channel CCRx
 *  @param  : idx = index into table for timer|channel
 * *************************************************************************/
static void ry_setpwm(uint32_t pwmx, uint8_t idx)
{
	uint16_t pwm = pwmx * PWMPCTtoTICK;
	/* Set timer pwm for this relay. */
	switch(idx)
	{
	/* Group A */
	case 0: // OA1 T2C3
		TIM2->CCR3 = pwm;
		break;
	case 1: // OA2 T12C2
		TIM12->CCR2 = pwm;
		break;
	case 2: // OA3 T2C4
		TIM2->CCR4 = pwm;
		break;
	case 3: // OA4 T12C1
		TIM12->CCR1 = pwm;
		break;

	/* Group B */		
	case 4: // OB1 T3C2
		TIM3->CCR2 = pwm;
		break;
	case 5: // OB2 T3C3
		TIM3->CCR3 = pwm;
		break;
	case 6: // OB3 T3C1
		TIM3->CCR1 = pwm;
		break;
	case 7: // OB4 T3C4
		TIM3->CCR4 = pwm;
		break;

	/* Group C */
	case 8: // OC1 T1C2
		TIM1->CCR2 = pwm;
		break;
	case 9: // OC2 T1C4
		TIM1->CCR4 = pwm;
		break;
	case 10: // OC3 T1C1
		TIM1->CCR1 = pwm;
		break;
	case 11: // OC4 T1C3
		TIM1->CCR3 = pwm;
		break;
	default:
		morse_trap(882);
		break;
	}

	return;	
}

/* *************************************************************************
 * void StartRyTask(void const * argument);
 *	@brief	: Task startup
 * *************************************************************************/
void StartRyTask(void* argument)
{
	BaseType_t ret;
	struct RYREQ_Q* pssb; // Pointer to struct for request details
	struct RELAYDBUF* prybuf_tmp;
	struct RELAYDBUF* ptmp;		
	uint32_t pwmtim;

	emc_idx_v_struct_hardcode_params(&emclc);

	ry_init(); // Start timers that drive relay fets

	for (;;)
	{
		/* Enable interrupts since disabled below.  */
		TIM9->DIER  = 0x1; //  Delay timer: UG interrupt
		TIM5->DIER |= 0x2; //  Keep-alive timer: OC1 

		/* Check queue of loaded items. */
		ret = xQueueReceive(RyTaskReadReqQHandle,&pssb,500000);
		if (ret == pdPASS)
		{ // Here, not a timeout waiting for a request
			// Out-of-range relay index check
debugry[debugryx] = pssb->idx;
debugryx += 1;
if (debugryx >= 16) debugryx = 0;

			if (pssb->idx >= NRELAYS) morse_trap(716); // Argh!

			/* Disable delay timer interrupt jic. */
			TIM9->DIER = 0;	
			/* Disable keep-live timer jic. */
			TIM5->DIER &= ~0x2; // Capture 1 disable	
			/* update the keep-alive timeout. */
			relaywv[pssb->idx].kp_wv = emcfunction.lc.relay[pssb->idx].kp; 

			pssb->cancel = 0; // JIC!

			/* Convert incoming pwm (0 - 100%) to timer pwm ticks */
			if (pssb->pwm == 255) // Use parameter value?
				pwmtim = emcfunction.lc.relay[pssb->idx].pwm;
			else
				pwmtim = pssb->pwm; // Use value in request

			/* Simple case where no pull-in delay is used. */
			if (emcfunction.lc.relay[pssb->idx].pulldelay == 0)
			{ // Here, no pull-in delay involved (e.g. pump motor)
				relaywv[pssb->idx].pwm_wv = pwmtim; // Update working pwm
				ry_setpwm(pssb->pwm,pssb->idx); // Update FET pwm drive
morse_trap(111);
				continue; // Onward!
			}

			/* Here--this relay has a pull-in delay parameter specified. */
			if (pssb->pwm == 0) // Is request to turn relay OFF?
			{ // Here, new pwm request turns relay off
debugry2 += 1;				
				ry_setpwm(pssb->pwm, pssb->idx); // Set relay FET off
				relaywv[pssb->idx].on = 0; // Ry status: OFF
				// Set cancel flag for all buffered entries for this relay
				rybuf_cancel(pssb->idx);
				// Special case: timer is timing this relay now?
				if (idx_delay == pssb->idx)
				{ // Here delay timer in progress for this relay
					TIM9->CR1 = 0x0; // Stop timer
					TIM9->SR = ~0x1; // Clear interrupt flag if on.
					idx_abort = 0; // ISR skips updating pwm

					/* More relays waiting for pull-in delay? */
					if ((ptmp=rybuf_next()) != NULL)
					{ // Here, buffer has a not-cancelled entry
						rydbuf_startdelay(ptmp->idx); // Load ry & start TIM9						
						continue; 
					}					
				}
			}
			else
			{ // Here, request is to start the relay (w pull-in delay)
//morse_trap(333);
				if (relaywv[pssb->idx].on == 0)
				{ // Relay is off, start 100% pwm and pull-in delay
					// Check relay is already in the delay buffer
//morse_trap(555);
					prybuf_tmp = rydbuf_search(pssb->idx);
					if (prybuf_tmp != NULL)
					{ // Here, this relay is already on the request list
						// Either it is currently doing a pull-in or
						// it will be come up as the buffer is processed.
						ry_setpwm(pwmtim,pssb->idx); // Set pwm
					}
					else
					{ // Here, not on the delay buffer 
						// Add this relay to the delay buffer.
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // RED ON
osDelay(10);
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET); // RED OFF							
						rybuf_add(pwmtim, pssb->idx); // Add to request buffer
						if (delay_flag == 0)
						{ // Here, TIM9 is not currently timing a delay.
							rydbuf_startdelay(pssb->idx);
debugry3 += 1;							
						}
					}
				}
				else
				{ // Here, relay is already ON, so update pwm
//morse_trap(666);				
					ry_setpwm(pwmtim,pssb->idx); // Set pwm
				}
			}
		}
	}
}
/* *************************************************************************
 * TaskHandle_t xRyTaskCreate(uint32_t taskpriority);
 * @brief	: Create task; task handle created is global for all to enjoy!
 * @param	: taskpriority = Task priority (just as it says!)
 * @return	: RyTaskHandle
 * *************************************************************************/
TaskHandle_t xRyTaskCreate(uint32_t taskpriority)
{
	BaseType_t ret = xTaskCreate(StartRyTask, "RyTask",\
     (128+64), NULL, taskpriority,\
     &RyTaskHandle);
	if (ret != pdPASS) return NULL;

	RyTaskReadReqQHandle = xQueueCreate(QUEUESIZE, sizeof(struct RyREQ_Q*) );
	if (RyTaskReadReqQHandle == NULL) return NULL;

	return RyTaskHandle;
}
/* #############################################################################
 * void TIM5_IRQ_Handler(void);
 * KP (keep-alive) timer (0.1ms)
 * ############################################################################# */
void TIM5_IRQ_Handler(void)
{
//morse_trap(456);
debugry1 += 1;

	TIM5->SR = ~0x2; // Clear flag CH1

	/* Next interrupt time (10 ms between OC interrupts) */
	TIM5->CCR1 += KPUPDATEDUR;

	/* Check one relay each OC interrupt. */
	idx_kp += 1;
	if (idx_kp >= NRELAYS) idx_kp = 0;

	/* Countdown the keep-alive time. */
	if (relaywv[idx_kp].kp_wv != 0)
	{ // Here, keep-alive countdown in progress
		relaywv[idx_kp].kp_wv -= 1;
		if (relaywv[idx_kp].kp_wv == 0)
		{ // Here, countdown ended
			relaywv[idx_kp].on = 0; // Set status to OFF
			ry_setpwm(0,idx_kp);    // Set 0% duty pwm
			rybuf_cancel(idx_kp);   // Clear any pending updates
			if ((delay_flag != 0) &&
				(idx_kp == idx_delay))
			{ // Here pull-in delay in progress for this relay
				idx_abort = 1; //
			}
		}
	}
	else
	{ // Here, keep-alive count at zero 
		if (relaywv[idx_kp].on != 0)
		{ // Here, ka expired, but relay shows ON
			relaywv[idx_kp].on = 0; // Set status to OFF jic
			ry_setpwm(0,idx_kp);    // Set 0% duty pwm jic
			rybuf_cancel(idx_kp);   // Clear any pending updates jic
			if ((delay_flag != 0) && (idx_kp == idx_delay))
			{ // Here pull-in delay in progress for this relay
				idx_abort = 1; //
			}
		}
	}
	return;
}
/* #############################################################################
 * void TIM9_IRQ_Handler(void);
 * @brief	: TIM9--pull-in delay timer (prescale:0.1 ms ticks)
 * ############################################################################# */
uint32_t debugry8;
void TIM9_IRQ_Handler(void)
{
	struct RELAYDBUF* ptmp;	
	TIM9->SR = ~0x1; // Clear UG interrupt flag

	delay_flag = 0; // Delay timing is not active	

if (prybuf_take == prybuf_add) morse_trap(715);

	/* Skip updating pwm and status if this delay aborted. */
	if (idx_abort == 0)
	{ // Here, no request to abort this current pull-in.
		/* Switch pwm from 100% for pullin to holding pwm */
		if (prybuf_take->pwm == 255) // Parameter versus request pwm
			ry_setpwm(emclc.relay[idx_delay].pwm,idx_delay); // use parameter
		else
			ry_setpwm(prybuf_take->pwm,idx_delay); // Use request value

		relaywv[idx_delay].on = 1; // Ry status: ON
	}
	idx_abort = 0;

	/* More relays waiting for pull-in delay? */
	ptmp = rybuf_next();

	if (prybuf_take != prybuf_add)
	{ // Here, buffer holds a not-cancelled entry
		rydbuf_startdelay(ptmp->idx); // Load ry & start TIM9		
debugry4 += 1;		
		return; 
	}
debugryx = 0;	
	return;
}
/* #############################################################################
 * void TIM13_IRQ_Handler(void);
 * @brief	: TIM13-- (originally for pull-in delay, but doesn't do one shot mode)
 * ############################################################################# */

void TIM13_IRQ_Handler(void)
{
	TIM13->SR = ~0x3; // 
	return;
}


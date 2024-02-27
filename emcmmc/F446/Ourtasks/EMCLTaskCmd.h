/******************************************************************************
* File Name          : EMCLTaskCmd.h
* Date First Issued  : 01/31/2024
* Description        : EMCLTask: Command codes for CAN msgs 
*******************************************************************************/

#ifndef __EMCLTASKCMD
#define __EMCLTASKCMD

// Numbering chosen to not conflict with loader commands
#define EMCL_COOLING_STATUS1    36 // GET: Alert status & temperature report
#define EMCL_MOTORPWM_SETPWMX   37 // SET: PWM PCT for all 4: pump, blower, DMOC, JIC
#define EMCL_MOTORPWM_GETPWMX   38 // GET: PWM PCT for all 4: pump, blower, DMOC, JIC


/* Payload layout
[0] EMCL_COOLING_STATUS1; // Code for payload
[1] Alert status: 0 = NO ALERTS; otherwise bits set--
    7: 
    6:
    5: DMOC motor greater than over-temperature parameter
    4: Greater than over-temperature parameter: pump outlet
    3: Greater than over-temperature parameter: winch motor
    2: Greater than over-temperature parameter: heat exchanger
    1: Greater than over-temperature parameter: ambient
    0: Greater than over-temperature parameter: jic
    // Readings: deg C; 255 = not installed; less than 0 deg C = 0;
[2] adc1.abs[p->tx_pmpo]: pump outlet
[3] adc1.abs[p->tx_moto]: winch motor
[4] adc1.abs[p->tx_hexo]: heat exchanger
[5] adc1.abs[p->tx_amb ]: ambient
[6] adc1.abs[p->tx_jic ]: JIC
[7] Reserve for DMOC report of motor temperature 
*/

#endif
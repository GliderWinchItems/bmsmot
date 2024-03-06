/******************************************************************************
* File Name          : EMCLTaskCmd.h
* Date First Issued  : 01/31/2024
* Description        : EMCLTask: Command codes for CAN msgs 
*******************************************************************************/

#ifndef __EMCLTASKCMD
#define __EMCLTASKCMD

// Numbering chosen to not conflict with loader commands
#define EMCL_COOLING_STATUS1    36 // GET: Alert status & temperature report
#define EMCL_MOTOR_RY_SET       37 // SET: Relays and PWM PCT for motors
#define EMCL_MOTOR_RY_STATUS2   38 // GET: Relay status groups OA, OB, and PWM PCT for OC motors
#define EMCL_RY_HOLD_PWM_SET    39 // SET: Special relay hold pwm (after pullin testing)



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

/* Payload layout: EMCL_MOTOR_RY_STATUS2   38 // GET: Relay status groups OA, OB, and PWM PCT for OC motors
[0] EMCL_MOTOR_RY_STATUS2; // Get Relay & motor settings
[1] 
[2]
[3] Relay on/off bits: 1 = ON
    7: OCB4
    6: OCB3
    5: OCB2
    4: OCB1
    3: OCA4
    2: OCA3
    1: OCA2
    0: OCA1
[4] OC1 motor percent
[5] OC2 motor percent
[6] OC3 motor percent
[7] OC4 motor percent
*/
/* Payload layout: EMCL_MOTOR_RY_SET       37 // SET: Relays and PWM PCT for motors
[0] EMCL_MOTOR_RY_SET; // Set relays & motors
[1] Motors to be changed bits: 0 = no change
    7: 1 = revert to automatic control (following payload gets ignored)
    6: reserved
    5: reserved
    4: reserved
    3: 1 = OC4 motor percent change to payload [7] relay array [11]
    2: 1 = OC3 motor percent change to payload [6] relay array [10]
    1: 1 = OC2 motor percent change to payload [5] relay array [ 9]
    0: 1 = OC1 motor percent change to payload [4] relay array [ 8]
[2] Relays to be set: bits 0 = no change
    7: OCB4
    6: OCB3
    5: OCB2
    4: OCB1
    3: OCA4
    2: OCA3
    1: OCA2
    0: OCA1
[3] Relay on/off bits: 1 = ON
    7: OCB4
    6: OCB3
    5: OCB2
    4: OCB1
    3: OCA4
    2: OCA3
    1: OCA2
    0: OCA1
[4] OC1 motor percent
[5] OC2 motor percent
[6] OC3 motor percent
[7] OC4 motor percent
*/

/* Payload layout: EMCL_RY_HOLD_PWM_SET    39 // SET: Special relay hold pwm (after pullin testing)
[0] EMCL_RY_HOLD_PWM_SET; // Set relays & motors
[1] Relay index (0 - 11)
[2] holding pwm (0 - 100)
[3]-[7] reserved
*/


#endif
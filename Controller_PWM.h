/********************************************/
/*                                          */
/* Title: Controller_PWM                    */
/* Programmer: Anass Dahany                 */
/* Brief Description: Defines the           */
/* functions for all PWM activity of the    */
/* BLDC controller                          */
/*                                          */
/********************************************/

#ifndef CONTROLLER_PWM_H
#define	CONTROLLER_PWM_H

/**/

void fault_blink(int frequency){
    /* starts the LED blinking at a particular frequency */




}



void pwm_l(float duty_cycle){
    /* applies a PWM waveform with a specified duty cycle to the throttle
     *  intercept MOSFET on the LEFT side*/




}

void pwm_r(float duty_cycle){
    /* applies a PWM waveform with a specified duty cycle to the throttle
     *  intercept MOSFET on the RIGHT side*/





}

void pwm_cc_r(float duty_cycle){
    /* applies a PWM waveform with the specified duty cycle to the cruise
     *  control MOSFET on the RIGHT side*/





}

void pwm_cc_l(float duty_cycle){
    /* applies a PWM waveform with the specified duty cycle to the cruise
     *  control MOSFET on the LEFT side*/





}










#endif	


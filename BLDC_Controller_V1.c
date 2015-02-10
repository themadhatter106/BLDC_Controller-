
/********************************************/
/*                                          */
/* Title: BLDC Controller                   */
/* Programmer: Chris Leger                  */
/* Brief Description: Software Cooperates   */
/* With the MC33035 BLDC controller chip to */
/* add additional functionality to the motor*/
/* controller                               */
/********************************************/

/*********************************************************************/
/*IN DEPTH SOFTWARE FUNCTIONAL DISCRIPTION:                          */
/* @1. Low Voltage Cutoff                                             */
/* @2. Cruise Control                                                 */
/* @3. Economy Mode                                                   */
/* @4. Motor Stall Protection                                         */
/* @5. Fault Indication (LED)                                         */
/* @6. Motor Torque Modulation for Turns                              */
/* @7. Soft Reverse                                                   */
/*********************************************************************/


// PIC16F1788 Configuration Bit Settings

// 'C' source line config statements

#include <xc.h>

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

// CONFIG1
#pragma config FOSC = ECH       // Oscillator Selection (ECH, External Clock, High Power Mode (4-32 MHz): device clock supplied to CLKIN pin)
#pragma config WDTE = ON        // Watchdog Timer Enable (WDT enabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = ON        // Internal/External Switchover (Internal/External Switchover mode is enabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config VCAPEN = OFF     // Voltage Regulator Capacitor Enable bit (Vcap functionality is disabled on RA6.)
#pragma config PLLEN = ON       // PLL Enable (4x PLL enabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LPBOR = OFF      // Low Power Brown-Out Reset Enable Bit (Low power brown-out is disabled)
#pragma config LVP = ON         // Low-Voltage Programming Enable (Low-voltage programming enabled)

/*Microchip Libraries*/
#include <stdio.h>
#include <stdlib.h>
#include <pic16f1788.h>
#include <xc.h>

/* Custom Library files written by Anass*/
#include "Controller_PWM.h"
#include "Controller_Measure.h"

/*Set Low Voltage Cutoff*/
#define BATT_MIN 30




main(){


/*NOTE: port directions need to be figured out and changed in initialize*/
/*all initailizations need to be confirmed really as they are all old*/
initialize();

/*value of steering sensor*/
float steering_sensor;

/*pwm ratio for each side*/
float left_pwm = 1;
float right_pwm = 1;


int reverse_counter;
int i;
int batt_fault;
int motor_fault;

/*indicates steering PWM change*/
int left,right,straight = 0;

/*cruise control status*/
int cruise_control;
/*cruise control voltage*/
float cruise_control_v;
/*record current reverse switch state*/
int reverse_switch = PORTAbits.RA1;

/*counts presses and releases of button*/
int button_counter = 0;

while(1){

    //////////////////////////////////////////////////
    /*             Low Voltage Cutoff               */
    //////////////////////////////////////////////////



    if(measure_batt() <= BATT_MIN){

        /*Indicate battery fault*/
        batt_fault = 1;

    }else{

       /*Clear battery fault*/
        batt_fault = 0;
    }




 /////////////////////////////////////////////////////
 /*               Motor Stall Protection            */
 /////////////////////////////////////////////////////


    if( measure_hall() == 0 && measure_throttle() > 2 || cruise_control_v() > 2 ){

        sleep(1);

        /*wait a second and remeasure to make sure the motor is really stalled*/
        if( measure_hall() == 0 && measure_throttle() > 2 || cruise_control_v() > 2 ){

            motor_fault = 1;

        }
    }


////////////////////////////////////////////////////////
/*              Fault Control                         */
////////////////////////////////////////////////////////

    /*NOTE: Need to add fault output support!!!*/
    
    if(batt_fault == 1){

        /*in case of battery fault blink LED at 2 Hz*/
        fault_blink(2);
        /*Turn output Enable off*/
        PORTAbits.RA0 = 1;
    }

    if(motor_fault == 1){

        /*in case of motor stall fault blink LED at 1 Hz*/
        fault_blink(1);
        /*Turn output Enable off*/
        PORTAbits.RA0 = 1;

    }

    

   if(motor_fault == 0 && batt_fault == 0){

       /*in case of no fault turn off LED*/
        fault_blink(0);
        /* Turn Output Enable on*/
        PORTAbits.RA0 = 0;


    }




    
    /////////////////////////////////////////////////////
    /* Motor Torque Modulation for Turns & Economy Mode*/
    /////////////////////////////////////////////////////


    /*read in and store value of steering sensor*/

    steering_sensor = measure_steering(void);

    /*Determine the PWM for the intercept MOSFET on each motor*/
    /*12 bit ADC -> 8191 as maximum value*/
    /* Right now the speed scaling is a constant value for simplicity
       can be upgraded later                                        */
    
    /*when turning to the left slow down the left*/
    if(steering_sensor <= 8191*(0.3) && left == 0){
        /*scale the left side*/
        left_pwm=left_pwm*0.75;
        /*return right side to full power*/
        if(cruise_control == 1){
            right_pwm=(cruise_control_v/6.5);
        }else{
             right_pwm=1;
        }

        /*If the economy mode switch is turned on scale throttle by %75*/
    if(PORTAbits.RA3 == 1){

        left_pwm=left_pwm*0.75;
        right_pwm=right_pwm*0.75;

    }


        /*Change the PWM values of the throttle intercept mosfets or cruise
         control mosfets*/
        if( cruise_control == 0){
            
        pwm_l(left_pwm);
        pwm_r(right_pwm);
            
        }else{
            
        pwm_cc_l(left_pwm);
        pwm__cc_r(right_pwm);
            
        }


        /*set PWM status*/
        left = 1;
        right = 0;
        straight = 0;

    }

    /*When turning to the right slow down the right */
    if(steering_sensor >= 8191*(0.7) && right == 0){

       
        right_pwm=right_pwm*0.75;
         /*return left side to full power*/
        if(cruise_control == 1){
            left_pwm=(cruise_control_v/6.5);
        }else{
            left_pwm=1;
        }

        /*If the economy mode switch is turned on scale throttle by %75*/
         if(PORTAbits.RA3 == 1){

        left_pwm=left_pwm*0.75;
        right_pwm=right_pwm*0.75;

         }

       /*Change the PWM values of the throttle intercept mosfets or cruise
         control mosfets*/
        if( cruise_control == 0){
            
        pwm_l(left_pwm);
        pwm_r(right_pwm);
            
        }else{
            
        pwm_cc_l(left_pwm);
        pwm__cc_r(right_pwm);
            
        }

        /*set PWM status*/
        left = 0;
        right = 1;
        straight = 0;

    }

    /*when going straight make no change to the throttle signal*/
    if(steering_sensor >= 8191*(7/10) && steering_sensor <= 8191*(3/10)
            && straight == 0){

         /*return left & right side to full power*/
        if(cruise_control == 1){
            right_pwm=(cruise_control_v/6.5);
            left_pwm=(cruise_control_v/6.5);

        }else{
             right_pwm=1;
              left_pwm=1;
        }

        /*If the economy mode switch is turned on scale throttle by %75*/
        if(PORTAbits.RA3 == 1){

        left_pwm=left_pwm*0.75;
        right_pwm=right_pwm*0.75;

        }

       /*Change the PWM values of the throttle intercept mosfets or cruise
         control mosfets*/
        if( cruise_control == 0){
            
        pwm_l(left_pwm);
        pwm_r(right_pwm);
            
        }else{
            
        pwm_cc_l(left_pwm);
        pwm__cc_r(right_pwm);
            
        }

        /*set PWM status*/
        left = 0;
        right = 0;
        straight = 1;

    }




   ///////////////////////////////////////////////////////
   /*                   Soft Reverse                    */
   ///////////////////////////////////////////////////////

   /*Performs a soft reverse if the vehicle is moving and performs
    a hard reverse if the vehicle is stationary*/

    /*look for a state change in the reverse switch to activte soft reverse*/

    if( reverse_switch != PORTAbits.RA1 ){

        /*save change in reverse switch state*/
        reverse_switch = PORTAbits.RA1;

        /*if the motor is moving do the soft reverse routine*/
        if( measure_hall() > 0 ){

            /*ramp down throttle*/
            for(i=100; i>0; i--){

                pwm_l(i/100);
                pwm_r(i/100);
                sleep(0.01);

            }

            /*Change State of FWD/REV output*/
            PORTCbits.RC6 = ~PORTCbits.RC6;

             /*ramp up throttle*/
            for(i=0; i<100; i++){

                pwm_l(i/100);
                pwm_r(i/100);
                sleep(0.01);

            }
            /*if the motor is stopped perform a hard reverse*/
        }else{

             /*Change State of FWD/REV output*/
            PORTCbits.RC6 = ~PORTCbits.RC6;

        }

    }

   //////////////////////////////////////////////////////////
   /*                   Cruise Control                     */
   //////////////////////////////////////////////////////////

    /*NOTE: This is the simpleist version of the cruise control algorithm which
     maintains the set throttle voltage, it does not maintain the wheel
     speed by using a feedback loop, although this could be upgraded*/

    /*cruise control momentary switch press*/
    if( PORTAbits.RA2 == 1 && button_counter == 0){

        button_counter++;
    }
     /*cruise control momentary switch release*/
    if( PORTAbits.RA2 == 0 && button_counter == 1){

        button_counter++;
        cruise_control = 1;

        /*store current throttle voltage*/
        cruise_control_v = measure_throttle();


        /*Disable the throttle intercept mosfets*/
        pwm_l(0);
        pwm_r(0);

        /*PWM cruise control mosfets to maintain throttle voltage*/
        pwm_cc_r(right_pwm);
        pwm_cc_l(left_pwm);



    }

/*cruise control momentary switch press*/
    if( PORTAbits.RA2 == 1 && button_counter == 2){

        button_counter++;

    }

    /*cruise control momentary switch release*/
    /*turn off cruise control*/
 if( PORTAbits.RA2 == 0 && button_counter == 3){

        cruise_control = 0;
        button_counter = 0;

        /*Disable cruise control MOSFETS*/
         pwm_cc_r(0);
         pwm_cc_l(0);


        /*enable intercept mosfets*/
         pwm_l(1);
        pwm_r(1);

    }



}


}


void initialize(void)
{



    OSCCON = 0b01110101;
    //Configure Oscillator
                                    //bit 7,unimplemented
                                    //Internal Osc. Freq. 111, 8 MHz
                                    //bit 3, OSC is running from internal osc
                                    //bit 2,High frequency is stable
                                    //bit 1, low frequency is unstable
                                    //bit 0, internal osc is defined by system clock

    //I/O Configuration

    PORTA = 0b00000000;

    PORTC = 0b00000000;

    ANSEL = 0b00110000;

    CMCON0 = 0b00000111;

    TRISA = 0b00000000;           //All portA pins configured as outputs


    TRISC= 0b00100011;



    //ADC configuration
    ADCON0 = 0b10010001;
    //bit 7,right justified
    //bit 6, vref = VDD
    //bit 5, unimplemented
    //bits 4-2, Analog Channel Select AN1(RA1)
    //bit 1, Conversion status bit(GO/nDONE) cleared
    //bit 0,ADC Enable bit set.

    ADCON1 = 0b01110000;
    //bit 7, unimplemented
    //bit 6-4, x11, clock is system clock(500 kHZ)
    //bits 3-0, unimplemented




}




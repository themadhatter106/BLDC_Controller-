
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
/* 1. Low Voltage Cutoff                                             */
/* 2. Cruise Control                                                 */
/* 3. Economy Mode                                                   */
/* 4. Motor Stall Protection                                         */
/* 5. Fault Indication (LED)                                         */
/* 6. Motor Torque Modulation for Turns                              */
/* 7. Soft Reverse                                                   */
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

#include <stdio.h>
#include <stdlib.h>
#include <pic16f1824.h>
#include <xc.h>








/* Everything below is from the old code*/
/* needs extensive modification*/


void initialize(void);
void pwm_initalize(void);
int read_sensor(void);
int read_throttle(void);


main(){


    /* code for the left side*/

initialize();
pwm_initalize();
int throttle;
int sensor;
int throttle_scaled;
float temp;

        while(1){

            /* Get the ADC values of the throttle and sensor*/
            throttle = read_throttle();
            sensor = read_sensor();

            /* scale throttle corresponding to the value of PR2 register,
             needed for changing the duty cycle*/
            temp=throttle *0.3978;
            throttle_scaled=temp;

            if(PIR1bits.TMR2IF == 1){
                TRISCbits.TRISC5 = 0;
            }



/*1023(3/10)=307*/
            if(sensor>307){

/* If making a right turn or going straight then don't modulate throttle signal*/


                /* settung registers for duty cycle */
                CCP1CONbits.DC1B0 = (throttle_scaled & 0x01);

                CCP1CONbits.DC1B1 = ((throttle_scaled) & 0x02)>>1;

                CCPR1L = (throttle_scaled)>>2;
            }else{




                /*making a Left turn, this side must slow down*/
                temp=throttle_scaled*0.75;
                throttle_scaled=temp;

                /* settung registers for duty cycle */
                CCP1CONbits.DC1B0 = (throttle_scaled & 0x01);

                CCP1CONbits.DC1B1 = ((throttle_scaled) & 0x02)>>1;

                CCPR1L = (throttle_scaled)>>2;



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


void pwm_initalize(void){

    PR2 = 0x65;

    CCP1CON = 0b00111100;
    //Configure PWM mode and...
    //bits 4-5 are the two LSBs of PWM duty cycle

    CCPR1L=00000111;
    //First 6 LSBs are the 6 MSBs of PWM duty cycle

    PIR1bits.TMR2IF=0;
    //Clear the timer 2 interrupt flag

    T2CON = 0b00000100;
    //Enable timer and set prescaler to 1

}

int read_sensor(void){
     ADCON0 = 0b10010001;


     int ADC_VAL;

     ADCON0bits.GO = 1;          //start conversion cycle

while(ADCON0bits.GO != 0);
ADC_VAL = ((ADRESH << 8)| ADRESL);

return ADC_VAL;

}


int read_throttle(void){
     ADCON0 = 0b10010101;

     int ADC_VAL;

     ADCON0bits.GO = 1;          //start conversion cycle

while(ADCON0bits.GO != 0);
ADC_VAL = ((ADRESH << 8)| ADRESL);

return ADC_VAL;
}
/*Programmer: Chris Leger */
/*Title: Right Motor Steering Control*/

/*code from the steering control box on the blue quad*/

#include <stdio.h>
#include <stdlib.h>
#include <pic16f684.h>
#include <xc.h>


__CONFIG (INTIO & WDTDIS & PWRTEN & MCLRDIS & UNPROTECT & UNPROTECT & BORDIS
        & IESODIS & FCMDIS);

void initialize(void);
void pwm_initalize(void);
int read_sensor(void);
int read_throttle(void);


main(){

     /* Master controller, sets status LEDs*/
    
/*setup oscilator, ADC, and I/O ports*/
initialize();
/*Setup registers for PWM*/
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
            /*truncate the decimal value*/
            throttle_scaled=temp;
            
            if(PIR1bits.TMR2IF == 1){
                TRISCbits.TRISC5 = 0;
            }



/* if voltage of sensor is between 0V and 3.2V*/
            
            if(sensor<614){

/* If making a left turn or going straight then
   don't modulate throttle signal*/

                if(sensor>409){
                /*Straight LED on*/
                PORTCbits.RC2 = 0;
                PORTCbits.RC3 = 1;
                PORTCbits.RC4 = 0;
                }else{
                /*Left turn LED on*/
                PORTCbits.RC2 = 0;
                PORTCbits.RC3 = 0;
                PORTCbits.RC4 = 1;
                }

                /* settung registers for duty cycle */
                CCP1CONbits.DC1B0 = (throttle_scaled & 0x01);

                CCP1CONbits.DC1B1 = ((throttle_scaled) & 0x02)>>1;

                CCPR1L = (throttle_scaled)>>2;
            }else{


                /*Right Turn LEDS*/
                PORTCbits.RC2 = 1;
                PORTCbits.RC3 = 0;
                PORTCbits.RC4 = 0;

                /*making a right turn, this side must slow down*/
                temp=throttle_scaled*0.5;
                throttle_scaled=temp;

                /* settung registers for duty cycle */
                CCP1CONbits.DC1B0 = (throttle_scaled & 0x01);

                CCP1CONbits.DC1B1 = (throttle_scaled & 0x02)>>1;

                CCPR1L = throttle_scaled>>2;

                
                
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
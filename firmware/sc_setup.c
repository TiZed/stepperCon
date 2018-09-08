/* 
 * File:   sc_setup.c
 * Author: TiZed
 *
 * Created on 12 August 2016
 * 
 *  Copyright (C) 2016 TiZed
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * ----------------------------------------------------------------------------
 * 
 */

#include <pic18fregs.h>
#include <stdint.h>

#define TMR_10MS  45535

void ioSetup(void) {
    TRISBbits.RB4 = 0 ;     // Start with a cleared 'fault' signal
    PORTBbits.RB4 = 0 ;
    
    ANSELDbits.ANSD2 = 0 ;
    TRISDbits.RD2 = 0 ;     // Start with blue LED off
    PORTDbits.RD2 = 0 ;
    
    ANSELC = 0x00 ;         // Disable Port-C as analog input
   
    TRISCbits.RC5 = 0 ;     // Start with red LED off
    PORTCbits.RC5 = 0 ;
    
    TRISAbits.RA4 = 0 ;
    LATAbits.LATA4 = 0 ;
    
    ANSELAbits.ANSA5 = 0 ;
    TRISAbits.RA5 = 0 ;
    LATAbits.LATA5 = 0 ;
    
    ANSELB = 0x00 ;         // Disable port B analog input
    
    ANSELDbits.ANSD5 = 0 ;  
   
    TRISBbits.RB0 = 1 ;     // 'step', 'dir' and 'enable' as inputs
    TRISBbits.RB2 = 1 ;
    TRISBbits.RB3 = 1 ;
}

void pwmSetup(uint8_t db_time) {
    // ECCP2 - Phase A PWM 
    // ECCP1 - Phase B PWM
    
    TRISCbits.TRISC0 = 1 ;          // Disable port drivers for setup
    TRISCbits.TRISC1 = 1 ;
    TRISCbits.TRISC2 = 1 ;
    TRISDbits.TRISD5 = 1 ;
    TRISDbits.TRISD1 = 1 ;
    TRISEbits.TRISE2 = 1 ;
    
    CCPTMRS0bits.C2TSEL = 0b10 ;    // Timer6 to ECCP2
    CCPTMRS0bits.C1TSEL = 0b10 ;    // Timer6 to ECCP1  
    CCPTMRS1bits.C5TSEL = 0b01 ;    // Timer4 to CCP5
    CCPTMRS1bits.C4TSEL = 0b01 ;    // Timer4 to CCP4
    
    T6CONbits.TMR6ON = 0 ;          // Turn Timer6 off
    PR6 = 0xff ;                    // Set Timer6 for 15.625kHz @64MHz, 10-bit res
    T6CONbits.T6CKPS = 0b01 ;       // Set prescaler to 1:4
    
    T4CONbits.TMR4ON = 0 ;          // Turn Timer4 off
    PR4 = 0xff ;                    // Set Timer4 for 62.5kHz @64MHz, 10-bit res
    T4CONbits.T4CKPS = 0b00 ;       // Set prescaler to 1:1
    
    ECCP2ASbits.PSS2AC = 0b00  ;    // Pin A1 set to '0' at shutdown
    ECCP2ASbits.PSS2BD = 0b00  ;    // Pin A2 set to '0' at shutdown
    ECCP1ASbits.PSS1AC = 0b00  ;    // Pin B1 set to '0' at shutdown
    ECCP1ASbits.PSS1BD = 0b00  ;    // Pin B2 set to '0' at shutdown
    
    ECCP2ASbits.CCP2AS = 0x00  ;    // Disable auto-shutdown
    ECCP1ASbits.CCP1AS = 0x00  ;    
    
    ECCP2ASbits.CCP2ASE = 1    ;    // Force shutdown
    ECCP1ASbits.CCP1ASE = 1    ;    
    
    CCP2CONbits.P2M = 0b10 ;
    CCP2CONbits.CCP2M = 0b1111 ;    // Phase A Half-bridge mode
    CCP1CONbits.P1M = 0b10 ;
    CCP1CONbits.CCP1M = 0b1111 ;    // Phase B Half-bridge mode
    
    CCP5CONbits.CCP5M = 0b1100 ;    // A Ref PWM
    CCP4CONbits.CCP4M = 0b1100 ;    // B Ref PWM
    
    PWM2CONbits.P2DC = db_time ;    // 16 * 4 * Tosc = 1us dead-band
    PWM1CONbits.P1DC = db_time ; 
    
    TRISCbits.TRISC0 = 0 ;          // Enable port drivers
    TRISCbits.TRISC1 = 0 ;
    TRISCbits.TRISC2 = 0 ;
    TRISDbits.TRISD5 = 0 ;
    TRISDbits.TRISD1 = 0 ;
    TRISEbits.TRISE2 = 0 ;
    
    CCPR2L = 128 ;
    CCP2CONbits.DC2B = 0 ;
    
    CCPR1L = 128 ;
    CCP1CONbits.DC1B = 0 ;
            
    CCPR5L = 32 ;
    CCP5CONbits.DC5B = 0 ;
    
    CCPR4L = 32 ;
    CCP4CONbits.DC4B = 0 ;
    
    T6CONbits.TMR6ON = 1 ;          // Start PWM timer
    T4CONbits.TMR4ON = 1 ;          // Start PWM timer
    
    ECCP2ASbits.CCP2ASE = 0    ;    // Enable PWM
    ECCP1ASbits.CCP1ASE = 0    ;    
}

void pwmOut(void) {
    TRISBbits.TRISB5 = 1 ;
    
    CCPTMRS0bits.C3TSEL = 0b00 ;    // Timer2 to CCP3
    
    T2CONbits.TMR2ON = 0 ;          // Turn Timer6 off
    PR2 = 0xff ;                    // Set Timer6 for 15.625kHz @64MHz, 10-bit res
    T2CONbits.T2CKPS = 0b00 ;       // Set prescaler to 1:1
    
    CCP3CONbits.P3M = 0b00 ;
    CCP3CONbits.CCP3M = 0b1100 ; 
    
    TRISBbits.TRISB5 = 0 ;
    
    CCPR3L = 128 ;
    CCP3CONbits.DC3B = 0 ;
    
    T2CONbits.TMR2ON = 1 ; 
}

void compsSetup(void) {
    // Enable Fixed Voltage Reference (FVR)
    VREFCON0bits.FVREN = 1 ;
    while (!VREFCON0bits.FVRST) ;   // Wait for FVR to stabilize
    VREFCON0bits.FVRS = 0b01 ;      // set FVR to 1.024V
    
    // Enable DAC
    VREFCON1bits.DACNSS = 0 ;       // Set DAC neg. ref. to Vss
    VREFCON1bits.DACPSS = 0b10 ;    // Set DAC pos. ref. to FVR
    VREFCON1bits.DACOE = 0 ;        // Disable DAC output pin
    VREFCON1bits.DACEN = 1 ;        // Enable DAC
    VREFCON2bits.DACR = 7 ;         // = 224mV
    
    // Comparator 1 (Phase A) setup
    CM1CON0bits.C1CH = 0b01 ;       // C12N1- input to C1-
    CM1CON0bits.C1R = 0 ;           // C1+ to Vref input
    CM2CON1bits.C1RSEL = 0 ;        // Use DAC as Vref
    CM1CON0bits.C1POL = 1 ;         // Invert logic
    CM1CON0bits.C1SP = 1 ;          // Normal power, high speed mode
    CM2CON1bits.C1HYS = 0 ;         // Enable hysteresis
    
    // Comparator 2 (Phase B) setup
    CM2CON0bits.C2CH = 0b00 ;       // C12N0- input to C2-
    CM2CON0bits.C2R = 0 ;           // C2+ to Vref input
    CM2CON1bits.C2RSEL = 0 ;        // Use DAC as Vref
    CM2CON0bits.C2POL = 1 ;         // Invert logic
    CM2CON0bits.C2SP = 1 ;          // Normal power, high speed mode
    CM2CON1bits.C2HYS = 0 ;         // Enable hysteresis
    
    PIR2bits.C1IF = 0 ;             // Clear any pending interrupts 
    PIR2bits.C2IF = 0 ; 
    
    CM1CON0bits.C1ON = 1 ;          // Enable comparator 1
    CM2CON0bits.C2ON = 1 ;          // Enable comparator 2
}

void intSetup(void) {
    INTCONbits.GIE_GIEH = 0 ;   // Disable interrupts
    INTCONbits.PEIE_GIEL = 0 ;
    
    RCONbits.IPEN = 1 ;         // Enable interrupts priority

    INTCON2bits.RBPU = 1 ;      // Disable Port B pull-ups
    INTCON2bits.INTEDG0 = 1 ;   // 'step' interrupt on rising edge
    
    INTCON3bits.INT2IP = 0 ;    // Low priority to 'dir' interrupt
    IPR1bits.ADIP = 1 ;         // ADC High priority interrupt
    IPR1bits.SSP1IP = 0 ;       // Low priority to I2C interrupt
    
    IPR2bits.C1IP = 1 ;         // Set comparators interrupts to high priority
    IPR2bits.C2IP = 1 ;
    
    INTCONbits.GIE_GIEH = 1 ;   // Enable interrupts
    INTCONbits.PEIE_GIEL = 1 ;
}

// Setup ADC for current capture
void adc_setup(void) {
    ADCON0bits.ADON = 0 ;      // ADC Off
    
    ADCON2bits.ADCS = 0b110 ;  // ADC clock = Fosc/64 = 1us
    ADCON2bits.ACQT = 0b101 ;  // 12 Tad acquisition time
    ADCON2bits.ADFM = 1 ; 
    
    ADCON1bits.PVCFG = 0b00 ;  // Pos. Ref. voltage = AVdd
    ADCON1bits.NVCFG = 0b00 ;  // Nrg. Ref. voltage = AVss
    ADCON1bits.TRIGSEL = 1  ;  // Set GO trigger to CCP5
    
    TRISAbits.TRISA0 = 1 ;
    ANSELAbits.ANSA0 = 1 ;     // Enable RA0 as analog input
    
    TRISAbits.TRISA1 = 1 ;
    ANSELAbits.ANSA1 = 1 ;     // Enable RA1 as analog input
    
    PIR1bits.ADIF = 0 ;        // Reset ADC interrupt 
    PIE1bits.ADIE = 0 ;        // Disable ADC interrupt
    
    ADCON0bits.ADON = 1 ;      // ADC On
}

void resetCheck(void) {
    uint8_t i ;
    
    // On WDT timeout, Power down detect or brown-out
    if(!RCONbits.TO || !RCONbits.PD || !RCONbits.BOR) {
        // Reset RCON flags
        RCONbits.TO = 1 ;
        RCONbits.PD = 1 ;
        RCONbits.BOR = 0 ;
        RCONbits.POR = 0 ;

        PORTBbits.RB4 = 1 ;     // Rise 'fault' signal
        PORTCbits.RC5 = 1 ;     // Turn on red LED

        while(1) ; 
    }

    // On stack over/underflow
    if(STKPTRbits.STKFUL || STKPTRbits.STKUNF) {
        // Clear stack pointer flags
        STKPTRbits.STKFUL = 0 ;
        STKPTRbits.STKUNF = 0 ;

        PORTBbits.RB4 = 1 ;     // Rise 'fault' signal
        PORTCbits.RC5 = 1 ;     // Turn on red LED

        // Blink RED LED
        T1CONbits.TMR1CS = 0 ;      // Timer1 to Fosc/4
        i = 120 ;

        while(1) {
            T1CONbits.TMR1ON = 0 ;
            PIR1bits.TMR1IF = 0 ;
            T1CONbits.T1CKPS = 3 ;      // Timer1 1:8 prescale
            TMR1H = TMR_10MS >> 8 ;
            TMR1L = TMR_10MS ;
            T1CONbits.TMR1ON = 1 ;

            while(!PIR1bits.TMR1IF) ;

            if (--i == 0) {
                LATCbits.LATC5 = !LATCbits.LATC5 ;
                i = 120 ;
            }
        }
    }
}
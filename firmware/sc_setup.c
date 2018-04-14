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
    
    TRISBbits.RB0 = 1 ;     // 'step', 'dir' and 'enable' as inputs
    TRISBbits.RB2 = 1 ;
    TRISBbits.RB3 = 1 ;
}

void pwmSetup(void) {
    TRISCbits.TRISC0 = 1 ;          // Disable port drivers for setup
    TRISCbits.TRISC1 = 1 ;
    TRISCbits.TRISC2 = 1 ;
    TRISDbits.TRISD5 = 1 ;
    
    CCPTMRS0bits.C2TSEL = 0b00 ;    // Timer2 to ECCP2
    CCPTMRS0bits.C1TSEL = 0b00 ;    // Timer2 to ECCP1      
    PR2 = 0xff ;                    // Set timer for 62.5kHz @64MHz, 10-bit res
    T2CONbits.T2CKPS = 0x0 ;        // Set prescaler to 1:1
    T2CONbits.TMR2ON = 1 ;          // Turn timer on
    
    ECCP2ASbits.PSS2AC = 0b00  ;    // Pin A1 set to '0' at shutdown
    ECCP2ASbits.PSS2BD = 0b00  ;    // Pin A2 set to '0' at shutdown
    ECCP1ASbits.PSS1AC = 0b00  ;    // Pin B1 set to '0' at shutdown
    ECCP1ASbits.PSS1BD = 0b00  ;    // Pin B2 set to '0' at shutdown
    
    ECCP2ASbits.CCP2AS = 0x00  ;    // Disable auto-shutdown
    ECCP1ASbits.CCP1AS = 0x00  ;    
    
    ECCP2ASbits.CCP2ASE = 1    ;    // Force shutdown
    ECCP1ASbits.CCP1ASE = 1    ;    
    
    CCP2CONbits.CCP2M = 0b1100 ;    // Phase A Half-bridge mode
    CCP2CONbits.P2M = 0b10 ;
    CCP1CONbits.CCP1M = 0b1100 ;    // Phase B Half-bridge mode
    CCP1CONbits.P1M = 0b10 ;
    
    PWM2CONbits.P2DC = 16 ;             // 16 * 4 * Tosc = 1us dead-band
    PWM1CONbits.P1DC = 16 ; 
    
    TRISCbits.TRISC0 = 0 ;          // Enable port drivers
    TRISCbits.TRISC1 = 0 ;
    TRISCbits.TRISC2 = 0 ;
    TRISDbits.TRISD5 = 0 ;
    
    ECCP2ASbits.CCP2ASE = 0    ;    // Enable PWM
    ECCP1ASbits.CCP1ASE = 0    ;    
}

void intSetup(void) {
    INTCONbits.GIE_GIEH = 0 ;   // Disable interrupts
    INTCONbits.PEIE_GIEL = 0 ;
    
    RCONbits.IPEN = 1 ;         // Enable interrupts priority

    INTCON2bits.RBPU = 1 ;      // Disable Port B pull-ups
    INTCON2bits.INTEDG0 = 1 ;   // 'step' interrupt on rising edge
    
    INTCON3bits.INT2IP = 1 ;    // High priority to 'dir' interrupt
    
    IPR1bits.ADIP = 1 ;         // ADC High priority interrupt
    
    IPR1bits.SSP1IP = 0 ;       // Low priority to I2C interrupt
    
    
    INTCONbits.GIE_GIEH = 1 ;   // Enable interrupts
    INTCONbits.PEIE_GIEL = 1 ;
}

// Setup ADC for current capture
void adc_setup(void) {
    ADCON0bits.ADON = 0 ;      // ADC Off
    
    ADCON2bits.ADCS = 0b110 ;  // ADC clock = Fosc/64 = 1us
    ADCON2bits.ADFM = 1 ; 
    
    ADCON1bits.PVCFG = 0b00 ;  // Pos. Ref. voltage = AVdd
    ADCON1bits.NVCFG = 0b00 ;  // Nrg. Ref. voltage = AVss
    
    ANSELAbits.ANSA0 = 1 ;     // Enable RA0 as analog input
    TRISAbits.TRISA0 = 1 ;
    
    ANSELAbits.ANSA1 = 1 ;     // Enable RA1 as analog input
    TRISAbits.TRISA1 = 1 ;
    
    PIR1bits.ADIF = 0 ;        // Reset ADC interrupt 
    PIE1bits.ADIE = 1 ;        // Enable ADC interrupt
    
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
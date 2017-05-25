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

void compsSetup(void) {
    ANSELAbits.ANSA0 = 1 ;      // Enable RA0 as analog input
    TRISAbits.TRISA0 = 1 ;
    
    ANSELAbits.ANSA1 = 1 ;      // Enable RA1 as analog input
    TRISAbits.TRISA1 = 1 ;
    
    ANSELAbits.ANSA2 = 1 ;      // Enable RA2 as analog input
    TRISAbits.TRISA2 = 1 ;
    
    ANSELAbits.ANSA3 = 1 ;      // Enable RA3 as analog input
    TRISAbits.TRISA3 = 1 ;
    
    CM1CON0 = 0x09 ;            // C12IN1- Goes to C1, Current of Phase-A
    CM2CON0 = 0x08 ;            // C12IN0- Goes to C2, Current of Phase-B
     
    CM2CON1bits.C1HYS = 0 ;     // Enable hysteresis for both comparators
    CM2CON1bits.C2HYS = 0 ;           
}

void ioSetup(void) {
    TRISBbits.RB4 = 0 ;     // Start with a cleared 'fault' signal
    PORTBbits.RB4 = 0 ;
    
    ANSELDbits.ANSD2 = 0 ;
    TRISDbits.RD2 = 0 ;     // Start with blue LED off
    PORTDbits.RD2 = 0 ;
    
    ANSELC = 0x00 ;         // Disable Port-C as analog input
   
    TRISCbits.RC5 = 0 ;     // Start with red LED off
    PORTCbits.RC5 = 0 ;
    
    TRISCbits.RC0 = 0 ;     // Phase-A outputs
    TRISCbits.RC1 = 0 ;
 //   ANSELAbits.ANSA4 = 0 ;
    TRISAbits.RA4 = 0 ;
    
    LATCbits.LATC0 = 0 ;
    LATCbits.LATC1 = 0 ;
    LATAbits.LATA4 = 0 ;
    
    TRISCbits.RC2 = 0 ;     // Phase-B outputs
    ANSELDbits.ANSD5 = 0 ;
    TRISDbits.RD5 = 0 ;
    ANSELAbits.ANSA5 = 0 ;
    TRISAbits.RA5 = 0 ;
    
    LATCbits.LATC2 = 0 ;
    LATDbits.LATD5 = 0 ; 
    LATAbits.LATA5 = 0 ;
    
    ANSELB = 0x00 ;         // Disable port B analog input
    
    TRISBbits.RB0 = 1 ;     // 'step', 'dir' and 'enable' as inputs
    TRISBbits.RB2 = 1 ;
    TRISBbits.RB3 = 1 ;
}

void pwmSetup(void) {
    ANSELEbits.ANSE2 = 0 ;
    TRISEbits.TRISE2 = 1 ;          // Disable driver for setup
    ANSELDbits.ANSD1 = 0 ;
    TRISDbits.TRISD1 = 1 ;
    
    CCPTMRS1 = 0x00 ;               // Use Timer2 for references
    PR2 = 0xff ;                    // Set timer for 62.5kHz @64MHz, 10-bit res
    T2CONbits.T2CKPS = 0x0 ;        // Set prescaler to 1:1
    T2CONbits.TMR2ON = 1 ;          // Turn timer on
    
    CCP5CONbits.CCP5M = 0x0c ;      // Enable Phase-A current reference
    CCP4CONbits.CCP4M = 0x0c ;      // Enable Phase-B current reference
    
    TRISEbits.TRISE2 = 0 ;          // Enable driver
    TRISDbits.TRISD1 = 0 ;
}

void intSetup(void) {
    INTCONbits.GIE_GIEH = 0 ;   // Disable interrupts
    INTCONbits.PEIE_GIEL = 0 ;
    
    RCONbits.IPEN = 1 ;         // Enable interrupts priority

    INTCON2bits.RBPU = 1 ;      // Disable Port B pull-ups
    INTCON2bits.INTEDG0 = 1 ;   // 'step' interrupt on rising edge
    
    INTCON3bits.INT2IP = 1 ;    // High priority to 'dir' interrupt
    
    IPR2bits.C1IP = 1 ;         // High priority to Phase-A comparator
    IPR2bits.C2IP = 1 ;         // High priority to Phase-B comparator
    
    IPR1bits.CCP1IP = 1 ;       // Hight priority to Phase-A compare timer
    IPR2bits.CCP2IP = 1 ;       // Hight priority to Phase-B compare timer
    
    IPR1bits.SSP1IP = 0 ;       // Low priority to I2C interrupt
    
    INTCONbits.GIE_GIEH = 1 ;   // Enable interrupts
    INTCONbits.PEIE_GIEL = 1 ;
}

void phTimersSetup(void) {
    // Prepare Timers 1 & 3 with 1:8 prescaler 
    // For Toff counters
    T1CONbits.TMR1CS = 0 ;
    T1CONbits.T1CKPS = 3 ;
    T1CONbits.TMR1ON = 0 ;
    TMR1L = 0 ;
    TMR1H = 0 ;
    
    T3CONbits.TMR3CS = 0 ;
    T3CONbits.T3CKPS = 3 ;
    T3CONbits.TMR3ON = 0 ;
    TMR3L = 0 ;
    TMR3H = 0 ;
    
    CCPTMRS0bits.C1TSEL = 0 ;   // Timer1 -> CCP1
    CCPTMRS0bits.C2TSEL = 1 ;   // Timer3 -> CCP2
    
    // CCP1 & 2 set to software interrupt mode
    CCP1CONbits.CCP1M = 0xa ;
    CCP2CONbits.CCP2M = 0xa ;
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
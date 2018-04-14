/* 
 * File:   stepperCon.c
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
 * 
 * * Phase A:
 * -------
 * RA1 - C12IN1- - Current sense in
 * RC1 - P2A - A1 Output
 * RC0 - P2B - A2 Output
 * RE2 - CCP5 - Analog out, connects to phase enable or AN3
 * RA3 - C1IN+ - Get reference voltage from PWM
 * RA4 - C1OUT - Phase enable signal from comparator
 * 
 * Phase B:
 * -------
 * RA0 - C12IN0- - Current sense in
 * RC2 - P1A - B1 Output
 * RD5 - P1B - B2 Output
 * RD1 - CCP4 - Analog out, connects to phase enable or AN3
 * RA2 - C2IN+ - Get reference voltage from PWM
 * RA5 - C2OUT - Phase enable signal from comparator
 * 
 * Controller I/F:
 * --------------
 * RB2 - INT2 - Direction Input
 * RB0 - INT0 - Step Input
 * RB3 - Enable Input
 * RB4 - Fault output
 * 
 * RC4 - SDA1 - I2C Data
 * RC3 - SCL1 - Clock
 * 
 * RD2 - Blue LED
 * RC5 - Red LED
 * 
 */

#include <pic18fregs.h>
#include <stdint.h>
#include <float.h>
#include <math.h>

#include "registers.h"


// 64MHz clock 
#pragma config  FOSC    = HSHP      // HS ocsillator, high power
#pragma config  PLLCFG  = ON        // x4 PLL on
#pragma config  WDTEN   = SWON      // WDT software controlled
#pragma config  XINST   = OFF       // For SDCC
#pragma config  CCP3MX  = PORTE0    // CCP3 on port E0   
#pragma config  P2BMX   = PORTC0    // P2B on port C0
#pragma config  MCLRE   = EXTMCLR   // Enable MCLR pin (E3 disabled)
// #pragma config  LVP     = ON        // Low voltage programming enabled

#define STEPS 16
#define FULL_CYCLE (4 * STEPS)
#define HALF_CYCLE (2 * STEPS)
#define THREE_QUARTERS (3 * STEPS)

// Some default values
#define CUR_GAIN 90         // Current sensor gain

#define I2C_ADDRESS 0x40    // X- 0x40, Y- 0x42, Z- 0x44 
#define I2C_REGISTERS 9


#define MAX_AMP 3704        // [mA] Maximum driver amperage
#define SET_AMP 1500        // [mA] Default driver limit
#define T_OFF   20          // [usec.] fixed phase time off
#define T_BLANK_HIGH 1      // [usec.] blank time for forward current
#define T_BLANK_LOW  10     // [usec.] blank time for revers current

// Stepping modes
#define STEP_16  1
#define STEP_8   2
#define STEP_4   4
#define STEP_2   8
#define STEP_1  16

// Decay modes
#define SLOW_DECAY 0
#define FAST_DECAY 1

// System states
#define IDLE      0
#define START     1
#define RUNNING   2
#define HALT      3
#define STEP      4
#define NEXT_STEP 6

// Phase states
#define PH_BLANK    0
#define PH_OFF      1
#define T_DRIVE     2

// ADC Capture states
#define CAPTURE_A   0
#define CAPTURE_B   1

#define TMR_1MS   63536     // @1:8 Prescaller, Fosc/4
#define TMR_10MS  45536     // @1:8 Prescaller, Fosc/4
#define TMR_500NS 1         // @1:8 Prescaller, Fosc/4

#define PWM_MAX   1023

// Default EEPROM data
typedef unsigned char eeprom ;
__code eeprom __at 0xf00000 __EEPROM[] = { I2C_ADDRESS, STEP_8, SET_AMP, 
                                           SET_AMP >> 8, MAX_AMP, 
                                           MAX_AMP >> 8, T_OFF,
                                           T_BLANK_LOW, T_BLANK_HIGH};

// PWM micro-stepping base table
static const __data int16_t pwm_base[] = {  
        0,   50,  100,  149,  196,  241,  284,  325,    //  0-7 
      362,  396,  426,  452,  473,  490,  502,  510,    //  8-15
      512,  510,  502,  490,  473,  452,  426,  396,    // 16-23 
      362,  325,  284,  241,  196,  149,  100,   50,    // 24-31
        0,  -50, -100, -149, -196, -241, -284, -325,    // 32-39
     -362, -396, -426, -453, -473, -490, -502, -510,    // 40-47
     -512, -510, -502, -490, -473, -452, -426, -396,    // 48-55
     -362, -325, -284, -241, -196, -149, -100,  -50     // 56-63
} ;

uint8_t state ;             // Current state of operation state machine
uint8_t skip ;              // Reg 0x01 - Micro-stepping 

uint16_t set_amp ;          // Reg 0x02 (LSB) & 0x03 (MSB) 
uint16_t max_amp ;          // Reg 0x04 (LSB) & 0x05 (MSB)

uint8_t a_decay ;           // Phase A decay mode
uint8_t b_decay ;           // Phase B decay mode

uint8_t t_off ;             // Phase fixed off time, Reg 0x06
uint8_t t_blank_low ;       // Phase blanking in reverse current Reg 0x7
uint8_t t_blank_high ;      // Phase blanking in forward current Reg 0x8

int8_t step_a ;            // Current phase A step
int8_t step_b ;            // Current phase B step
uint8_t pol_a ;             // Phase A polarity
uint8_t pol_b ;             // Phase B polarity
int8_t  dir ;               // Direction

uint16_t pwm_lu[FULL_CYCLE] ;   // PWM lookup table
uint16_t zero_cross ;

uint16_t a_adc, b_adc ;
uint8_t adc_cap ;

uint8_t i2c_address ;       // Reg 0x00
uint8_t i2c_counter ;       // I2C bytes counter
uint8_t i2c_reg_addr ;      // Register address to read/write
uint8_t i2c_regs[I2C_REGISTERS] ;
uint8_t i2c_dirty ;         // I2C "dirty", does not match EEPROM

uint8_t a_state, b_state ;


extern void ioSetup(void) ;
extern void pwmSetup(void) ;
extern void intSetup(void) ;
extern void adc_setup(void) ;
extern void phTimersSetup(void) ;
extern void resetCheck(void) ;

// High priority interrupt
static void highInt(void) __interrupt(1) {
    // 'step' Interrupt
    if (INTCONbits.INT0IF) {
        state = STEP ;
        INTCONbits.INT0IF = 0 ;     // clear 'step' interrupt
    }
    
    // 'dir' Interrupts
    if (INTCON3bits.INT2IF) {
        
        if(PORTBbits.RB2) {
            // 'dir' is high trigger on falling edge next
            INTCON2bits.INTEDG2 = 0 ;   
            dir = 1 ;
        }
        else { 
            // 'dir' is low trigger on rising edge next
            INTCON2bits.INTEDG2 = 1 ;
            dir = -1 ;
        }

        state = NEXT_STEP ;         // Force recalculate of next step
        INTCON3bits.INT2IF = 0 ;    // clear 'dir' interrupt
    }
    
    // ADC Done
    if (PIR1bits.ADIF) {
        if (adc_cap) {
            b_adc = (ADRESH << 8) + ADRESL ;
            ADCON0bits.CHS = 0b0000 ;           // Set capture to RA0
            adc_cap = CAPTURE_A ;
        }
        else {
            a_adc = (ADRESH << 8) + ADRESL ;   
            ADCON0bits.CHS = 0b0001 ;           // Set capture to RA1
            adc_cap = CAPTURE_B ;
        }
            
        ADCON0bits.GO = 1 ;                     // Start next capture
        PIR1bits.ADIF = 0 ;                     // Clear ADC interrupt
    }
}

// Low priority interrupt
static void lowInt(void) __interrupt(2) {
    uint8_t i2c_buf = 0 ;
    
    // I2C Interrupt
    if (PIR1bits.SSP1IF) {
        if (SSP1STATbits.BF) i2c_buf = SSP1BUF ;    // Read data, clear BF
        
        // If address was received
        if (!SSP1STATbits.D) {
            i2c_counter = 0 ;                       // Reset counter
            if (SSP1STATbits.R) {                   // On read,
                SSP1BUF = i2c_regs[i2c_reg_addr++] ; // Send register value
            }
        }
        // If data received
        else {
            i2c_counter++ ;                         // Increment counter
            if (SSP1STATbits.R) {                   // On read,
                SSP1BUF = i2c_regs[i2c_reg_addr++] ; // Send register value
            }
            else {                                  // On write,
                if (i2c_counter == 1) {             // First byte is register
                    i2c_reg_addr = i2c_buf ;        // value
                }
                else {                              // Next bytes are data to 
                    i2c_regs[i2c_reg_addr++] = i2c_buf ; // write
                    i2c_dirty = 1 ;                 // set I2C dirty flag
                }
            }
        }
        
        i2c_reg_addr %= sizeof(i2c_regs) ;          // Prevent buffer overflow
        
        PIR1bits.SSP1IF = 0 ;       // Clear interrupt
        SSP1CON1bits.CKP = 1 ;      // Release clock
    }
    
    else if (PIR2bits.BCL1IF) {
        PIR2bits.BCL1IF = 0 ;       // Clear I2C collision interrupt
    } 
}

// Setup I2C I/F
void i2cSetup(void) {
    TRISCbits.TRISC3 = 1 ;      // Set I2C pins as input
    TRISCbits.TRISC4 = 1 ;
    
    SSP1CON1bits.SSPEN = 0 ;
    SSP1CON2bits.GCEN = 1 ;     // Enable general call on address 0x00
    SSP1CON2bits.SEN = 1 ;      // Enable clock stretching
    SSP1ADD = i2c_address ;     // Set device I2C address
    
    i2c_reg_addr = 0 ;
    
    SSP1CON1bits.SSPM = 0x6 ;   // I2C Slave mode, 7-bit address
}

// Prepare interrupts
void activeInts(void) {
    PIE1bits.SSP1IE = 0 ;       // Disable I2C interrupt
    SSP1CON1bits.SSPEN = 0 ;    // Disable I2C port
    PIE2bits.BCL1IE = 0 ;       // Disable I2C collision detection interrupt
    
    // 'dir' interrupt; falling edge if '1', rising edge '0' 
    if(PORTBbits.RB2) { 
        INTCON2bits.INTEDG2 = 0 ;
        dir = 1 ;
    }
    else { 
        INTCON2bits.INTEDG2 = 1 ;
        dir = -1 ;
    }
    
    INTCONbits.INT0IE = 1 ;     // Enable 'step' interrupt
    INTCON3bits.INT2IE = 1 ;    // Enable 'dir' interrupt
}

void idleInts(void) {
    INTCONbits.INT0IE = 0 ;     // Disable 'step' interrupt
    INTCON3bits.INT2IE = 0 ;    // Disable 'dir' interrupt
    
    PIE1bits.SSP1IE = 1 ;       // Enable I2C interrupt
    PIE2bits.BCL1IE = 1 ;       // Enable I2C collision detection interrupt
    
    SSP1CON1bits.SSPOV = 0 ;
    PIR1bits.SSP1IF = 0 ;
    SSP1CON1bits.CKP = 1 ;      // Release clock
    SSP1CON1bits.SSPEN = 1 ;    // Enable I2C port
}

// Calculate PWM lookup table
void prep_pwm_lu(void) {
    float ratio, set, bias ;
    uint8_t i ;
    
    ratio = (float)set_amp / (float)max_amp ;
    bias = (float)PWM_MAX / 2.0 ;
    
    zero_cross = __fs2uint(bias) ;
    
    for(i = 0 ; i < (sizeof(pwm_base) / sizeof(int16_t)) ; i++) {
        set = (float)pwm_base[i] * ratio + bias ;
        pwm_lu[i] = __fs2uint(set) ;
    }
}

// Get operation variables
void set_op_vars(void) {
    i2c_address = i2c_regs[0x00] ;
    skip        = i2c_regs[0x01] ;
    set_amp     = i2c_regs[0x02] ;
    set_amp    += i2c_regs[0x03] << 8 ;
    max_amp     = i2c_regs[0x04] ;
    max_amp    += i2c_regs[0x05] << 8 ;
    t_off       = i2c_regs[0x06] * 2 * TMR_500NS ;
    t_blank_low = i2c_regs[0x07] * 2 * TMR_500NS ;
    t_blank_high= i2c_regs[0x08] * 2 * TMR_500NS ;
}

// Basic msec. delay loop
void delay_ms(uint16_t time) {
    uint16_t i = time ;
    
    T0CON = 0x02 ;
    
    while(--i) {
        T0CONbits.TMR0ON = 0 ;
        INTCONbits.TMR0IF = 0 ;
        TMR0H = TMR_1MS >> 8 ;
        TMR0L = TMR_1MS ;
        T0CONbits.TMR0ON = 1 ;

        while(!INTCONbits.TMR0IF) ;
    }
    
    T0CONbits.TMR0ON = 0 ;
    INTCONbits.TMR0IF = 0 ;
}

int main(void) {
    uint8_t pwm_a_l = 0, pwm_a_h = 0, pwm_b_l = 0, pwm_b_h = 0 ;
    
    ioSetup() ;                 // Setup IO ports
    
    // We didn't have a proper power-on
    if(RCONbits.POR) {
        resetCheck() ;
    }
    
    // Set POR bit to one. If there is a sudden reset,
    // this bit won't reset indicating a fault.
    RCONbits.POR = 1 ;
    RCONbits.BOR = 1 ;
    
    read_regs(i2c_regs, sizeof(i2c_regs)) ;  // Populate I2C registers from EEPROM
    set_op_vars() ;             // Set operation variables from I2C registers
    i2c_dirty = 0 ;             // Clear I2C dirty flag
    
    i2cSetup() ;                // Setup I2C I/F
    intSetup() ;                // Interrupts setup
    adc_setup() ;
    idleInts() ;                // Set interrupts to 'idle' state
    
    // Reset state
    state = IDLE ;              // Start in 'idle' state
    
    // Blink Blue LED once - indicate ready
    PORTDbits.RD2 = 1 ;     // Turn blue LED on
    delay_ms(300) ;    
    PORTDbits.RD2 = 0 ;     // Turn blue LED off
    
    WDTCON = 1 ;        // Enable watchdog timer
    
    while(1) {
        // Reset watchdog timer
        ClrWdt() ;
        
        switch(state) {
            case START:
                LATDbits.LATD2 = 1 ;
                
                // On full step phase A starts at 45 deg, 0 deg otherwise
                if (skip == STEP_1) step_a = STEPS / 2 ;
                else step_a = 0 ;
                
                // phase B is +90 deg from phase A
                step_b = step_a + STEPS ;
                
                if (step_a > THREE_QUARTERS) a_decay = FAST_DECAY ;
                else if (step_a >= HALF_CYCLE) a_decay = SLOW_DECAY ;
                else if (step_a > STEPS) a_decay = FAST_DECAY ;
                else a_decay = SLOW_DECAY ;
                
                if (step_b > THREE_QUARTERS) b_decay = FAST_DECAY ;
                else if (step_b >= HALF_CYCLE) b_decay = SLOW_DECAY ;
                else if (step_b > STEPS) b_decay = FAST_DECAY ;
                else b_decay = SLOW_DECAY ;
                
                prep_pwm_lu() ;         // prepare PWM lookup table
                
                pwmSetup() ;            // Activate PWM
                activeInts() ;          // Set active state interrupts
                
                a_state = T_DRIVE ;
                b_state = T_DRIVE ;
                
                // Set current polarity
                if (step_a == HALF_CYCLE || step_a == 0) pol_a = 2 ;
                else if (step_a > HALF_CYCLE) pol_a = 0 ;
                else pol_a = 1 ;
                
                if (step_b == HALF_CYCLE || step_b == 0) pol_b = 2 ;
                else if (step_b > HALF_CYCLE) pol_b = 0 ;
                else pol_b = 1 ;
                
                
                // Pre-calculate PWM duty-cycle registers values
                CCP5CONbits.DC5B = 0x0c + (pwm_lu[step_a] & 0x3) << 4 ;
                CCPR5L = pwm_lu[step_a] >> 2 ;
                
                CCP4CONbits.DC4B = 0x0c + (pwm_lu[step_b] & 0x3) << 4 ;
                CCPR4L = pwm_lu[step_b] >> 2 ;
                
                PORTAbits.RA4 = 1 ;
                PORTAbits.RA5 = 1 ; 
                
                state = NEXT_STEP ;
                break ;
                
            case STEP:
                CCP5CONbits.DC5B = pwm_a_l ;             // Set PWM duty cycle, Phase-A
                CCPR5L = pwm_a_h ;
                
                CCP4CONbits.DC4B = pwm_b_l ;             // Set PWM duty cycle, Phase-B
                CCPR4L = pwm_b_h ;
                
                if (step_a > THREE_QUARTERS && step_a < (FULL_CYCLE - 1)) {
                    a_decay = FAST_DECAY ;
                    PORTAbits.RA4 = 1 ;
                }
                else if (step_a >= HALF_CYCLE) a_decay = SLOW_DECAY ;
                else if (step_a > STEPS) {
                    a_decay = FAST_DECAY ;
                    PORTAbits.RA4 = 1 ;
                }
                else a_decay = SLOW_DECAY ;
                
                if (step_b > THREE_QUARTERS && step_b < (FULL_CYCLE - 1)) {
                    b_decay = FAST_DECAY ;
                    PORTAbits.RA5 = 1 ;
                }
                else if (step_b >= HALF_CYCLE) b_decay = SLOW_DECAY ;
                else if (step_b > STEPS) {
                    b_decay = FAST_DECAY ;
                    PORTAbits.RA5 = 1 ;
                }
                else b_decay = SLOW_DECAY ;
               
                state = NEXT_STEP ;
                break ;
                
            case NEXT_STEP:
                state = RUNNING ;
                
                // Forward motion
                if (dir == 1) {
                    step_a += skip ;
                    step_b += skip ;
                    if (step_a > FULL_CYCLE - 1) step_a -= FULL_CYCLE ;
                    if (step_b > FULL_CYCLE - 1) step_b -= FULL_CYCLE ;
                }
                // Reverse motion
                else {
                    step_a -= skip ;
                    step_b -= skip ;
                    if (step_a < 0) step_a += FULL_CYCLE ;
                    if (step_b < 0) step_b += FULL_CYCLE ;
                }
                
                // Set current polarity
                if (step_a == HALF_CYCLE || step_a == 0) pol_a = 2 ;
                else if (step_a > HALF_CYCLE) pol_a = 0 ;
                else pol_a = 1 ;
                
                if (step_b == HALF_CYCLE || step_b == 0) pol_b = 2 ;
                else if (step_b > HALF_CYCLE) pol_b = 0 ;
                else pol_b = 1 ;
                               
                // Pre-calculate PWM duty-cycle registers values
                pwm_a_l = 0x0c + (pwm_lu[step_a] & 0x3) << 4 ;
                pwm_a_h = pwm_lu[step_a] >> 2 ;
                
                pwm_b_l = 0x0c + (pwm_lu[step_b] & 0x3) << 4 ;
                pwm_b_h = pwm_lu[step_b] >> 2 ;
                
                break ;
                
            case RUNNING:
                if(CM1CON0bits.C1OUT && a_state == T_DRIVE) {
                    if (a_decay == FAST_DECAY) {            
                        if (pol_a) {                    // Forward current, Phase-A
                            LATCbits.LATC0 = 0 ;
                            LATCbits.LATC1 = 1 ;  
                        }
                        else {                          // Reverse current, Phase-A
                            LATCbits.LATC1 = 0 ;
                            LATCbits.LATC0 = 1 ;
                        }
                    }     
                    else PORTAbits.RA4 = 0 ;
                    
                    a_state = PH_OFF ;
                    T1CONbits.TMR1ON = 1 ;
                }
                
                if(CM2CON0bits.C2OUT && b_state == T_DRIVE) {
                    if (b_decay == FAST_DECAY) {
                        if (pol_b) {                    // Forward current, Phase-B
                            LATDbits.LATD5 = 0 ;
                            LATCbits.LATC2 = 1 ;
                        }
                        else {                          // Reverse current, Phase-B
                            LATCbits.LATC2 = 0 ;
                            LATDbits.LATD5 = 1 ;
                        }
                    }
                    else PORTAbits.RA5 = 0 ;
                    
                    b_state = PH_OFF ;
                    T3CONbits.TMR3ON = 1 ;
                }
                
                // Enable signal dropped
                if (!PORTBbits.RB3) {
                    INTCONbits.GIE = 0 ;
                    idleInts() ;
                    
                    LATAbits.LATA4 = 0 ;    // Shut phases down
                    LATAbits.LATA5 = 0 ; 
                    
                    PORTDbits.RD2 = 0 ;     // Turn blue LED off
                    state = IDLE ;
                    INTCONbits.GIE = 1 ;
                }
                break ;
                
            case IDLE: 
                // Enable stepping
                if (PORTBbits.RB3) state = START ;
                
                // If I2C registers were written, store to EEPROM
                if (i2c_dirty) {
                    store_regs(i2c_regs, sizeof(i2c_regs)) ;
                    i2c_dirty = 0 ;
                    set_op_vars() ; 
                }
                
                break ;
        }
    }
}


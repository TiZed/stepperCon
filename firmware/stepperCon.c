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

#include "pi_controller.h"
#include "registers.h"
#include "sc_setup.h"


// 64MHz clock 
#pragma config  FOSC    = HSHP      // HS ocsillator, high power
#pragma config  PLLCFG  = ON        // x4 PLL on
#pragma config  WDTEN   = SWON      // WDT software controlled
#pragma config  XINST   = OFF       // For SDCC
#pragma config  CCP3MX  = PORTB5    // CCP3 on port E0   
#pragma config  P2BMX   = PORTC0    // P2B on port C0
#pragma config  MCLRE   = EXTMCLR   // Enable MCLR pin (E3 disabled)
// #pragma config  LVP     = ON        // Low voltage programming enabled

#define STEPS 16
#define FULL_CYCLE (4 * STEPS)
#define HALF_CYCLE (2 * STEPS)
#define THREE_QUARTERS (3 * STEPS)

// Some default values
#define CUR_GAIN 90          // Current sensor gain

// PI Parameters
#define KP      30           // Proportional coefficient
#define KI      10           // Integral coefficient

#define I2C_ADDRESS 0x44     // X- 0x40, Y- 0x42, Z- 0x44 
#define I2C_REGISTERS 11

#define MAX_AMP 1852         // [mA] Maximum driver amperage
#define SET_AMP 1400         // [mA] Default driver limit
#define T_BLANK 24           // [62.5nsec.] dead-band for transistors transition

// Stepping modes
#define STEP_16  1
#define STEP_8   2
#define STEP_4   4
#define STEP_2   8
#define STEP_1  16

// System states
#define IDLE      0
#define START     1
#define RUNNING   2
#define CALC_PI_A 3
#define CALC_PI_B 4
#define STEP      6

#define TMR_1MS   63536     // @1:8 Prescaller, Fosc/4
#define TMR_10MS  45536     // @1:8 Prescaller, Fosc/4
#define TMR_500NS 1         // @1:8 Prescaller, Fosc/4

#define PWM_MAX   1023

// Default EEPROM data
typedef unsigned char eeprom ;
__code eeprom __at 0xf00000 __EEPROM[] = { I2C_ADDRESS, STEP_16, SET_AMP, 
                                           SET_AMP >> 8, MAX_AMP, 
                                           MAX_AMP >> 8, T_BLANK,
                                           KP, KP >> 8, KI, KI >> 8 };

// PWM micro-stepping 10bit base table
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

volatile uint8_t state ;             // Current state of operation state machine
volatile uint8_t micro_steps ;       // Reg 0x01 - Micro-stepping 

volatile uint16_t set_amp ;          // Reg 0x02 (LSB) & 0x03 (MSB) 
volatile uint16_t max_amp ;          // Reg 0x04 (LSB) & 0x05 (MSB)

volatile uint8_t t_blank ;           // Phase blanking in reverse current Reg 0x06

volatile int16_t pid_kp ;            // Reg 0x07 (LSB) & 0x08 (MSB) 
volatile int16_t pid_ki ;            // Reg 0x09 (LSB) & 0x0a (MSB)

volatile int8_t  dir ;               // Direction
volatile int8_t  skip ;              // Current steps, based on micro-stepping mode 

volatile uint16_t pwm_lu[FULL_CYCLE] ;   // PWM lookup table
volatile uint16_t zero_cross ;

uint8_t i2c_address ;       // Reg 0x00
uint8_t i2c_counter ;       // I2C bytes counter
uint8_t i2c_reg_addr ;      // Register address to read/write
volatile uint8_t i2c_regs[I2C_REGISTERS] ;
uint8_t i2c_dirty ;         // I2C "dirty", does not match EEPROM
volatile uint8_t i2c_buf ;

volatile uint16_t adc_wdt ;

// High priority interrupt
static void highInt(void) __interrupt(1) {
    // ADC Done
    if (PIR1bits.ADIF) {
        adc_wdt = 0 ;
        
        if (state != STEP) {
            if (ADCON0bits.CHS == 0b0001) state = CALC_PI_A ;
            else state = CALC_PI_B ;
        }
           
        PIR1bits.ADIF = 0 ;                     // Clear ADC interrupt
    }
    
     // 'step' Interrupt
    if (INTCONbits.INT0IF) {
        state = STEP ;
        INTCONbits.INT0IF = 0 ;     // clear 'step' interrupt
    }
    
    // On Comparator 1 change
    if (PIR2bits.C1IF && PIE2bits.C1IE) {
        if (CM1CON0bits.C1OUT) {
            ADCON0bits.CHS = 0b0001 ; 
            ADCON0bits.GO = 1 ;
            PIE2bits.C1IE = 0 ;
        }
        
        PIR2bits.C1IF = 0 ;        
    }
    
    // On Comparator 2 change
    if (PIR2bits.C2IF && PIE2bits.C2IE) {
        if (CM2CON0bits.C2OUT) {
            ADCON0bits.CHS = 0b0000 ; 
            ADCON0bits.GO = 1 ;
            PIE2bits.C2IE = 0 ;
        }
        
        PIR2bits.C2IF = 0 ;        
    }
}

// Low priority interrupt
static void lowInt(void) __interrupt(2) {
   
    // 'dir' Interrupts
    if (INTCON3bits.INT2IF) {
        
        if(PORTBbits.RB2) {
            // 'dir' is high trigger on falling edge next
            INTCON2bits.INTEDG2 = 0 ;   
            dir = 1 ;
            skip = micro_steps ;
        }
        else { 
            // 'dir' is low trigger on rising edge next
            INTCON2bits.INTEDG2 = 1 ;
            dir = -1 ;
            skip = -micro_steps ;
        }

        INTCON3bits.INT2IF = 0 ;    // clear 'dir' interrupt
    }
    
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
    PIE1bits.ADIE = 1 ;         // Enable ADC interrupt
}

void idleInts(void) {
    INTCONbits.INT0IE = 0 ;     // Disable 'step' interrupt
    INTCON3bits.INT2IE = 0 ;    // Disable 'dir' interrupt
    
    PIE1bits.ADIE = 0 ;         // Disable ADC interrupt
    
    PIE2bits.C1IE = 0 ;         // Disable comparators int. 
    PIE2bits.C2IE = 0 ;
    
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
    uint16_t i ;
    int16_t max = -32000 ;
    int16_t min = 32000 ;
    
    ratio = (float)set_amp / (float)max_amp ;
    bias = (float)PWM_MAX / 2.0 + 1 ;
    
    zero_cross = bias ;
    
    for(i = 0 ; i < (sizeof(pwm_base) / sizeof(int16_t)) ; i++) {
        set = (float)pwm_base[i] * ratio + bias ;
        pwm_lu[i] = set ;
        
        if ((int16_t)pwm_lu[i] > max) max = pwm_lu[i] ;
        if ((int16_t)pwm_lu[i] < min) min = pwm_lu[i] ;
    }
    
    max += 15 ;
    min -= 15 ;
    
    set_max_out(max) ;
    set_min_out(min) ;
}

// Get operation variables
void set_op_vars(void) {
    i2c_address  = i2c_regs[0x00] ;
    micro_steps  = i2c_regs[0x01] ;
    set_amp      = i2c_regs[0x02] ;
    set_amp     += i2c_regs[0x03] << 8 ;
    max_amp      = i2c_regs[0x04] ;
    max_amp     += i2c_regs[0x05] << 8 ;
    t_blank      = i2c_regs[0x06] ;
    pid_kp       = i2c_regs[0x07] ;
    pid_kp      += i2c_regs[0x08] << 8 ;
    pid_ki       = i2c_regs[0x09] ;
    pid_ki      += i2c_regs[0x0a] << 8 ;
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
    int16_t pwm_a = 0, pwm_b = 0 ;
    uint16_t adc_res ;
    pi_result_t pi_result_a ;
    pi_result_t pi_result_b ;   
    uint8_t dummy ;
  
    int16_t step_a = 0 ;                         // Current phase A step
    int16_t step_b = 0 ;                         // Current phase B step
    
    ioSetup() ;                 // Setup IO ports
    
    // We didn't have a proper power-on
    if(RCONbits.POR) resetCheck() ;
    
    // Set POR bit to one. If there is a sudden reset,
    // this bit won't reset indicating a fault.
    RCONbits.POR = 1 ;
    RCONbits.BOR = 1 ;
    
    read_regs(i2c_regs, sizeof(i2c_regs)) ;  // Populate I2C registers from EEPROM
    set_op_vars() ;             // Set operation variables from I2C registers
    i2c_dirty = 0 ;             // Clear I2C dirty flag
    
    i2cSetup() ;                // Setup I2C I/F
    intSetup() ;                // Interrupts setup
    compsSetup() ;
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
                state = RUNNING ;
                
                // Turn blue LED on
                LATDbits.LATD2 = 1 ;
                
                skip = dir * micro_steps ;
                
                // On full step phase A starts at 45 deg, 0 deg otherwise
                if (micro_steps == STEP_1) step_a = STEPS / 2 ;
                else step_a = 0 ;
                
                // phase B is +90 deg from phase A
                step_b = step_a + STEPS ;
                
                init_result(&pi_result_a, pid_kp, pid_ki) ;
                init_result(&pi_result_b, pid_kp, pid_ki) ;
                
                prep_pwm_lu() ;            // prepare PWM lookup table
                
                pwm_a = pwm_lu[step_a] ;
                pwm_b = pwm_lu[step_b] ;
                
                LATAbits.LATA4 = 1 ;
                LATAbits.LATA5 = 1 ; 
                LATCbits.LATC5 = 0 ;
               
                pwmSetup(t_blank) ;        // Activate PWM
                pwmOut() ;
                activeInts() ;             // Set active state interrupts
                
                ADCON0bits.CHS = 0b0001 ;  // Set capture to RA0
                ADCON0bits.GO = 1 ;
                
                break ;
                
            case STEP:
                state = RUNNING ;
                
                step_a += skip ;
                step_b += skip ;

                if (step_a > FULL_CYCLE - 1) step_a -= FULL_CYCLE ;
                else if (step_a < 0) step_a += FULL_CYCLE ;

                if (step_b > FULL_CYCLE - 1) step_b -= FULL_CYCLE ; 
                else if (step_b < 0) step_b += FULL_CYCLE ;
                
                pwm_a = pwm_lu[step_a] ;
                pwm_b = pwm_lu[step_b] ;
                
                if (!PIE2bits.C2IE && !PIE2bits.C1IE && !ADCON0bits.GO) {
                    dummy = CM1CON0 ;
                    PIR2bits.C1IF = 0 ; 
                    PIE2bits.C1IE = 1 ;
                }
                
                break ;
                
            case CALC_PI_A:
                state = RUNNING ;
                
                adc_res = ADRESH ;
                adc_res <<= 8 ;
                adc_res += ADRESL ;
                
                dummy = CM2CON0 ;
                PIR2bits.C2IF = 0 ; 
                PIE2bits.C2IE = 1 ;
                
                LATCbits.LATC5 = 1 ;
                calc_pi(&pi_result_a, adc_res, pwm_a) ;
                
                CCPR2L = pi_result_a.output >> 2 ;
                CCP2CONbits.DC2B = pi_result_a.output ;
                LATCbits.LATC5 = 0 ;
                
                CCPR5L = pi_result_a.output >> 3 ;
                CCP5CONbits.DC5B = pi_result_a.output >> 1 ;
                
                break ;

            case CALC_PI_B:
                state = RUNNING ;
                
                adc_res = ADRESH  ;
                adc_res <<= 8 ;
                adc_res += ADRESL ;

                dummy = CM1CON0 ;
                PIR2bits.C1IF = 0 ; 
                PIE2bits.C1IE = 1 ;
                        
                calc_pi(&pi_result_b, adc_res, pwm_b) ;
                
                CCPR1L = pi_result_b.output >> 2 ;
                CCP1CONbits.DC1B = pi_result_b.output ;
                
                adc_res >>= 1 ;
                
                CCPR4L = pi_result_b.output >> 3 ;
                CCP4CONbits.DC4B = pi_result_b.output >> 1 ;
                
                break ;
                
            case RUNNING:
                if(++adc_wdt > 600 && !ADCON0bits.GO) {
                    adc_wdt = 0 ;
                    ADCON0bits.CHS = 0b0001 ;  // Set capture to RA0
                    ADCON0bits.GO = 1 ;
                }
                
                // Enable signal dropped
                if (!PORTBbits.RB3) {
                    INTCONbits.GIE = 0 ;
                    idleInts() ;
                    
                    LATAbits.LATA4 = 0 ;            // Shut phases down
                    LATAbits.LATA5 = 0 ; 
                    
                    ECCP2ASbits.CCP2ASE = 1    ;    // Shutdown PWM 
                    ECCP1ASbits.CCP1ASE = 1    ;    
                    
                    ADCON0bits.GO = 0 ;
                    
                    PORTDbits.RD2 = 0 ;             // Turn blue LED off
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


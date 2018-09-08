/* 
 * File:   sc_setup.h
 * Author: tized
 *
 * Created on August 31, 2018, 10:55 PM
 */

#ifndef SC_SETUP_H
#define	SC_SETUP_H

#ifdef	__cplusplus
extern "C" {
#endif

void ioSetup(void) ;
void pwmSetup(uint8_t) ;
void compsSetup(void) ;
void intSetup(void) ;
void adc_setup(void) ;
void resetCheck(void) ;

void pwmOut(void) ;


#ifdef	__cplusplus
}
#endif

#endif	/* SC_SETUP_H */


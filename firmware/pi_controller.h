/* 
 * File:   pi_controller.h
 * Author: tized
 *
 * Created on 15 June 2018, 10:37
 */

#ifndef PI_CONTROLLER_H
#define	PI_CONTROLLER_H

#include <stdint.h>

#ifdef	__cplusplus
extern "C" {
#endif

extern volatile int16_t _max_out ;
extern volatile int16_t _min_out ;

typedef struct {
    int16_t output ;
    int16_t out_integ ;
    
    int16_t kp ;
    int16_t ki ;
} pi_result_t ;


void set_max_out(int16_t) ;
void set_min_out(int16_t) ;

void init_result(pi_result_t *, int16_t, int16_t) ;
void calc_pi(pi_result_t *, int16_t, int16_t) ;


#ifdef	__cplusplus
}
#endif

#endif	/* PI_CONTROLLER_H */


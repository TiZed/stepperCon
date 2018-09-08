/* 
 * File:   pi_controller.c
 * Author: TiZed
 *
 * Created on 12 July 2017
 * 
 *  Copyright (C) 2017 TiZed
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
 */

#include "pi_controller.h"
#include "fast_mult.h"

#include <pic18fregs.h>
#include <stdint.h>

volatile int16_t _max_out ;
volatile int16_t _min_out ;
volatile int16_t _integ_sat ;


void set_max_out(int16_t max_out_val) {
    _max_out = max_out_val ;
    _integ_sat = mult_int16(2, _max_out) ;
}

void set_min_out(int16_t min_out_val) {
    _min_out = min_out_val ;
}

void init_result(pi_result_t * result, int16_t kp, int16_t ki) {
    result->out_integ = 0 ;
    result->output = 0 ;
    
    result->kp = kp ;
    result->ki = ki ;
}

void calc_pi(pi_result_t * result, int16_t measured, int16_t setpoint) {
    int16_t error ;
    int32_t output ;
    
    if (result->kp == 0) {
        output = setpoint ;
    }
    
    else {
        error = setpoint - measured ;
        output = mult_int16(result->kp, error) >> 4 ;

        if(result->kp != 0) {
            result->out_integ += mult_int16(result->ki, error) >> 9 ;

            if (result->out_integ > _integ_sat) result->out_integ = _integ_sat ;
            if (result->out_integ < -_integ_sat) result->out_integ = -_integ_sat ;

            output += result->out_integ ;
        }
    }
   
    if (output > _max_out) output = _max_out ;
    else if (output < _min_out) output = _min_out ;
    
    result->output = output ;  
}
/* 
 * File:   registers.c
 * Author: TiZed
 *
 * Created on 11 October 2016
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

void read_regs(uint8_t * i2c_regs, uint8_t num_regs) {
    uint8_t i ;
    
    EEADRH = 0x00 ;
    
    for(i = 0 ; i < num_regs ; i++) {
        EEADR = i ;              // Read I2C address from 00 location
        EECON1bits.EEPGD = 0 ;      // Access data EEPROM
        EECON1bits.CFGS = 0 ;
        EECON1bits.RD = 1 ;         // Initiate read
        i2c_regs[i] = EEDATA ;
    }
}

void store_regs(uint8_t * i2c_regs, uint8_t num_regs) {
    uint8_t i, val ;
    
    EEADRH = 0x00 ;
    
    for(i = 0 ; i < num_regs ; i++) {
        EEADR = i ; 
        EECON1bits.EEPGD = 0 ;      // Access data EEPROM
        EECON1bits.CFGS = 0 ;
        EECON1bits.RD = 1 ;         // Initiate read
        val = EEDATA ;
        
        // Only write changed values
        if (val != i2c_regs[i]) {
            EEDATA = i2c_regs[i] ;      // Prepare data
            
            EECON1bits.EEPGD = 0 ;      // Access data EEPROM
            EECON1bits.CFGS = 0 ;
            EECON1bits.WREN = 1 ;       // Allow write cycle
            
            INTCONbits.GIE = 0 ;        // Disable interrupts
            
            EECON2 = 0x55 ;             // "magic" sequence for write
            EECON2 = 0xaa ;
            
            EECON1bits.WR = 1 ;         // Initiate write
            
            INTCONbits.GIE = 1 ;        // Enable interrupts
            
            EECON1bits.WREN = 0 ;       // Prevent write cycle
        }   
    }
}

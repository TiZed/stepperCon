/* 
 * File:   fast_mult.h
 * Author: tized
 *
 * Created on 26 July 2018, 11:47
 */

#ifndef FAST_MULT_H
#define	FAST_MULT_H


#ifdef	__cplusplus
extern "C" {
#endif
    
#include <pic18fregs.h>
    
extern inline uint16_t mult_uint8(uint8_t, uint8_t) ;
extern inline int16_t mult_int8(int8_t a, int8_t b) ;

extern inline uint32_t mult_uint16(uint16_t, uint16_t) ;
extern inline int32_t mult_int16(int16_t, int16_t) ;

#pragma udata bank0 res_ a16_ b16_ a_ b_
volatile uint8_t a_, b_ ;
volatile uint16_t a16_, b16_ ;
volatile uint32_t res_ ;


// 8x8 unsigned integer fast multiplier  
inline uint16_t mult_uint8(uint8_t a, uint8_t b) {
    a_ = a ;
    b_ = b ;
    
    __asm
        movf _a_, W
        mulwf _b_
    __endasm;
    
    a16_ = PRODH ;
    a16_ <<= 8 ;
    a16_ += PRODL ;
    
    return(a16_) ;
}

// 8x8 signed integer fast multiplier 
inline int16_t mult_int8(int8_t a, int8_t b) {
    a_ = a ;
    b_ = b ;
    
    __asm
        movf _a_, W
        mulwf _b_
        btfsc _b_, 7
        subwf PRODH, F
        movf _b_, W 
        btfsc _a_, 7
        subwf PRODH, F
    __endasm ;
    
    a16_ = PRODH ;
    a16_ <<= 8 ;
    a16_ += PRODL ;
    
    return(a16_) ;
}

// 16x16 unsigned integer fast multiplier 
inline uint32_t mult_uint16(uint16_t a, uint16_t b) {
    a16_ = a ;
    b16_ = b ;
    
    __asm
            movf _a16_, W
            mulwf _b16_
            movff PRODH, (_res_ + 1)
            movff PRODL, _res_
            movf (_a16_ + 1), W
            mulwf (_b16_ + 1)
            movff PRODH, (_res_ + 3)
            movff PRODL, (_res_ + 2)
            movf _a16_, W
            mulwf (_b16_ + 1)
            movf PRODL, W
            addwf (_res_ + 1), F
            movf PRODH, W
            addwfc (_res_ + 2), F
            clrf _WREG
            addwfc (_res_ + 3), F
            movf (_a16_ + 1), W
            mulwf _b16_
            movf PRODL, W
            addwf (_res_ + 1), F
            movf PRODH, W
            addwfc (_res_ + 2), F
            clrf _WREG
            addwfc (_res_ + 3), F
    __endasm ;
    
    return(res_) ;
    
}

// 16x16 signed integer fast multiplier 
inline int32_t mult_int16(int16_t a, int16_t b) {
    a16_ = a ;
    b16_ = b ;
    
    __asm
            movf _a16_, W
            mulwf _b16_
            movff PRODH, (_res_ + 1)
            movff PRODL, _res_
            movf (_a16_ + 1), W
            mulwf (_b16_ + 1)
            movff PRODH, (_res_ + 3)
            movff PRODL, (_res_ + 2)
            movf _a16_, W
            mulwf (_b16_ + 1)
            movf PRODL, W
            addwf (_res_ + 1), F
            movf PRODH, W
            addwfc (_res_ + 2), F
            clrf _WREG
            addwfc (_res_ + 3), F
            movf (_a16_ + 1), W
            mulwf _b16_
            movf PRODL, W
            addwf (_res_ + 1), F
            movf PRODH, W
            addwfc (_res_ + 2), F
            clrf _WREG
            addwfc (_res_ + 3), F
            btfss (_b16_ + 1), 7
            bra $+10 // _SIGN_A16
            movf _a16_, W
            subwf (_res_ + 2), 1
            movf (_a16_ + 1), W
            subwfb (_res_ + 3), 1
//            _SIGN_A16:
            btfss (_a16_ + 1), 7
            bra $+10 // _CONT_CODE
            movf _b16_, W
            subwf (_res_ + 2), 1
            movf (_b16_ + 1), W
            subwfb (_res_ + 3), 1
//            _CONT_CODE:
            
    __endasm ;
    
    return(res_) ;
}


#ifdef	__cplusplus
}
#endif

#endif	/* FAST_MULT_H */


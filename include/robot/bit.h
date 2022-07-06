/**
 * @file bit.h
 * @brief ビット演算処理
 */
#pragma once
#include <iostream>
#include <bitset>

extern const unsigned int BIT_FLAG[16];

inline bool get_bit(unsigned int  data, int bit){ return(data & BIT_FLAG[bit]); }
inline void set_bit(unsigned int &data, int bit){ data |= BIT_FLAG[bit]; }
inline void clr_bit(unsigned int &data, int bit){ data &= ~BIT_FLAG[bit]; }


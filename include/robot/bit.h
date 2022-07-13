/**
 * @file bit.h
 * @brief ビット演算処理
 */
#pragma once
#include <iostream>
#include <bitset>

extern const unsigned int BIT_FLAG[16];

inline bool get_bit(unsigned int  data, unsigned int bit){ return(data & BIT_FLAG[bit]); }
inline void set_bit(unsigned int &data, unsigned int bit){ data |= BIT_FLAG[bit]; }
inline void clr_bit(unsigned int &data, unsigned int bit){ data &= ~BIT_FLAG[bit]; }

/**
 * @brief 複数の連続ビットを抽出
 */
inline unsigned int get_bits(unsigned int data, unsigned int bit0, unsigned int bit1)
{
    int size = 8*sizeof(data)-1;
    assert(bit0 < bit1 && bit1<=size);
    data = data << (size-bit1);
    data = data >> (size-bit1+bit0);
    return data;
}


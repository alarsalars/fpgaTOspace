
// Taha Alars

// Simple PRBS for hardware loop back error test with 7 taps based on lfsr and XOR
#include "prbs.h"



bool first_func(){
#pragma HLS inline off
    static ap_uint<7> lfsr_shift =0b0101011;
     bool th5;
     bool th6;

    th5 = (lfsr_shift >> 5) & 1;
    th6 = (lfsr_shift >> 6) & 1;
    // th7 = lfsr_shift.get_bit(7);
    // th6 = lfsr_shift.get_bit(7);
    bool bit_xor;
    ap_uint<7> mask;
    bit_xor = th6 ^ th5;
    mask = bit_xor <<6;
    lfsr_shift = lfsr_shift >> 1;
    lfsr_shift = lfsr_shift | mask;
    // lfsr_shift.set_bit(7,bit_xor);
    return bit_xor;

}


bool second_func(bool loop_bit, bool &bit_out2 ){
#pragma HLS inline off
    static ap_uint<7> lfsr_shift =0b1100110;
     bool th5;
     bool th6;
    ap_uint<7> mask;

    th5 = (lfsr_shift >> 5) & 1;
    th6 = (lfsr_shift >> 6) & 1;
    // th7 = lfsr_shift.get_bit(7);
    // th6 = lfsr_shift.get_bit(7);
    bool bit_xor;

    bit_xor = th6 ^ th5;
    mask = loop_bit <<6;
    lfsr_shift = lfsr_shift >> 1;
    lfsr_shift = lfsr_shift | mask;
    // lfsr_shift.set_bit(7,bit_xor);
    bit_out2 = loop_bit;
    return bit_xor;

}




void main_func( bool in,  bool &out,  bool &compare){
//void main_func( bool &compare){  // test
#pragma HLS PIPELINE
#pragma HLS INTERFACE ap_none port=in
#pragma HLS INTERFACE ap_none port=out
#pragma HLS INTERFACE ap_none port=compare
#pragma HLS INTERFACE ap_ctrl_none port=return

     bool out_from_func1;
     bool out_from_func2;
     bool in_func1;
     bool out_func1;
     //bool out;  // test
     out_func1 = first_func();
     in_func1 = in;
     //out_from_func2 = second_func(out, out_from_func1 ); // test
     out_from_func2 = second_func(in_func1, out_from_func1 );
     compare = out_from_func1 ^ out_from_func2;
     out=out_func1;

}

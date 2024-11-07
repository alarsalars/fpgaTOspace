

// Taha Alars
// Very simple wishbone slave uart sender reciver half duplux. It has large room to improve also to manage
// the send and recive data package, I just wanted to test wb-uart with HLS in this code
// I use only one baudrate = 115200

#include "top.h"


void top(ap_uint<8> adr, bool we, bool cyc, bool stb,ap_uint<8> wb_in, bool rx
		,bool &tx, bool &ack,ap_uint<10> &uart_out){
#pragma HLS PIPELINE
#pragma HLS INTERFACE ap_none port=adr
#pragma HLS INTERFACE ap_none port=we
#pragma HLS INTERFACE ap_none port=cyc
#pragma HLS INTERFACE ap_none port=stb
#pragma HLS INTERFACE ap_none port=wb_in
#pragma HLS INTERFACE ap_none port=rx
#pragma HLS INTERFACE ap_none port=tx
#pragma HLS INTERFACE ap_none port=ack
#pragma HLS INTERFACE ap_none port=uart_out
#pragma HLS INTERFACE ap_ctrl_none port=return


	static wb_fsm state = idle;
	static unsigned int baud_count  = DIVISOR -1;
	static int i = 0;
	static int j = 1;


	ap_uint<8> adr_reg ;
	static ap_uint<8> wb_in_reg ;
	static ap_uint<11> uart_wr_shift ;
	static ap_uint<11> uart_rd_shift ;
	static bool tx_ff = 1;
	static bool rx_ff = 1;

	wb_fsm next_state = idle;

	adr_reg = adr;
	wb_in_reg = wb_in;
	rx_ff = rx;
	//uart_wr_shift = 0b1010111110;

	switch (state){
	case idle:
		tx_ff = 1;
		i = 0;
		j = 1;
		baud_count  = DIVISOR -1;
		if (adr_reg == 0xfc){
			ack = 0;
			if ( stb == 1 & cyc == 1 & we==1){
				next_state = write;
				uart_wr_shift = (1 << 10) | (1 << 9) | (wb_in_reg << 1) | 0b0;
				tx_ff = uart_wr_shift[i];
				i++;

			}
			else if ( stb == 1 & cyc == 1 & we==0){
					next_state = read_wait;
			}
			else {
				next_state = idle;
			}

		}
		else {
			next_state = idle;
			ack = 0;
		}
		break;

	case write:
			if (baud_count == 1) {
				        tx_ff = uart_wr_shift[i];
				        baud_count = DIVISOR - 1;
				        i++;
				        if (i >= 11) {
				            next_state = idle;
				            ack = 1;
				            i = 0;
				        } else {
				            next_state = write;
				            ack = 0;
				        }
			} else
			{
						ack = 0;
				        baud_count--;
				        next_state = write;
			 }

	    break;

	case read_wait:
		 if (!rx_ff){
			 uart_rd_shift[0] = rx_ff;
			 next_state = read;}
		 else{
			 next_state = idle;
			 ack=0;}
		 break;


	 case read:
			if (baud_count ==  1){
				 uart_rd_shift[j] = rx_ff;
				 baud_count  = DIVISOR -1;
				 j++;
				 if (j >=10){
					 next_state = idle;
					 j=1;
					 ack=1;
				 }
				 else{
				 next_state = read;
				 ack=0;
				 }

			 }
			 else{
				 baud_count--;
				 next_state = read;
				 ack=0;

			 }

		 break;


	 default:
		break;



		 } // switch



	tx = tx_ff;
	state = next_state;
	uart_out = (uart_rd_shift)& 0b1111111111;



	} // function



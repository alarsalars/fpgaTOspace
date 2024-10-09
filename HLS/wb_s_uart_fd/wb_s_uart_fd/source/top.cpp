
// I use only one baudrate = 115200

#include "top.h"


void top(ap_uint<8> adr, bool we, bool cyc, bool stb,ap_uint<8> wb_in, bool rx
		,bool &tx, bool &ack,ap_uint<8> &uart_out){
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
	static int uart_count = 0;
	static unsigned int baud_count  = DIVISOR -1;


	ap_uint<8> adr_reg 		= 0;
	ap_uint<8> wb_in_reg 		= 0;
	static ap_uint<8> uart_wr_shift  = 0;
	static ap_uint<8> uart_rd_shift  = 0;
	ap_uint<8> uart_out_reg = 0;
	ap_uint<8> data_out_reg = 0;
	bool tx_ff = 1;

	wb_fsm next_state = idle;

	adr_reg = adr;
	wb_in_reg = wb_in;

	switch (state){
	case idle:
		ack = 0;
		tx_ff = 1;
		uart_count = 0;
		baud_count  = DIVISOR -1;
		if (adr_reg == 0xfc){
			if ( stb == 1 & cyc == 1 & we==1){
				next_state = write;
				uart_wr_shift = (0b1,wb_in_reg,0b0);

			}
			else if ( stb == 1 & cyc == 1 & we==0){
				next_state = read;
				uart_rd_shift = 0;
			}
			else {
				next_state = idle;
			}

		}
		else {
			next_state = idle;
		}
		break;

	 case write:
		 for (int i = 0; i < 10;i++){
			 for (int j = 0; j<DIVISOR-1;j++)
				 if (baud_count ==  1){
					 tx_ff = uart_wr_shift[i];
					 baud_count  = DIVISOR -1;
					 next_state = write;

				 }
				 else{
					 baud_count = baud_count -1;
					 next_state = write;
				 }

		 }
		 ack = 1;
		 next_state = idle;
		 tx_ff = 1;
		 baud_count  = DIVISOR -1;
		 break;

	 case read:
		 if (rx == 0){
			for (int i=0; i<8;i++){
				for (int j=0;j<DIVISOR -1;j++){
					 if (baud_count ==  1){
						 uart_rd_shift[i] = rx;
						 baud_count  = DIVISOR -1;
						 next_state = read;
					 }
					 else{
						 baud_count = baud_count -1;
						 next_state = read;
					 }
				}
			 }
			ack = 1;
			next_state = idle;
			baud_count  = DIVISOR -1;
			uart_out_reg = uart_rd_shift;
		 }
		 else{
			 next_state = read;
		 }
		 break;
	 default:
		break;



		 } // switch



	tx = tx_ff;
	state = next_state;
	uart_out = uart_out_reg;



	} // function



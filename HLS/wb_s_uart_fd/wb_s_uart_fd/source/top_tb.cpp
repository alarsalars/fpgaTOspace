
/// Taha Alars

// Wishbone uart testbench



#include "top_tb.h"



//void top(ap_uint<8> adr, bool we, bool cyc, bool stb,ap_uint<8> wb_in, bool rx
//		,bool &tx, bool &ack,ap_uint<8> &uart_out);



int main(){
	int status1, status2, status = 0;

	ap_uint<8> adr = 0xfc;
	static bool we = 0;
	static bool cyc = 0;
	static bool stb = 0;
	ap_uint<8> wb_in  = 0b01011010;
	ap_uint<11> wb_in_reg  = 0b11111001111;
	static bool rx = 1;
	static bool tx = 1;
	static ap_uint<10> tx_reg = 0;
	static bool ack;
	static ap_uint<10>	uart_out;
	static int count_rx=0;

	top(adr,we,cyc,stb,wb_in,rx,tx,ack,uart_out);
	std::cout<<"--------------"<<std::endl;
	std::cout<<"Start the write test"<<std::endl;
	std::cout<<"--------------"<<std::endl;
	std::cout<<"Load the Write signals"<<std::endl;

	we  = 1;
	cyc = 1;
	stb = 1;
	std::cout<<" adr = "<< std::hex <<adr<<" we = "<<we<<" cyc = "<<cyc<<" stb = "<<stb<<" wb_in = "<< std::hex <<wb_in<<" rx = "<<rx<<" tx = "<<tx<<" ack = "<<ack<<" uart_out = "<< std::hex <<uart_out<<std::endl;
	for (int i=0;i<10000;i++){
		top(adr,we,cyc,stb,wb_in,rx,tx,ack,uart_out);
		if(i % 55 ==0){
			std::cout<<" tx = "<<tx<<std::endl;
			tx_reg = (tx_reg >> 1) | (tx << 9);
		}
		if (ack == 1){
		break;
		}
	}
	std::cout<<" adr = "<< std::hex <<adr<<" we = "<<we<<" cyc = "<<cyc<<" stb = "<<stb<<" wb_in = "<< std::hex <<wb_in<<" rx = "<<rx<<" tx = "<<tx<<" ack = "<<ack<<" uart_out = "<< std::hex <<uart_out<<std::endl;
	we  = 0;
	cyc = 0;
	stb = 0;
	top(adr,we,cyc,stb,wb_in,rx,tx,ack,uart_out);
	std::cout<<"--------------"<<std::endl;
	std::cout<<" adr = "<< std::hex <<adr<<" we = "<<we<<" cyc = "<<cyc<<" stb = "<<stb<<" wb_in = "<< std::hex <<wb_in<<" rx = "<<rx<<" tx = "<<tx<<" ack = "<<ack<<" uart_out = "<< std::hex <<uart_out<<std::endl;
	std::cout<<"--------------"<<std::endl;
	std::cout << "TX_REG = " << tx_reg << std::endl;
	std::cout<<"--------------"<<std::endl;
	std::cout<<"--------------"<<std::endl;
	std::cout<<"END Write"<<std::endl;
	std::cout<<"--------------"<<std::endl;

	if (tx_reg == ((1 << 9) | (wb_in << 1))) {
	    status1 = 0;
	} else {
	    status1 = 1;
	    std::cout<<"Status one failed"<<std::endl;
	}

	std::cout<<" adr = "<< std::hex <<adr<<" we = "<<we<<" cyc = "<<cyc<<" stb = "<<stb<<" wb_in = "<< std::hex <<wb_in<<" rx = "<<rx<<" tx = "<<tx<<" ack = "<<ack<<" uart_out = "<< std::hex <<uart_out<<std::endl;
	std::cout<<"--------------"<<std::endl;
	std::cout<<"Start the read test"<<std::endl;
	std::cout<<"--------------"<<std::endl;
	std::cout<<"Load the read signals"<<std::endl;

	we  = 0;
	cyc = 1;
	stb = 1;
	//top(adr,we,cyc,stb,wb_in,rx,tx,ack,uart_out);
	std::cout<<" adr = "<< std::hex <<adr<<" we = "<<we<<" cyc = "<<cyc<<" stb = "<<stb<<" wb_in = "<< std::hex <<wb_in<<" rx = "<<rx<<" tx = "<<tx<<" ack = "<<ack<<" uart_out = "<< std::hex <<uart_out<<std::endl;
	std::cout<<"--------------"<<std::endl;
	std::cout<<"--------------"<<std::endl;
	std::cout<<" RX = "<<rx<<std::endl;
	std::cout<<"--------------"<<std::endl;
	std::cout<<"--------------"<<std::endl;
	rx = 0;
	top(adr,we,cyc,stb,wb_in,rx,tx,ack,uart_out);
	top(adr,we,cyc,stb,wb_in,rx,tx,ack,uart_out);
	top(adr,we,cyc,stb,wb_in,rx,tx,ack,uart_out);
	for (int i=0;i<10000;i++){
		top(adr,we,cyc,stb,wb_in,rx,tx,ack,uart_out);
		if (i % 55==0){
			std::cout<<" adr = "<< std::hex <<adr<<" we = "<<we<<" cyc = "<<cyc<<" stb = "<<stb<<" wb_in = "<< std::hex <<wb_in<<" rx = "<<rx<<" tx = "<<tx<<" ack = "<<ack<<" uart_out = "<< std::hex <<uart_out<<std::endl;
			rx = wb_in_reg.bit(count_rx);
			std::cout<<" RX = "<<rx<<std::endl;
			count_rx++;
			if (count_rx >=11){
				count_rx = 0;
				break;
			}
			else if (ack==1){
						count_rx = 0;
						break;
					}
		}


	}

	std::cout<<" adr = "<< std::hex <<adr<<" we = "<<we<<" cyc = "<<cyc<<" stb = "<<stb<<" wb_in = "<< std::hex <<wb_in<<" rx = "<<rx<<" tx = "<<tx<<" ack = "<<ack<<" uart_out = "<< std::hex <<uart_out<<std::endl;
	std::cout<<"UART_OUT = "<<std::hex <<uart_out<<std::endl;
	std::cout<<"--------------"<<std::endl;
	std::cout<<"--------------"<<std::endl;
	std::cout<<"END Read"<<std::endl;
	std::cout<<"--------------"<<std::endl;

	if (uart_out == ((wb_in_reg << 1) & 0b1111111110)) {
	    status2 = 0;
	} else {
	    status2 = 1;
	    std::cout<<"Status two failed"<<std::endl;
	}
	top(adr,we,cyc,stb,wb_in,rx,tx,ack,uart_out);
	std::cout<<" adr = "<< std::hex <<adr<<" we = "<<we<<" cyc = "<<cyc<<" stb = "<<stb<<" wb_in = "<< std::hex <<wb_in<<" rx = "<<rx<<" tx = "<<tx<<" ack = "<<ack<<" uart_out = "<< std::hex <<uart_out<<std::endl;

	we  = 0;
	cyc = 0;
	stb = 0;
	top(adr,we,cyc,stb,wb_in,rx,tx,ack,uart_out);
	std::cout<<" adr = "<< std::hex <<adr<<" we = "<<we<<" cyc = "<<cyc<<" stb = "<<stb<<" wb_in = "<< std::hex <<wb_in<<" rx = "<<rx<<" tx = "<<tx<<" ack = "<<ack<<" uart_out = "<< std::hex <<uart_out<<std::endl;

	status = status1 ^ status2;
	if (!status){
		std::cout<<"Test Pass"<<std::endl;
	}
	else{
		std::cout<<"Test Failed"<<std::endl;
	}

	std::cout<<"--------------"<<std::endl;
	std::cout<<"END Test"<<std::endl;
	std::cout<<"--------------"<<std::endl;

	return status;

} // main function

#include "top_tb.h"



//void top(ap_uint<8> adr, bool we, bool cyc, bool stb,ap_uint<8> wb_in, bool rx
//		,bool &tx, bool &ack,ap_uint<8> &uart_out);



int main(){
	int status = 0;

	ap_uint<8> adr = 0xfc;
	bool we = 0;
	bool cyc = 0;
	bool stb = 0;
	ap_uint<8> wb_in  = 0x53;
	ap_uint<9> wb_in_reg  = 0b101011010;
	bool rx = 0;
	bool tx  = 0;
	ap_uint<8> tx_reg = 0;
	bool ack = 0;
	ap_uint<8>	uart_out  = 0;

	top(adr,we,cyc,stb,wb_in,rx,tx,ack,uart_out);
	std::cout<<"--------------"<<std::endl;
	std::cout<<"Start the write test"<<std::endl;
	std::cout<<"--------------"<<std::endl;
	std::cout<<"Load the Write signals"<<std::endl;

	we  = 1;
	cyc = 1;
	stb = 1;
	for (int i=0;i<10;i++){
		top(adr,we,cyc,stb,wb_in,rx,tx,ack,uart_out);
		std::cout<<" adr = "<<adr<<" we = "<<we<<" cyc = "<<cyc<<" stb = "<<stb<<" wb_in = "<<wb_in<<" rx = "<<rx<<" tx = "<<tx<<" ack = "<<ack<<" uart_out = "<<uart_out<<std::endl;
		tx_reg[i]= tx;
	}
	std::cout<<"TX_REG = "<<tx_reg<<std::endl;
	std::cout<<"--------------"<<std::endl;
	std::cout<<"--------------"<<std::endl;
	std::cout<<"END Write"<<std::endl;
	std::cout<<"--------------"<<std::endl;
	we  = 0;
	cyc = 0;
	stb = 0;
	std::cout<<" adr = "<<adr<<" we = "<<we<<" cyc = "<<cyc<<" stb = "<<stb<<" wb_in = "<<wb_in<<" rx = "<<rx<<" tx = "<<tx<<" ack = "<<ack<<" uart_out = "<<uart_out<<std::endl;

	top(adr,we,cyc,stb,wb_in,rx,tx,ack,uart_out);
	std::cout<<"--------------"<<std::endl;
	std::cout<<"Start the read test"<<std::endl;
	std::cout<<"--------------"<<std::endl;
	std::cout<<"Load the read signals"<<std::endl;

	we  = 0;
	cyc = 1;
	stb = 1;

	for (int i=0;i<9;i++){
		top(adr,we,cyc,stb,wb_in,wb_in_reg[i],tx,ack,uart_out);
		std::cout<<" adr = "<<adr<<" we = "<<we<<" cyc = "<<cyc<<" stb = "<<stb<<" wb_in = "<<wb_in<<" rx = "<<wb_in_reg[i]<<" tx = "<<tx<<" ack = "<<ack<<" uart_out = "<<uart_out<<std::endl;
	}
	std::cout<<"UART_OUT = "<<uart_out<<std::endl;
	std::cout<<"--------------"<<std::endl;
	std::cout<<"--------------"<<std::endl;
	std::cout<<"END Read"<<std::endl;
	std::cout<<"--------------"<<std::endl;

	we  = 0;
	cyc = 0;
	stb = 0;


	std::cout<<"--------------"<<std::endl;
	std::cout<<"END Test"<<std::endl;
	std::cout<<"--------------"<<std::endl;

	return status;

} // main function

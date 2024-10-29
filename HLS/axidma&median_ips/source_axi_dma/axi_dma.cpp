      
// Taha Alars



// AXI DMA it could be wrapped in one IP but for simplicity i use it as different IP for each main function
// AXI Stream Pass by Reference (&)
// AXI Burst  Pass by Value

/*
AXI Stream (AXIS), is for continuous, unidirectional data transfer.
TVALID,TREADY,TDATA:,TLAST:

AXI Burst (AXI Memory-Mapped) is used for memory-mapped data transfers in bursts.
ARADDR/AWADDR,ARVALID/AWVALID,RDATA/WDATA,RVALID/WVALID,BRESP/RRESP

Not all the signals are used, but it is important to learn what options we have

*/


#include "axi_dma.h"
#include <ap_axi_sdata.h>
#include <complex>
#include <ap_fixed.h>
#include <ap_utils.h>





/*  
------------- the below lines are only to make the c testbench pass through, it is a bug by vitis hls solved in 2023.2
------------- but I am using 2022.2, therefore, those lines by magic make the testbench run, reference by one amd member
------------- Link: https://adaptivesupport.amd.com/s/question/0D54U00005ZLkyVSAT/i-couldnt-manage-to-run-c-simulation-with-axi-manual-burst-hlsburstmaxi-on-windows-1011?language=zh_CN&t=1729758638532
*/
#include "hls_fft.h"
struct fft_param: hls::ip_fft::params_t {
    static const unsigned stages_block_ram = 0;
    static const unsigned max_nfft = 9;
};

void dummy_fft(std::complex<ap_fixed<16, 1>> _in[512], std::complex<ap_fixed<16, 1>> _out[512]) {
    hls::fft<fft_param>(_in, _out, NULL, NULL);
}

/*  
------------- 
------------- the bug fix lines finished and now we are into the AXI TX and RX codes
------------- 
*/





/*

---------------------    AXI DMA TX ---------------------------------
---------------------    AXI DMA TX ---------------------------------
---------------------    AXI DMA TX ---------------------------------
---------------------    AXI DMA TX ---------------------------------

We have three main buses here:
1- axi lite to configure the IP from length and width
2- axi master burst to connect to the ddr via the HP bus in the zynq processor
3- axi master steeam it used to stream the recived data from the ddr to the required slave in our case we will stream to fifo and read the data back by AXI RX

TODO: Do data processing 


*/



// First we need to read by the axi master port from ddr 
void burst_read(burst_data read_mem, uint16_t length, uint16_t width, data_width array_write[N]){
    #pragma HLS INLINE // to keep the hirachy
    Outer_loop_for_length_1: for (_loop outer = 0; outer < length; outer ++ ) // size_t is used to match the to match with the system 32 bits or 64 bits
    {// we loop  for the number of 512/width
        #pragma HLS PIPELINE
       // #pragma HLS LOOP_TRIPCOUNT max = 4 min = 1 // I can limit my loop 
       // regarding the length and width entries from axiLITE but I will comment it for now 
       read_mem.read_request(width*outer,width); // we made a No. of request for the number of width*length (256 = N)

        inner_loop_for_width_1 : for ( _loop inner = 0; inner < width;inner ++){ // we loop for the number of 256/length 
            // we can also use the min max loop mimitation if we know the max and min of length and width
            #pragma HLS PIPELINE II=1
            array_write[inner] = read_mem.read(); // we try to read length * width times and store them in array with 256 memory location
        }
    
    }

}


// stream out to fifo
void axi_write(data_width array_read[N],uint16_t length, uint16_t width,stream_data &axi_out){
    #pragma HLS INLINE // to keep the hirachy
    axi_stream_bus tmp_tx_bus;
    // each byte of keep and strb are vaild therefore all the bits of keep and strb should be one, assigning -1 to them will give the same meaning of 0xffff
    tmp_tx_bus.keep = -1;
    tmp_tx_bus.strb = -1;

    outer_loop_for_length_2 : for (_loop outer = 0; outer < length; outer ++){
        #pragma HLS PIPELINE
        // we can also use the min max loop limitation if we know the max and min of length and width
        inner_loop_for_width_2 : for(_loop inner= 0 ; inner < width; inner ++){
        #pragma HLS PIPELINE II=1   
        // NOTE: we can do parsing data for handling useful information and easy data handling in Hw/Sw
        // example 
        // tmp_tx_bus.data(15,0)=array_read[inner](15,0); // etc for other as well
        // but we will do whole package transfer
        tmp_tx_bus.data = array_read[inner];
        tmp_tx_bus.last = (inner == width-1); // when finish we can assign one
        axi_out.write(tmp_tx_bus);
        }
    }
}





void AXIDMA_TX(burst_data axi_sb, uint16_t length, uint16_t width,stream_data &axi_ms){
    #pragma HLS INTERFACE mode= s_axilite port= length
    #pragma HLS INTERFACE mode= s_axilite port= width
    #pragma HLS INTERFACE mode= s_axilite port= return
    #pragma HLS INTERFACE mode= m_axi bundle=hp_zynq depth= 256 latency=0 max_write_burst_length=256 num_read_outstanding=32 num_write_outstanding=32 port=axi_sb // we define a master axi with bundle to link with the axidma_rx
    #pragma HLS INTERFACE mode= axis register_mode = both port= axi_ms // allow axi stream to read and write
    #pragma HLS DATAFLOW // to allow pipeline between different functions

    data_width stream_array[N];
    #pragma HLS STREAM type=fifo variable=stream_array  depth = 256 // define the stream_array as stream array
    burst_read(axi_sb, length, width, stream_array);
    axi_write(stream_array, length, width,axi_ms);

}


/*



---------------------    AXI DMA RX ---------------------------------
---------------------    AXI DMA RX ---------------------------------
---------------------    AXI DMA RX ---------------------------------
---------------------    AXI DMA RX ---------------------------------

We have three main buses here:
1- axi lite to configure the IP from length and width
2- axi master burst to connect to the ddr via the HP bus in the zynq processor
3- axi slave steeam it used to recive the data from the fifo and send it back to the ddr by the HP bus in the zynq processor by the axi master burst



*/


// read the stream axi from fifo 
void axi_read(stream_data &axi_ss, uint16_t length, uint16_t width, data_width stream_in_arr[N] ){
#pragma HLS INLINE
    axi_stream_bus tmp_rx_bus;
    outer_loop_for_rx_1 : for (_loop outer = 0; outer < length ; outer ++){
        #pragma HLS PIPELINE
        inner_loop_for_rx_1 : for (_loop inner = 0; inner < width; inner ++){
		#pragma HLS PIPELINE II=1
        tmp_rx_bus = axi_ss.read();
        stream_in_arr[width*outer+inner] = tmp_rx_bus.data;
        }
    }
}


// write to the ddr by the HP of zynq
void burst_write(data_width stream_pr_arr[N], uint16_t length, uint16_t width, burst_data write_mem ){
    #pragma HLS INLINE 
    outer_loop_for_rx_2 : for (_loop outer = 0; outer < length; outer ++){
        #pragma HLS PIPELINE
        write_mem.write_request(width*outer,width);
        inner_loop_for_rx_2 : for ( _loop inner = 0; inner < width ; inner ++){
        #pragma HLS PIPELINE
        write_mem.write(stream_pr_arr[width*outer+inner]);
        }
        write_mem.write_response();
    }
}



void AXIDMA_RX(stream_data &axi_stream_in, uint16_t length, uint16_t width, burst_data axi_burst_out){
    #pragma HLS INTERFACE mode= s_axilite port= length
    #pragma HLS INTERFACE mode= s_axilite port= width
    #pragma HLS INTERFACE mode= s_axilite port= return
    #pragma HLS INTERFACE mode= m_axi bundle=hp_zynq depth= 256 latency=0 max_write_burst_length=256 num_read_outstanding=32 num_write_outstanding=32 port=axi_burst_out // we define a master axi with bundle to link with the axidma_rx
    #pragma HLS INTERFACE mode= axis register_mode = both port= axi_stream_in // allow axi stream to read and write
    #pragma HLS DATAFLOW // to allow pipeline between different functions

    data_width stream_rx_array[N];
    #pragma HLS STREAM type=fifo variable=stream_rx_array  depth = 256 // define the stream_array as stream array
    axi_read(axi_stream_in, length, width, stream_rx_array);
    burst_write(stream_rx_array, length, width,axi_burst_out);

}



void AXIDMA_TX_RX(burst_data axi_burst_in,stream_data &axi_stream_out,  stream_data &axi_stream_in,burst_data axi_burst_out , uint16_t length, uint16_t width){
#pragma HLS INTERFACE mode= s_axilite port= length
#pragma HLS INTERFACE mode= s_axilite port= width
#pragma HLS INTERFACE mode= s_axilite port= return
	//TX interfaces
#pragma HLS INTERFACE mode= m_axi bundle=zynq_hp1 channel=1 depth= 256 latency=0 max_write_burst_length=256 num_read_outstanding=32 num_write_outstanding=32 port=axi_burst_in // we define a master axi with bundle and channel
#pragma HLS INTERFACE mode= axis register_mode = both port= axi_stream_out // allow axi stream to read and write
// RX Interfaces
#pragma HLS INTERFACE mode= m_axi bundle=zynq_hp2 channel=2 depth= 256 latency=0 max_write_burst_length=256 num_read_outstanding=32 num_write_outstanding=32 port=axi_burst_out // we define a master axi with bundle and channel
#pragma HLS INTERFACE mode= axis register_mode = both port= axi_stream_in // allow axi stream to read and write

#pragma HLS DATAFLOW // important: to allow pipeline between different functions,otherwise, we  will not get the pipeline between the functions and the dataflow will be on sequance between functions

data_width stream_array[N];
#pragma HLS STREAM type=fifo variable=stream_array  depth = 256 // define the stream_array as stream array
burst_read(axi_burst_in, length, width, stream_array);
axi_write(stream_array, length, width,axi_stream_out);

data_width stream_rx_array[N];
#pragma HLS STREAM type=fifo variable=stream_rx_array  depth = 256 // define the stream_array as stream array
axi_read(axi_stream_in, length, width, stream_rx_array);
burst_write(stream_rx_array, length, width,axi_burst_out);
}












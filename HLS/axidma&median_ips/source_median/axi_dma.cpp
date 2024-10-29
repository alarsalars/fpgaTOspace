      
// Taha Alars

// update with median implementation, I will consider the image data is stored in DDR first and I deal with large block of data 256 bits per clock 
// in case of rgb 24 bits direct stream process, go to the github project refrence with sobel filter, we need in this case 2 fifos to access 9 different pixels per call

// AXI DMA it could be wrapped in one IP but for simplicity i use it as different IP for each main function
// AXI Stream Pass by Reference (&)
// AXI Burst  Pass by Value

/*
AXI Stream (AXIS), is for continuous, unidirectional data transfer.
TVALID,TREADY,TDATA:,TLAST:

AXI Burst (AXI Memory-Mapped) is used for memory-mapped data transfers in bursts.
ARADDR/AWADDR,ARVALID/AWVALID,RDATA/WDATA,RVALID/WVALID,BRESP/RRESP

Not all the signals of AXI are used, but it is important to learn what options we have

*/


#include "axi_dma.h"
#include <ap_axi_sdata.h>
#include <complex>
#include <ap_fixed.h>
#include <ap_utils.h>


#define DATA_WIDTH  256
#define OUTPUT_WIDTH 216 // 9 pixels of 24 bits
#define col 9
#define row 256

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
------------- the vitis bug fix lines finished and now we are into the AXI TX and RX codes
------------- 
*/

void rgb_to_gray(bool done_in, color_width data[27],color_width grayscale[9]  ,bool &done_out ){
    #pragma HLS PIPELINE
    //ap_uint<8> grayscale[9];
    done_out = 0;
    if (done_in ){
        for (size_t i = 0; i < 9; i++){
        #pragma HLS UNROLL
        grayscale[i] =  ((data[i*3]>>2) + (data[i*3]>>5)) + 
                        ((data[i*3+1]>>1) + (data[i*3+1]>>4)) + 
                        ((data[i*3+2]>>4) + (data[i*3+2]>>5)) ;
        }
        done_out = 1;
    }
}





void padding(color_width grayscale_in[9],int icol, int irow,color_width grayscale_out[9], bool &done_pad ){
    done_pad= 0;
    if (icol == 0 &&  irow == 0){
        grayscale_out[0] = 0;
        grayscale_out[1] = 0;
        grayscale_out[2] = 0;
        grayscale_out[3] = 0;
        grayscale_out[4] = grayscale_in[4];
        grayscale_out[5] = grayscale_in[5];
        grayscale_out[6] = 0;
        grayscale_out[7] = grayscale_in[7];
        grayscale_out[8] = grayscale_in[8];
    }
    else if (irow == 0 && icol > 0 && icol < col-1){
        grayscale_out[0] = 0;
        grayscale_out[1] = 0;
        grayscale_out[2] = 0;
        grayscale_out[3] = grayscale_in[3];
        grayscale_out[4] = grayscale_in[4];
        grayscale_out[5] = grayscale_in[5];
        grayscale_out[6] = grayscale_in[6];
        grayscale_out[7] = grayscale_in[7];
        grayscale_out[8] = grayscale_in[8];
    }
    else if (irow == 0 && icol == col-1){
        grayscale_out[0] = 0;
        grayscale_out[1] = 0;
        grayscale_out[2] = 0;
        grayscale_out[3] = grayscale_in[3];
        grayscale_out[4] = grayscale_in[4];
        grayscale_out[5] = 0;
        grayscale_out[6] = grayscale_in[6];
        grayscale_out[7] = grayscale_in[7];
        grayscale_out[8] = 0;       
    }
    else if (icol == 0 && irow > 0 && irow < row-1){
        grayscale_out[0] = 0;
        grayscale_out[1] = grayscale_in[1];
        grayscale_out[2] = grayscale_in[2];
        grayscale_out[3] = 0;
        grayscale_out[4] = grayscale_in[4];
        grayscale_out[5] = grayscale_in[5];
        grayscale_out[6] = 0;
        grayscale_out[7] = grayscale_in[7];
        grayscale_out[8] = grayscale_in[8];    
    }
    else if (icol > 0 && icol < col-1 && irow > 0 && irow < row-1){
        grayscale_out[0] = grayscale_in[0];
        grayscale_out[1] = grayscale_in[1];
        grayscale_out[2] = grayscale_in[2];
        grayscale_out[3] = grayscale_in[3];
        grayscale_out[4] = grayscale_in[4];
        grayscale_out[5] = grayscale_in[5];
        grayscale_out[6] = grayscale_in[6];
        grayscale_out[7] = grayscale_in[7];
        grayscale_out[8] = grayscale_in[8];        
    }
    else if (icol == col -1  && irow > 0 && irow < row-1){
        grayscale_out[0] = grayscale_in[0];
        grayscale_out[1] = grayscale_in[1];
        grayscale_out[2] = 0;
        grayscale_out[3] = grayscale_in[3];
        grayscale_out[4] = grayscale_in[4];
        grayscale_out[5] = 0;
        grayscale_out[6] = grayscale_in[6];
        grayscale_out[7] = grayscale_in[7];
        grayscale_out[8] = grayscale_in[8]; 
    }
    else if (icol == 0 && irow == row-1){
        grayscale_out[0] = 0;
        grayscale_out[1] = grayscale_in[1];
        grayscale_out[2] = grayscale_in[2];
        grayscale_out[3] = 0;
        grayscale_out[4] = grayscale_in[4];
        grayscale_out[5] = grayscale_in[5];
        grayscale_out[6] = 0;
        grayscale_out[7] = 0;
        grayscale_out[8] = 0;        
    } 
    else if (irow == row -1  && icol > 0 && icol < col-1){
        grayscale_out[0] = grayscale_in[0];
        grayscale_out[1] = grayscale_in[1];
        grayscale_out[2] = grayscale_in[2];
        grayscale_out[3] = grayscale_in[3];
        grayscale_out[4] = grayscale_in[4];
        grayscale_out[5] = grayscale_in[5];
        grayscale_out[6] = 0;
        grayscale_out[7] = 0;
        grayscale_out[8] = 0;           
    }   
    else if (icol == col -1  && irow == row-1){
        grayscale_out[0] = grayscale_in[0];
        grayscale_out[1] = grayscale_in[1];
        grayscale_out[2] = 0;
        grayscale_out[3] = grayscale_in[3];
        grayscale_out[4] = grayscale_in[4];
        grayscale_out[5] = 0;
        grayscale_out[6] = 0;
        grayscale_out[7] = 0;
        grayscale_out[8] = 0;          
    }  
    done_pad = 1;
}


void median_bubble( color_width grayscale_padd[9], ap_uint<8> &median_value ){
    color_width grayscale_sort[9] ;
    #pragma HLS PIPELINE
    for (int i = 0; i < 9 ; i++){
        grayscale_sort[i] = grayscale_padd[i];
    }
    for (int i = 0; i < 9 ; i++){
        for (int j = 0; j < 8 - i; j++){
            #pragma HLS UNROLL
            if (grayscale_sort[j]  > grayscale_sort[j+1] ){
                ap_uint<8> tmp = grayscale_sort[j] ;
                grayscale_sort[j] = grayscale_sort[j+1];
                grayscale_sort[j+1] = tmp;
            }
        }
    }

    median_value = grayscale_sort[4];
    //for (int i = 0; i < 9 ; i++){
    //    if (i % 4 == 0){
    //        grayscale_median[i] = median_value;
    //    }
    //    else{
    //        grayscale_median[i] = grayscale_sort[i];
    //    }
    //    
    //}

}

void gray_to_rgb(color_width grayscale_median[9], data_width &output_row, bool &done_rgb    ){
    #pragma HLS PIPELINE
        data_width tmp_rgb = 0;
        done_rgb = 0;

    for ( int i = 0; i < 9; i++){
        #pragma HLS UNROLL
        tmp_rgb(i * 24 + 7, i * 24)         = grayscale_median[i]; // red channel
        tmp_rgb(i * 24 + 15, i * 24 + 8)    = grayscale_median[i]; // green channel
        tmp_rgb(i * 24 + 23, i * 24 + 16)   = grayscale_median[i]; // blue channel
    }
    output_row = tmp_rgb;
    done_rgb = 1;
}

void median_filter(stream_data &axi_ss_median,uint16_t length, uint16_t width,stream_data &median_filter ){
    #pragma HLS INTERFACE mode= s_axilite port= length
    #pragma HLS INTERFACE mode= s_axilite port= width
    #pragma HLS INTERFACE mode= s_axilite port= return
    #pragma HLS INTERFACE mode= axis register_mode = both port= median_filter
    #pragma HLS INTERFACE mode= axis register_mode = both port= axi_ss_median
    #pragma HLS PIPELINE
    axi_stream_bus tmp_fifo;
    data_width data_pr;
    data_width median_arr[N];
    axi_stream_bus tmp_median_bus;
    data_width stream_median_arr[N];
    color_width grayscale[9];
    color_width grayscale_padding[9];
    ap_uint<8> d[27], grayscale_median[9];

    bool done_in = 0, done_out = 0, done_pad = 0;
    bool done_rgb = 0;  
    //fifo_stream fifo_buffer("fifo_buffer");
    // Loop 1: Reading data into FIFO buffer
    outer_loop_for_fifo_1 : for (_loop outer = 0; outer < length ; outer ++){
        #pragma HLS PIPELINE
        inner_loop_for_fifo_1 : for (_loop inner = 0; inner < width ; inner ++){
        //    if (!axi_ss_median.empty()){
        tmp_median_bus = axi_ss_median.read(); // read the axi signals
        stream_median_arr[width*outer+inner] =  tmp_median_bus.data; // read only the data from the tmp reg  
        //    }
        }
    }
    // Loop 2: Processing each pixel row
    outer_loop_for_fifo_2 : for (_loop irow = 0; irow < length*width ; irow ++){
        #pragma HLS PIPELINE
        data_pr = stream_median_arr[irow];
        //if (!fifo_buffer.empty()){
            
            // using only the 216 bits of 256, the rest will be ignored
            // it is not a good practice as it cause memory loss but it is just to match the current example
                for (_loop i = 0; i < 27; i++){
                    #pragma HLS UNROLL
                    d[i]     = data_pr(i*8+7,i*8);
                    //d[i*3+1]   = data_pr(i*8*3+7,i*3*8+8);
                    //d[i*3+2]   = data_pr(i*8*3+7,i*3*8+16);
                }
            // RGB to grayscale conversion
            done_in = 1;
            rgb_to_gray(done_in,d, grayscale, done_out );
            done_in = 0;
                if (done_out){
                    // Padding and median filtering
                    done_pad = 0;
                    for (_loop icol = 0; icol < 9; icol++){
                        #pragma HLS UNROLL
                        
                        padding(grayscale, icol, irow,grayscale_padding, done_pad);
                        if (done_pad){
                            median_bubble( grayscale_padding, grayscale_median[icol]);
                            done_pad = 0;
                        }
                    }

                }
        //}

        gray_to_rgb(grayscale_median, median_arr[irow],done_rgb);
    }
    if (done_rgb){
        axi_write(median_arr, length, width,median_filter);
    }
}





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
        tmp_rx_bus = axi_ss.read(); // read the axi signals 
        stream_in_arr[width*outer+inner] = tmp_rx_bus.data; // read only the data from the tmp reg
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

















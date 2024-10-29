#pragma once

#include "ap_int.h"
#include <ap_fixed.h>
#include <hls_stream.h>
#include <ap_axi_sdata.h>
#include <stdint.h>
#include "hls_burst_maxi.h"
#include <ap_utils.h>



#define N       256   // 64*4                                      // Max nummber of read write burst, it has to be pow of 2 also not more than 256


typedef ap_uint<256>                            data_width;   // data package width per read/write
typedef hls::axis<data_width,0,0,0>             axi_stream_bus; // define the axi data stream bus stream data hls::axis<data_t, tlast, tid, tdest, tkeep>;
typedef hls::stream<axi_stream_bus>             stream_data;    //  FIFO-based stream based on the axi
typedef hls::burst_maxi<data_width>             burst_data;    // burst master axi send recive busrt data
typedef size_t                                  _loop;
typedef hls::stream<data_width>                 fifo_stream;
typedef ap_uint<8>                              color_width;


void AXIDMA_RX(stream_data &axi_stream_in, uint16_t length, uint16_t width, burst_data axi_burst_out);
void AXIDMA_TX(burst_data axi_sb, uint16_t length, uint16_t width,stream_data &axi_ms);
void AXIDMA_TX_RX(burst_data axi_burst_in,stream_data &axi_stream_out,  stream_data &axi_stream_in,burst_data axi_burst_out , uint16_t length, uint16_t width);
void axi_write(data_width array_read[N],uint16_t length, uint16_t width,stream_data &axi_out);
void median_filter(stream_data &axi_ss_median,uint16_t length, uint16_t width,stream_data &median_filter );

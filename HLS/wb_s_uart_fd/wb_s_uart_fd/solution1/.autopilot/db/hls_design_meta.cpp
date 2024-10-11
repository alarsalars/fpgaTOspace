#include "hls_design_meta.h"
const Port_Property HLS_Design_Meta::port_props[]={
	Port_Property("ap_clk", 1, hls_in, -1, "", "", 1),
	Port_Property("ap_rst", 1, hls_in, -1, "", "", 1),
	Port_Property("adr", 8, hls_in, 0, "ap_none", "in_data", 1),
	Port_Property("we", 1, hls_in, 1, "ap_none", "in_data", 1),
	Port_Property("cyc", 1, hls_in, 2, "ap_none", "in_data", 1),
	Port_Property("stb", 1, hls_in, 3, "ap_none", "in_data", 1),
	Port_Property("wb_in", 8, hls_in, 4, "ap_none", "in_data", 1),
	Port_Property("rx", 1, hls_in, 5, "ap_none", "in_data", 1),
	Port_Property("tx", 1, hls_out, 6, "ap_none", "out_data", 1),
	Port_Property("ack", 1, hls_out, 7, "ap_none", "out_data", 1),
	Port_Property("uart_out", 10, hls_out, 8, "ap_none", "out_data", 1),
};
const char* HLS_Design_Meta::dut_name = "top";

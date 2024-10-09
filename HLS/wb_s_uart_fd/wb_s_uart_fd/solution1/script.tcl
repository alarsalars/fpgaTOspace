############################################################
## This file is generated automatically by Vitis HLS.
## Please DO NOT edit it.
## Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
############################################################
open_project wb_s_uart_fd
set_top top
add_files wb_s_uart_fd/source/top.cpp
add_files wb_s_uart_fd/source/top.h
add_files -tb wb_s_uart_fd/source/top_tb.cpp
add_files -tb wb_s_uart_fd/source/top_tb.h
open_solution "solution1" -flow_target vivado
set_part {xcvu11p-flga2577-1-e}
create_clock -period 10 -name default
#source "./wb_s_uart_fd/solution1/directives.tcl"
csim_design
csynth_design
cosim_design
export_design -format ip_catalog

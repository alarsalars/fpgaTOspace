############################################################
## This file is generated automatically by Vitis HLS.
## Please DO NOT edit it.
## Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
############################################################
open_project wb_s_uart_fd
set_top top
add_files wb_s_uart_fd/source/top.cpp
add_files wb_s_uart_fd/source/top.h
add_files -tb wb_s_uart_fd/source/top_tb.cpp -cflags "-Wno-unknown-pragmas -Wno-unknown-pragmas -Wno-unknown-pragmas -Wno-unknown-pragmas" -csimflags "-Wno-unknown-pragmas"
add_files -tb wb_s_uart_fd/source/top_tb.h -cflags "-Wno-unknown-pragmas -Wno-unknown-pragmas -Wno-unknown-pragmas -Wno-unknown-pragmas" -csimflags "-Wno-unknown-pragmas"
open_solution "solution1" -flow_target vivado
set_part {xczu9eg-ffvb1156-2-e}
create_clock -period 10 -name default
config_export -description wb_s_uart -display_name wb_uart -format ip_catalog -output C:/Users/TAlars/Documents/vivado_projects_tests/fpgaTOspace/HLS/wb_s_uart_fd/wb_s_uart_fd/ip_wb_uart -rtl verilog
source "./wb_s_uart_fd/solution1/directives.tcl"
csim_design
csynth_design
cosim_design -trace_level all
export_design -rtl verilog -format ip_catalog -output C:/Users/TAlars/Documents/vivado_projects_tests/fpgaTOspace/HLS/wb_s_uart_fd/wb_s_uart_fd/ip_wb_uart

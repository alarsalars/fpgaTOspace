############################################################
## This file is generated automatically by Vitis HLS.
## Please DO NOT edit it.
## Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
############################################################
open_project prbs_7_taps
set_top main_func
add_files prbs_7_taps/source/prbs.cpp
add_files prbs_7_taps/source/prbs.h
add_files -tb prbs_7_taps/source/prbs_tb.cpp -cflags "-Wno-unknown-pragmas -Wno-unknown-pragmas" -csimflags "-Wno-unknown-pragmas"
add_files -tb prbs_7_taps/source/prbs_tb.h -cflags "-Wno-unknown-pragmas -Wno-unknown-pragmas" -csimflags "-Wno-unknown-pragmas"
open_solution "solution1" -flow_target vivado
set_part {xczu9eg-ffvb1156-2-e}
create_clock -period 10 -name default
config_export -display_name prbs7 -format ip_catalog -output C:/Users/TAlars/Documents/vivado_projects_tests/fpgaTOspace/HLS/prbs_7_taps/ip_prbs -rtl verilog
source "./prbs_7_taps/solution1/directives.tcl"
csim_design
csynth_design
cosim_design -trace_level all
export_design -rtl verilog -format ip_catalog -output C:/Users/TAlars/Documents/vivado_projects_tests/fpgaTOspace/HLS/prbs_7_taps/ip_prbs

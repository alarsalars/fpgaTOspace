//Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
//--------------------------------------------------------------------------------
//Tool Version: Vivado v.2022.2 (win64) Build 3671981 Fri Oct 14 05:00:03 MDT 2022
//Date        : Fri May 24 19:10:30 2024
//Host        : IT05676 running 64-bit major release  (build 9200)
//Command     : generate_target design_1_wrapper.bd
//Design      : design_1_wrapper
//Purpose     : IP block netlist
//--------------------------------------------------------------------------------
`timescale 1 ps / 1 ps

module design_1_wrapper
   (rgb1_led_3bits_tri_o);
  output [2:0]rgb1_led_3bits_tri_o;

  wire [2:0]rgb1_led_3bits_tri_o;

  design_1 design_1_i
       (.rgb1_led_3bits_tri_o(rgb1_led_3bits_tri_o));
endmodule

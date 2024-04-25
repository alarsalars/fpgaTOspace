//Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
//--------------------------------------------------------------------------------
//Tool Version: Vivado v.2022.2 (lin64) Build 3671981 Fri Oct 14 04:59:54 MDT 2022
//Date        : Tue Mar 19 09:16:27 2024
//Host        : IT05676 running 64-bit Ubuntu 22.04.4 LTS
//Command     : generate_target design_1_wrapper.bd
//Design      : design_1_wrapper
//Purpose     : IP block netlist
//--------------------------------------------------------------------------------
`timescale 1 ps / 1 ps

module design_1_wrapper
   (rgb1_led_3bits_tri_o,
    rgb2_led_3bits_tri_o);
  output [2:0]rgb1_led_3bits_tri_o;
  output [2:0]rgb2_led_3bits_tri_o;

  wire [2:0]rgb1_led_3bits_tri_o;
  wire [2:0]rgb2_led_3bits_tri_o;

  design_1 design_1_i
       (.rgb1_led_3bits_tri_o(rgb1_led_3bits_tri_o),
        .rgb2_led_3bits_tri_o(rgb2_led_3bits_tri_o));
endmodule

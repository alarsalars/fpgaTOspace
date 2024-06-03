//Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
//--------------------------------------------------------------------------------
//Command: generate_target bd_f60c_wrapper.bd
//Design : bd_f60c_wrapper
//Purpose: IP block netlist
//--------------------------------------------------------------------------------
`timescale 1 ps / 1 ps

module bd_f60c_wrapper
   (SLOT_0_BRAM_addr,
    SLOT_0_BRAM_clk,
    SLOT_0_BRAM_din,
    SLOT_0_BRAM_dout,
    SLOT_0_BRAM_en,
    SLOT_0_BRAM_rst,
    SLOT_0_BRAM_we,
    SLOT_1_AXI_araddr,
    SLOT_1_AXI_arburst,
    SLOT_1_AXI_arcache,
    SLOT_1_AXI_arlen,
    SLOT_1_AXI_arlock,
    SLOT_1_AXI_arprot,
    SLOT_1_AXI_arqos,
    SLOT_1_AXI_arready,
    SLOT_1_AXI_arsize,
    SLOT_1_AXI_aruser,
    SLOT_1_AXI_arvalid,
    SLOT_1_AXI_awaddr,
    SLOT_1_AXI_awburst,
    SLOT_1_AXI_awcache,
    SLOT_1_AXI_awlen,
    SLOT_1_AXI_awlock,
    SLOT_1_AXI_awprot,
    SLOT_1_AXI_awqos,
    SLOT_1_AXI_awready,
    SLOT_1_AXI_awsize,
    SLOT_1_AXI_awuser,
    SLOT_1_AXI_awvalid,
    SLOT_1_AXI_bready,
    SLOT_1_AXI_bresp,
    SLOT_1_AXI_bvalid,
    SLOT_1_AXI_rdata,
    SLOT_1_AXI_rlast,
    SLOT_1_AXI_rready,
    SLOT_1_AXI_rresp,
    SLOT_1_AXI_rvalid,
    SLOT_1_AXI_wdata,
    SLOT_1_AXI_wlast,
    SLOT_1_AXI_wready,
    SLOT_1_AXI_wstrb,
    SLOT_1_AXI_wvalid,
    clk,
    resetn);
  input [12:0]SLOT_0_BRAM_addr;
  input SLOT_0_BRAM_clk;
  input [31:0]SLOT_0_BRAM_din;
  input [31:0]SLOT_0_BRAM_dout;
  input SLOT_0_BRAM_en;
  input SLOT_0_BRAM_rst;
  input [3:0]SLOT_0_BRAM_we;
  input [12:0]SLOT_1_AXI_araddr;
  input [1:0]SLOT_1_AXI_arburst;
  input [3:0]SLOT_1_AXI_arcache;
  input [7:0]SLOT_1_AXI_arlen;
  input [0:0]SLOT_1_AXI_arlock;
  input [2:0]SLOT_1_AXI_arprot;
  input [3:0]SLOT_1_AXI_arqos;
  input SLOT_1_AXI_arready;
  input [2:0]SLOT_1_AXI_arsize;
  input [15:0]SLOT_1_AXI_aruser;
  input SLOT_1_AXI_arvalid;
  input [12:0]SLOT_1_AXI_awaddr;
  input [1:0]SLOT_1_AXI_awburst;
  input [3:0]SLOT_1_AXI_awcache;
  input [7:0]SLOT_1_AXI_awlen;
  input [0:0]SLOT_1_AXI_awlock;
  input [2:0]SLOT_1_AXI_awprot;
  input [3:0]SLOT_1_AXI_awqos;
  input SLOT_1_AXI_awready;
  input [2:0]SLOT_1_AXI_awsize;
  input [15:0]SLOT_1_AXI_awuser;
  input SLOT_1_AXI_awvalid;
  input SLOT_1_AXI_bready;
  input [1:0]SLOT_1_AXI_bresp;
  input SLOT_1_AXI_bvalid;
  input [31:0]SLOT_1_AXI_rdata;
  input SLOT_1_AXI_rlast;
  input SLOT_1_AXI_rready;
  input [1:0]SLOT_1_AXI_rresp;
  input SLOT_1_AXI_rvalid;
  input [31:0]SLOT_1_AXI_wdata;
  input SLOT_1_AXI_wlast;
  input SLOT_1_AXI_wready;
  input [3:0]SLOT_1_AXI_wstrb;
  input SLOT_1_AXI_wvalid;
  input clk;
  input resetn;

  wire [12:0]SLOT_0_BRAM_addr;
  wire SLOT_0_BRAM_clk;
  wire [31:0]SLOT_0_BRAM_din;
  wire [31:0]SLOT_0_BRAM_dout;
  wire SLOT_0_BRAM_en;
  wire SLOT_0_BRAM_rst;
  wire [3:0]SLOT_0_BRAM_we;
  wire [12:0]SLOT_1_AXI_araddr;
  wire [1:0]SLOT_1_AXI_arburst;
  wire [3:0]SLOT_1_AXI_arcache;
  wire [7:0]SLOT_1_AXI_arlen;
  wire [0:0]SLOT_1_AXI_arlock;
  wire [2:0]SLOT_1_AXI_arprot;
  wire [3:0]SLOT_1_AXI_arqos;
  wire SLOT_1_AXI_arready;
  wire [2:0]SLOT_1_AXI_arsize;
  wire [15:0]SLOT_1_AXI_aruser;
  wire SLOT_1_AXI_arvalid;
  wire [12:0]SLOT_1_AXI_awaddr;
  wire [1:0]SLOT_1_AXI_awburst;
  wire [3:0]SLOT_1_AXI_awcache;
  wire [7:0]SLOT_1_AXI_awlen;
  wire [0:0]SLOT_1_AXI_awlock;
  wire [2:0]SLOT_1_AXI_awprot;
  wire [3:0]SLOT_1_AXI_awqos;
  wire SLOT_1_AXI_awready;
  wire [2:0]SLOT_1_AXI_awsize;
  wire [15:0]SLOT_1_AXI_awuser;
  wire SLOT_1_AXI_awvalid;
  wire SLOT_1_AXI_bready;
  wire [1:0]SLOT_1_AXI_bresp;
  wire SLOT_1_AXI_bvalid;
  wire [31:0]SLOT_1_AXI_rdata;
  wire SLOT_1_AXI_rlast;
  wire SLOT_1_AXI_rready;
  wire [1:0]SLOT_1_AXI_rresp;
  wire SLOT_1_AXI_rvalid;
  wire [31:0]SLOT_1_AXI_wdata;
  wire SLOT_1_AXI_wlast;
  wire SLOT_1_AXI_wready;
  wire [3:0]SLOT_1_AXI_wstrb;
  wire SLOT_1_AXI_wvalid;
  wire clk;
  wire resetn;

  bd_f60c bd_f60c_i
       (.SLOT_0_BRAM_addr(SLOT_0_BRAM_addr),
        .SLOT_0_BRAM_clk(SLOT_0_BRAM_clk),
        .SLOT_0_BRAM_din(SLOT_0_BRAM_din),
        .SLOT_0_BRAM_dout(SLOT_0_BRAM_dout),
        .SLOT_0_BRAM_en(SLOT_0_BRAM_en),
        .SLOT_0_BRAM_rst(SLOT_0_BRAM_rst),
        .SLOT_0_BRAM_we(SLOT_0_BRAM_we),
        .SLOT_1_AXI_araddr(SLOT_1_AXI_araddr),
        .SLOT_1_AXI_arburst(SLOT_1_AXI_arburst),
        .SLOT_1_AXI_arcache(SLOT_1_AXI_arcache),
        .SLOT_1_AXI_arlen(SLOT_1_AXI_arlen),
        .SLOT_1_AXI_arlock(SLOT_1_AXI_arlock),
        .SLOT_1_AXI_arprot(SLOT_1_AXI_arprot),
        .SLOT_1_AXI_arqos(SLOT_1_AXI_arqos),
        .SLOT_1_AXI_arready(SLOT_1_AXI_arready),
        .SLOT_1_AXI_arsize(SLOT_1_AXI_arsize),
        .SLOT_1_AXI_aruser(SLOT_1_AXI_aruser),
        .SLOT_1_AXI_arvalid(SLOT_1_AXI_arvalid),
        .SLOT_1_AXI_awaddr(SLOT_1_AXI_awaddr),
        .SLOT_1_AXI_awburst(SLOT_1_AXI_awburst),
        .SLOT_1_AXI_awcache(SLOT_1_AXI_awcache),
        .SLOT_1_AXI_awlen(SLOT_1_AXI_awlen),
        .SLOT_1_AXI_awlock(SLOT_1_AXI_awlock),
        .SLOT_1_AXI_awprot(SLOT_1_AXI_awprot),
        .SLOT_1_AXI_awqos(SLOT_1_AXI_awqos),
        .SLOT_1_AXI_awready(SLOT_1_AXI_awready),
        .SLOT_1_AXI_awsize(SLOT_1_AXI_awsize),
        .SLOT_1_AXI_awuser(SLOT_1_AXI_awuser),
        .SLOT_1_AXI_awvalid(SLOT_1_AXI_awvalid),
        .SLOT_1_AXI_bready(SLOT_1_AXI_bready),
        .SLOT_1_AXI_bresp(SLOT_1_AXI_bresp),
        .SLOT_1_AXI_bvalid(SLOT_1_AXI_bvalid),
        .SLOT_1_AXI_rdata(SLOT_1_AXI_rdata),
        .SLOT_1_AXI_rlast(SLOT_1_AXI_rlast),
        .SLOT_1_AXI_rready(SLOT_1_AXI_rready),
        .SLOT_1_AXI_rresp(SLOT_1_AXI_rresp),
        .SLOT_1_AXI_rvalid(SLOT_1_AXI_rvalid),
        .SLOT_1_AXI_wdata(SLOT_1_AXI_wdata),
        .SLOT_1_AXI_wlast(SLOT_1_AXI_wlast),
        .SLOT_1_AXI_wready(SLOT_1_AXI_wready),
        .SLOT_1_AXI_wstrb(SLOT_1_AXI_wstrb),
        .SLOT_1_AXI_wvalid(SLOT_1_AXI_wvalid),
        .clk(clk),
        .resetn(resetn));
endmodule

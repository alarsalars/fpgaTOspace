// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2022.2 (win64) Build 3671981 Fri Oct 14 05:00:03 MDT 2022
// Date        : Mon Jun  3 13:42:50 2024
// Host        : IT05676 running 64-bit major release  (build 9200)
// Command     : write_verilog -force -mode funcsim
//               c:/Users/TAlars/Documents/vivado_projects_tests/vipix/bram/bram/bram.gen/sources_1/bd/design_1/ip/design_1_axi_bram_ctrl_0_bram_0/design_1_axi_bram_ctrl_0_bram_0_sim_netlist.v
// Design      : design_1_axi_bram_ctrl_0_bram_0
// Purpose     : This verilog netlist is a functional simulation representation of the design and should not be modified
//               or synthesized. This netlist cannot be used for SDF annotated simulation.
// Device      : xczu1cg-sbva484-1-e
// --------------------------------------------------------------------------------
`timescale 1 ps / 1 ps

(* CHECK_LICENSE_TYPE = "design_1_axi_bram_ctrl_0_bram_0,blk_mem_gen_v8_4_5,{}" *) (* downgradeipidentifiedwarnings = "yes" *) (* x_core_info = "blk_mem_gen_v8_4_5,Vivado 2022.2" *) 
(* NotValidForBitStream *)
module design_1_axi_bram_ctrl_0_bram_0
   (clka,
    rsta,
    ena,
    wea,
    addra,
    dina,
    douta,
    rsta_busy);
  (* x_interface_info = "xilinx.com:interface:bram:1.0 BRAM_PORTA CLK" *) (* x_interface_parameter = "XIL_INTERFACENAME BRAM_PORTA, MEM_SIZE 8192, MEM_WIDTH 32, MEM_ECC NONE, MASTER_TYPE BRAM_CTRL, READ_WRITE_MODE READ_WRITE, READ_LATENCY 1" *) input clka;
  (* x_interface_info = "xilinx.com:interface:bram:1.0 BRAM_PORTA RST" *) input rsta;
  (* x_interface_info = "xilinx.com:interface:bram:1.0 BRAM_PORTA EN" *) input ena;
  (* x_interface_info = "xilinx.com:interface:bram:1.0 BRAM_PORTA WE" *) input [3:0]wea;
  (* x_interface_info = "xilinx.com:interface:bram:1.0 BRAM_PORTA ADDR" *) input [31:0]addra;
  (* x_interface_info = "xilinx.com:interface:bram:1.0 BRAM_PORTA DIN" *) input [31:0]dina;
  (* x_interface_info = "xilinx.com:interface:bram:1.0 BRAM_PORTA DOUT" *) output [31:0]douta;
  output rsta_busy;

  wire [31:0]addra;
  wire clka;
  wire [31:0]dina;
  wire [31:0]douta;
  wire ena;
  wire rsta;
  wire rsta_busy;
  wire [3:0]wea;
  wire NLW_U0_dbiterr_UNCONNECTED;
  wire NLW_U0_rstb_busy_UNCONNECTED;
  wire NLW_U0_s_axi_arready_UNCONNECTED;
  wire NLW_U0_s_axi_awready_UNCONNECTED;
  wire NLW_U0_s_axi_bvalid_UNCONNECTED;
  wire NLW_U0_s_axi_dbiterr_UNCONNECTED;
  wire NLW_U0_s_axi_rlast_UNCONNECTED;
  wire NLW_U0_s_axi_rvalid_UNCONNECTED;
  wire NLW_U0_s_axi_sbiterr_UNCONNECTED;
  wire NLW_U0_s_axi_wready_UNCONNECTED;
  wire NLW_U0_sbiterr_UNCONNECTED;
  wire [31:0]NLW_U0_doutb_UNCONNECTED;
  wire [31:0]NLW_U0_rdaddrecc_UNCONNECTED;
  wire [3:0]NLW_U0_s_axi_bid_UNCONNECTED;
  wire [1:0]NLW_U0_s_axi_bresp_UNCONNECTED;
  wire [31:0]NLW_U0_s_axi_rdaddrecc_UNCONNECTED;
  wire [31:0]NLW_U0_s_axi_rdata_UNCONNECTED;
  wire [3:0]NLW_U0_s_axi_rid_UNCONNECTED;
  wire [1:0]NLW_U0_s_axi_rresp_UNCONNECTED;

  (* C_ADDRA_WIDTH = "32" *) 
  (* C_ADDRB_WIDTH = "32" *) 
  (* C_ALGORITHM = "1" *) 
  (* C_AXI_ID_WIDTH = "4" *) 
  (* C_AXI_SLAVE_TYPE = "0" *) 
  (* C_AXI_TYPE = "1" *) 
  (* C_BYTE_SIZE = "8" *) 
  (* C_COMMON_CLK = "0" *) 
  (* C_COUNT_18K_BRAM = "0" *) 
  (* C_COUNT_36K_BRAM = "2" *) 
  (* C_CTRL_ECC_ALGO = "NONE" *) 
  (* C_DEFAULT_DATA = "0" *) 
  (* C_DISABLE_WARN_BHV_COLL = "0" *) 
  (* C_DISABLE_WARN_BHV_RANGE = "0" *) 
  (* C_ELABORATION_DIR = "./" *) 
  (* C_ENABLE_32BIT_ADDRESS = "1" *) 
  (* C_EN_DEEPSLEEP_PIN = "0" *) 
  (* C_EN_ECC_PIPE = "0" *) 
  (* C_EN_RDADDRA_CHG = "0" *) 
  (* C_EN_RDADDRB_CHG = "0" *) 
  (* C_EN_SAFETY_CKT = "1" *) 
  (* C_EN_SHUTDOWN_PIN = "0" *) 
  (* C_EN_SLEEP_PIN = "0" *) 
  (* C_EST_POWER_SUMMARY = "Estimated Power for IP     :     3.867232 mW" *) 
  (* C_FAMILY = "zynquplus" *) 
  (* C_HAS_AXI_ID = "0" *) 
  (* C_HAS_ENA = "1" *) 
  (* C_HAS_ENB = "0" *) 
  (* C_HAS_INJECTERR = "0" *) 
  (* C_HAS_MEM_OUTPUT_REGS_A = "0" *) 
  (* C_HAS_MEM_OUTPUT_REGS_B = "0" *) 
  (* C_HAS_MUX_OUTPUT_REGS_A = "0" *) 
  (* C_HAS_MUX_OUTPUT_REGS_B = "0" *) 
  (* C_HAS_REGCEA = "0" *) 
  (* C_HAS_REGCEB = "0" *) 
  (* C_HAS_RSTA = "1" *) 
  (* C_HAS_RSTB = "0" *) 
  (* C_HAS_SOFTECC_INPUT_REGS_A = "0" *) 
  (* C_HAS_SOFTECC_OUTPUT_REGS_B = "0" *) 
  (* C_INITA_VAL = "0" *) 
  (* C_INITB_VAL = "0" *) 
  (* C_INIT_FILE = "NONE" *) 
  (* C_INIT_FILE_NAME = "no_coe_file_loaded" *) 
  (* C_INTERFACE_TYPE = "0" *) 
  (* C_LOAD_INIT_FILE = "0" *) 
  (* C_MEM_TYPE = "0" *) 
  (* C_MUX_PIPELINE_STAGES = "0" *) 
  (* C_PRIM_TYPE = "1" *) 
  (* C_READ_DEPTH_A = "2048" *) 
  (* C_READ_DEPTH_B = "2048" *) 
  (* C_READ_LATENCY_A = "1" *) 
  (* C_READ_LATENCY_B = "1" *) 
  (* C_READ_WIDTH_A = "32" *) 
  (* C_READ_WIDTH_B = "32" *) 
  (* C_RSTRAM_A = "0" *) 
  (* C_RSTRAM_B = "0" *) 
  (* C_RST_PRIORITY_A = "CE" *) 
  (* C_RST_PRIORITY_B = "CE" *) 
  (* C_SIM_COLLISION_CHECK = "ALL" *) 
  (* C_USE_BRAM_BLOCK = "1" *) 
  (* C_USE_BYTE_WEA = "1" *) 
  (* C_USE_BYTE_WEB = "1" *) 
  (* C_USE_DEFAULT_DATA = "0" *) 
  (* C_USE_ECC = "0" *) 
  (* C_USE_SOFTECC = "0" *) 
  (* C_USE_URAM = "0" *) 
  (* C_WEA_WIDTH = "4" *) 
  (* C_WEB_WIDTH = "4" *) 
  (* C_WRITE_DEPTH_A = "2048" *) 
  (* C_WRITE_DEPTH_B = "2048" *) 
  (* C_WRITE_MODE_A = "WRITE_FIRST" *) 
  (* C_WRITE_MODE_B = "WRITE_FIRST" *) 
  (* C_WRITE_WIDTH_A = "32" *) 
  (* C_WRITE_WIDTH_B = "32" *) 
  (* C_XDEVICEFAMILY = "zynquplus" *) 
  (* downgradeipidentifiedwarnings = "yes" *) 
  (* is_du_within_envelope = "true" *) 
  design_1_axi_bram_ctrl_0_bram_0_blk_mem_gen_v8_4_5 U0
       (.addra({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,addra[12:2],1'b0,1'b0}),
        .addrb({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .clka(clka),
        .clkb(1'b0),
        .dbiterr(NLW_U0_dbiterr_UNCONNECTED),
        .deepsleep(1'b0),
        .dina(dina),
        .dinb({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .douta(douta),
        .doutb(NLW_U0_doutb_UNCONNECTED[31:0]),
        .eccpipece(1'b0),
        .ena(ena),
        .enb(1'b0),
        .injectdbiterr(1'b0),
        .injectsbiterr(1'b0),
        .rdaddrecc(NLW_U0_rdaddrecc_UNCONNECTED[31:0]),
        .regcea(1'b0),
        .regceb(1'b0),
        .rsta(rsta),
        .rsta_busy(rsta_busy),
        .rstb(1'b0),
        .rstb_busy(NLW_U0_rstb_busy_UNCONNECTED),
        .s_aclk(1'b0),
        .s_aresetn(1'b0),
        .s_axi_araddr({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .s_axi_arburst({1'b0,1'b0}),
        .s_axi_arid({1'b0,1'b0,1'b0,1'b0}),
        .s_axi_arlen({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .s_axi_arready(NLW_U0_s_axi_arready_UNCONNECTED),
        .s_axi_arsize({1'b0,1'b0,1'b0}),
        .s_axi_arvalid(1'b0),
        .s_axi_awaddr({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .s_axi_awburst({1'b0,1'b0}),
        .s_axi_awid({1'b0,1'b0,1'b0,1'b0}),
        .s_axi_awlen({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .s_axi_awready(NLW_U0_s_axi_awready_UNCONNECTED),
        .s_axi_awsize({1'b0,1'b0,1'b0}),
        .s_axi_awvalid(1'b0),
        .s_axi_bid(NLW_U0_s_axi_bid_UNCONNECTED[3:0]),
        .s_axi_bready(1'b0),
        .s_axi_bresp(NLW_U0_s_axi_bresp_UNCONNECTED[1:0]),
        .s_axi_bvalid(NLW_U0_s_axi_bvalid_UNCONNECTED),
        .s_axi_dbiterr(NLW_U0_s_axi_dbiterr_UNCONNECTED),
        .s_axi_injectdbiterr(1'b0),
        .s_axi_injectsbiterr(1'b0),
        .s_axi_rdaddrecc(NLW_U0_s_axi_rdaddrecc_UNCONNECTED[31:0]),
        .s_axi_rdata(NLW_U0_s_axi_rdata_UNCONNECTED[31:0]),
        .s_axi_rid(NLW_U0_s_axi_rid_UNCONNECTED[3:0]),
        .s_axi_rlast(NLW_U0_s_axi_rlast_UNCONNECTED),
        .s_axi_rready(1'b0),
        .s_axi_rresp(NLW_U0_s_axi_rresp_UNCONNECTED[1:0]),
        .s_axi_rvalid(NLW_U0_s_axi_rvalid_UNCONNECTED),
        .s_axi_sbiterr(NLW_U0_s_axi_sbiterr_UNCONNECTED),
        .s_axi_wdata({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .s_axi_wlast(1'b0),
        .s_axi_wready(NLW_U0_s_axi_wready_UNCONNECTED),
        .s_axi_wstrb({1'b0,1'b0,1'b0,1'b0}),
        .s_axi_wvalid(1'b0),
        .sbiterr(NLW_U0_sbiterr_UNCONNECTED),
        .shutdown(1'b0),
        .sleep(1'b0),
        .wea(wea),
        .web({1'b0,1'b0,1'b0,1'b0}));
endmodule
`pragma protect begin_protected
`pragma protect version = 1
`pragma protect encrypt_agent = "XILINX"
`pragma protect encrypt_agent_info = "Xilinx Encryption Tool 2022.2"
`pragma protect key_keyowner="Synopsys", key_keyname="SNPS-VCS-RSA-2", key_method="rsa"
`pragma protect encoding = (enctype="BASE64", line_length=76, bytes=128)
`pragma protect key_block
VHPlDkoDlWlBfBMvPBmGYmaek3s9hXXhjF28kllYPnaNm3TSnzzpXHWHc8Ye9/2L2yiQfJ1hTWou
Ia/zeQ8h9/dtr6QB5YkyW4wlb/LbMgXb+DGIXPSllNl0IMsRQIcQDbcQm1bO/nlhb+2pjxiuaQrl
DbvxoDwPs7z3LunRxsg=

`pragma protect key_keyowner="Aldec", key_keyname="ALDEC15_001", key_method="rsa"
`pragma protect encoding = (enctype="BASE64", line_length=76, bytes=256)
`pragma protect key_block
lmIhoX8hXuc7tNV1sXY1K2/gXL7Y7Hq73qQF7+x03UWWTRd3uhGmVQtOMVbhIW+66UkWUHiD26zL
fzqGor8bgSNGpSFyS11k4TwLQT4OfAMGO8C9Qmmh4+VENBnpS9TW+wHzCv8oUwht7xYtYRZvOvYK
F3fMppz2sBkUd1lciw98ZE/UmNkhqBuMfIYF43j45DEJ55PBhOZNg91Ls4v3qBHyBAaYPFFoMry3
d5Fw1PZyFQSEOSSpwgyds2aN0g6oIwl7zm0LJrM9VDAOxBUE50hk+oHr4jj8J8UhHQJnlEHm1Idm
rvxKygNKRvfSpa90NYxZJFYgqnrMYg+19+9aZA==

`pragma protect key_keyowner="Mentor Graphics Corporation", key_keyname="MGC-VELOCE-RSA", key_method="rsa"
`pragma protect encoding = (enctype="BASE64", line_length=76, bytes=128)
`pragma protect key_block
VkyCjO2onoeZWEoYQ/4ue7X5mkHyTYVW9xjdoTsGS4GdP/Q64VaCZL/jr6R8DVDXPMnH7tRMrDpo
jpYBnyzSgOkfgqM+96ioC2fDyAaG4gYgGLmrBR6qK3/mxXwAZZX+GJ9R/eWXkc9h8xN+gsSSX6/M
jIQCgeT6q7PB4dWT6KY=

`pragma protect key_keyowner="Mentor Graphics Corporation", key_keyname="MGC-VERIF-SIM-RSA-2", key_method="rsa"
`pragma protect encoding = (enctype="BASE64", line_length=76, bytes=256)
`pragma protect key_block
Iub91V+TnhVlZCSLu6iKmFjix71y6/l83OPTs8uewWvkE7WcqYxEKi9fonXEkzAtWzuKwEUqnOlN
VBsNJqPUdKcd22q523mrdt89mpdosWD+hvZdO7ELhJniY5u9h49FFkubpN2JiUTcIcKEYxVNlds4
wyvaYUqbPVH5v2ooJwDdimS4GVn9HerCOgPwfshvQDNlMTxLcYju4v8BHMc5Rub9Q/ihvpQU74v2
ouZ9XIwA+C6pBLwvaqS8jE7HXOokgqJilaX/W/t+KEgiFry/txRTMU9WMD7tCN7lcfjCydmS3Lq+
3u6Hsr0S8BwNjcaDpZDnBTygUJd4JSqREnk33w==

`pragma protect key_keyowner="Real Intent", key_keyname="RI-RSA-KEY-1", key_method="rsa"
`pragma protect encoding = (enctype="BASE64", line_length=76, bytes=256)
`pragma protect key_block
U46EWFmKmpZGaWfyL+dokyQtJtaOYsa7HCW/+fdtw9/yHKTWFpmqKBZngBj5rPkNhtTDDCJkqsYj
tUXg1j4tgIBaCQn9B0q/aG+B3gPLrudp9hLL25mVbsfiTzdekiV2hJMmhuMoavKKPJHC6zyW7kZi
80er82OQy8h+Df/fe6TRjH9xEt3/b80tRKUMbxkLfnnkAyyf1KfOhB6/uyI4mwXuQR+DsAbzybKR
YtXpOiW72tGrXTFlzcwbHamWZefqsilVpBw6V5dh33vYKGx50xwWpj76maAkpQrOpB7zufeldJe4
W1UOEN84AZdRTLkVSxamWo/wp8nP9fiGS/ItRw==

`pragma protect key_keyowner="Xilinx", key_keyname="xilinxt_2021_07", key_method="rsa"
`pragma protect encoding = (enctype="BASE64", line_length=76, bytes=256)
`pragma protect key_block
qczgIJYpE/SzErzK7eWJBGcDFEzDLm8cKbwJbPXuM6YnJxx44W+E60R3war7K2QGFAkOoCDUtDC7
SghJGF32btaDLzeKm0tQ669sBtQmMIaBrlt7I9QBkNM8zN9GL92qxNC9o3UVWMOYy5BmH8nUPgcE
O6lRubeltlrTuDe7UJQ2nEPHcXjpUJJ8dxktyW+LovBy1OxW8g4GRAsmEJsoOEg0HuDdWcc4IshJ
PvwPJ7LblELAKsdkSt65y9VaklaEm7MlH4ImlgIa74TgRmutLUbWxM1QYhGE5rAzFhGU5i3RJOdx
L3N7GGGvLMW2z9NSHbIFX+/eNII9fNJ9nZbgLA==

`pragma protect key_keyowner="Metrics Technologies Inc.", key_keyname="DSim", key_method="rsa"
`pragma protect encoding = (enctype="BASE64", line_length=76, bytes=256)
`pragma protect key_block
Ti1NUgDv8YPk90APMwfu/mRr38QYwAxZfv0T6zQ89YS55t2EquEGVqrEafYX6rTydLOw8le1Oucv
f2oERpSSSTih/ScZneSZmuPE/Zh2BU1Ajv0j+/+0uEWXU+5lLPbDJjnapTmJXih1MYPf0SHpZZmE
BKj2IEBI9MPZlh6bxpa5BWJnyPdAvHf+UNaMXU9+pmbtrzUVebql4mFJu45Z3+ehmFY4FBW3zXMF
44C4TlHACLwL3vHVMCVfeKhgdVDbpE+/IFhTStz7mZ9h9RKGanQcs6YDVM1R+2RKA1QT1fX4FiQc
1V+FGmrm1ujxmFGXwpfNKByVlfCY0oWhRJCYYQ==

`pragma protect key_keyowner="Atrenta", key_keyname="ATR-SG-RSA-1", key_method="rsa"
`pragma protect encoding = (enctype="BASE64", line_length=76, bytes=384)
`pragma protect key_block
HuEXFK0NXt09xU2yxxjng1OLsT+ZEM4EhqBgpr9D2ljw2vDaMBrqEsRQTc2B9soDq3ewDduHJXBd
OGYxkPnoN6LhjULtB2nTgjcH6NxA4puZ1ZNcndDndVBo8rTW5W1OqHq6InAG0CqPpTIkuqz3ECPl
EysI++MCDfH6tIzlekxJFIJ1McJsTq5rFuLzMMcrmkBxgcayDpOcCFuzZzCczxmt/cCCIKmDybwT
OQXmOcLJoYLP4sFu6R9c6xO8i6p++crv2N3eIxZHKbek9xBBZqQM9EYuEtsbkqAs9XZpa16i5njR
BDFxTKcP6r7JgFALJE89AZhBbate5JXWp0v4ECZD18aEL17CipwcWPutNMdG1apzSPP5y59n7rMG
yxBPz1gKHc3Emkl4WcO0hjICxqmO6dMXoY8JvBSf6ry2l0sH9Ihr3Bq5WWmlhPHnoaNr5jl//vNe
KfToWtn97eoVSt1LnmXXnSpdigbHr0UIg8AdkpdkuNRaWdVicDdgSo49

`pragma protect key_keyowner="Cadence Design Systems.", key_keyname="CDS_RSA_KEY_VER_1", key_method="rsa"
`pragma protect encoding = (enctype="BASE64", line_length=76, bytes=256)
`pragma protect key_block
mokwst2bn6UxD6V9UdIgCIG1QQ/d0FiJqYGOTI2eHPV6YElaLjnJ8DnQmZnGS95o3x93FDOoa58C
RwYsX1fVoVtXkj1LuZq0k7q9vEe4T8xMjpkeYtIHY9k0Xhy1Lq/xRlfzGAf9fvf9e+f4r7aR/Sb/
uCZxxugG5niTwLENY1n3NthYL0jvo8Fmdw4Qg0nTCGWlVCws+09K0g9/lx6I9EcuHHemcHO3fOZG
lMc4NaPNozKwnyDMoWUkwiVxyFEPFaQLNYqzjvR+CqrWfhFLo96JWhL+eaDoNuZoBVYQtNH5ZwBL
BoO27Pw10lgcReGlZBz3BLO7T4ddynCx0+eSnw==

`pragma protect key_keyowner="Synplicity", key_keyname="SYNP15_1", key_method="rsa"
`pragma protect encoding = (enctype="BASE64", line_length=76, bytes=256)
`pragma protect key_block
PiP7AjOQqqouyQMoBQqgWIDhUSViq94rIvGiIJ/UKMDspM/yXw1caE8AhWHTjYckC4yLpPAz5P6s
1Z6flzDPrzVwg4e59X2cc4IMCHhedna0rDO804njcc6amRDTeLsMLTkWfvomB4xwszm2AgT+PRnB
WHd09ZUDVFjiBXT+Oa9AicgGJHrX3w823yBPuAa704kje/SzgtiDpcTU1eLmLhLW7LpEd9KIHd9s
ER7Uk9Orws0Kq9PMTqMX4hMn5K5mFakOeOURiEbUjdv5RiIJ2g/PlQXSItM8fHsBTQa6fOaJwQTI
vHwK3a8ZBHpfT1YH+n7wNiNUZwD4SFXm1QVx4g==

`pragma protect key_keyowner="Mentor Graphics Corporation", key_keyname="MGC-PREC-RSA", key_method="rsa"
`pragma protect encoding = (enctype="BASE64", line_length=76, bytes=256)
`pragma protect key_block
Ul5ZfTHJwMctaNhYRortUZizYMPYRef7uYqPSuMkxsArnxI/cjGh+KRMwzV86hyp/6TXSJIjm5ec
2wX2UONdPN+DOJ84jYC4JbgJQrPnTj7ioD8uLX/WlyPcQzyF5keqFgj5eR5s13FskVWCuAWf5m9w
mhFEKFjVXDAr7gVgAJh/hL8P6Psrnf+LGfiM8JhnDepsHEYykGlpD3fzru2BGgqHWqPqFMcnyVGl
vysaIXiJz/eYKvO8RGcgd3DJAM/wPm9A0m/DWcmSnczOgTjoqkHcBg2H5uJMLvufzmjImi6LYEqq
v04ESDEN31cSUzqUYcayvMFOnI/WNsWbFIa5+Q==

`pragma protect data_method = "AES128-CBC"
`pragma protect encoding = (enctype = "BASE64", line_length = 76, bytes = 57280)
`pragma protect data_block
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
`pragma protect end_protected
`ifndef GLBL
`define GLBL
`timescale  1 ps / 1 ps

module glbl ();

    parameter ROC_WIDTH = 100000;
    parameter TOC_WIDTH = 0;
    parameter GRES_WIDTH = 10000;
    parameter GRES_START = 10000;

//--------   STARTUP Globals --------------
    wire GSR;
    wire GTS;
    wire GWE;
    wire PRLD;
    wire GRESTORE;
    tri1 p_up_tmp;
    tri (weak1, strong0) PLL_LOCKG = p_up_tmp;

    wire PROGB_GLBL;
    wire CCLKO_GLBL;
    wire FCSBO_GLBL;
    wire [3:0] DO_GLBL;
    wire [3:0] DI_GLBL;
   
    reg GSR_int;
    reg GTS_int;
    reg PRLD_int;
    reg GRESTORE_int;

//--------   JTAG Globals --------------
    wire JTAG_TDO_GLBL;
    wire JTAG_TCK_GLBL;
    wire JTAG_TDI_GLBL;
    wire JTAG_TMS_GLBL;
    wire JTAG_TRST_GLBL;

    reg JTAG_CAPTURE_GLBL;
    reg JTAG_RESET_GLBL;
    reg JTAG_SHIFT_GLBL;
    reg JTAG_UPDATE_GLBL;
    reg JTAG_RUNTEST_GLBL;

    reg JTAG_SEL1_GLBL = 0;
    reg JTAG_SEL2_GLBL = 0 ;
    reg JTAG_SEL3_GLBL = 0;
    reg JTAG_SEL4_GLBL = 0;

    reg JTAG_USER_TDO1_GLBL = 1'bz;
    reg JTAG_USER_TDO2_GLBL = 1'bz;
    reg JTAG_USER_TDO3_GLBL = 1'bz;
    reg JTAG_USER_TDO4_GLBL = 1'bz;

    assign (strong1, weak0) GSR = GSR_int;
    assign (strong1, weak0) GTS = GTS_int;
    assign (weak1, weak0) PRLD = PRLD_int;
    assign (strong1, weak0) GRESTORE = GRESTORE_int;

    initial begin
	GSR_int = 1'b1;
	PRLD_int = 1'b1;
	#(ROC_WIDTH)
	GSR_int = 1'b0;
	PRLD_int = 1'b0;
    end

    initial begin
	GTS_int = 1'b1;
	#(TOC_WIDTH)
	GTS_int = 1'b0;
    end

    initial begin 
	GRESTORE_int = 1'b0;
	#(GRES_START);
	GRESTORE_int = 1'b1;
	#(GRES_WIDTH);
	GRESTORE_int = 1'b0;
    end

endmodule
`endif

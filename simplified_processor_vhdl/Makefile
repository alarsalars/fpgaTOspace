SIM ?= ghdl
TOPLEVEL_LANG ?= vhdl
VHDL_SOURCES += $(PWD)/top.vhd
SIM_ARGS ?= --vcd=waveform1.vcd

all: ad 

ad:
#rm -rf sim_build
#$(MAKE) sim MODULE=test_tb TOPLEVEL=top
    MODULE = tb_cocotb
    TOPLEVEL = processor_top
	
	
include $(shell cocotb-config --makefiles)/Makefile.sim

#PYTHON_LIBDIR:=$(shell python -c 'from distutils import sysconfig; print sysconfig.get_config_var("LIBDIR")')
#LIBPYTHON_LOC=/usr/lib/x86_64-linux-gnu/libpython3.10.so
#SIM_ARGS ?= --vcd=waveform.vcd
SIM ?= ghdl
TOPLEVEL_LANG ?= vhdl
VHDL_SOURCES += $(PWD)/top.vhd
SIM_ARGS ?= --vcd=waveform1.vcd

all: ad 

ad:
#rm -rf sim_build
#$(MAKE) sim MODULE=test_tb TOPLEVEL=top
    MODULE = test_tb
    TOPLEVEL = top
	
	
include $(shell cocotb-config --makefiles)/Makefile.sim

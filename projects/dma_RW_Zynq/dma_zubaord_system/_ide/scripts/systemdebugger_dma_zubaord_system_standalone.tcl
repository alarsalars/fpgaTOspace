# Usage with Vitis IDE:
# In Vitis IDE create a Single Application Debug launch configuration,
# change the debug type to 'Attach to running target' and provide this 
# tcl script in 'Execute Script' option.
# Path of this script: C:\Users\TAlars\Documents\vivado_projects_tests\vipix\dma\dma_RW_Zynq\dma_zubaord_system\_ide\scripts\systemdebugger_dma_zubaord_system_standalone.tcl
# 
# 
# Usage with xsct:
# To debug using xsct, launch xsct and run below command
# source C:\Users\TAlars\Documents\vivado_projects_tests\vipix\dma\dma_RW_Zynq\dma_zubaord_system\_ide\scripts\systemdebugger_dma_zubaord_system_standalone.tcl
# 
connect -url tcp:127.0.0.1:3121
source C:/Xilinx/Vitis/2022.2/scripts/vitis/util/zynqmp_utils.tcl
targets -set -nocase -filter {name =~"APU*"}
rst -system
after 3000
targets -set -filter {jtag_cable_name =~ "Avnet ZUBoard1CG 1234-oj1A" && level==0 && jtag_device_ctx=="jsn-ZUBoard1CG-1234-oj1A-04688093-0"}
fpga -file C:/Users/TAlars/Documents/vivado_projects_tests/vipix/dma/dma_RW_Zynq/dma_zubaord/_ide/bitstream/design_1_wrapper.bit
targets -set -nocase -filter {name =~"APU*"}
loadhw -hw C:/Users/TAlars/Documents/vivado_projects_tests/vipix/dma/dma_RW_Zynq/design_1_wrapper/export/design_1_wrapper/hw/design_1_wrapper.xsa -mem-ranges [list {0x80000000 0xbfffffff} {0x400000000 0x5ffffffff} {0x1000000000 0x7fffffffff}] -regs
configparams force-mem-access 1
targets -set -nocase -filter {name =~"APU*"}
set mode [expr [mrd -value 0xFF5E0200] & 0xf]
targets -set -nocase -filter {name =~ "*R5*#0"}
rst -processor
dow C:/Users/TAlars/Documents/vivado_projects_tests/vipix/dma/dma_RW_Zynq/design_1_wrapper/export/design_1_wrapper/sw/design_1_wrapper/boot/fsbl.elf
set bp_57_27_fsbl_bp [bpadd -addr &XFsbl_Exit]
con -block -timeout 60
bpremove $bp_57_27_fsbl_bp
targets -set -nocase -filter {name =~ "*A53*#0"}
rst -processor
dow C:/Users/TAlars/Documents/vivado_projects_tests/vipix/dma/dma_RW_Zynq/dma_zubaord/Debug/dma_zubaord.elf
configparams force-mem-access 0
targets -set -nocase -filter {name =~ "*A53*#0"}
con

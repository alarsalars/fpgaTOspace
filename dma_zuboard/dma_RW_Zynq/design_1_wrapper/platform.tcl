# 
# Usage: To re-create this platform project launch xsct with below options.
# xsct C:\Users\TAlars\Documents\vivado_projects_tests\vipix\dma\dma_RW_Zynq\design_1_wrapper\platform.tcl
# 
# OR launch xsct and run below command.
# source C:\Users\TAlars\Documents\vivado_projects_tests\vipix\dma\dma_RW_Zynq\design_1_wrapper\platform.tcl
# 
# To create the platform in a different location, modify the -out option of "platform create" command.
# -out option specifies the output directory of the platform project.

platform create -name {design_1_wrapper}\
-hw {C:\Users\TAlars\Documents\vivado_projects_tests\vipix\dma\dma_RW_Zynq\design_1_wrapper.xsa}\
-arch {64-bit} -fsbl-target {psu_cortexa53_0} -out {C:/Users/TAlars/Documents/vivado_projects_tests/vipix/dma/dma_RW_Zynq}

platform write
domain create -name {standalone_psu_cortexa53_0} -display-name {standalone_psu_cortexa53_0} -os {standalone} -proc {psu_cortexa53_0} -runtime {cpp} -arch {64-bit} -support-app {empty_application}
platform generate -domains 
platform active {design_1_wrapper}
domain active {zynqmp_fsbl}
domain active {zynqmp_pmufw}
domain active {standalone_psu_cortexa53_0}
platform generate -quick
platform clean
platform clean
platform generate
platform config -fsbl-target psu_cortexr5_0
platform write
bsp reload
catch {bsp regenerate}
domain active {zynqmp_fsbl}
bsp reload
catch {bsp regenerate}
domain active {zynqmp_pmufw}
bsp reload
catch {bsp regenerate}
platform generate -domains zynqmp_fsbl,zynqmp_pmufw 
platform active {design_1_wrapper}
platform generate -domains 

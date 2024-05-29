# 
# Usage: To re-create this platform project launch xsct with below options.
# xsct C:\Users\TAlars\Documents\vivado_projects_tests\vipix\multi_proc\design_1_wrapper\platform.tcl
# 
# OR launch xsct and run below command.
# source C:\Users\TAlars\Documents\vivado_projects_tests\vipix\multi_proc\design_1_wrapper\platform.tcl
# 
# To create the platform in a different location, modify the -out option of "platform create" command.
# -out option specifies the output directory of the platform project.

platform create -name {design_1_wrapper}\
-hw {C:\Users\TAlars\Documents\vivado_projects_tests\vipix\multi_proc\multi_proc\design_1_wrapper.xsa}\
-arch {64-bit} -fsbl-target {psu_cortexa53_0} -out {C:/Users/TAlars/Documents/vivado_projects_tests/vipix/multi_proc}

platform write
domain create -name {standalone_psu_cortexa53_0} -display-name {standalone_psu_cortexa53_0} -os {standalone} -proc {psu_cortexa53_0} -runtime {cpp} -arch {64-bit} -support-app {hello_world}
platform generate -domains 
platform active {design_1_wrapper}
domain active {zynqmp_fsbl}
domain active {zynqmp_pmufw}
domain active {standalone_psu_cortexa53_0}
platform generate -quick
platform active {design_1_wrapper}
domain create -name {standalone_microblaze_0} -display-name {standalone_microblaze_0} -os {standalone} -proc {microblaze_0} -runtime {cpp} -arch {32-bit} -support-app {empty_application}
platform generate -domains 
platform write
domain active {zynqmp_fsbl}
domain active {zynqmp_pmufw}
domain active {standalone_psu_cortexa53_0}
domain active {standalone_microblaze_0}
platform generate -quick
platform generate

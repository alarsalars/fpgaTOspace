set moduleName top
set isTopModule 1
set isCombinational 0
set isDatapathOnly 0
set isPipelined 0
set pipeline_type function
set FunctionProtocol ap_ctrl_none
set isOneStateSeq 1
set ProfileFlag 0
set StallSigGenFlag 0
set isEnableWaveformDebug 1
set hasInterrupt 0
set C_modelName {top}
set C_modelType { void 0 }
set C_modelArgList {
	{ adr int 8 regular  }
	{ we uint 1 regular  }
	{ cyc uint 1 regular  }
	{ stb uint 1 regular  }
	{ wb_in int 8 regular  }
	{ rx uint 1 regular  }
	{ tx int 1 regular {pointer 1}  }
	{ ack int 1 regular {pointer 1}  }
	{ uart_out int 10 regular {pointer 1}  }
}
set C_modelArgMapList {[ 
	{ "Name" : "adr", "interface" : "wire", "bitwidth" : 8, "direction" : "READONLY"} , 
 	{ "Name" : "we", "interface" : "wire", "bitwidth" : 1, "direction" : "READONLY"} , 
 	{ "Name" : "cyc", "interface" : "wire", "bitwidth" : 1, "direction" : "READONLY"} , 
 	{ "Name" : "stb", "interface" : "wire", "bitwidth" : 1, "direction" : "READONLY"} , 
 	{ "Name" : "wb_in", "interface" : "wire", "bitwidth" : 8, "direction" : "READONLY"} , 
 	{ "Name" : "rx", "interface" : "wire", "bitwidth" : 1, "direction" : "READONLY"} , 
 	{ "Name" : "tx", "interface" : "wire", "bitwidth" : 1, "direction" : "WRITEONLY"} , 
 	{ "Name" : "ack", "interface" : "wire", "bitwidth" : 1, "direction" : "WRITEONLY"} , 
 	{ "Name" : "uart_out", "interface" : "wire", "bitwidth" : 10, "direction" : "WRITEONLY"} ]}
# RTL Port declarations: 
set portNum 11
set portList { 
	{ ap_clk sc_in sc_logic 1 clock -1 } 
	{ ap_rst sc_in sc_logic 1 reset -1 active_high_sync } 
	{ adr sc_in sc_lv 8 signal 0 } 
	{ we sc_in sc_logic 1 signal 1 } 
	{ cyc sc_in sc_logic 1 signal 2 } 
	{ stb sc_in sc_logic 1 signal 3 } 
	{ wb_in sc_in sc_lv 8 signal 4 } 
	{ rx sc_in sc_logic 1 signal 5 } 
	{ tx sc_out sc_logic 1 signal 6 } 
	{ ack sc_out sc_logic 1 signal 7 } 
	{ uart_out sc_out sc_lv 10 signal 8 } 
}
set NewPortList {[ 
	{ "name": "ap_clk", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "clock", "bundle":{"name": "ap_clk", "role": "default" }} , 
 	{ "name": "ap_rst", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "reset", "bundle":{"name": "ap_rst", "role": "default" }} , 
 	{ "name": "adr", "direction": "in", "datatype": "sc_lv", "bitwidth":8, "type": "signal", "bundle":{"name": "adr", "role": "default" }} , 
 	{ "name": "we", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "we", "role": "default" }} , 
 	{ "name": "cyc", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "cyc", "role": "default" }} , 
 	{ "name": "stb", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "stb", "role": "default" }} , 
 	{ "name": "wb_in", "direction": "in", "datatype": "sc_lv", "bitwidth":8, "type": "signal", "bundle":{"name": "wb_in", "role": "default" }} , 
 	{ "name": "rx", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "rx", "role": "default" }} , 
 	{ "name": "tx", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "tx", "role": "default" }} , 
 	{ "name": "ack", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "ack", "role": "default" }} , 
 	{ "name": "uart_out", "direction": "out", "datatype": "sc_lv", "bitwidth":10, "type": "signal", "bundle":{"name": "uart_out", "role": "default" }}  ]}

set RtlHierarchyInfo {[
	{"ID" : "0", "Level" : "0", "Path" : "`AUTOTB_DUT_INST", "Parent" : "",
		"CDFG" : "top",
		"Protocol" : "ap_ctrl_none",
		"ControlExist" : "0", "ap_start" : "0", "ap_ready" : "0", "ap_done" : "0", "ap_continue" : "0", "ap_idle" : "0", "real_start" : "0",
		"Pipeline" : "None", "UnalignedPipeline" : "0", "RewindPipeline" : "0", "ProcessNetwork" : "0",
		"II" : "1",
		"VariableLatency" : "0", "ExactLatency" : "0", "EstimateLatencyMin" : "0", "EstimateLatencyMax" : "0",
		"Combinational" : "0",
		"Datapath" : "0",
		"ClockEnable" : "0",
		"HasSubDataflow" : "0",
		"InDataflowNetwork" : "0",
		"HasNonBlockingOperation" : "0",
		"IsBlackBox" : "0",
		"Port" : [
			{"Name" : "adr", "Type" : "None", "Direction" : "I"},
			{"Name" : "we", "Type" : "None", "Direction" : "I"},
			{"Name" : "cyc", "Type" : "None", "Direction" : "I"},
			{"Name" : "stb", "Type" : "None", "Direction" : "I"},
			{"Name" : "wb_in", "Type" : "None", "Direction" : "I"},
			{"Name" : "rx", "Type" : "None", "Direction" : "I"},
			{"Name" : "tx", "Type" : "None", "Direction" : "O"},
			{"Name" : "ack", "Type" : "None", "Direction" : "O"},
			{"Name" : "uart_out", "Type" : "None", "Direction" : "O"},
			{"Name" : "state", "Type" : "OVld", "Direction" : "IO"},
			{"Name" : "tx_ff", "Type" : "OVld", "Direction" : "IO"},
			{"Name" : "i", "Type" : "OVld", "Direction" : "IO"},
			{"Name" : "j", "Type" : "OVld", "Direction" : "IO"},
			{"Name" : "baud_count", "Type" : "OVld", "Direction" : "IO"},
			{"Name" : "uart_rd_shift_V", "Type" : "OVld", "Direction" : "IO"},
			{"Name" : "uart_wr_shift_V", "Type" : "OVld", "Direction" : "IO"}]}]}


set ArgLastReadFirstWriteLatency {
	top {
		adr {Type I LastRead 0 FirstWrite -1}
		we {Type I LastRead 0 FirstWrite -1}
		cyc {Type I LastRead 0 FirstWrite -1}
		stb {Type I LastRead 0 FirstWrite -1}
		wb_in {Type I LastRead 0 FirstWrite -1}
		rx {Type I LastRead 0 FirstWrite -1}
		tx {Type O LastRead -1 FirstWrite 0}
		ack {Type O LastRead -1 FirstWrite 0}
		uart_out {Type O LastRead -1 FirstWrite 0}
		state {Type IO LastRead -1 FirstWrite -1}
		tx_ff {Type IO LastRead -1 FirstWrite -1}
		i {Type IO LastRead -1 FirstWrite -1}
		j {Type IO LastRead -1 FirstWrite -1}
		baud_count {Type IO LastRead -1 FirstWrite -1}
		uart_rd_shift_V {Type IO LastRead -1 FirstWrite -1}
		uart_wr_shift_V {Type IO LastRead -1 FirstWrite -1}}}

set hasDtUnsupportedChannel 0

set PerformanceInfo {[
	{"Name" : "Latency", "Min" : "0", "Max" : "0"}
	, {"Name" : "Interval", "Min" : "1", "Max" : "1"}
]}

set PipelineEnableSignalInfo {[
]}

set Spec2ImplPortList { 
	adr { ap_none {  { adr in_data 0 8 } } }
	we { ap_none {  { we in_data 0 1 } } }
	cyc { ap_none {  { cyc in_data 0 1 } } }
	stb { ap_none {  { stb in_data 0 1 } } }
	wb_in { ap_none {  { wb_in in_data 0 8 } } }
	rx { ap_none {  { rx in_data 0 1 } } }
	tx { ap_none {  { tx out_data 1 1 } } }
	ack { ap_none {  { ack out_data 1 1 } } }
	uart_out { ap_none {  { uart_out out_data 1 10 } } }
}

set maxi_interface_dict [dict create]

# RTL port scheduling information:
set fifoSchedulingInfoList { 
}

# RTL bus port read request latency information:
set busReadReqLatencyList { 
}

# RTL bus port write response latency information:
set busWriteResLatencyList { 
}

# RTL array port load latency information:
set memoryLoadLatencyList { 
}

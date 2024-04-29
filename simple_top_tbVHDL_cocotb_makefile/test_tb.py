import os
import random
import sys
# import numpy as np 
from pathlib import Path
from typing import List
import cocotb
from cocotb.runner import get_runner
from cocotb.triggers import Timer, FallingEdge, RisingEdge
from cocotb.clock import Clock

@cocotb.test()
async def hardcoded_test(dut):
    """generate cocotb test."""
    # Start VCD trace
    #vcd_file = "dump.vcd"
    #cocotb.log.info(f"Dumping VCD to {vcd_file}")
    #await cocotb.triggers.Timer(0)
    #cocotb.triggers.start_vcd_trace(vcd_file)
    val_array = List[int]
    myarray: val_array =   [int("101010101000011011111010",2),
                            int("100010101111011011111011",2),
                            int("100110110101110111111101",2),
                            int("010001111111111011111101",2),
                            int("101101001010100110111100",2),
                            int("100001001101010010111100",2),
                            int("111010100000111010111111",2),
                            int("011101101110000110111111",2),
                            int("010111010000101111111111",2),
                            int("100000010000010011111111",2)]
    led_reg = 0xFFFFFF
    dut.led_dat.value           = 0
    dut.led_lnk.value           = 0
    dut.led_pwrgd.value         = 0
    dut.led_cpu_act.value       = 0
    dut.led_autotst.value       = 0
    dut.led_poe_pwrgd.value     = 0
    dut.bypass_relay_on.value   = 0
    dut.poe_ctrl_poe.value      = 0
    dut.poe_cfg_poe.value       = 0
    await Timer(1, 'ns')
    dut.rst_n.value = 0
    await Timer(1, 'ns')
    dut.rst_n.value = 1
    clock = Clock(dut.clk_32MHz, 31.25, units="ns")
    clock_tb = Clock(dut.clk_tb, 22, units="ms")
    cocotb.start_soon(clock.start(start_high=False))
    cocotb.start_soon(clock_tb.start(start_high=False))
    counter: int = 0
    while counter < 10:
        buff_reg = myarray[counter]  # Use modulo to cycle through the array
        cocotb.log.info(f"buff_reg: {(buff_reg)}")
        cocotb.log.info(f"led_reg: {(led_reg)}")
        cocotb.log.info(f"waiting.............buff_reg: {(buff_reg)}")
        await RisingEdge(dut.clk_tb)
        cocotb.log.info(f"waiting....................buff_reg: {(buff_reg)}")
        await RisingEdge(dut.rec_en)
        dut.led_dat.value            = buff_reg >> 16
        dut.led_lnk.value            = (buff_reg >> 8) & 0xFF
        dut.led_pwrgd.value          = (buff_reg >> 7) & 0x01
        dut.led_cpu_act.value        = (buff_reg >> 6) & 0x01
        dut.led_autotst.value        = (buff_reg >> 5) & 0x01
        dut.led_poe_pwrgd.value      = (buff_reg >> 4) & 0x01
        dut.bypass_relay_on.value    = (buff_reg >> 3) & 0x01
        dut.poe_ctrl_poe.value       = (buff_reg >> 1) & 0x03
        dut.poe_cfg_poe.value        = buff_reg & 0x01
        await RisingEdge(dut.led_clk)
        cocotb.log.info(f"led_reg: {(led_reg)}")
        mask = 0xFFF000
        cocotb.log.info(f"led_reg: {bin(led_reg)}")
        cocotb.log.info(f"led_reg: {bin(led_reg)}")
        cocotb.log.info(f"led_reg: {bin(led_reg)}")
        for i in range(23):
            erase = 0xffffff ^ (1 << i)
            led_reg = (led_reg & erase) | (dut.led_data.value << i)
            led_reg_rev = int(format(led_reg, '24b')[::-1], 2)
            buff_reg_rev = int(buff_reg)
            await RisingEdge(dut.led_clk)
            cocotb.log.info(f"buff_reg: {bin(buff_reg)}")
            cocotb.log.info(f"buff_reg_rev: {bin(buff_reg_rev)}")
            cocotb.log.info(f"led_reg: {bin(led_reg)}")
            cocotb.log.info(f"led_reg_rev: {bin(led_reg_rev)}")
        masked_var1 = led_reg_rev     & mask
        masked_var2 = buff_reg_rev    & mask
        counter += 1
        assert masked_var1 == masked_var2, f"output q was incorrect on the cycle{counter} {buff_reg_rev} = {led_reg_rev}"
    cocotb.log.info("........................finish............................")
    cocotb.log.info("........................finish............................")
    cocotb.log.info("........................finish............................")



@cocotb.test()
async def randomised_test(dut):
    """Test for  random numbers multiple times"""
    cnt = random.randint(0, 15)
    cocotb.log.info(f"...........cnt..............: {(cnt)}")
    myarray = List[int]
    bit_length = 24
    myarray = [random.getrandbits(bit_length) for _ in range(cnt)]  # Assuming the size is 10
    cocotb.log.info(f"..........myarray..................: {(myarray)}")
    led_reg = 0xFFFFFF
    dut.led_dat.value           = 0
    dut.led_lnk.value           = 0
    dut.led_pwrgd.value         = 0
    dut.led_cpu_act.value       = 0
    dut.led_autotst.value       = 0
    dut.led_poe_pwrgd.value     = 0
    dut.bypass_relay_on.value   = 0
    dut.poe_ctrl_poe.value      = 0
    dut.poe_cfg_poe.value       = 0
    await Timer(1, 'ns')
    dut.rst_n.value = 0
    await Timer(1, 'ns')
    dut.rst_n.value = 1
    clock = Clock(dut.clk_32MHz, 31.25, units="ns")
    clock_tb = Clock(dut.clk_tb, 22, units="ms")
    cocotb.start_soon(clock.start(start_high=False))
    cocotb.start_soon(clock_tb.start(start_high=False))
    counter: int = 0
    while counter < cnt:
        buff_reg = myarray[counter]  # Use modulo to cycle through the array
        cocotb.log.info(f"..........buff_reg..................: {(buff_reg)}")
        await RisingEdge(dut.clk_tb)
        await RisingEdge(dut.rec_en)
        dut.led_dat.value            = buff_reg >> 16
        dut.led_lnk.value            = (buff_reg >> 8) & 0xFF
        dut.led_pwrgd.value          = (buff_reg >> 7) & 0x01
        dut.led_cpu_act.value        = (buff_reg >> 6) & 0x01
        dut.led_autotst.value        = (buff_reg >> 5) & 0x01
        dut.led_poe_pwrgd.value      = (buff_reg >> 4) & 0x01
        dut.bypass_relay_on.value    = (buff_reg >> 3) & 0x01
        dut.poe_ctrl_poe.value       = (buff_reg >> 1) & 0x03
        dut.poe_cfg_poe.value        = buff_reg & 0x01
        await RisingEdge(dut.led_clk)
        mask = 0xFFF000
        for i in range(23):
            erase = 0xffffff ^ (1 << i)
            led_reg = (led_reg & erase) | (dut.led_data.value << i)
            led_reg_rev = int(format(led_reg, '24b')[::-1], 2)
            buff_reg_rev = int(buff_reg)
            await RisingEdge(dut.led_clk)
        cocotb.log.info(f"..........led_reg..................: {(buff_reg)}")
        masked_var1 = led_reg_rev     & mask
        masked_var2 = buff_reg_rev    & mask
        counter += 1
        assert masked_var1 == masked_var2, f"output q was incorrect on the cycle{counter} {buff_reg_rev} = {led_reg_rev}"
    cocotb.log.info("........................finish............................")
    cocotb.log.info("........................finish............................")
    cocotb.log.info("........................finish............................")
    # Stop VCD trace
    #await cocotb.triggers.Timer(0)
    #cocotb.triggers.dump_vcd(vcd_file)


'''
def test_runner():

    hdl_toplevel_lang = os.getenv("HDL_TOPLEVEL_LANG", "vhdl")
    sim = os.getenv("SIM", "ghdl")

    proj_path = Path(__file__).resolve().parent

    #verilog_sources = []
    vhdl_sources = []

    #if hdl_toplevel_lang == "verilog":
    #    verilog_sources = [proj_path / "top_sv.sv"]
    #else:
     #   vhdl_sources = [proj_path / "top.vhd"]

    runner = get_runner(sim)
    runner.build(
        #verilog_sources=verilog_sources,
        vhdl_sources=vhdl_sources,
        hdl_toplevel="top",
        always=True,
    )

    runner.test(hdl_toplevel="top", test_module="test_tb,")


if __name__ == "__main__":
    test_runner()

'''
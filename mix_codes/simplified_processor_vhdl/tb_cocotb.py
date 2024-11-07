
## it is not complete yet, I need to go through the instructions and the internal memory that include all the INs
## This is only a basic structur to load in and out but it is not totally matching with the required input.
## I will change the internal memory to external so I can cover more cases and the edges as well in python and use the real power of COCOTB




import cocotb
import logging
import random
 
from cocotb.triggers import RisingEdge, Timer
from cocotb.utils import get_sim_time
from cocotb.clock import Clock
 
async def rst_stimuli(dut):
      logging.info('Applying reset to DUT @ %0s',str(get_sim_time('ns')))	
      dut.RST.value = 1;
      await Timer(100,'ns')
      dut.RST.value = 0
      logging.info('System Reset Done @ %0s',str(get_sim_time('ns')))





@cocotb.test()
async def test(dut):
      logging.getLogger().setLevel(logging.INFO)	
      
      cocotb.start_soon(rst_stimuli(dut))
      cocotb.start_soon(Clock(dut.CLK,20,'ns').start())
      
      await Timer(100,'ns')
      logging.info('Sending Stimuli to DUT @ %0s',str(get_sim_time('ns')))
      err = 0	     
      for i in range(10):
          DIN = random.randint(0,65535)
          dut.DIN.value = DIN
          await RisingEdge(dut.CLK)
          await RisingEdge(dut.CLK)
          logging.info('Din: %0d and Dout : %0d',DIN,dut.DOUT.value)
          if dut.DOUT.value != DIN :
             err += 1
          else:
             err = err   
 
      if err > 0:
         logging.error('Test Failed for %0d test cases',err)
      else:
         logging.info('All Test Passes') 






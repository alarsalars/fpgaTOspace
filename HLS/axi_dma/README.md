# AXI DMA TX & RX + Median Filtering

The main concept of the origin idea is to learn how to deal with various AXI interfaces in HLS, the current concept is to recive data from DDR by TXDMA and stream it to any other IP in my case I am using FIFO to recive it and stream it again to RXDMA, the last send the data to the DDR again. So, it is just a loop, to make sure it works. 

Next step, I want to modify the design and include data processing with Median Filtering, so insteade of using FIFO I will make new HLS IP, to process the data and stream it back.



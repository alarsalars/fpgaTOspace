
/*
 *
 *
 *
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "xaxidma.h"
#include "xparameters.h"
#include "sleep.h"
#include "xil_cache.h"
#define ARRAY_SIZE 1024

u32 checkHalted(u32 baseAddress,u32 offset);

int main(){
	u32 a[ARRAY_SIZE];
	u32 b[ARRAY_SIZE];
    for (int i = 0; i < ARRAY_SIZE; i++) {
        a[i] = rand(); // Generate a random integer
    }

    u32 status;

	XAxiDma_Config *myDmaConfig;
	XAxiDma myDma;

	myDmaConfig = XAxiDma_LookupConfigBaseAddr(XPAR_AXI_DMA_0_BASEADDR);
	status = XAxiDma_CfgInitialize(&myDma, myDmaConfig);
	if(status != XST_SUCCESS){
		print("DMA initialization failed\n");
		return -1;
	}
	print("DMA initialization success..\n");
	status = checkHalted(XPAR_AXI_DMA_0_BASEADDR,0x4);
	xil_printf("Status before data transfer %0x\n",status);
	Xil_DCacheFlushRange((u32)a,ARRAY_SIZE*sizeof(u32));
	Xil_DCacheFlushRange((u32)b,ARRAY_SIZE*sizeof(u32));
	status = XAxiDma_SimpleTransfer(&myDma, (u32)b, ARRAY_SIZE*sizeof(u32),XAXIDMA_DEVICE_TO_DMA);
	status = XAxiDma_SimpleTransfer(&myDma, (u32)a, ARRAY_SIZE*sizeof(u32),XAXIDMA_DMA_TO_DEVICE);
	if(status != XST_SUCCESS){
		print("DMA initialization failed\n");
		return -1;
	}
    status = checkHalted(XPAR_AXI_DMA_0_BASEADDR,0x4);
    while(status != 1){
    	status = checkHalted(XPAR_AXI_DMA_0_BASEADDR,0x4);
    }
    status = checkHalted(XPAR_AXI_DMA_0_BASEADDR,0x34);
    while(status != 1){
    	status = checkHalted(XPAR_AXI_DMA_0_BASEADDR,0x34);
    }
	print("DMA transfer success..\n");
	print("write..\n");
	for(int i=0;i<ARRAY_SIZE;i++)
			xil_printf("%0x\n",a[i]);
	print("read..\n");

	for(int i=0;i<ARRAY_SIZE;i++)
		xil_printf("%0x\n",b[i]);
	print("compare data as inverted..\n");


	//xil_printf("%0x\n",a[ARRAY_SIZE]);
	//xil_printf("%0x\n",b[ARRAY_SIZE]);
}


u32 checkHalted(u32 baseAddress,u32 offset){
	u32 status;
	status = (XAxiDma_ReadReg(baseAddress,offset))&XAXIDMA_HALTED_MASK;
	return status;
}
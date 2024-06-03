#include <stdio.h>
#include "platform.h"
#include "xil_printf.h"
#include "xil_io.h"
#include "xparameters.h"
#include "string.h"
#include "xbram.h"

XBram_Config *bramC;
XBram bramI;

int addr = XPAR_PSU_DDR_0_S_AXI_BASEADDR + 0x00600000;
int baddr = XPAR_AXI_BRAM_CTRL_0_S_AXI_BASEADDR;
int d[50];
int b[50];

void bram_init(){

	int status;
	bramC = XBram_LookupConfig(XPAR_AXI_BRAM_CTRL_0_DEVICE_ID);
	status = XBram_CfgInitialize(&bramI, bramC, XPAR_AXI_BRAM_CTRL_0_S_AXI_BASEADDR);
		if (status == XST_SUCCESS){
			xil_printf("bram is success\n");
		}
		else{
			xil_printf("bram is faild\n");
		}

}



int main(){

    init_platform();
    bram_init();



    for(int i = 0 ; i < 50 ; i++)
    {
    	Xil_Out32(addr + 4*i,5*i);
    }


    for(int i = 0 ; i < 50 ; i++)
    {
    	 d[i] = Xil_In32(addr + 4*i);
    	Xil_Out32(baddr + 4*i,d[i]);
    }

    for(int i = 0 ; i < 50 ; i++)
    {
    	b[i] = Xil_In32(baddr + 4*i);
    	if(b[i] == d[i])
    	{
    		printf("pass\n");
    		xil_printf("Value read : %0d\n",b[i]);
    	}
    	else
    	{
    		printf("fail\n");
    	}
    }


    /*
    for(int i = 0 ; i < 50 ; i++)
    {
    	arin[i] = 5*i;
    }


    //memcpy(addr,arin,50*sizeof(int));//destination addr, source addr, no. of sample
    //memcpy(arout,addr,50*sizeof(int));

    for(int i = 0 ; i < 50 ; i++)
    {
    	int temp = arout[i];
    	xil_printf("Value read : %0d\n",temp);
    }

*/
    cleanup_platform();
    return 0;
}

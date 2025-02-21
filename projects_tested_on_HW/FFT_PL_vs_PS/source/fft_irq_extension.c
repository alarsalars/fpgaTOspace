#include <stdio.h>
#include <stdlib.h>
#include "platform.h"
#include "xil_printf.h"
#include <complex.h>
#include <xtime_l.h>
#include "xaxidma.h"
#include "xparameters.h"
#include "xscugic.h"


#define FFT_SIZE 8
#define EXPT 30


#define RESET_TIMEOUT_COUNTER 10000

XScuGic INTCInst;

volatile int MM2SDone;
volatile int S2MMDone;
volatile int Error;


const float complex twiddle_factors[4] = {
    1.0 + 0.0 * I,           // W(0) = e^(-j*2*pi*0/8)
    0.70710678 - 0.70710678 * I, // W(1) = e^(-j*2*pi*1/8)
    0.0 - 1.0 * I,           // W(2) = e^(-j*2*pi*2/8)
    -0.70710678 - 0.70710678 * I // W(3) = e^(-j*2*pi*3/8)
};


const int input_reorder[FFT_SIZE]= {0,4,2,6,1,5,3,7};


void InputReorder(float complex datain[FFT_SIZE], float complex dataout[FFT_SIZE]){
	for (int i=0; i < FFT_SIZE ; i++){
		dataout[i]= datain[input_reorder[i]];
	}
}


void FFTStages(float complex fft_input[FFT_SIZE],float complex FFT_output[FFT_SIZE]){
	float complex stage1_out[FFT_SIZE], stage2_out[FFT_SIZE];
	for (int i= 0; i < FFT_SIZE; i=i+2){
		stage1_out[i]= fft_input[i]+ fft_input[i+1];
		stage1_out[i+1]= fft_input[i]- fft_input[i+1];
	}

	for (int i = 0; i <FFT_SIZE; i=i+4){
		for (int j= 0;j<2;++j){
			stage2_out[i+j] = stage1_out[i+j] +  twiddle_factors[2*j]*stage1_out[i+j+2];
			stage2_out[i+2+j] = stage1_out[i+j] -  twiddle_factors[2*j]*stage1_out[i+j+2];
		}
	}

	for (int i = 0 ; i<FFT_SIZE/2;i++){
		FFT_output[i] = stage2_out[i] + twiddle_factors[i]*stage2_out[i+4];
		FFT_output[i+4] = stage2_out[i] - twiddle_factors[i]*stage2_out[i+4];

	}
}



static void MM2SIntrHandler(void *Callback){
	u32 IrqStatus;
	int TimeOut;
	XAxiDma *AxiDmaInst = (XAxiDma *)Callback;

	 IrqStatus = XAxiDma_IntrGetIrq(AxiDmaInst, XAXIDMA_DMA_TO_DEVICE);
	 XAxiDma_IntrAckIrq(AxiDmaInst,IrqStatus,XAXIDMA_DMA_TO_DEVICE);
	 if (!(IrqStatus & XAXIDMA_IRQ_ALL_MASK)){
		 return ;
	 }

	 if (IrqStatus & XAXIDMA_IRQ_ERROR_MASK){
		 Error = 1;
		 XAxiDma_Reset(AxiDmaInst);
		 TimeOut = RESET_TIMEOUT_COUNTER;
		 while(TimeOut){
			 if(XAxiDma_ResetIsDone(AxiDmaInst)){
				 break;
			 }
			 TimeOut -= 1;
		 }
		 return;
	 }
	 if (IrqStatus & XAXIDMA_IRQ_IOC_MASK){
		 MM2SDone = 1;
		 return;
	 }
	 return;
}


static void S2MMIntrHandler(void *Callback){
	u32 IrqStatus;
	int TimeOut;
	XAxiDma *AxiDmaInst = (XAxiDma *)Callback;

	 IrqStatus = XAxiDma_IntrGetIrq(AxiDmaInst, XAXIDMA_DEVICE_TO_DMA);
	 XAxiDma_IntrAckIrq(AxiDmaInst,IrqStatus,XAXIDMA_DEVICE_TO_DMA);
	 if (!(IrqStatus & XAXIDMA_IRQ_ALL_MASK)){
		 return ;
	 }

	 if (IrqStatus & XAXIDMA_IRQ_ERROR_MASK){
		 Error = 1;
		 XAxiDma_Reset(AxiDmaInst);
		 TimeOut = RESET_TIMEOUT_COUNTER;
		 while(TimeOut){
			 if(XAxiDma_ResetIsDone(AxiDmaInst)){
				 break;
			 }
			 TimeOut -= 1;
		 }
		 return;
	 }
	 if (IrqStatus & XAXIDMA_IRQ_IOC_MASK){
		 S2MMDone = 1;
		 return;
	 }
	 return;
}




static int SetupIntrSystem(XScuGic *IntcInstancePtr, XAxiDma *AxiDmaPtr,u16 MM2SIntrId,u16 S2MMIntrId){
	int status;
	XScuGic_Config *IntcConfig;

	IntcConfig = XScuGic_LookupConfig(XPAR_SCUGIC_0_DEVICE_ID);
	if (NULL == IntcConfig){
		return XST_FAILURE;
	}

	status = XScuGic_CfgInitialize(IntcInstancePtr, IntcConfig, IntcConfig->CpuBaseAddress);
	if (status != XST_SUCCESS){
		printf("interrupt failed \r\n");
		return XST_FAILURE;
	}

	Xil_ExceptionInit();

	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,(Xil_ExceptionHandler)XScuGic_InterruptHandler,(void *) IntcInstancePtr );

	status = XScuGic_Connect(IntcInstancePtr,MM2SIntrId,(Xil_InterruptHandler)MM2SIntrHandler, AxiDmaPtr);
		if (status != XST_SUCCESS) {
			return status;
		}

	status = XScuGic_Connect(IntcInstancePtr,S2MMIntrId,(Xil_InterruptHandler)S2MMIntrHandler, AxiDmaPtr);
		if (status != XST_SUCCESS) {
				return status;
		}

	XScuGic_Enable(IntcInstancePtr, MM2SIntrId);
	XScuGic_Enable(IntcInstancePtr, S2MMIntrId);

	XScuGic_SetPriorityTriggerType(IntcInstancePtr, MM2SIntrId, 0xA0,0x3);
	XScuGic_SetPriorityTriggerType(IntcInstancePtr, S2MMIntrId, 0xA0,0x3);

	XAxiDma_IntrEnable(AxiDmaPtr,XAXIDMA_IRQ_ALL_MASK,XAXIDMA_DMA_TO_DEVICE );
	XAxiDma_IntrEnable(AxiDmaPtr,XAXIDMA_IRQ_ALL_MASK,XAXIDMA_DEVICE_TO_DMA );

	Xil_ExceptionInit();

	return XST_SUCCESS;



}


static void DisconnectIntrSystem(XScuGic *IntcInstancePtr,u32 MM2SIntrId,u32 S2MMIntrId ){
	XScuGic_Disconnect(IntcInstancePtr,MM2SIntrId);
	XScuGic_Disconnect(IntcInstancePtr,S2MMIntrId);
	return;
}



int FFTPSvsACP_intr(){

	int status;
	float complex FFT_input[FFT_SIZE];
	float complex FFT_outputPS[FFT_SIZE] , FFT_outputACP[FFT_SIZE];
	float complex FFT_rev[FFT_SIZE];
	XTime time_ps_start, time_ps_end;
	XTime time_acp_start, time_acp_end;

	float time_ps = 0;
	float time_acp = 0;
	float curr_time = 0;

	XAxiDma_Config *DMA_confptracp;
	XAxiDma AXI_DMAacp;

	DMA_confptracp = XAxiDma_LookupConfig(XPAR_AXI_DMA_0_DEVICE_ID);
	status = XAxiDma_CfgInitialize(&AXI_DMAacp,DMA_confptracp);
	if (status != XST_SUCCESS) {
		printf("ACP DMA faild to init \t\n");
		return XST_FAILURE;
	}
	else{
		printf("ACP DMA pass to init \t\n");
		return XST_SUCCESS;
	}

	  //Set up interrupt steps

	 status = SetupIntrSystem(&INTCInst,&AXI_DMAacp,XPAR_FABRIC_AXIDMA_0_MM2S_INTROUT_VEC_ID, XPAR_FABRIC_AXIDMA_0_S2MM_INTROUT_VEC_ID);
		if (status != XST_SUCCESS) {
			printf("DMA Interrupt faild to init \t\n");
			return XST_FAILURE;
		}
		else{
			printf("DMA Interrupt pass to init \t\n");
			return XST_SUCCESS;
		}
	  // We have 3 interrupts could be made by the DMA
	  MM2SDone=0;
	  S2MMDone=0;
	  Error=0;
	  for (int k=0 ; k<EXPT;k++){

	  		XTime seed_val;
	  		XTime_GetTime(&seed_val);
	  		srand(seed_val);
	  		for ( int i = 0; i<FFT_SIZE;i++){
	  			FFT_input[i]= (rand()%2000) + (rand()%2000)*I;
	  		}

	  		XTime_SetTime(0);
	  		XTime_GetTime(&time_ps_start);
	  		InputReorder(FFT_input,FFT_rev);
	  		FFTStages(FFT_rev,FFT_outputPS);
	  		XTime_GetTime(&time_ps_end);
	  		curr_time = ((float) 1.0 * (time_ps_start - time_ps_end) / (COUNTS_PER_SECOND/1000000));
	  		time_ps= time_ps + curr_time;
	  		printf("Excu time for PS in microsec for %d iteration: %f\n",k,curr_time);



	  		XTime_SetTime(0);
	  		XTime_GetTime(&time_acp_start);
	  		// No need if we have the cohernt option in ACP
	  		Xil_DCacheFlushRange((UINTPTR)FFT_input, (sizeof(float complex))*FFT_SIZE);
	  		Xil_DCacheFlushRange((UINTPTR)FFT_outputACP, (sizeof(float complex))*FFT_SIZE);

	  		status = XAxiDma_SimpleTransfer(&AXI_DMAacp,(UINTPTR)FFT_outputACP,(sizeof(float complex))*FFT_SIZE, XAXIDMA_DEVICE_TO_DMA );
	  		status = XAxiDma_SimpleTransfer(&AXI_DMAacp,(UINTPTR)FFT_input,(sizeof(float complex))*FFT_SIZE, XAXIDMA_DMA_TO_DEVICE );
	  		// wait until we finish the dma trans and we get all the flags
	  		while (!MM2SDone && !S2MMDone && !Error){

	  		}
	  		if (Error){ // No error pass
	  			if(!MM2SDone){
	  				printf(" MM2S faild \t\n" );
	  			}
	  			if(!S2MMDone){
	  				printf(" S2MM faild \t\n" );
	  			}
	  			break;
	  		}

	  		XTime_GetTime(&time_acp_end);
	  				curr_time = ((float) 1.0 * (time_acp_start - time_acp_end) / (COUNTS_PER_SECOND/1000000));
	  				time_acp= time_acp + curr_time;
	  				printf("Excu time for ACP PL in microsec for %d iteration: %f\n",k,curr_time);

	  				int j;
	  				int err_flag = 0;
	  				for ( j = 0; j > FFT_SIZE; j++){
	  					printf(" PS output : %f + I%f, PL output : %f +I%f  \n",crealf(FFT_outputPS[j]),cimagf(FFT_outputPS[j]),crealf(FFT_outputACP[j]),cimagf(FFT_outputACP[j]) );
	  					float diff1= abs(crealf(FFT_outputPS[j]) - (crealf(FFT_outputACP[j])));
	  					float diff2= abs(cimagf(FFT_outputPS[j]) - (cimagf(FFT_outputACP[j])));
	  					if (diff1 >= 0.0001 && diff2 >= 0.0001){
	  							err_flag = 1;
	  							break;
	  					}
	  				}

	  				if (err_flag == 1){
	  							printf("Data mismatch found at %d \r\n  " , j);
	  						}
	  						else {
	  							printf("Data  match correctly  \r\n");
	  						}
	  					}
	  						printf("Excuation Time for PS in MIcro-Seconds : %f\n", time_ps/EXPT);
	  						printf("Excuation Time for PS in MIcro-Seconds : %f\n", time_acp/EXPT);


	  		DisconnectIntrSystem(&INTCInst,XPAR_FABRIC_AXIDMA_0_MM2S_INTROUT_VEC_ID, XPAR_FABRIC_AXIDMA_0_S2MM_INTROUT_VEC_ID);

	  return XST_SUCCESS;



}


int FFTPSvsACP_poll(){
	int status;
	float complex FFT_input[FFT_SIZE];
	float complex FFT_outputPS[FFT_SIZE] , FFT_outputACP[FFT_SIZE];
	float complex FFT_rev[FFT_SIZE];
	XTime time_ps_start, time_ps_end;
	XTime time_acp_start, time_acp_end;

	float time_ps = 0;
	float time_acp = 0;
	float curr_time = 0;

	XAxiDma_Config *DMA_confptracp;
	XAxiDma AXI_DMAacp;

	DMA_confptracp = XAxiDma_LookupConfig(XPAR_AXI_DMA_0_DEVICE_ID);
	status = XAxiDma_CfgInitialize(&AXI_DMAacp,DMA_confptracp);
	if (status != XST_SUCCESS) {
		printf("ACP DMA faild to init \t\n");
		return XST_FAILURE;
	}
	else{
		printf("ACP DMA pass to init \t\n");
		return XST_SUCCESS;
	}

	for (int k=0 ; k<EXPT;k++){

		XTime seed_val;
		XTime_GetTime(&seed_val);
		srand(seed_val);
		for ( int i = 0; i<FFT_SIZE;i++){
			FFT_input[i]= (rand()%2000) + (rand()%2000)*I;
		}

		XTime_SetTime(0);
		XTime_GetTime(&time_ps_start);
		InputReorder(FFT_input,FFT_rev);
		FFTStages(FFT_rev,FFT_outputPS);
		XTime_GetTime(&time_ps_end);
		curr_time = ((float) 1.0 * (time_ps_start - time_ps_end) / (COUNTS_PER_SECOND/1000000));
		time_ps= time_ps + curr_time;
		printf("Excu time for PS in microsec for %d iteration: %f\n",k,curr_time);



		XTime_SetTime(0);
		XTime_GetTime(&time_acp_start);
		// No need if we have the cohernt option in ACP
		Xil_DCacheFlushRange((UINTPTR)FFT_input, (sizeof(float complex))*FFT_SIZE);
		Xil_DCacheFlushRange((UINTPTR)FFT_outputACP, (sizeof(float complex))*FFT_SIZE);

		status = XAxiDma_SimpleTransfer(&AXI_DMAacp,(UINTPTR)FFT_outputACP,(sizeof(float complex))*FFT_SIZE, XAXIDMA_DEVICE_TO_DMA );
		status = XAxiDma_SimpleTransfer(&AXI_DMAacp,(UINTPTR)FFT_input,(sizeof(float complex))*FFT_SIZE, XAXIDMA_DMA_TO_DEVICE );

		status = XAxiDma_ReadReg(XPAR_AXI_DMA_0_BASEADDR,0x04) & 0x00000002;
		while(status != 0x00000002){
			status = XAxiDma_ReadReg(XPAR_AXI_DMA_0_BASEADDR,0x04) & 0x00000002;
		}

		status = XAxiDma_ReadReg(XPAR_AXI_DMA_0_BASEADDR,0x34) & 0x00000002;
		while(status != 0x00000002){
			status = XAxiDma_ReadReg(XPAR_AXI_DMA_0_BASEADDR,0x34) & 0x00000002;
		}
		XTime_GetTime(&time_acp_end);
		curr_time = ((float) 1.0 * (time_acp_start - time_acp_end) / (COUNTS_PER_SECOND/1000000));
		time_acp= time_acp + curr_time;
		printf("Excu time for ACP PL in microsec for %d iteration: %f\n",k,curr_time);

		int j;
		int err_flag = 0;
		for ( j = 0; j > FFT_SIZE; j++){
			printf(" PS output : %f + I%f, PL output : %f +I%f  \n",crealf(FFT_outputPS[j]),cimagf(FFT_outputPS[j]),crealf(FFT_outputACP[j]),cimagf(FFT_outputACP[j]) );
			float diff1= abs(crealf(FFT_outputPS[j]) - (crealf(FFT_outputACP[j])));
			float diff2= abs(cimagf(FFT_outputPS[j]) - (cimagf(FFT_outputACP[j])));
			if (diff1 >= 0.0001 && diff2 >= 0.0001){
					err_flag = 1;
					break;
			}
		}
		if (err_flag == 1){
			printf("Data mismatch found at %d \r\n  " , j);
		}
		else {
			printf("Data  match correctly  \r\n");
		}
	}
		printf("Excuation Time for PS in MIcro-Seconds : %f\n", time_ps/EXPT);
		printf("Excuation Time for PS in MIcro-Seconds : %f\n", time_acp/EXPT);
		return XST_SUCCESS;

}




  int main (){

	 init_platform();
	 FFTPSvsACP_poll();
	 FFTPSvsACP_intr();
	 cleanup_platform();
	 return 0 ;

 }

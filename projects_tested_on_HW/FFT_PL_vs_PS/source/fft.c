
#include <stdio.h>
#include <stdlib.h>
#include "platform.h"
#include "xil_printf.h"
#include <complex.h>
#include <xtime_l.h>
#include "xaxidma.h"
#include <xparameters.h>



#define FFT_SIZE 8
#define DMA_IDLE_MASK 0x00000002
#define DMA_MM2S_MASk 0x04
#define DMA_S2MM_MASk 0x34

const float complex twiddle_factors[4] = {
    1.0 + 0.0 * I,           // W(0) = e^(-j*2*pi*0/8)
    0.70710678 - 0.70710678 * I, // W(1) = e^(-j*2*pi*1/8)
    0.0 - 1.0 * I,           // W(2) = e^(-j*2*pi*2/8)
    -0.70710678 - 0.70710678 * I // W(3) = e^(-j*2*pi*3/8)
};


const int input_reorder[FFT_SIZE]= {0,4,2,6,1,5,3,7};


void input_reorder_fun(float complex datain[FFT_SIZE], float complex dataout[FFT_SIZE]){
	for (int i=0; i < FFT_SIZE ; i++){
		dataout[i]= datain[input_reorder[i]];
	}
}


void fftstages(float complex fft_input[FFT_SIZE],float complex FFT_output[FFT_SIZE]){
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


u32 checkDMAIDLE(u32 BaseAddress, u32 offset){
	u32 status = XAxiDma_ReadReg(BaseAddress, offset) & DMA_IDLE_MASK;
	return status;

}

int main (){
	float complex FFT_input[FFT_SIZE] = {11+23*I,32+10*I,91+94*I,15+69*I,47+96*I,44+12*I,96+17*I,49+58*I};
	float complex FFT_output_PS[FFT_SIZE];
	float complex FFT_rev[FFT_SIZE];
	XTime time_PS_start, time_PS_end;
	XTime time_PL_start, time_PL_end;
	float complex FFT_output_PL[FFT_SIZE];

	printf("\n FFT input: \r\n");
	for (int i=0; i<FFT_SIZE; i++){
		printf("%f %f \n",crealf(FFT_input[i]),cimagf(FFT_input[i]));
	}
	XTime_SetTime(0);
	XTime_GetTime(&time_PS_start);

	input_reorder_fun(FFT_input,FFT_rev);
	fftstages(FFT_rev,FFT_output_PS);
	XTime_GetTime(&time_PS_end);

	XAxiDma_Config *DMA_confptr;
	XAxiDma AxiDMA;
	DMA_confptr = XAxiDma_LookupConfig(XPAR_AXI_DMA_0_DEVICE_ID);
	int status = XAxiDma_CfgInitialize(&AxiDMA,DMA_confptr );
	if ( status == XST_SUCCESS) {
		printf("\n DMA PASS \r\n");
	}
	else {
		printf("\n DMA Failed \r\n");
		return XST_FAILURE;
	}

	XTime_SetTime(0);
	XTime_GetTime(&time_PL_start);
		Xil_DCacheFlushRange((UINTPTR)FFT_input, (sizeof(float complex))*FFT_SIZE);
		Xil_DCacheFlushRange((UINTPTR)FFT_output_PL, (sizeof(float complex))*FFT_SIZE);

	status = XAxiDma_SimpleTransfer(&AxiDMA,(UINTPTR)FFT_output_PL,(sizeof(float complex))*FFT_SIZE, XAXIDMA_DEVICE_TO_DMA );
	if ( status == XST_SUCCESS) {
		printf("\n recive PASS \r\n");
	}
	else {
		printf("\n recive Failed \r\n");
		return XST_FAILURE;
	}

	status = XAxiDma_SimpleTransfer(&AxiDMA,(UINTPTR)FFT_input,(sizeof(float complex))*FFT_SIZE, XAXIDMA_DMA_TO_DEVICE );
	if ( status == XST_SUCCESS) {
		printf("\n send PASS \r\n");
	}
	else {
		printf("\n send Failed \r\n");
		return XST_FAILURE;
	}

	status = checkDMAIDLE(XPAR_AXI_DMA_0_BASEADDR,DMA_MM2S_MASk);
	while(status != DMA_IDLE_MASK){
		status = checkDMAIDLE(XPAR_AXI_DMA_0_BASEADDR,DMA_MM2S_MASk);
	}

	status = checkDMAIDLE(XPAR_AXI_DMA_0_BASEADDR,DMA_S2MM_MASk);
	while(status != DMA_IDLE_MASK){
		status = checkDMAIDLE(XPAR_AXI_DMA_0_BASEADDR,DMA_S2MM_MASk);
	}
	XTime_GetTime(&time_PL_end);




	//// compare

	int j;
	int err_flag = 0;
	for ( j = 0; j < FFT_SIZE; j++){
		printf(" PS output : %f + I%, PL output : %f +I%f  \n",crealf(FFT_output_PS[j]),cimagf(FFT_output_PS[j]),crealf(FFT_output_PL[j]),cimagf(FFT_output_PL[j]) );
		float diff1= abs(crealf(FFT_output_PS[j]) - (crealf(FFT_output_PL[j])));
		float diff2= abs(cimagf(FFT_output_PS[j]) - (cimagf(FFT_output_PL[j])));

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



/*
	printf("\n FFT PS output: \r\n");
	for ( int i = 0; i < FFT_SIZE ; i++){
		printf("%f %f\n",crealf(FFT_output_PS[i]),cimagf(FFT_output_PS[i]) );
	}
*/
	printf("\n ------------------------------------------------------- Execution Time-------------\n");

	float time_processor = 0;
	time_processor = (float)1.0 * (time_PS_end - time_PS_start) /(COUNTS_PER_SECOND/1000000);
	printf("Excuation Time for PS in MIcro-Seconds : %f\n", time_processor);

	float time_hdl = 0;
	time_hdl = (float)1.0 * (time_PL_end - time_PL_start) /(COUNTS_PER_SECOND/1000000);
	printf("Excuation Time for PL in MIcro-Seconds : %f\n", time_hdl);

}

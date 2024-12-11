#include <iostream>
#include <fstream>
#include <algorithm>
#include <stdlib.h>
#include "fft.h"
#include <cstdlib>
fft_complex fftin_or[FFTSIZE];

// there is a python script to generate the input binary values in cases we need it as external .bin file
void generate_data(fft_complex fftin_or[FFTSIZE]) {
    srand(time(NULL));

    for (int k = 0; k < FFTSIZE; k++) {
        float real_part = (float)(rand() % 2001) / 100.0f - 10.0f; // -10.0 to 10.0
        float imag_part = (float)(rand() % 2001) / 100.0f - 10.0f;
        fftin_or[k] = fft_complex(real_part, imag_part);
    }
}


int main(){
	using namespace std;
	fft_stream fft_in_data, fft_out_data;
	fft_complex fftout_fft_array[FFTSIZE];
	cout << "START SIMULATION" << endl;
	cout << "Generate INPUT DATA" << endl;
	generate_data(fftin_or);
	cout << "Finish generating" << endl;
	cout <<" Print the Orignal generated array\n"<< endl;
    for (int i = 0 ; i < FFTSIZE; i++){
    	cout << "Element " << i << ": " << real(fftin_or[i]) << " + " << imag(fftin_or[i]) << "I" << endl;
    }
	cout << "PUSHING DATA INTO AXIS" << endl;
	for (int i =0; i < FFTSIZE;i++){
		data_axis input_value;
		input_value.data = fftin_or[i];
		input_value.last= (i==FFTSIZE-1);
		fft_in_data.write(input_value);
		cout << "Element " << i << ": " << real(input_value.data) << " + " << imag(input_value.data) << "I" << endl;
	}
	cout << "CALLING FFT TRANSFORM HLS" << endl;
	FFT_exc(fft_in_data,fft_out_data);
	cout << "PUSHING DATA OUT OF AXIS" << endl;
	unsigned ptr = 0;
	data_axis output_value;
	do {
		output_value = fft_out_data.read();
		fftout_fft_array[ptr++] = output_value.data;
	}while(!output_value.last);
	cout << "Print BINARY OUTPUT DATA" << endl;
    for (int i = 0 ; i < FFTSIZE; i++){
    	cout << "Element " << i << ": " << real(fftout_fft_array[i]) << " + " << imag(fftout_fft_array[i]) << "I" << endl;
    }
    cout << "SIMULATION DONE" << endl;
    return 0;

}



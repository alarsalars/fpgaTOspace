
// Taha Alars

#include "fft.h"


// axi read function and load the input array of fft

void axis_read(fft_stream &input_stream, fft_complex output_array[FFTSIZE]){
    data_axis tmp_data_in;
    read_loop: for (int i = 0; i < FFTSIZE; i++){
        #pragma HLS PIPELINE II = 1
        tmp_data_in = input_stream.read();
        output_array[i]= (fft_complex)tmp_data_in.data;
    }
}


/// Cooley-Tukey Fast Fourier Transform algorithm

// Bit-Reversal
void bit_reversal(fft_complex fftin_or[FFTSIZE],fft_complex fftin_re[FFTSIZE] ){
    uint16_t bit_count = stages_bits_length; // (uint16_t)log2(FFTSIZE) not synthesizable
    value_loop : for (uint16_t i = 0; i < FFTSIZE; i ++){
	#pragma HLS PIPELINE II=1
        uint16_t reversed_val = 0;
        uint16_t count = i;
        reverse_loop : for (uint16_t j=0;j < bit_count  ;j++){
	#pragma HLS UNROLL factor=4
                reversed_val = (reversed_val << 1) | (count & 1);
                count >>= 1;
        }
        fftin_re[reversed_val] = fftin_or[i];
    }

}

// Twiddle factor
void wfft_ini(fft_complex wfft[FFTSIZE/2]) {
    for (int i = 0; i < FFTSIZE / 2; i++) {
    #pragma HLS UNROLL
        wfft[i] = fft_complex(cos(-TWO_PI * (float)i / FFTSIZE),    // remove 2 * M_PI to save resources
                        sin(-TWO_PI * (float)i / FFTSIZE));
    }
}

// the following function is mainly coded by PhD student that I dont know his name but I learnt it by " Dr. Sumit J Darak" by refernce to the student
void fft(fft_complex fft_input[FFTSIZE],fft_complex wfft[FFTSIZE], fft_complex fft_output[FFTSIZE] ){
    int dft_points = 0;
    int butterfly_w = 0;
    fft_complex temp_fft_upper;
    fft_complex butterfly_wfft;
    // copy input array
    copy_array_loop: for (int k=0; k < FFTSIZE;k++){
	#pragma HLS PIPELINE II = 1
        fft_output[k] = fft_input[k];
    }
    // The first implementation loop which is for the stages no
    stages_loop: for (int stage = 1; stage < FFTSTAGES; stage++){
	#pragma HLS PIPELINE II=1
        dft_points = 1 << stage;// 2*stages ,  Number of points per DFT stage
        butterfly_w = dft_points >>1; // fft sub points/2 ,  Half points per DFT stage
    // Second loop for the Twiddle factor used
    butterfly_weight_loop: for (int i = 0; i < butterfly_w; i++){
	#pragma HLS PIPELINE II=1
            butterfly_wfft = wfft[i * (FFTSIZE >> stage)]; // OR (1 << stage) *i
    //  third loop for calculate the fft output for each weight iteration  with butterfly
    fft_loop: for (int upper = i; upper < FFTSIZE; upper += dft_points){
            int lower = upper + butterfly_w;
            temp_fft_upper = fft_output[upper]; // old upper
            fft_output[upper] = temp_fft_upper + fft_output[lower]* butterfly_wfft;
            fft_output[lower] = temp_fft_upper - fft_output[lower]* butterfly_wfft;
            }
        }
    }
}

void axis_write(fft_complex fft_output_array[FFTSIZE],fft_stream &output_stream ){
    data_axis tmp_data_out;
    tmp_data_out.keep = -1;
    tmp_data_out.strb = -1;
    stream_out_loop: for ( int k = 0; k < FFTSIZE; k++){
        #pragma HLS PIPELINE II = 1
        tmp_data_out.data = (fft_complex)fft_output_array[k];
        tmp_data_out.last = (k == FFTSIZE-1); // when finish we can assign one
        output_stream.write(tmp_data_out);
    }
}





void FFT_exc(fft_stream &s_axis_fft,fft_stream &m_axis_fft ){
	#pragma HLS INTERFACE axis port = s_axis_fft
	#pragma HLS INTERFACE axis port = m_axis_fft
	#pragma HLS INTERFACE ap_ctrl_none port=return
	fft_complex fft_input_array[FFTSIZE];
	fft_complex rv_bit_array[FFTSIZE];
	fft_complex wfft[FFTSIZE/2];
	fft_complex fft_output_array[FFTSIZE];
	#pragma HLS DATAFLOW
	axis_read(s_axis_fft, fft_input_array);
	bit_reversal(fft_input_array,rv_bit_array);
	wfft_ini(wfft);
	fft(rv_bit_array,wfft,fft_output_array);
	axis_write(fft_output_array,m_axis_fft );

}




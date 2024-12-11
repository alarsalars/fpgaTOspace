#ifndef FFT_H
#define FFT_H

// #define _USE_MATH_DEFINES  // Should be a preprocessor directive
#include <ap_fixed.h>
#include "ap_int.h"
#include <hls_stream.h>
#include <hls_math.h>
#include <ap_axi_sdata.h>
#include <stdint.h>
#include <complex>

#define FFTSIZE 16
#define stages_bits_length 4  // ((int)(log2(FFTSIZE)))
#define FFTSTAGES stages_bits_length // ((int)(log2(FFTSIZE)))
#define TWO_PI 6.283185307179586f
// stream --> axi --> complex --> fixed
typedef ap_fixed<32,16> fft_val ;
typedef hls::axis<std::complex<fft_val>,0,0,0>    data_axis;
typedef hls::stream<data_axis>      fft_stream;

typedef std::complex<fft_val>     fft_complex;
void FFT_exc(fft_stream &s_axis_fft,fft_stream &m_axis_fft );
#endif



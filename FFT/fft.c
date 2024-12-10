


#include "fft_new.h"  
#ifndef CMPLX
#define CMPLX(x, y) ((x) + (y)*I)
#endif

void generate_data(float complex fftin_or[FFTSIZE]){
        for (int k=0; k<FFTSIZE; k++){
        fftin_or[k]   =(k*7/4)+(k*11/2)*I;
    }
}




// Cooley-Tukey
void bit_reversal(float complex fftin_or[FFTSIZE],float complex fftin_re[FFTSIZE] ){
    uint16_t bit_count = (uint16_t)log2(FFTSIZE);
    value_loop : for (uint16_t i = 0; i < FFTSIZE; i ++){
        uint16_t reversed_val = 0;
        uint16_t count = i;
        reverse_loop : for (uint16_t j=0;j < bit_count  ;j++){
                reversed_val <<= 1;
                reversed_val |= count & 1;
                count >>= 1;
        }
        fftin_re[reversed_val] = fftin_or[i];
    }
     
}


void wfft_ini(float complex wfft[FFTSIZE/2]) {
    for (int i = 0; i < FFTSIZE / 2; i++) {
        wfft[i] = CMPLX(0.9998 * cos(-2 * M_PI * (float)i / FFTSIZE), 
                        0.9998 * sin(-2 * M_PI * (float)i / FFTSIZE));
    }
}

// the following function is mainly coded by PhD student that I dont know his name but I learnt it by " Dr. Sumit J Darak" by refernce to the student
void fft(float complex fft_input[FFTSIZE],float complex wfft[FFTSIZE], float complex fft_output[FFTSIZE] ){
    int dft_points = 0;
    int butterfly_w = 0;
    float complex temp_fft_upper;
    float complex butterfly_wfft = 0;
    int lower =0;
    ////////////////////////////////
    copy_fft_loop: for (int k=0; k < FFTSIZE;k++){
        fft_output[k] = fft_input[k];
    }
    ////////////////////////////////
    stages_loop: for (int stage = 1; stage < FFTSTAGES; stage++){
        dft_points = 1 << stage;// 2*stages // Number of points per DFT stage
        butterfly_w = dft_points >>1; // fft sub points/2 // Half points per DFT stage
    ////////////////////////////////
        butterfly_weight_loop: for (int i = 0; i < butterfly_w; i++){
                                        butterfly_wfft = wfft[i * (FFTSIZE >> stage)]; // OR (1 << stage) *i
    ////////////////////////////////                                    
                                        fft_loop: for (int upper = i; upper < FFTSIZE; upper += dft_points){
                                                            lower = upper + butterfly_w; 
                                                            temp_fft_upper = fft_output[upper]; // old upper
                                                            fft_output[upper] = temp_fft_upper + fft_output[lower]* butterfly_wfft;
                                                            fft_output[lower] = temp_fft_upper - fft_output[lower]* butterfly_wfft;
                                        }
        }
    }
}





int main(){

    float complex fftin_or[FFTSIZE];
    float complex fftin_re[FFTSIZE];
    float complex fft_out[FFTSIZE];
    float complex wfft[FFTSIZE];
    printf(" Generate complex array\n");
    generate_data(fftin_or);
    printf(" Print the Orignal generated array\n");
    for (int i = 0 ; i < FFTSIZE; i++){
        printf("Element %d: %f + %fi\n", i, creal(fftin_or[i]), cimag(fftin_or[i]));
    }
    printf(" Bit reversal \n");
    bit_reversal(fftin_or , fftin_re);
    printf(" Print the reverses array\n");
    for (int i = 0 ; i < FFTSIZE; i++){
        printf("Element %d: %f + %fi\n", i, creal(fftin_re[i]), cimag(fftin_re[i]));
    }
    printf(" Calculate the Weight Wfft\n ");
    wfft_ini(wfft);
        printf(" Print the wfft\n");
    for (int i = 0 ; i < FFTSIZE/2; i++){
        printf("Element %d: %f + %fi\n", i, creal(wfft[i]), cimag(wfft[i]));
    }
    printf(" Start fft calculation\n ");
    fft(fftin_re,wfft,fft_out);
    printf(" Print the fft output array\n");
    for (int i = 0 ; i < FFTSIZE; i++){
        printf("Element %d: %f + %fi\n", i, creal(fft_out[i]), cimag(fft_out[i]));
    }
    return 0;

}







#include "fft.h"  


void generate_data(float complex fftin_or[FFTSIZE]){
        for (int k=0; k<FFTSIZE; k++){
        fftin_or[k]   =(k/4)+(k/2)*I;
    }
}




// Cooley-Tukey
void bit_reversal(float complex fftin_or[FFTSIZE],float complex fftin_re[FFTSIZE] ){
    
    value_loop : for (uint16_t i = 0; i < FFTSIZE; i ++){
        uint16_t reversed_val = 0;
        uint16_t count = i;
        reverse_loop : for (uint16_t j=0;j < sizeof(count) * 8;j++){
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





int main(){

    float complex fftin_or[FFTSIZE];
    float complex fftin_re[FFTSIZE];
    printf(" Generate complex array");
    generate_data(fftin_or);
    printf(" Print the Orignal generated array");
    for (int i = 0 ; i < FFTSIZE; i++){
        printf("Element %d: %f + %fi\n", i, creal(fftin_or[i]), cimag(fftin_or[i]));
    }
    printf(" Bit reversal ");
    bit_reversal(fftin_or , fftin_re);
    printf(" Print the reverses array");
        for (int i = 0 ; i < FFTSIZE; i++){
        printf("Element %d: %f + %fi\n", i, creal(fftin_re[i]), cimag(fftin_re[i]));
    }
    


}




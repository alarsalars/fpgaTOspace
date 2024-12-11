
## quick script to generate values and compare with the FFT original code. 

import random
import struct
import numpy as np

N = 16

real_parts = [random.uniform(-1.0, 1.0) for _ in range(N)]  # Random real parts
imag_parts = [random.uniform(-1.0, 1.0) for _ in range(N)]  # Random imaginary parts

# Open a binary file for writing
with open("fft_complex_input.bin", "wb") as f:
    for i in range(N):
        # Pack the real and imaginary parts as 32-bit floats
        f.write(struct.pack('f', real_parts[i]))  # Real part
        f.write(struct.pack('f', imag_parts[i]))  # Imaginary part

print("Binary file 'fft_complex_input.bin' created with 16 complex values.")





input_data = [
    0.299988 - 3.81001j, -7.32001 + 3.20999j, -4.67001 - 7.24001j,
    0.599991 + 0.909988j, 9.37999 + 7.09j, -5.44 - 8.19j,
    5.5 + 8.64j, 1.54999 - 7.82001j, 6.68999 + 8.98j,
    3.82999 - 9.27j, -0.520004 - 8j, -4.75 - 7.33j,
    -6.38 + 3.46999j, -4.86 + 7.07999j, 9.92 + 5.78999j,
    -4.34001 + 0.289993j
]


fft_output = np.fft.fft(input_data)


for i, val in enumerate(fft_output):
    print(f"Element {i}: {val.real:.5f} + {val.imag:.5f}I")



def compute_twiddle_factors(N):
    k = np.arange(N)
    W_N = np.exp(-2j * np.pi * k / N)
    return W_N

twiddle_factors = compute_twiddle_factors(N)
print("Twiddle Factors:", twiddle_factors)
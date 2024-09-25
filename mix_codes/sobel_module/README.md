# Sobel module for edge detection

This project is a concept from @HUI HU, I did the coding for the module by using VHDL and the testbench by using verilog to be easier to deal with .bmp files.
Each submodule is tested with VHDl except the sobel top is tested with verilog cuz I read and write a .bmp image.
I provided the image i used, however feel free to change the image but you need to change the following:
1. image path
2. the one line fifo depth
3. sobel_data_module the rows and cols need to be changed to fit with the image, here we add a black frame to the image for padding when some pixel dont have full 3x3 neighborhood 
4. the testbench memory make sure it always bigger than the original image 




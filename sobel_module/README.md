# Sobel Edge Detection Module

This project implements a Sobel edge detection module in VHDL, based on a concept from @HUI HU. I developed the VHDL module, while the testbench is written in Verilog for easier handling of .bmp image files.

Key Features:
Submodule Testing: Each submodule is tested individually using VHDL.
Top-Level Module: The Sobel top module is tested using Verilog, as it reads and writes .bmp images for edge detection.
Usage Instructions:
I have included a sample image, but you can replace it with your own by updating the following parameters:

Image Path: Modify the file path to point to your new image.
FIFO Depth: Adjust the depth of the one-line FIFO based on the new image.
Sobel Data Module: Update the number of rows and columns to match the dimensions of your image. A black frame (padding) is added to ensure proper 3x3 neighborhood processing at the edges.
Testbench Memory: Ensure the testbench memory size exceeds the dimensions of the original image to avoid overflow.




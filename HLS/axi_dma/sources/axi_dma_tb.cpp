
// Taha Alars   
// First step is to verify the DMA TX and RX by sending the same data in a sequance start from 0 - 255
// we send random data
// we make tx and rx as one IP and verify the design
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <stdlib.h>
#include "axi_dma.h"
#include <random>




int main (){
    int status =0;
    using namespace std;

    std::random_device rd; // Obtain a random number from hardware
    std::mt19937_64 gen(rd()); // Seed the generator
    std::uniform_int_distribution<uint64_t> distr; // we cant use data_width directly cuz its not defined by c++ libraries
    cout << " Test AXI DMA TX and RX" << endl;

    data_width array_TX[2048]; // just to make the testbench more flex with the memory, we need only 256 for the real array
    data_width array_RX[2048];
    stream_data rx_st, tx_st;
    uint16_t length = 4 ;
    uint16_t width = 64;

    // we load the axi burst array with a random array through the whole array 
    for ( _loop i = 0; i<length ; i++){
        for (_loop j = 0; j < width; j++){
        uint64_t upper = distr(gen);
        uint64_t lower = distr(gen);
            array_TX[i*width+j] = (data_width)((upper << 64) | lower); // // Combine into a 128-bit value
            // array_TX[i*width+j] = (data_width)(i*width+j)
        }
    }


    // load the array to rx stream 
    for ( _loop i = 0; i<length ; i++){
        for (_loop j = 0; j < width; j++){
            axi_stream_bus tmp_bus;
            tmp_bus.data = array_TX[i*width+j]; 
            tmp_bus.last = (j==width-1);
            rx_st.write(tmp_bus);
        }
    }



AXIDMA_TX_RX(array_TX,tx_st, rx_st,array_RX,length,width);




// we use the lines below if we need to treat the TX and RX as seperate IP with 2 different fucntions
/*
    cout << " Stream to AXI with RX IP "<<endl;
    AXIDMA_RX(rx_st, length, width, array_RX); // array RX should contain the same values now with array TX
    cout << " AXI to stream with TX IP"<<endl;
    AXIDMA_TX(array_TX, length, width,tx_st);
*/
    for ( _loop i = 0; i<length ; i++){
        cout << setfill('0') << setw(2) << "length 'row' " << i << ":"<<endl;
        for (_loop j = 0; j < width; j++){
            cout << setfill('0') << setw(4) << array_TX[i*width+j] << " ";
            cout << setfill('0') << setw(4) << array_RX[i*width+j] << " ";
            if (array_TX[i*width+j] != array_RX[i*width+j]){
                status  = 1;
                cout << "Mismatch found at [" << i << "][" << j << "]" << endl;
                break;  
            }
        }
        cout<<endl;
        if (status){
            cout << "Mismatch found"<<endl;
        break; 
        }
    }
    cout << "Simulation Passed No Mismatch found"<<endl;
    cout<< "End the simulation with AXI DMA TX RX " << endl;

    return status;

} 

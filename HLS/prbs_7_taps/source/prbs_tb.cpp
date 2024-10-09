
// Auther Taha Alars
/// in the testbench we will see the compare value will be mismatched for a bit of an one clock because we are looping back the
// output value with the input value in the testbench. In hardware this should not appear. I made sure to test the loop internally in the main function and it shows no latency


#include "prbs_tb.h"
#include <iostream>



int main(){
    int status =0;

    static bool in_tb = 1;
    static bool out_tb;
    bool compare_tb;
    main_func(in_tb, out_tb, compare_tb);
    //main_func(compare_tb);
    std::cout<<"IN = "<<in_tb<<" OUT = "<<out_tb<<" COMPARE = "<<compare_tb<<std::endl;
    //std::cout<<" COMPARE = "<<compare_tb<<std::endl;
    in_tb = out_tb;


    for (int i = 0; i < 7;i++){
        main_func(in_tb, out_tb, compare_tb);
        //main_func(compare_tb);
        std::cout<<"IN = "<<in_tb<<" OUT = "<<out_tb<<" COMPARE = "<<compare_tb<<std::endl;
        //std::cout<<" COMPARE = "<<compare_tb<<std::endl;
        in_tb = out_tb;
    }

    for (int i = 0; i < 21;i++){
        main_func(in_tb, out_tb, compare_tb);
        //main_func(compare_tb);
        std::cout<<"IN = "<<in_tb<<" OUT = "<<out_tb<<" COMPARE = "<<compare_tb<<std::endl;
        //std::cout<<" COMPARE = "<<compare_tb<<std::endl;
        in_tb = out_tb;
        status = compare_tb;
         if (status == 0){
            std::cout<<"Test passed"<<std::endl;
        }
        else {
           std::cout<<"Test Failed"<<std::endl;
        }
    }
    return status;
}



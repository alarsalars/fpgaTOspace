// --------------------------------
// --------- Reference @HUI HU---------- 
// I had to use verilog as @HUI HU did cuz it gives more flexibality to deal with image files
// --------------------------------  

// Testbench

`define clk_period 10   

module rgb_to_grayscale_tb ();

    reg clk, rst;
    reg [7:0] red_i, green_i, blue_i;
    reg done_i;

    wire [7:0] grayscale_o;
    wire done_o;

    rgb_to_grayscale rgb_to_grayscale(
        clk,     
        rst,     
        red_i,    
        green_i,    
        blue_i,    
        done_i, 
        grayscale_o,
        done_o     
    );

initial clk = 1'b1;
always #(`clk_period/2) clk = ~clk; 
// -------------------------------THE Gray results array ----------------------------------
integer i=0;
integer j=0;
localparam results_array_len = 1024*1024; 
reg[7:0] results_array[0 : results_array_len-1]; // new array to store the new grayscale data
always @(posedge clk) begin
    if (rst) begin 
        j = 8'd0; 
    end else begin
        if(done_o)begin // load the new data to the new array
            results_array[j]  = grayscale_o;
            results_array[j+1]= grayscale_o;
            results_array[j+2]= grayscale_o;
            j=j+3;
        end
    end

end

//--------------------------------------------------------------------


`define read_filename "C:\\Users\\TAlars\\Downloads\\sample1.bmp"
localparam bmp_array_len = 1024*1024;

reg[7:0] bmp_data[0:bmp_array_len -1 ];

integer bmp_file_size; //the size of the whole data in bytes
integer bmp_start_pos; // start offset of data
integer bmp_width;     // the width of the pixel bitmap (image width)
integer bmp_hight; // the hight of pixel bitmap  (image hight)
integer bit_count_pix; // the data width 24 bits 8 bits for each color 

task readbmp; // Read the image data
integer fileid, i;
    begin
        fileid = $fopen(`read_filename,"rb");
            if (fileid == 0) begin 
                $display("Error open file\n");
            end else begin 
                $fread(bmp_data,fileid); 
                $fclose(fileid);
                // the following are refrenced from wikipedia bmp file format first example image data with 24 bits 
                bmp_file_size = {bmp_data[5],bmp_data[4],bmp_data[3],bmp_data[2]};
                $display("BMP size= %d!\n", bmp_file_size);
                bmp_start_pos = {bmp_data[13],bmp_data[12],bmp_data[11],bmp_data[10]};
                $display("BMP start position = %d!\n", bmp_start_pos);
                bmp_width = {bmp_data[21],bmp_data[20],bmp_data[19],bmp_data[18]};
                $display("BMP pixel width bitmap = %d!\n", bmp_width);
                bmp_hight = {bmp_data[25],bmp_data[24],bmp_data[23],bmp_data[22]};
                $display("BMP pixel hight bitmap = %d!\n", bmp_hight);
                bit_count_pix = {bmp_data[29],bmp_data[28]};
                $display("BMP pixel data width = %d!\n", bit_count_pix);
                if (bit_count_pix != 24) begin  // the data has to be 24 in order to have 8 bits for each RGB
                $display("bit_count_pix not equal to 24\n");  
                $finish;
                end
                if (bmp_width % 4 ) begin  // it should be divided by 4 otherwise it needs to be fixed
                    $display("bmp_width needs to be zero \n");
                    $finish;
                end
                //for (i = bmp_start_pos; i < bmp_file_size ; i = i+1) begin // print the image read data you can confirm the same data by opening the image with binary read software like HXD
                //    $display("%h",bmp_data[i]);
               // end
            end 
    end

endtask 

`define write_filename "C:\\Users\\TAlars\\Downloads\\results.bmp"

task writebmp;  // Write the data again in a image .bmp
    integer fileid, i;
    begin
        fileid = $fopen(`write_filename, "wb"); 
        for (i = 0; i < bmp_start_pos; i= i+1 ) begin // write the image info first start from 0 to data position
            $fwrite(fileid, "%c", bmp_data[i]);

        end

        for(i=0; i<bmp_file_size;i=i+1)begin /// second write the image new  grayscale data from the new array to the file start from position till the image size
            $fwrite(fileid, "%c", results_array[i]);
        end
        $fclose(fileid);
        $display("write bmp is done \n");
        
    end

endtask;

    initial begin
        rst = 1'b1;
        done_i = 1'b0;

        red_i   = 8'd0;
        green_i = 8'd0;
        blue_i  = 8'd0;

        #(`clk_period);   
        rst = 1'b0;
        readbmp;
        for (i= bmp_start_pos; i<bmp_file_size; i = i+3) begin //the data in the array loaded as Blue green red so when we assign start from i+2 to i
            red_i   = bmp_data[i+2];
            green_i = bmp_data[i+1];
            blue_i  = bmp_data[i];
            #(`clk_period);
            done_i = 1'b1;
            
        end
        #(`clk_period);
        done_i = 1'b0;
        #(`clk_period);
        writebmp;
        #(`clk_period);
        $stop;   

    end

endmodule   

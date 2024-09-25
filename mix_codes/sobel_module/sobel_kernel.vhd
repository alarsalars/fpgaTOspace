LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.numeric_std.all;



ENTITY sobel_kernel IS 
    PORt ( 
        -------------- system ----------------------
        CLK     : IN std_logic;
        RST     : IN std_logic;
        ----------- // INPUT  /// ------ 
        grayscale_i : IN std_logic_vector(7 DOWNTO 0);
        done_i      : IN std_logic;        
        -----------  /// OUTPUT /// ------
        grayscale_o : OUT std_logic_vector(7 DOWNTO 0);
        done_o      : OUT std_logic
       

    );

END ENTITY;



ARCHITECTURE Arch_sobel_kernel  OF  sobel_kernel IS 


SIGNAL data_reg0  : std_logic_vector(7 DOWNTO 0);
SIGNAL data_reg1  : std_logic_vector(7 DOWNTO 0);
SIGNAL data_reg2  : std_logic_vector(7 DOWNTO 0);
SIGNAL data_reg3  : std_logic_vector(7 DOWNTO 0);
SIGNAL data_reg4  : std_logic_vector(7 DOWNTO 0);
SIGNAL data_reg5  : std_logic_vector(7 DOWNTO 0);
SIGNAL data_reg6  : std_logic_vector(7 DOWNTO 0);
SIGNAL data_reg7  : std_logic_vector(7 DOWNTO 0);
SIGNAL data_reg8  : std_logic_vector(7 DOWNTO 0);
SIGNAL done_ff   : std_logic;

component sobel_data_buffer is
    port (
        clk         : IN std_logic;
        rst         : IN std_logic;
        -----------  /// INPUT /// ------
        grayscale_i : IN std_logic_vector(7 DOWNTO 0);
        done_i      : IN std_logic;
        ----------- // OUTPUT  /// ------ 
        data0_o     : OUT std_logic_vector(7 DOWNTO 0);
        data1_o     : OUT std_logic_vector(7 DOWNTO 0);
        data2_o     : OUT std_logic_vector(7 DOWNTO 0);
        data3_o     : OUT std_logic_vector(7 DOWNTO 0);
        data4_o     : OUT std_logic_vector(7 DOWNTO 0);
        data5_o     : OUT std_logic_vector(7 DOWNTO 0);
        data6_o     : OUT std_logic_vector(7 DOWNTO 0);
        data7_o     : OUT std_logic_vector(7 DOWNTO 0);
        data8_o     : OUT std_logic_vector(7 DOWNTO 0);
        done_o      : OUT std_logic

    );
end component;

component sobel_calc is
    port (
        clk         : IN std_logic;
        rst         : IN std_logic;
        ----------- // INPUT  /// ------ 
        data0_i     : IN std_logic_vector(7 DOWNTO 0);
        data1_i     : IN std_logic_vector(7 DOWNTO 0);
        data2_i     : IN std_logic_vector(7 DOWNTO 0);
        data3_i     : IN std_logic_vector(7 DOWNTO 0);
        data4_i     : IN std_logic_vector(7 DOWNTO 0);
        data5_i     : IN std_logic_vector(7 DOWNTO 0);
        data6_i     : IN std_logic_vector(7 DOWNTO 0);
        data7_i     : IN std_logic_vector(7 DOWNTO 0);
        data8_i     : IN std_logic_vector(7 DOWNTO 0);
        done_i      : IN std_logic;
        -----------  /// OUTPUT /// ------
        grayscale_o : OUT std_logic_vector(7 DOWNTO 0);
        done_out      : OUT std_logic

    );
end component;


BEGIN 




sobel_data_buffer_int: sobel_data_buffer

    port map (
        clk          => clk,
        rst          => rst,    
        grayscale_i  => grayscale_i,  
        done_i       => done_i,  
        data0_o      => data_reg0,   
        data1_o      => data_reg1,   
        data2_o      => data_reg2,   
        data3_o      => data_reg3,   
        data4_o      => data_reg4,   
        data5_o      => data_reg5,   
        data6_o      => data_reg6,   
        data7_o      => data_reg7,   
        data8_o      => data_reg8,   
        done_o       => done_ff    
        
    );

sobel_calc_int : sobel_calc

    port map (
        clk          => clk,
        rst          => rst,
        data0_i      => data_reg0,
        data1_i      => data_reg1,
        data2_i      => data_reg2,
        data3_i      => data_reg3,
        data4_i      => data_reg4,
        data5_i      => data_reg5,
        data6_i      => data_reg6,
        data7_i      => data_reg7,
        data8_i      => data_reg8,
        done_i       => done_ff,
        grayscale_o  => grayscale_o,
        done_out       => done_o
        
    );







END Arch_sobel_kernel;
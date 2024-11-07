LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.numeric_std.all;


ENTITY sobel_data_buffer IS 
    PORT (
        clk : IN std_logic;
        rst : IN std_logic;
        -----------  /// INPUT /// ------
        grayscale_i : IN std_logic_vector(7 DOWNTO 0);
        done_i      : IN std_logic;
        ----------- // OUTPUT  /// ------ 
        data0_o  : OUT std_logic_vector(7 DOWNTO 0);
        data1_o  : OUT std_logic_vector(7 DOWNTO 0);
        data2_o  : OUT std_logic_vector(7 DOWNTO 0);
        data3_o  : OUT std_logic_vector(7 DOWNTO 0);
        data4_o  : OUT std_logic_vector(7 DOWNTO 0);
        data5_o  : OUT std_logic_vector(7 DOWNTO 0);
        data6_o  : OUT std_logic_vector(7 DOWNTO 0);
        data7_o  : OUT std_logic_vector(7 DOWNTO 0);
        data8_o  : OUT std_logic_vector(7 DOWNTO 0);
        done_o   : OUT std_logic
    );

END ENTITY;



ARCHITECTURE Arch_sobel_data_buffer  OF  sobel_data_buffer IS 

SIGNAL data0_reg : std_logic_vector(7 DOWNTO 0);
SIGNAL data1_reg : std_logic_vector(7 DOWNTO 0);
SIGNAL data2_reg : std_logic_vector(7 DOWNTO 0);
SIGNAL done_f    : std_logic;

component fifo_double_module is
    port (
        clk     : IN std_logic;
        rst     : IN std_logic;
        ------- //// INPUT 
        we_i    : IN std_logic;
        data_i  : IN std_logic_vector(7 DOWNTO 0);
        ------- //// OUTPUT 
        data0_o : OUT std_logic_vector(7 DOWNTO 0);
        data1_o : OUT std_logic_vector(7 DOWNTO 0);
        data2_o : OUT std_logic_vector(7 DOWNTO 0);
        done_o  : OUT std_logic
        
    );
end component;


component sobel_data_module is
    port (
        clk     : IN std_logic;
        rst     : IN std_logic;
        ------- //// INPUT 
        data0_i :IN std_logic_vector(7 DOWNTO 0);
        data1_i :IN std_logic_vector(7 DOWNTO 0);
        data2_i :IN std_logic_vector(7 DOWNTO 0);
        done_i  :IN std_logic;
        ------- //// OUTPUT
        data0_o  : OUT std_logic_vector(7 DOWNTO 0);
        data1_o  : OUT std_logic_vector(7 DOWNTO 0);
        data2_o  : OUT std_logic_vector(7 DOWNTO 0);
        data3_o  : OUT std_logic_vector(7 DOWNTO 0);
        data4_o  : OUT std_logic_vector(7 DOWNTO 0);
        data5_o  : OUT std_logic_vector(7 DOWNTO 0);
        data6_o  : OUT std_logic_vector(7 DOWNTO 0);
        data7_o  : OUT std_logic_vector(7 DOWNTO 0);
        data8_o  : OUT std_logic_vector(7 DOWNTO 0);
        done_o   : OUT std_logic
        
    );
end component;



BEGIN 


fifo_double_module_inst : fifo_double_module
    port map (
        clk      => clk,
        rst      => rst,
        -------  => 
        we_i     => done_i,
        data_i   => grayscale_i,
        -------  => 
        data0_o  => data0_reg,
        data1_o  => data1_reg,
        data2_o  => data2_reg,
        done_o   => done_f

        
    );


    sobel_data_module_inst : sobel_data_module
    port map (
        clk      => clk,
        rst      => rst,
        -------  => 
        data0_i  => data0_reg,
        data1_i  => data1_reg,
        data2_i  => data2_reg,
        done_i   => done_f,
        -------  => 
        data0_o  => data0_o,
        data1_o  => data1_o,
        data2_o  => data2_o,
        data3_o  => data3_o,
        data4_o  => data4_o,
        data5_o  => data5_o,
        data6_o  => data6_o,
        data7_o  => data7_o,
        data8_o  => data8_o,
        done_o   => done_o 
    );


    END Arch_sobel_data_buffer;
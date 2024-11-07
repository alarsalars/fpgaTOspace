-- --------------------------------
-- --------- Reference @HUI HU---------- 
-- --------------------------------  --
-- Testbench

LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.numeric_std.all;



ENTITY sobel_data_buffer_tb Is 

END ENTITY;


ARCHITECTURE arch_sobel_data_buffer_tb  OF sobel_data_buffer_tb IS 


component sobel_data_buffer is
    port (
        clk          : IN std_logic;
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


-- /// clock 100MHZ
CONSTANT   clk_period  : TIME := 10 ns;

-- //// port singnls
SIGNAL clk     :   std_logic := '0';
SIGNAL rst     :   std_logic := '0';

-----------  /// INPUT /// ------
SIGNAL grayscale_i :  std_logic_vector(7 DOWNTO 0);
SIGNAL done_i      :  std_logic;
----------- // OUTPUT  /// ------ 
SIGNAL data0_o     :  std_logic_vector(7 DOWNTO 0);
SIGNAL data1_o     :  std_logic_vector(7 DOWNTO 0);
SIGNAL data2_o     :  std_logic_vector(7 DOWNTO 0);
SIGNAL data3_o     :  std_logic_vector(7 DOWNTO 0);
SIGNAL data4_o     :  std_logic_vector(7 DOWNTO 0);
SIGNAL data5_o     :  std_logic_vector(7 DOWNTO 0);
SIGNAL data6_o     :  std_logic_vector(7 DOWNTO 0);
SIGNAL data7_o     :  std_logic_vector(7 DOWNTO 0);
SIGNAL data8_o     :  std_logic_vector(7 DOWNTO 0);
SIGNAL done_o      :  std_logic ; 

BEGIN 

-- /////// Clock process
clk_proc : PROCESS
BEGIN 
    clk <= '1';
    wait for clk_period/2;
    clk <= '0';
    wait for clk_period/2;
END PROCESS;



---- /////////////////////////
sobel_data_buffer_inst: sobel_data_buffer

    port map (
        clk         => clk,
        rst         => rst,
        ----------- => 
        grayscale_i => grayscale_i,
        done_i      => done_i     ,
        ----------- => 
        data0_o     => data0_o,
        data1_o     => data1_o,
        data2_o     => data2_o,
        data3_o     => data3_o,
        data4_o     => data4_o,
        data5_o     => data5_o,
        data6_o     => data6_o,
        data7_o     => data7_o,
        data8_o     => data8_o,
        done_o      => done_o 
        
    );

simu_proc : process 

begin 
    
    rst <= '1';
    grayscale_i <= (others =>'0');
    done_i      <= '0';
    wait for clk_period;
    rst         <= '0';
    done_i      <= '1';

    FOR i IN 1 TO 36 LOOP
    grayscale_i  <= std_logic_vector(to_unsigned(i, 8));
    wait for clk_period;
    END LOOP;
    done_i      <= '0';
    wait for clk_period;

    assert false report "Simulation finished" severity failure;


END PROCESS;

END arch_sobel_data_buffer_tb;

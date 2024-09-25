-- --------------------------------
-- --------- Reference @HUI HU---------- 
-- --------------------------------  --
-- Testbench

--- g = gx_d + gy_d
--- gx_d = gx_p - gx_n 
--- gx_p = d6 +d3*2 + d0 
--- gx_n = d8 + d5*2 + d2

--- gy_d = gy_p - gy_n 
--- gy_p = d0 +d1 *2 +d2 
--- gy_n = d6 +d7*2 +d8



LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.numeric_std.all;



ENTITY sobel_calc_tb Is 

END ENTITY;


ARCHITECTURE arch_sobel_calc_tb  OF sobel_calc_tb IS 

component sobel_calc is
    port (
        clk         : IN std_logic;
        rst         : IN std_logic;
        -----------  /// OUTPUT /// ------
        grayscale_o : OUT std_logic_vector(7 DOWNTO 0);
        done_o      : OUT std_logic;
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
        done_i      : IN std_logic
    );
end component;


-- /// clock 100MHZ
CONSTANT   clk_period  : TIME := 10 ns;

-- //// port singnls
SIGNAL clk     :   std_logic := '0';
SIGNAL rst     :   std_logic := '0';


SIGNAL grayscale_o :  std_logic_vector(7 DOWNTO 0);
SIGNAL done_o      :  std_logic;
SIGNAL data0_i     :  std_logic_vector(7 DOWNTO 0);
SIGNAL data1_i     :  std_logic_vector(7 DOWNTO 0);
SIGNAL data2_i     :  std_logic_vector(7 DOWNTO 0);
SIGNAL data3_i     :  std_logic_vector(7 DOWNTO 0);
SIGNAL data4_i     :  std_logic_vector(7 DOWNTO 0);
SIGNAL data5_i     :  std_logic_vector(7 DOWNTO 0);
SIGNAL data6_i     :  std_logic_vector(7 DOWNTO 0);
SIGNAL data7_i     :  std_logic_vector(7 DOWNTO 0);
SIGNAL data8_i     :  std_logic_vector(7 DOWNTO 0);
SIGNAL done_i      :  std_logic;


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
sobel_calc_inst: sobel_calc

    port map (
        clk         => clk        ,
        rst         => rst        ,
        grayscale_o => grayscale_o,
        done_o      => done_o     ,
        data0_i     => data0_i    ,
        data1_i     => data1_i    ,
        data2_i     => data2_i    ,
        data3_i     => data3_i    ,
        data4_i     => data4_i    ,
        data5_i     => data5_i    ,
        data6_i     => data6_i    ,
        data7_i     => data7_i    ,
        data8_i     => data8_i    ,
        done_i      => done_i     
        
    );
    simu_proc : process 

    begin 
        
        rst <= '1';
        done_i      <= '0';
        wait for clk_period;
        rst         <= '0';
    -- Initialize input data
        data0_i <= std_logic_vector(to_unsigned(1, 8));
        data1_i <= std_logic_vector(to_unsigned(2, 8));
        data2_i <= std_logic_vector(to_unsigned(3, 8));
        data3_i <= std_logic_vector(to_unsigned(4, 8));
        data4_i <= std_logic_vector(to_unsigned(5, 8));
        data5_i <= std_logic_vector(to_unsigned(6, 8));
        data6_i <= std_logic_vector(to_unsigned(7, 8));
        data7_i <= std_logic_vector(to_unsigned(8, 8));
        data8_i <= std_logic_vector(to_unsigned(9, 8));
        done_i      <= '1';
        wait for 9* clk_period;
        done_i      <= '0';

        assert false report "Simulation finished" severity failure;
    
    
    END PROCESS;
    
    END arch_sobel_calc_tb;
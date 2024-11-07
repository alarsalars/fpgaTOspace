-- --------------------------------
-- --------- Reference @HUI HU---------- 
-- --------------------------------  --
-- Testbench

LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.numeric_std.all;




ENTITY fifo_single_line_tb Is 

END ENTITY;



ARCHITECTURE arch_fifo_single_line_tb  OF  fifo_single_line_tb IS 

COMPONENT fifo_fr_line IS 
    PORT (
        clk     : IN std_logic;
        rst     : IN std_logic;
        we_i    : IN std_logic;
        data_i  : IN std_logic_vector(7 DOWNTO 0);
        data_o  : OUT std_logic_vector(7 DOWNTO 0);
        done_o  : OUT std_logic

    );

END COMPONENT;

-- /// clock 100MHZ
CONSTANT   clk_period  : TIME := 10 ns;

-- //// port singnls
SIGNAL clk     :   std_logic := '0';
SIGNAL rst     :   std_logic := '0';
SIGNAL we_i    :   std_logic;
SIGNAL data_i  :   std_logic_vector(7 DOWNTO 0);
SIGNAL data_o  :   std_logic_vector(7 DOWNTO 0);
SIGNAL done_o  :   std_logic;

BEGIN 

fifo_fr_line_inis : fifo_fr_line 
    PORT MAP (
        clk       => clk   ,
        rst       => rst   ,
        we_i      => we_i  ,
        data_i    => data_i,
        data_o    => data_o,
        done_o    => done_o

    );



-- /////// Clock process
clk_proc : PROCESS
BEGIN 
    clk <= '1';
    wait for clk_period/2;
    clk <= '0';
    wait for clk_period/2;
END PROCESS;

-- //// The process to operate the fifo 

fifo_proc : PROCESS
    BEGIN 
        rst <= '1';
        wait for 3* clk_period;
        rst     <= '0';
        we_i    <= '0';
        data_i  <= (others => '0');
        wait for 3* clk_period;
        we_i    <= '1';
        FOR i IN 0 TO 639 LOOP
            data_i  <= std_logic_vector(to_unsigned(i, 8));
            wait for clk_period;
        END LOOP;
        we_i    <= '0';
        wait for 5* clk_period;
        we_i    <= '1';
        wait for 800 * clk_period;
        assert false report "Simulation finished" severity failure;
END PROCESS;

END arch_fifo_single_line_tb;
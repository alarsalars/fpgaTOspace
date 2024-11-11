LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.numeric_std.all;


ENTITY median_top_tb Is 

END ENTITY;



ARCHITECTURE median_tb_arch  OF  median_top_tb IS 


COMPONENT medina_top IS 
generic (

bit_width : integer := 8
);

port ( 
    clk          : IN std_logic;
    rst          : IN std_logic;
    ----------- // INPUT  /// ------ 
    data_in1     : IN std_logic_vector(bit_width - 1 DOWNTO 0);
    data_in2     : IN std_logic_vector(bit_width - 1 DOWNTO 0);
    data_in3     : IN std_logic_vector(bit_width - 1 DOWNTO 0);
    en_i         : IN std_logic;
    -----------  /// OUTPUT /// ------
    median       : OUT std_logic_vector(bit_width - 1 DOWNTO 0);
    done_med     : OUT std_logic

);


END COMPONENT;


CONSTANT   clk_period  : TIME := 10 ns;
SIGNAL clk : std_logic:= '0';
SIGNAL rst : std_logic:= '0';

SIGNAL data_in1     : std_logic_vector(7 DOWNTO 0);
SIGNAL data_in2     : std_logic_vector(7 DOWNTO 0);
SIGNAL data_in3     : std_logic_vector(7 DOWNTO 0);
SIGNAL en_i         : std_logic;
SIGNAL median       : std_logic_vector(7 DOWNTO 0);
SIGNAL done_med     : std_logic;




begin 


mean_top_inis : medina_top 
    PORT MAP (
        clk        => clk,
        rst        => rst,
        data_in1   => data_in1,
        data_in2   => data_in2,
        data_in3   => data_in3,
        en_i       => en_i    ,
        median     => median  ,
        done_med   => done_med 
    );


clk_proc : PROCESS Is 
BEGIN 
    clk <= '1';
    wait for clk_period/2;
    clk <= '0';
    wait for clk_period/2;
END PROCESS;



mean_test_process : process 
BEGIN 
    rst             <= '1';
    data_in1        <= (others => '0') ;
    data_in2        <= (others => '0') ;
    data_in3        <= (others => '0') ;
    en_i            <= '0';

    wait for 4* clk_period;
    rst             <= '0';
    wait for  clk_period;
    en_i        <='1';
    data_in1    <= x"05";
    data_in2    <= x"09";
    data_in3    <= x"03";
    wait for  clk_period;
    data_in1    <= x"0a";
    data_in2    <= x"0f";
    data_in3    <= x"bb";
    wait for  clk_period;
    data_in1    <= x"c1";
    data_in2    <= x"02";
    data_in3    <= x"f4";
    wait for clk_period;
    en_i        <='0';
    data_in1    <= x"00";
    data_in2    <= x"00";
    data_in3    <= x"00";
    wait for 10* clk_period;
    assert false report "Condition met, stopping simulation" severity failure;
end process;

end median_tb_arch;
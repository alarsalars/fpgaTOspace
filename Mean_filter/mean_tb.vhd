LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.numeric_std.all;




ENTITY mean_top_tb Is 

END ENTITY;


ARCHITECTURE mean_tb_arch  OF  mean_top_tb IS 


COMPONENT mean_top IS 
    generic(
    out_bits : integer := 12
    );
    PORT (
        clk : IN std_logic;
        rst : IN std_logic;
        ----------- // INPUT  /// ------ 
        data_in_en  : IN std_logic;
        data_in     : IN std_logic_vector(7 DOWNTO 0);
        done_in     : IN std_logic;
        -----------  /// OUTPUT /// ------
        data_out      : OUT std_logic_vector(out_bits-1 DOWNTO 0); -- consider the expected number of inputs is 10 packages, otherwise we need to 
        done_out      : OUT std_logic

    );

END COMPONENT;


CONSTANT   clk_period  : TIME := 10 ns;
SIGNAL clk : std_logic:= '0';
SIGNAL rst : std_logic:= '0';

SIGNAL data_in_en   :  std_logic:= '0';
SIGNAL data_in      :  std_logic_vector(7 DOWNTO 0) := (others => '0');
SIGNAL done_in      :  std_logic:= '0';
SIGNAL data_out     :  std_logic_vector(11 DOWNTO 0):= (others => '0'); -- consider the expected number of inputs is 10 packages, otherwise we need to 
SIGNAL done_out     :  std_logic:= '0';
SIGNAL data_tmp      :  unsigned(7 DOWNTO 0) := (others => '0');

BEGIN 

mean_top_inis : mean_top 
    PORT MAP (
        clk             =>   clk       ,
        rst             =>   rst       ,
        data_in_en      =>   data_in_en,
        data_in         =>   data_in   ,
        done_in         =>   done_in   ,
        data_out        =>   data_out  ,
        done_out        =>   done_out     
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
    data_in_en      <= '0';
    data_in         <= (others => '0');
    done_in         <= '0';
    data_tmp        <= x"55";
    wait for 5 * clk_period;
    rst             <= '0';
    wait for     clk_period;
    data_in_en <= '1';
    for i in 0 to 9 loop 
        data_tmp <= data_tmp + x"12";
        data_in  <= std_logic_vector(data_tmp);
        wait for     clk_period;
    end loop;
    done_in         <= '1';
    data_in_en      <= '0';
    wait for   12*   clk_period;
    data_in         <= (others => '0');
    done_in         <= '0';
    wait for  10 *    clk_period;
    assert false report "Condition met, stopping simulation" severity failure;
end process;

end mean_tb_arch;
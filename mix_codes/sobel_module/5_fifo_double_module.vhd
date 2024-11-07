

LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.numeric_std.all;



entity fifo_double_module is 
port( 
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
end entity; 

ARCHITECTURE fifo_double_module_Arch  OF  fifo_double_module IS 


SIGNAL  dataf1_o  : std_logic_vector(7 DOWNTO 0) := (others => '0');
SIGNAL  donef_o  : std_logic := '0';

component fifo_fr_line is
    port (
        clk     : IN std_logic;
        rst     : IN std_logic;
        we_i    : IN std_logic;
        data_i  : IN std_logic_vector(7 DOWNTO 0);
        data_o  : OUT std_logic_vector(7 DOWNTO 0);
        done_o  : OUT std_logic
        
    );
end component;


BEGIN 


data0_o <= data_i;
data1_o <= dataf1_o;
done_o  <= donef_o;


fifo_single_line_1_ins: fifo_fr_line
    port map (
        clk    => clk,    
        rst    => rst  ,  
        we_i   => we_i  , 
        data_i => data_i ,
        data_o => dataf1_o,
        done_o => donef_o 
        
    );


fifo_single_line_2_ins: fifo_fr_line
    port map (
        clk    => clk  ,  
        rst    => rst  ,  
        we_i   => donef_o  , 
        data_i => dataf1_o ,
        data_o => data2_o, 
        done_o => open 
        
    );


END fifo_double_module_Arch;

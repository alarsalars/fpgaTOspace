-- Taha Alars 


-- in this filter i wanted to meet the minimum timing with simple, less resources
-- it takes 3 clocks to get the input array and one more clock to get the median, the enable signal is '1' only for 3 clocks

LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.numeric_std.all;

entity medina_top is 
generic ( 

bit_width : integer := 8
);

port ( 
    clk : IN std_logic;
    rst : IN std_logic;
    ----------- // INPUT  /// ------ 
    data_in1     : IN std_logic_vector(bit_width - 1 DOWNTO 0);
    data_in2     : IN std_logic_vector(bit_width - 1 DOWNTO 0);
    data_in3     : IN std_logic_vector(bit_width - 1 DOWNTO 0);
    en_i         : IN std_logic;
    -----------  /// OUTPUT /// ------
    median       : OUT std_logic_vector(bit_width - 1 DOWNTO 0);
    done_med     : OUT std_logic

);

end entity;

ARCHITECTURE Arch_median_top  OF  medina_top IS 


component sort is
    generic (
        out_bits : integer := 8  
        
    );
    port( 
        clk : IN std_logic;
        rst : IN std_logic;
        ----------- // INPUT  /// ------ 
        data1_in     : IN std_logic_vector(out_bits - 1 DOWNTO 0);
        data2_in     : IN std_logic_vector(out_bits - 1 DOWNTO 0);
        data3_in     : IN std_logic_vector(out_bits - 1 DOWNTO 0);
        en_hs        : IN std_logic;
        -----------  /// OUTPUT /// ------
        max_out     : OUT std_logic_vector(out_bits - 1 DOWNTO 0);
        med_out     : OUT std_logic_vector(out_bits - 1 DOWNTO 0);
        min_out     : OUT std_logic_vector(out_bits - 1 DOWNTO 0);
        done_out    : OUT std_logic
        );
end component;



TYPE DATA_buffer Is ARRAY (8 DOWNTO 0) OF STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL data_in_buf   : DATA_buffer := (others => (others => '0'));
SIGNAL data_sort_buf : DATA_buffer := (others => (others => '0'));


TYPE med_buffer Is ARRAY (2 DOWNTO 0) OF STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL max_med      :  med_buffer := (others => (others => '0'));
SIGNAL med_med      :  med_buffer := (others => (others => '0'));
SIGNAL min_med      :  med_buffer := (others => (others => '0'));



SIGNAL en_hs        : std_logic := '0';
SIGNAL done_out     : std_logic_vector(2 DOWNTO 0) := (others => '0');
SIGNAL done_med_ff  : std_logic_vector(2 DOWNTO 0) := (others => '0');

BEGIN 

gen_components_med: for i in 0 to 2 generate
    med_comp: sort 
        generic map (
            out_bits => bit_width
        )
        port map (
            clk             => clk      ,
            rst             => rst      ,
            data1_in        => data_in_buf(i)      ,
            data2_in        => data_in_buf(i + 3) ,
            data3_in        => data_in_buf(i + 6) ,
            en_hs           => done_out(2),
            max_out         => max_med(i)  ,  -- we need only the the value for i = 0 
            med_out         => med_med(i)  ,  -- we need only the the value for i = 1 , it is the median value
            min_out         => min_med(i)  ,  -- we need only the the value for i = 2 
            done_out        =>done_med_ff(i)
        );
 end generate gen_components_med;

 median    <= med_med(1);
 done_med  <= done_med_ff(2);

 gen_components_sort: for i in 0 to 2 generate
 sort_comp: sort 
     generic map (
         out_bits => bit_width
     )
     port map (
         clk             => clk      ,
         rst             => rst      ,
         data1_in        => data_sort_buf(i)         ,
         data2_in        => data_sort_buf(i + 3)   ,
         data3_in        => data_sort_buf(i + 6)    ,
         en_hs           => en_hs,
         max_out         => data_sort_buf(i*3+2)  ,
         med_out         => data_sort_buf(i*3+1)  ,
         min_out         => data_sort_buf(i*3) ,
         done_out        => done_out(i)
     );
end generate gen_components_sort;





buff_proc : process ( clk, rst) is 
variable count : integer := 0;
begin 
if rising_edge(clk) then 
    if (rst = '1') then 
        data_in_buf <= (others => (others => '0'));
        count := 0;
        en_hs <= '0';
    elsif (en_i = '1') then
        if (count < 9) then 
            data_in_buf(count)    <= data_in1;
            data_in_buf(count+1) <= data_in2;
            data_in_buf(count+2) <= data_in3;
            count := count + 3 ;
            en_hs <= '0';
            if (count = 9) then 
            count := 0;
            en_hs <= '1';
            end if;
        else
        count := 0;
        en_hs <= '0';   
        end if;
    else 
        count := 0;
        en_hs <= '0';
    end if ;
end if;
end process;

 
 

end Arch_median_top ;
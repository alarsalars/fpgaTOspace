-- Taha Alars


LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.numeric_std.all;

ENTITY sort is 
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
end ENTITY;



ARCHITECTURE Arch_sort  OF  sort IS 





BEGIN 

max_proc : process (clk,rst) IS 
begin 
    if rising_edge(clk) then 
        if (rst = '1') then
            max_out <= (others => '0');
            med_out <= (others => '0');
            min_out <= (others => '0');
            done_out<= '0';
        else
            if (en_hs = '1') then        
                if (data1_in >= data2_in AND data1_in >= data3_in) then 
                    max_out <= data1_in;
                elsif (data2_in >= data1_in AND data2_in >= data3_in)then
                    max_out <= data2_in;
                elsif (data3_in >= data2_in AND data3_in >= data1_in)then
                    max_out <= data3_in;
                else 
                    max_out <= (others => '0');
                end if;
                if (data1_in >= data2_in AND data1_in <= data3_in) OR (data1_in >= data3_in AND data1_in <= data2_in)then
                    med_out <= data1_in;
                elsif (data2_in >= data1_in AND data2_in <= data3_in) OR (data2_in >= data3_in AND data2_in <= data1_in) then
                    med_out <= data2_in;
                elsif (data3_in >= data2_in AND data3_in <= data1_in) OR (data3_in >= data1_in AND data3_in <= data2_in)then
                    med_out <= data3_in;
                else 
                    med_out <= (others => '0');
                end if;
                if (data1_in <= data2_in AND data1_in <= data3_in)then
                    min_out <= data1_in;
                elsif (data2_in <= data1_in AND data2_in <= data3_in)then
                    min_out <= data2_in;
                elsif (data3_in <= data2_in AND data3_in <= data1_in)then
                    min_out <= data3_in;
                else 
                    min_out <= (others => '0');
                end if;
                done_out <= '1';
            else
                max_out <= (others => '0');
                med_out <= (others => '0');
                min_out <= (others => '0');
                done_out<= '0';
            end if;

        end if;
    end if;
end process;

end Arch_sort ;
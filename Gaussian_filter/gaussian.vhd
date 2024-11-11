-- Taha Alars

-- Gaussian filter with 3x3 inputs of 3 clocks and use gaussian kernel(mask) as: 
--                                                                                  [1, 2, 1]
--                                                                                  [2, 4, 2]
--                                                                                  [1, 2, 1]
-- Sigma = 1
-- The kernel is normalized (sum of all elements = 16) when used for operations.
-- divide the final gaussian value by 16 for normalized

LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.numeric_std.all;



entity gaussian_top is 
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
    gaussian       : OUT std_logic_vector(bit_width - 1 DOWNTO 0);
    done_gau       : OUT std_logic

);

end entity;




ARCHITECTURE Arch_gaussian_top  OF  gaussian_top IS 


TYPE DATA_buffer Is ARRAY (8 DOWNTO 0) OF unsigned(7 DOWNTO 0);
SIGNAL data_in_buf   : DATA_buffer ;

SIGNAL row0,row1,row2 : unsigned(10 DOWNTO 0);  -- of 11 bits for sum 3 of 8 bits 
SIGNAL con_sum        : unsigned(13 DOWNTO 0); -- of 14 bits to sum 3 of 10 bits 
SIGNAL en_hs          : std_logic := '0';
SIGNAL done_state     : std_logic;

TYPE state_type IS (sum_array, sum_row, divide);
SIGNAL state, next_state : state_type;
SIGNAL gaussian_buf       : unsigned(7 DOWNTO 0);


BEGIN 

buff_proc : process ( clk, rst) is 
variable count : integer := 0;
begin 
if rising_edge(clk) then 
    if (rst = '1') then 
        data_in_buf <= (others => (others => '0'));
        count := 0;
        en_hs <= '0';
    elsif (en_i = '1') then
        if (count < 3) then 
            data_in_buf(count)   <= unsigned(data_in1); --sort the input array
            data_in_buf(count+3) <= unsigned(data_in2); --sort the input array
            data_in_buf(count+6) <= unsigned(data_in3); --sort the input array
            count := count + 1 ;
            en_hs <= '0';
            if (count >= 3) then 
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


conv_proc : process (clk,rst) is
begin  
    if rising_edge(clk) then 
        if (rst = '1') then 
            row0 <= to_unsigned(0, 11);
            row1 <= to_unsigned(0, 11);
            row2 <= to_unsigned(0, 11);
            next_state <= sum_array;
            done_state <= '0';
        else
            case state is  
                when sum_array =>
                done_state <= '0';
                    if en_hs = '1' then 
                        row0 <= resize(unsigned(data_in_buf(0) + (data_in_buf(1)*2) + data_in_buf(2)), 11);-- need 11 bits to sum 3 val of 8 bits
                        row1 <= resize(unsigned(data_in_buf(3)*2 + data_in_buf(4)*4 + data_in_buf(5)*2), 11);-- need 11 bits to sum 3 val of 8 bits
                        row2 <= resize(unsigned(data_in_buf(6) + data_in_buf(7) *2 + data_in_buf(8)), 11);-- need 11 bits to sum 3 val of 8 bits
                        next_state <= sum_row;
                    end if;
                when sum_row =>
                    con_sum <= resize(unsigned(row0 + row1 + row2), 14);
                    next_state <= divide;
                    done_state <= '0';
                when divide =>
                    gaussian_buf <= resize(unsigned(con_sum / 16),8); -- the sum of gaussian kernal 3x3, the max of 14 bits value / 16 needs max 8 bits which is the gaussian output value of one pixel
                    done_state <= '1';
                    next_state <= sum_array;
            end case;
        end if;
    end if;
end process;

gaussian <= std_logic_vector(gaussian_buf);
done_gau <= done_state;

PROCESS (clk, rst)
BEGIN
    if falling_edge(clk) then 
        IF rst = '1' THEN
            state <= sum_array;
        else
            state <= next_state;
        END IF;
    end if;
END PROCESS;


end Arch_gaussian_top ;
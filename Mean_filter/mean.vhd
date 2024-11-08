


-- Taha Alars

-- it should be much simpler than this but I wanted to use the inouts values as variable and the codes handle the count for us, 
-- by doing so, I had to deal with large latancy, it is about 10 clocks, sure, I could optimise it but I dont want to put more time in it.


-- normal mean equation is mean = sum/N
-- however, for smoother noise filtering we will immplement the following mean equation 
-- mean = (sum - min - max) / N-2
-- it would be better to add some Handshaking but for concept it should be sufficient

LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.numeric_std.all;

ENTITY mean_top is 
generic (
    -- Default value for output bits is 12 which means 10 inputs packages
    out_bits : integer := 12  -- 10 x (2^8 -1) = 2550 ~ 4096 = 12 bits
    
);
port( 
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
end ENTITY;



ARCHITECTURE Arch_mean_top  OF  mean_top IS 
    TYPE state_type IS (IDLE, LOAD,DONE,DONE2);
    SIGNAL current_state, next_state : state_type;
    
    TYPE state_machine IS (operate, send);
    SIGNAL op_state, send_state : state_machine := operate;
SIGNAL sum      : unsigned(11 downto 0) := (others => '0') ;
SIGNAL max, min :unsigned(7 downto 0) := (others => '0');
SIGNAL count  : integer  := 0; -- should be power of 2 
signal op_tmp  : unsigned(11 downto 0);
signal power_of_two : integer := 0;

SIGNAL div_done : std_logic := '0';
-- CONSTANT divider : integer := 3 ; --  8= 2 power 3 , 8 in N - 2, 
BEGIN 

-- I would buffer the data_in before use but for simplicity I will  use it directly from the port


sum_proc : process (clk, rst)
begin 
    if rising_edge(clk) then 
        if rst = '1' then
            sum <= (others => '0');   
        elsif data_in_en = '1' then 
            sum <= sum + unsigned(data_in);   
        end if;
    end if;
end process;

max_min_proc : process (clk, rst)
begin 
    if rising_edge(clk) then 
        if rst = '1' then
            max <= (others => '0');   
            min <= (others => '0');   
        elsif data_in_en = '1' then 
            -- Update max if new data is greater
            if unsigned(data_in) > max then
                max <= unsigned(data_in);
            end if;
            -- Update min if new data is smaller
            if (count = 0) then 
                min <= unsigned(data_in);
            elsif unsigned(data_in)< min then
                min <= unsigned(data_in);
            end if;
        end if;
    end if ;
end process;


count_proc : process (clk, rst)
begin
    if rising_edge(clk) then 
        if rst = '1' then
            count <= 0;  
        elsif data_in_en = '1' AND done_in = '0' then 
            count <= count + 1;   
        elsif data_in_en = '0' AND done_in = '1' then
            count <= count;  
        else 
            count <= 0;   
        end if;
    end if;
end process;


mean_proc : process (clk, rst)
begin 
    if rising_edge(clk) then 
        if rst = '1' then
            data_out <= (others => '0');   
            done_out <= '0';
            op_tmp <= (others => '0');
        elsif done_in = '1' then 
                if div_done = '1' then 
                case op_state is 
                when operate => 
                -- Perform the shift with count-2
                -- we need tp clocks to get the data_out or we do  concurrent signal assignment
                op_tmp <= shift_right(sum - max - min,power_of_two);
                send_state <= send;
                when send => 
                data_out <= std_logic_vector(op_tmp);
                done_out <= '1'; 
                send_state <= operate;  
                end case;
                end if; 
        else
        data_out <= (others => '0');   
        done_out <= '0';  
        end if;
    end if;
end process;

process(clk,rst)
    variable temp_div : integer := 0;
begin
    if rising_edge(clk) then 
        if rst = '1' then
            temp_div := 0;
            power_of_two <= 0;
            div_done <= '0';
        elsif (done_in = '1') then
        
        -- Loop to find the power of two
            case current_state is 
            when IDLE => 
                    power_of_two <= 0;
                    div_done <= '0';
                    temp_div := count-2;
                    next_state <= LOAD;
            when LOAD =>       
                if temp_div > 1 then
                temp_div := temp_div / 2;
                power_of_two <= power_of_two + 1;
                next_state <= LOAD;
                elsif temp_div = 1 then 
                div_done <= '0';
                next_state <= DONE;
                else 
                div_done <= '0';
                next_state <= IDLE;
                end if ;  
             when DONE => 
                div_done <= '1';
                next_state <= DONE2;
             when DONE2 => 
                div_done <= '1';
                next_state <= IDLE;
             END CASE;
         else 
            temp_div := 0;
            power_of_two <= 0;
            div_done <= '0';
        end if;
    end if;

end process;
    current_state_process : PROCESS (clk, rst)
    BEGIN
        IF rst = '1' THEN
            current_state <= IDLE;
        ELSIF rising_edge(clk) THEN
            current_state <= next_state;
        END IF;
    END PROCESS;
    operate_state_process : PROCESS (clk, rst)
    BEGIN
        IF rst = '1' THEN
            op_state <= operate;
        ELSIF rising_edge(clk) THEN
            op_state <= send_state;
        END IF;
    END PROCESS;
END Arch_mean_top;


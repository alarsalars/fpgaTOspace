---------------------------------------------------------------
-- Title     uart sender    
-- Project       
------------------
-- File          
-- Author    TAHA ALARS    
-- Email         
-- Organization  
-- Created       
------------------
-- Simulator     
-- Synthesis     
------------------
-- Description : 
---------------------------------------------------------------
-- Hierarchy: 



LIBRARY IEEE;
USE IEEE.std_logic_1164.ALL;
USE IEEE.numeric_std.ALL;


ENTITY uart_tx  IS
    GENERIC (
    length : INTEGER := 8
    );
    PORT (
        clk                 : IN std_logic;                                                                      
        tx_clk              : IN  std_logic;                        
        tx_start            : IN  std_logic;                            
        rst                 : IN  std_logic;                    
        tx_data             : IN  std_logic_vector(7 DOWNTO 0);                        
       -- length              : IN  std_logic_vector(3 DOWNTO 0);   -- 5,6,7,8                     
        parity_type         : IN  std_logic;                            
        parity_en           : IN  std_logic;                            
        stop2               : IN  std_logic;                        
        tx                  : OUT std_logic;                    
        tx_done             : OUT std_logic;                        
        tx_err              : OUT std_logic                      
    );

END uart_tx;


ARCHITECTURE rtl OF uart_tx IS

SIGNAL tx_reg        : std_logic_vector(7 DOWNTO 0) := (others => '0');
SIGNAL start_b       : std_logic := '0';
SIGNAL stop_b        : std_logic := '1';
SIGNAL parity_bit    : std_logic := '0';
SIGNAL count         : INTEGER   :=  0;

TYPE STATE_fsm IS (idle, start_bit, send_data, send_parity, send_first_stop, send_sec_stop, done); 
SIGNAL state : STATE_fsm := idle;
SIGNAL next_state : STATE_fsm := idle;

FUNCTION calculate_parity_odd(data : STD_LOGIC_VECTOR; length : INTEGER) return STD_LOGIC is
        VARIABLE result : STD_LOGIC := '0';
BEGIN
    FOR i IN 0 TO (length-1) LOOP
        result := result XOR data(i);
    END LOOP;
    return result;
END FUNCTION;

FUNCTION calculate_parity_even(data : STD_LOGIC_VECTOR; length : INTEGER) return STD_LOGIC is
    VARIABLE result : STD_LOGIC := '0';
BEGIN
    FOR i IN 0 TO (length-1) LOOP
        result := result XNOR data(i);
    END LOOP;
    return result;
END FUNCTION;


BEGIN 


paritiy_process : PROCESS(tx_clk)
    BEGIN 
        IF (parity_type = '1') THEN   -- Odd parity
            parity_bit <= calculate_parity_odd(tx_data, length);
        ELSE                          -- even parity
            parity_bit <= calculate_parity_even(tx_data, length);
        END IF;
END PROCESS;



state_process : PROCESS(tx_clk)
    BEGIN 
        IF (rst = '1') THEN
            state <= idle ;
        ELSE
            state <= next_state ;
        END IF;
END PROCESS;


tx_FSM : PROCESS(clk) 
    BEGIN 
        CASE(state) IS 
            WHEN idle => 
                tx_done  <= '0';
                tx       <= '1';
                tx_reg   <= (others => '0');
                tx_err   <= '0';
                IF (tx_start) THEN
                    next_state <= start_bit;
                ELSE
                  next_state <= idle;
                END IF;
            
            WHEN start_bit =>
                tx_reg      <= tx_data;
                tx          <= start_b;
                next_state  <= send_data;

            WHEN send_data => 
                IF (count < (length -1)) THEN
                    next_state <= send_data;
                    tx         <= tx_reg(count); 
                ELSIF (parity_en) THEN
                    tx          <= tx_reg(count);
                    next_state  <= send_parity;  
                ELSE
                    tx         <= tx_reg(count);
                    next_state <= send_first_stop;
                END IF;

            WHEN send_parity => 
                tx              <= parity_bit;
                next_state      <= send_first_stop;

            WHEN send_first_stop => 
                tx              <= stop_b;
                IF (stop2) THEN
                    next_state  <= send_sec_stop;
                ELSE 
                    next_state  <= done; 
                END IF;  

            WHEN send_sec_stop => 
                    tx          <= stop_b;
                    next_state  <= done;

            WHEN done => 
                    tx_done     <= '1';
                    next_state  <= idle;

            WHEN OTHERS =>
                    next_state  <= idle;
        END CASE;
END PROCESS;


counter_process : PROCESS(tx_clk) 
    BEGIN 
        CASE (state) IS 
            WHEN idle            => 
                count <= 0;

            WHEN start_bit       => 
                count <= 0;

            WHEN send_data       => 
                count <= count+1;

            WHEN send_parity     => 
                count <= 0;

            WHEN send_first_stop => 
                count <= 0;

            WHEN send_sec_stop   => 
                count <= 0;

            WHEN done            => 
                count <= 0;

            WHEN OTHERS          => 
                count <= 0;
        END  CASE;
END PROCESS;




END ARCHITECTURE rtl;
            










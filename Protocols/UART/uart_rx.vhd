---------------------------------------------------------------
-- Title    uart reciver     
-- Project       
------------------
-- File          
-- Author   TAHA ALARS      
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


ENTITY uart_rx IS 
    GENERIC (
    length : INTEGER := 8
    );
    PORT (
        clk          : IN std_logic;
        rx_clk       : IN STD_LOGIC;                       
        rx_start     : IN STD_LOGIC;                           
        rst          : IN STD_LOGIC;                   
        rx           : IN STD_LOGIC;                       
    --    length       : IN STD_LOGIC_VECTOR(3 DOWNTO 0);                       
        parity_type  : IN STD_LOGIC;                           
        parity_en    : IN STD_LOGIC;                           
        stop2        : IN STD_LOGIC;                       
        rx_out       : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);                       
        rx_done      : OUT STD_LOGIC;                       
        rx_error     : OUT STD_LOGIC                           
    );
END uart_rx;


ARCHITECTURE rtl OF uart_rx IS

SIGNAL parity    : STD_LOGIC;
SIGNAL datard    : STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL count     : INTEGER;
SIGNAL bit_count : INTEGER;

TYPE STATE_fsm IS (idle, start_bit, recive_data, check_parity, check_first_stop, check_sec_stop, done); 
SIGNAL state : STATE_fsm := idle;
SIGNAL next_state : STATE_fsm := idle;




BEGIN 




state_process : PROCESS(rx_clk)
    BEGIN 
        IF (rst = '1') THEN
            state <= idle ;
        ELSE
            state <= next_state ;
        END IF;
END PROCESS;


rx_FSM : PROCESS(clk)
    BEGIN 
        CASE(state) IS
            WHEN idle => 
                rx_done <= '0';
                rx_error <= '0';
                IF (rx_start = '1' AND ( rx = '0')) THEN 
                    next_state <= start_bit;
                ELSE 
                    next_state <= idle;
                END IF;
            
            WHEN start_bit =>
                IF (count = 7 AND rx = '1') THEN
                    next_state <= idle;
                ELSIF (count = 15) THEN
                    next_state <= recive_data;
                ELSE 
                    next_state <= start_bit;
                END IF;

            WHEN recive_data =>
                IF (count = 7) THEN
                    datard <= rx & datard(7 DOWNTO 1);
                ELSIF (count = 15 AND (bit_count = ((length -1)))) THEN
                        CASE (length) IS 
                            WHEN 5 =>
                                rx_out <= datard(7 downto 3);
                            WHEN 6 =>
                                rx_out <= datard(7 downto 2);
                            WHEN 7 =>
                                rx_out <= datard(7 downto 1);
                            WHEN 8 =>
                                rx_out <= datard(7 downto 0);
                            when others =>
                                null;
                         END CASE;
                         IF (parity_type) THEN
                            parity <= XOR datard;
                         ELSE 
                            parity <= XNOR datard;
                            IF (parity_en) THEN
                                next_state <= check_parity;
                            ELSE 
                                next_state <= check_first_stop;
                            END IF;
                          END IF;
                END IF;
                
            WHEN check_parity =>
                IF (count = 7) THEN
                    IF (rx = parity) THEN
                        rx_error <= '0';
                    ELSE 
                        rx_error <= '1';
                    END IF;
                ELSIF (count = 15) THEN
                    next_state <= check_first_stop;
                ELSE 
                    next_state <= check_parity;
                END IF;
            
            WHEN check_first_stop => 
                IF (count = 7) THEN 
                    IF (rx /= '1') THEN
                        rx_error <= '1';
                    ELSE    
                        rx_error <= '0';
                    END IF;
                ELSIF (count = 15) THEN
                    IF (stop2 = '1') THEN
                        next_state <= check_sec_stop;    
                    ELSE 
                        next_state <= done;
                    END IF;
                END IF;

            WHEN check_sec_stop => 
                IF (count = 7) THEN 
                    IF (rx /= '1') THEN
                        rx_error <= '1';
                    ELSE    
                        rx_error <= '0';
                    END IF;
                ELSIF (count = 15) THEN  
                        next_state <= done;
                END IF;
            
            WHEN done =>
                rx_done <= '1';
                next_state <= idle;
                rx_error <= '0';
            
            END CASE;
END PROCESS;

count_process : PROCESS(rx_clk)
    BEGIN 
        CASE(state) IS 
            WHEN idle =>
                count <= 0;
                bit_count <= 0;
            
            WHEN start_bit =>
                IF (count = 15) THEN 
                    count <= count +1;
                ELSE
                    count <= 0;
                END IF;
            
            WHEN recive_data =>
                IF (count = 15) THEN 
                    count <= count +1;
                ELSE
                    count <= 0;
                    bit_count <= 0;
                END IF;       

            WHEN check_parity =>
                IF (count = 15) THEN 
                    count <= count +1;
                ELSE
                    count <= 0;
                END IF;

            WHEN check_first_stop =>
                IF (count = 15) THEN 
                    count <= count +1;
                ELSE
                    count <= 0;
                END IF;

            WHEN check_sec_stop =>
                IF (count = 15) THEN 
                    count <= count +1;
                ELSE
                    count <= 0;
                END IF;

            WHEN done =>
                count <= 0;
                bit_count <= 0;
        END CASE;
END PROCESS;

END ARCHITECTURE rtl;

            


---------------------------------------------------------------
-- Title      clock gen for UART  
-- Project       
------------------
-- File          
-- Author       TAHA ALARS 
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


ENTITY clk_gen  IS
    PORT (

    clk                 : IN  std_logic;   -- clk= 50 MHz
    rst                 : IN  std_logic;
    baud                : IN  std_logic_vector(16 downto 0);
    tx_clk              : OUT std_logic :='0';
    rx_clk              : OUT std_logic :='0'  -- 16 times fater than tx
    );

END clk_gen;


ARCHITECTURE rtl OF clk_gen IS 

    -- TYPE STATE IS (4800, 9600, 14400, 19200, 38400, 57600, 115200, 128000);
    -- SIGNAL state            : STATE := 4800 ;
    SIGNAL rx_max           : INTEGER; -- only 11 bits  -- max count     
    SIGNAL tx_max           : INTEGER; -- only 11 bits  -- max count    
    SIGNAL rx_count         : INTEGER; -- only 14 bits   half count for the clock     
    SIGNAL tx_count         : INTEGER; -- only 14 bits   half count for the clock  



BEGIN 


counter_value : PROCESS(clk)
    BEGIN 
        IF(rst = '1') THEN 
            rx_max <= 0;
            tx_max <= 0;

        ELSE 
            CASE to_integer(unsigned(baud)) is
                WHEN 4800   => 
                rx_max  <= 651;    -- tx_max/16  => 10416/16 = 651
                tx_max  <= 10416;  -- clock/baud => 50M/4800 = 10416
                WHEN 9600 => 
                rx_max  <= 325;
                tx_max  <= 5208;
                WHEN 14400  => 
                rx_max  <= 217;
                tx_max  <= 3472;
                WHEN 19200  =>
                rx_max  <= 163;
                tx_max  <= 2604; 
                WHEN 38400  =>
                rx_max  <= 81;
                tx_max  <= 1302; 
                WHEN 57600  =>
                rx_max  <= 54;
                tx_max  <= 868; 
                WHEN 115200 =>
                rx_max  <= 27;
                tx_max  <= 434; 
                WHEN 128000 =>
                rx_max  <= 24;
                tx_max  <= 392; 
                WHEN OTHERS =>     
                rx_max  <= 25;
                tx_max  <= 5208; 
            END CASE;
        END IF;
END PROCESS;

rx_clk_ini : PROCESS(clk)
    BEGIN 
        IF(rst = '1') THEN
            rx_max      <= 0;
            rx_count    <= 0;
            rx_clk      <= '0';
        ELSE 
            IF(rx_count <= rx_max) THEN 
                rx_count <= rx_count +1;
            ELSE 
                rx_clk <= NOT rx_clk;
                rx_count <= 0;
            END IF;
        END IF;
END PROCESS;

tx_clk_ini : PROCESS(clk)
    BEGIN 
        IF(rst = '1') THEN
            tx_max      <= 0;
            tx_count    <= 0;
            tx_clk      <= '0';
        ELSE 
            IF(tx_count <= tx_max) THEN 
                tx_count <= tx_count +1;
            ELSE 
                tx_clk <= NOT tx_clk;
                tx_count <= 0;
            END IF;
        END IF;
END PROCESS;

END ARCHITECTURE rtl;








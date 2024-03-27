---------------------------------------------------------------
-- Title     spi master    
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
-- Description : it explains the simple state machine of the spi master no sclk used
--    a quick process can be used to generate sclk and send it to slave also make the state machine count on it
---------------------------------------------------------------
-- Hierarchy: 



LIBRARY IEEE;
USE IEEE.std_logic_1164.ALL;
USE IEEE.numeric_std.ALL;


ENTITY spi_m  IS
    GENERIC (
    length : INTEGER := 8
    );
    PORT (
        clk      : IN std_logic;       ----- master domain no sclk used here                            
        rst      : IN std_logic;                             
        wr       : IN std_logic;                            
        ready    : IN std_logic;                               
        op_done  : IN std_logic;                                 
        addr     : IN std_logic_vector(length-1 Downto 0);                              
        din      : IN std_logic_vector(length-1 Downto 0);                             
        dout     : OUT std_logic_vector(length-1 Downto 0);                              
        cs       : OUT std_logic;                             
        mosi     : OUT std_logic;                              
        miso     : IN std_logic;                              
        -- sclk  : std_logic  not used
        done     : OUT std_logic                                                        
                                  
    );

END spi_m;
    

ARCHITECTURE rtl OF spi_m IS

SIGNAL din_reg   : std_logic_vector(16 Downto 0);  -- data, addr and operation
SIGNAL dout_reg  : std_logic_vector(length-1 Downto 0); 

SIGNAL count     : INTEGER;

TYPE STATE_fsm IS (idle , load , check_op , send_data, read_data1 , read_data2 , check_ready);
SIGNAL state : STATE_fsm := idle;

 
BEGIN 


fsm_process : PROCESS(clk,rst)
    BEGIN 
        IF (rst = '1') THEN 
            state   <=  idle;
            count   <=  0;
            cs      <=  '1';
            mosi    <=  '0';
            done    <=  '0';
        ELSE 
            CASE(state)
                WHEN idle => 
                    count   <=  0;
                    cs      <=  '1';
                    mosi    <=  '0';
                    done    <=  '0';
                    state   <=  load;
                
                WHEN load => 
                    din_reg <= (din & addr & wr);
                    state <= check_op;

                WHEN check_op =>
                    IF (wr = '1' AND addr < 32) THEN
                        cs <= '0';
                        state <= send_data;
                    ELSIF (wr = '0' AND addr < 32) THEN
                        state <= read_data1;
                        cs <= '0';
                    ELSE 
                        cs      <=  '1';
                        state   <=  load;
                    END IF;

                WHEN send_data => 
                    IF (count <= 16) THEN 
                        count <= count +1;
                        mosi <= din_reg(count);
                        state <= send_data;
                    ELSE 
                        cs   <= '1';
                        mosi <= '0';
                        IF (op_done = '1') THEN
                            count <= 0:
                            done <= '1';
                            state <= idle;
                        else
                            state <= send_data;
                        END IF;
                    END IF;

                WHEN read_data1 => 
                    IF (count <= 8) THEN  -- only send wr and addr
                        count <= count +1;
                        mosi  <= din_reg(count);
                        state <= read_data2;
                    ELSE 
                        count <= 0;
                        cs    <= '1';
                        state <= check_ready;
                    END IF;
                    
                WHEN check_ready =>
                    IF (ready = '1') THEN 
                        state <= read_data2;
                    ELSE 
                        state <= check_ready;
                    END IF;
                WHEN read_data2 => 
                    IF (count <= 7) THEN 
                        count <= count +1;
                        dout_reg(count) <= miso;
                        state <= read_data2;
                    ELSE 
                        count <= 0;
                        done <= '1';
                        state <= idle;
                    END IF;
                WHEN others =>
                    count <= 0;
                    cs    <= '1';
                    mosi  <='0';

            END CASE;
            END IF;
END PROCESS;

dout <= dout_reg; ---------- should this be ready only when done is '1'

                            

END ARCHITECTURE rtl;
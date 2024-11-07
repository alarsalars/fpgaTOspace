---------------------------------------------------------------
-- Title     spi slave    
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
-- Description : it explains the simple state machine of the spi slave no sclk 
--    recived from the master so, it will use other clock domain for reciving the data,
-- I have a memory store option but it is commented that can be use as loop back for testing both 
-- the master and slave behaviour on the same data in testbench.
---------------------------------------------------------------
-- Hierarchy: 



LIBRARY IEEE;
USE IEEE.std_logic_1164.ALL;
USE IEEE.numeric_std.ALL;


ENTITY spi_s  IS
    GENERIC (
    length : INTEGER := 8
    );
    PORT (
        clk      : IN std_logic;       ----- slave domain no sclk used here                            
        rst      : IN std_logic;                             
        ready    : OUT std_logic;                               
        op_done  : OUT std_logic;                                 
        din      : IN std_logic_vector(length-1 Downto 0);                             
        dout     : OUT std_logic_vector(length-1 Downto 0);                              
        cs       : IN std_logic;                             
        mosi     : IN std_logic;                              
        miso     : OUT std_logic                              
        -- sclk  : std_logic  not used
                                  
    );

END spi_s;
    

ARCHITECTURE rtl OF spi_s IS

--     TYPE mem IS ARRAY (0 TO 31) OF std_logic_vector(7 DOWNTO 0);   -- in case if store memory is needed   (no din and no dout)
--     SIGNAL mem_array : mem;                                        -- in case if store memory is needed   (no din and no dout)          
SIGNAL count    : INTEGER;
SIGNAL datain   : std_logic_vector(15 DOWNTO 0);
SIGNAL dataout  : std_logic_vector(7 DOWNTO 0);
SIGNAL addr_reg : std_logic_vector(7 DOWNTO 0);

TYPE STATE_fsm IS  (idle , detect , store , send_addr , send_data)
SIGNAL  state : STATE_fsm := idle;


BEGIN



---------- fix reciving the input and output vector of 8 bits
-----------   check the state machine
-----------   SIGNAL datain   : std_logic_vector(15 DOWNTO 0);       those might need to double check
-----------   SIGNAL dataout  : std_logic_vector(7 DOWNTO 0);       those might need to double check

fsm_process : PROCESS(clk,rst)
    BEGIN 
        IF (rst = '1') THEN 
            state   <= idle;
            count   <= 0;
            miso    <= '0';
            ready   <= '0';
            op_done <= '0';

        ELSE 
            CASE (state) IS 
                WHEN idle => 
                    count   <= 0;
                    miso    <= '0';
                    ready   <= '0';
                    op_done <= '0';
                    datain <= (others => '0');
                    IF (cs ='0') THEN 
                        state <= detect;
                    ELSE 
                        state <= idle;
                    END IF;
                WHEN detect =>
                    IF ( mosi = '1') THEN 
                        state <= store;
                    ELSE 
                        state <= send_addr;
                    END IF;
                WHEN store => 
                    IF (count <= 15) THEN 
                        datain(count) <= mosi;
                        count <= count +1;
                        state <= store;
                    ELSE 
                        addr_reg <= datain(7 DOWNTO 0);
                        dout     <= datain(15 DOWNTO 8);              
                        -- mem_array(addr_reg) <= datain(15 DOWNTO 8);   -- in case if store memory is needed   
                        state <= idle;
                        count <= 0;
                        op_done <= '1';
                    END IF;

                WHEN send_addr => 
                    IF (count <= 7) THEN 
                        count <= count +1;
                        datain(count) <= mosi;
                        state <= send_addr;
                    ELSE 
                        count   <= 0;
                        state   <= send_data;
                        ready   <= '1';
                        dataout <= din;
                        dataout <= mem_array(datain); -- in case if store memory is needed 
                    END IF;
                WHEN send_data => 
                    ready <= '0';
                    IF (count < 8) THEN 
                        count <= count +1;
                        miso <= dataout(count);
                        state <= send_data;
                    else
                        count <= 0;
                        state <= idle;
                        op_done <= '1';
                    END IF;
                WHEN others => 
                    state <= idle;
            END CASE;
        END IF;
    END PROCESS;




                            

END ARCHITECTURE rtl;



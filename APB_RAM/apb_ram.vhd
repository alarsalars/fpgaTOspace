---------------------------------------------------------------
-- Title     APB RAM    
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
-- Description :  It is a simple memory with APB signals
---------------------------------------------------------------
-- Hierarchy: 



LIBRARY IEEE;
USE IEEE.std_logic_1164.ALL;
USE IEEE.numeric_std.ALL;


ENTITY apb_ram  IS
    PORT (      
        pclk     : IN  std_logic := '0';        
        preset   : IN  std_logic := '0';        
        psel     : IN  std_logic := '0';        
        penable  : IN  std_logic := '0';        
        pwrite   : IN  std_logic := '0';        
        paddr    : IN  std_logic_vector(31 DOWNTO 0) := (others => '0');        
        pwdata   : IN  std_logic_vector(31 DOWNTO 0) := (others => '0');    
        prdata   : OUT std_logic_vector(31 DOWNTO 0);       
        pready   : OUT std_logic;       
        pslverr  : OUT std_logic       
     );
END ENTITY apb_ram;

ARCHITECTURE rtl OF apb_ram IS 

TYPE mem_array IS ARRAY (0 TO 31) OF std_logic_vector(31 DOWNTO 0);
SIGNAL mem : mem_array := (others => (others => '0'));
TYPE STATE_fsm IS (idle, setup, accesss, transfer); 
SIGNAL state : STATE_fsm := idle;


BEGIN 

fsm_process : PROCESS(pclk) 
    	BEGIN 
            IF (preset = '0') THEN 
                state   <= idle;
                prdata  <= (others => '0');
                pready  <= '0';
                pslverr <= '0';
                mem <= (others => (others => '0'));

            ELSE 
                CASE(state) IS 
                    WHEN idle => 
                        prdata  <= (others => '0');
                        pready  <= '0';
                        pslverr <= '0';
                        state   <= setup;

                    WHEN setup => 
                        IF (psel ='1') THEN 
                            state <= accesss;
                        ELSE 
                            state <= setup;
                        END IF;
                    
                    WHEN accesss => 
                        IF (pwrite ='1' AND penable ='1') THEN -- WRITE
                            IF (to_integer(unsigned(paddr)) < 32) THEN
                                mem((to_integer(unsigned(paddr))))  <= pwdata; 
                                state       <= transfer;
                                pslverr     <= '0';
                                pready      <= '0';
                            ELSE 
                                state   <= transfer;
                                pready  <= '1';
                                pslverr <= '1';
                            END IF;
                        
                        ELSIF ( pwrite ='0' AND penable ='1') THEN  -- READ
                            IF (to_integer(unsigned(paddr)) < 32) THEN
                                prdata  <= mem(to_integer(unsigned(paddr)));
                                state   <= transfer;
                                pready  <= '1';
                                pslverr <='0';
                            ELSE 
                                state   <= transfer;
                                pready  <= '1';
                                pslverr <= '1';
                                prdata  <= (others => '0');
                            END IF; 

                        ELSE 
                            state <= setup;
                        END IF;
                    
                    WHEN transfer => 
                        state   <= setup;
                        pready  <= '0';
                        pslverr <= '0'; 

                    WHEN OTHERS =>
                        state   <= idle;

                END CASE;
            END IF;
END PROCESS;
                
END ARCHITECTURE rtl;                          





---------------------------------------------------------------
-- Title       
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



ENTITY i2c  IS
    GENERIC (
    length : INTEGER := 8
    );
    PORT (          
        clk      : IN  std_logic;   
        rst      : IN  std_logic;   
        wr       : IN  std_logic;   
        sda      : INOUT std_logic;      
        scl      : OUT std_logic;      
        addr     : IN  std_logic_vector(length-2 DOWNTO 0);      
        din      : IN  std_logic_vector(length-1 DOWNTO 0);   
        datard   : OUT std_logic_vector(length-1 DOWNTO 0);      
        done     : OUT std_logic;      

);

END i2c;



ARCHITECTURE rtl OF i2c IS 


SIGNAL addrt        : std_logic_vector(length-1 DOWNTO 0) := (others => '0');
SIGNAL temprd       : std_logic_vector(length-1 DOWNTO 0) := (others => '0');
SIGNAL en           : std_logic := '0';
SIGNAL sdat         : std_logic := '0';
SIGNAL count        : integer := '0';
SIGNAL countn       : integer := '0';
SIGNAL addrn, data_rd, datan   : std_logic_vector(length-1 DOWNTO 0) := (others => '0');
SIGNAL sdan         : std_logic := '0';
SIGNAL update       : std_logic := '0';


TYPE mem_array IS ARRAY (0 TO 128) OF std_logic_vector(length-1 DOWNTO 0);
SIGNAL mem : mem_array := (others => (others => '0'));

TYPE STATE_fsm IS (idle, start, send_addr, get_ack1, send_data, get_ack2, read_data, complete, get_addr, send_ack1, get_data, send_ack2); 
SIGNAL state, nstate : STATE_fsm := idle;



BEGIN   


sda_proc : PROCESS(clk)
        begin 
          IF rising_edge(clk) THEN 
            IF (rst = '1') THEN 
                addrt   <=  (others => '0');
                temprd  <=  (others => '0');
                en      <=  '0';
                sdat    <=  '0';
                count   <=   0;
            ELSE 
                CASE (state) IS     
                    WHEN idle => 
                        en     <= '1';
                        scl    <= '1';
                        sdat   <= '1';
                        state  <= start;
                        count  <= 0;
                        done   <= '0';
                        temprd <= (others => '0');
                    
                    WHEN start => 
                        sdat <= '0';
                        addrt <= (addr & wr);
                        state <= send_addr;

                    WHEN send_addr => 
                        IF (count <= 7) THEN 
                            sdat  <= addrt(count);
                            count <= count +1;
                        ELSE 
                            state <= get_ack1;
                            count <=  0;
                            en    <= '0';
                        END IF;
                    
                    WHEN get_ack1 => 
                        IF (sda = '0') THEN 
                            IF (wr == '1') THEN 
                                state <= send_data;
                                en    <= '1';
                            ELSIF (wr = '0') THEN 
                                state <= read_data;
                                en    <= '0';
                            END IF;
                        ELSE 
                            state <= get_ack1;
                        END IF;

                    WHEN send_data => 
                        IF (count <= 7) THEN 
                            sdat  <= din(count);
                            count <= count +1;
                        ELSE 
                            state <= get_ack2;
                            count <= 0;
                            en    <= '0';
                        END IF;
                    
                    WHEN get_ack2 => 
                        IF(sda = '0') THEN 
                            state <= complete;
                        ELSE 
                            state <= get_ack2;
                        END IF;

                    WHEN read_data => 
                        IF (count <= '0') THEN 
                            temprd(count) <= sda;
                            count <= count +1;
                        ELSE 
                            state <= complete;
                            count <= 0;
                        END IF;

                    WHEN complete => 
                        IF (update = '1') THEN 
                            done <= '1';
                            state <= idle;
                        ELSE 
                            state <= complete;
                        END IF;

                    WHEN OTHERS => 
                        state <= idle;

                END CASE;
            END IF;
        END IF;
END PROCESS;



state_proc : PROCESS(clk) 
    BEGIN 
        IF rising_edge(clk) THEN 
            IF (rst = '1') THEN 
                mem     <= (others => (others => '0'));
                addrn   <= (others => '0');
                datan   <= (others => '0');
                update  <= '0';
            ELSE 
                CASE (nstate) IS 
                    WHEN idle => 
                        addrn   <= (others => '0');
                        datan   <= (others => '0');
                        update  <= '0'';
                        data_rd <= (others => '0');
                        IF (scl ='1' AND sdat ='1') THEN 
                            nstate <= start;
                        ELSE 
                            nstate <= idle;
                        END IF;

                    WHEN start => 
                        IF (scl ='1' AND sdat = '0') THEN
                            nstate <= get_addr;
                        ELSE 
                            nstate <= start;
                        END IF;

                    WHEN get_addr => 
                        IF (countn <= 7) THEN 
                            addrn(countn) <= sdat;
                            countn <= countn+1;
                        ELSE 
                            nstate <= send_ack1;
                            countn <= 0;
                        END IF;

                    

                        
                    


























END ARCHITECTURE rtl; 













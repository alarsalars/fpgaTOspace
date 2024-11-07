LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.numeric_std.all;


ENTITY fifo_fr_line IS 
    PORT (
        clk     : IN std_logic;
        rst     : IN std_logic;
        we_i    : IN std_logic;
        data_i  : IN std_logic_vector(7 DOWNTO 0);
        data_o  : OUT std_logic_vector(7 DOWNTO 0);
        done_o  : OUT std_logic
    );
END ENTITY;

ARCHITECTURE behavioral OF fifo_fr_line IS 
CONSTANT depth : integer := 640;
-- CONSTANT depth : integer := 640;
-- simulation
--CONSTANT depth : integer := 6;
TYPE DATA_MEMORY Is ARRAY (0 TO depth-1 ) OF STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL mem : DATA_MEMORY := (others => (others => '0'));

SIGNAL wr_ptr : STD_LOGIC_VECTOR(9 DOWNTO 0); -- to hold 640 different addresses 640 equal to 10 bits
SIGNAL rd_ptr : STD_LOGIC_VECTOR(9 DOWNTO 0); -- to hold 640 different addresses 
SIGNAL icount : integer := 0; -- to increase from 0 to 640 

BEGIN 
-- Signal when FIFO is full
done_o <= '1' WHEN (icount = depth) else '0';
-- Read memory output
data_o <= mem(to_integer(unsigned(rd_ptr)));

----------------- handling the conter -----------------------
icount_proc : PROCESS(clk) IS 

    BEGIN 
        IF (rising_edge(clk)) THEN 
            IF (rst = '1') THEN 
                icount <= 0;
            ELSE 
                IF (we_i = '1') THEN 
                    icount <= (icount + 1) WHEN (icount /= depth) ELSE icount;
                END IF; 
            END IF;
        END IF;
END PROCESS; 


---------------------- handling write process --------------

write_proc : PROCESS(clk) IS    
    BEGIN 
        IF (rising_edge(clk)) THEN 
            IF (rst = '1') THEN
                wr_ptr <= (others => '0');
            ELSE 
                IF (we_i = '1') THEN 
                    mem(to_integer(unsigned(wr_ptr))) <= data_i;
                    wr_ptr <= std_logic_vector(unsigned(wr_ptr) + 1) WHEN ((to_integer(unsigned(wr_ptr))) /= depth - 1) ELSE (others => '0');
                END IF;  
            END IF; 
        END IF ;  
END PROCESS; 

---------------------- handling Read process --------------

read_proc : PROCESS(clk) IS    
    BEGIN 
        IF (rising_edge(clk)) THEN 
            IF (rst = '1') THEN
                rd_ptr <= (others => '0');
            ELSE 
                IF (we_i = '1') THEN 
                    IF (icount = depth) THEN 
                    rd_ptr <= std_logic_vector(unsigned(rd_ptr) + 1) WHEN ((to_integer(unsigned(rd_ptr))) /= depth - 1) ELSE (others => '0');
                    END IF;  
                END IF; 
            END IF ; 
         END IF; 
END PROCESS; 



END behavioral;
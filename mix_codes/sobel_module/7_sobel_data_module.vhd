

LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.numeric_std.all;

entity sobel_data_module is 
port( 
        clk     : IN std_logic;
        rst     : IN std_logic;
        ------- //// INPUT 
        data0_i :IN std_logic_vector(7 DOWNTO 0);
        data1_i :IN std_logic_vector(7 DOWNTO 0);
        data2_i :IN std_logic_vector(7 DOWNTO 0);
        done_i  :IN std_logic;
        ------- //// OUTPUT
        data0_o  : OUT std_logic_vector(7 DOWNTO 0);
        data1_o  : OUT std_logic_vector(7 DOWNTO 0);
        data2_o  : OUT std_logic_vector(7 DOWNTO 0);
        data3_o  : OUT std_logic_vector(7 DOWNTO 0);
        data4_o  : OUT std_logic_vector(7 DOWNTO 0);
        data5_o  : OUT std_logic_vector(7 DOWNTO 0);
        data6_o  : OUT std_logic_vector(7 DOWNTO 0);
        data7_o  : OUT std_logic_vector(7 DOWNTO 0);
        data8_o  : OUT std_logic_vector(7 DOWNTO 0);
        done_o   : OUT std_logic
);
end entity; 

ARCHITECTURE Arch_sobel_data_module  OF  sobel_data_module IS 
-- the image is 640x480
CONSTANT row : INTEGER := 480;
CONSTANT col : INTEGER := 640;

-- CONSTANT row : INTEGER := 480;
-- CONSTANT col : INTEGER := 640;

-- simulation 
-- CONSTANT row : INTEGER := 5;
-- CONSTANT col : INTEGER := 6;
SIGNAl irow, icol : integer := 0;
SIGNAL d0,d1,d2,d3,d4,d5,d6,d7,d8 : std_logic_vector(7 DOWNTO 0) := (others => '0');

SIGNAL count : integer := 0;
SIGNAL done_o_ff : std_logic;

BEGIN 


-- /////// handle rows and cols ///////////
rows_proc : process (clk) IS 
begin 
    IF rising_edge(clk) THEN 
        IF (rst = '1') THEN 
            irow <= 0;
            icol <= 0;
        ELSIF (done_o_ff = '1') THEN 
            icol <= icol + 1 when (icol < col-1 ) else  0;
            IF (icol = col-1) THEN 
                irow <= irow + 1 when (irow < row-1 ) else 0;
            END IF;
        

        END IF;
    END IF; 

END PROCESS;






-- ///////// handle data out ////////////

data_proc : process (all)
begin
    IF (rst = '1') THEN
        data0_o  <= (others => '0');
        data1_o  <= (others => '0');
        data2_o  <= (others => '0');
        data3_o  <= (others => '0');
        data4_o  <= (others => '0');
        data5_o  <= (others => '0');
        data6_o  <= (others => '0');
        data7_o  <= (others => '0');
        data8_o  <= (others => '0');
    ELSIF (done_o_ff = '1') THEN 
        IF (icol = 0 AND irow = 0) THEN 
            data0_o  <= (others => '0');
            data1_o  <= (others => '0');
            data2_o  <= (others => '0');
            data3_o  <= (others => '0');
            data4_o  <= d4;
            data5_o  <= d5;
            data6_o  <= (others => '0');
            data7_o  <= d7;
            data8_o  <= d8;
        ELSIF (irow = 0 AND icol > 0 AND icol < col-1) THEN 
            data0_o  <= (others => '0');
            data1_o  <= (others => '0');
            data2_o  <= (others => '0');
            data3_o  <= d3;
            data4_o  <= d4;
            data5_o  <= d5;
            data6_o  <= d6;
            data7_o  <= d7;
            data8_o  <= d8;
        ELSIF   (irow = 0 AND icol = col-1) THEN 
            data0_o  <= (others => '0');
            data1_o  <= (others => '0');
            data2_o  <= (others => '0');
            data3_o  <= d3;
            data4_o  <= d4;
            data5_o  <= (others => '0');
            data6_o  <= d6;
            data7_o  <= d7;
            data8_o  <= (others => '0');
        ELSIF   (icol = 0 AND irow > 0 AND irow < row-1) THEN 
            data0_o  <= (others => '0');
            data1_o  <= d1;
            data2_o  <= d2;
            data3_o  <= (others => '0');
            data4_o  <= d4;
            data5_o  <= d5;
            data6_o  <= (others => '0');
            data7_o  <= d7;
            data8_o  <= d8;
        ELSIF   (icol > 0 AND icol < col-1 AND irow > 0 AND irow < row-1) THEN 
            data0_o  <= d0;
            data1_o  <= d1;
            data2_o  <= d2;
            data3_o  <= d3;
            data4_o  <= d4;
            data5_o  <= d5;
            data6_o  <= d6;
            data7_o  <= d7;
            data8_o  <= d8;
        ELSIF   (icol = col -1  AND irow > 0 AND irow < row-1) THEN 
            data0_o  <= d0;
            data1_o  <= d1;
            data2_o  <= (others => '0');
            data3_o  <= d3;
            data4_o  <= d4;
            data5_o  <= (others => '0');
            data6_o  <= d6;
            data7_o  <= d7;
            data8_o  <= (others => '0');
        ELSIF   (icol = 0 AND irow = row-1) THEN 
            data0_o  <= (others => '0');
            data1_o  <= d1;
            data2_o  <= d2;
            data3_o  <= (others => '0');
            data4_o  <= d4;
            data5_o  <= d5;
            data6_o  <= (others => '0');
            data7_o  <= (others => '0');
            data8_o  <= (others => '0');
        ELSIF (irow = row -1  AND icol > 0 AND icol < col-1) THEN
            data0_o  <= d0;
            data1_o  <= d1;
            data2_o  <= d2;
            data3_o  <= d3;
            data4_o  <= d4;
            data5_o  <= d5;
            data6_o  <= (others => '0');
            data7_o  <= (others => '0');
            data8_o  <= (others => '0');
        ELSIF (icol = col -1  AND irow = row-1) THEN
            data0_o  <= d0;
            data1_o  <= d1;
            data2_o  <= (others => '0');
            data3_o  <= d3;
            data4_o  <= d4;
            data5_o  <= (others => '0');
            data6_o  <= (others => '0');
            data7_o  <= (others => '0');
            data8_o  <= (others => '0');
        END IF;

    END IF;
    
end process;    
done_o_ff <= '1' WHEN (count = 2) else '0';
done_o <= done_o_ff;
count_proc : PROCESS (clk) IS 
BEGIN 
    IF rising_edge(clk) THEN 
        IF (rst = '1') THEN 
            count <= 0;
        ELSIF (done_i = '1') THEN 
            count <= count WHEN (count = 2) ELSE count +1 ; 
        END IF;
    END IF; 
END PROCESS;




shift_proc : PROCESS(clk) IS 

BEGIN 
    IF rising_edge(clk) THEN 
        IF (rst = '1') THEN 
            d0  <= (others => '0');  
            d1  <= (others => '0');
            d2  <= (others => '0');
            d3  <= (others => '0');
            d4  <= (others => '0');
            d5  <= (others => '0');
            d6  <= (others => '0');
            d7  <= (others => '0');
            d8  <= (others => '0');
        ELSIF (done_i = '1') THEN 
        -- //// second buffer
            d0 <= d1;
            d1 <= d2;
            d2 <= data2_i;
        -- //// first buffer
            d3 <= d4;
            d4 <= d5;
            d5 <= data1_i;
        -- //// direct input data 
            d6 <= d7;
            d7 <= d8;
            d8 <= data0_i;
        END IF;
    END IF;


END PROCESS;


END Arch_sobel_data_module;

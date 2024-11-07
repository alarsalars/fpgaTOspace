
LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.numeric_std.all;


entity sobel_calc is 
port( 
    clk : IN std_logic;
    rst : IN std_logic;
    ----------- // INPUT  /// ------ 
    data0_i  : IN std_logic_vector(7 DOWNTO 0);
    data1_i  : IN std_logic_vector(7 DOWNTO 0);
    data2_i  : IN std_logic_vector(7 DOWNTO 0);
    data3_i  : IN std_logic_vector(7 DOWNTO 0);
    data4_i  : IN std_logic_vector(7 DOWNTO 0);
    data5_i  : IN std_logic_vector(7 DOWNTO 0);
    data6_i  : IN std_logic_vector(7 DOWNTO 0);
    data7_i  : IN std_logic_vector(7 DOWNTO 0);
    data8_i  : IN std_logic_vector(7 DOWNTO 0);
    done_i   : IN std_logic;
    -----------  /// OUTPUT /// ------
    grayscale_o : OUT std_logic_vector(7 DOWNTO 0);
    done_out      : OUT std_logic
);
end entity;


ARCHITECTURE Arch_sobel_calc  OF  sobel_calc IS 


-- // dividing the sobel equation to different part for pipeline purposes
-- // it will rquired 3 clocks to get the final results
---/// it is 10 bits cuz the max value of the equation would be 10 bits 
SIGNAL gx_p     : INTEGER;
SIGNAL gx_n     : INTEGER;
SIGNAL gx_d     : INTEGER;
SIGNAL gy_p     : INTEGER;
SIGNAL gy_n     : INTEGER;
SIGNAL gy_d     : INTEGER;
SIGNAL g_sum    : INTEGER;
SIGNAL g_sum_reg    : std_logic_vector(9 DOWNTO 0);

-- // for syc the different operations  with done_o
SIGNAL count : integer := 0;
--SIGNAL done_shit : std_logic_vctor(3 DOWNTO 0);


BEGIN 
-- Sobel Operator Kernels
--    | data0_i | data1_i | data2_i |
--    | data3_i | data4_i | data5_i |
--    | data6_i | data7_i | data8_i |
-- Gx Kernel (Gradient in X Direction)
-- | -1  0  1 |
-- | -2  0  2 |
-- | -1  0  1 |

-- Gy Kernel (Gradient in Y Direction)
-- |  1  2  1 |
-- |  0  0  0 |
-- | -1 -2 -1 |

-- Note: 
-- The Gx kernel is used to detect horizontal edges,
-- while the Gy kernel is used to detect vertical edges.
-- // process for the X AND Y section 

xY_proc : PROCESS(clk) 
BEGIN 
    IF rising_edge(clk) THEN 
        IF (rst = '1') THEN 
        -- // X
            gx_p  <= 0;
            gx_n  <= 0;
        -- // Y
            gy_p  <= 0;
            gy_n  <= 0;
        else
        --// X
            gx_p <= (to_integer(unsigned(data6_i))) + ((to_integer(unsigned(data3_i))) * 2) + (to_integer(unsigned(data0_i)));
            gx_n <= (to_integer(unsigned(data8_i))) + ((to_integer(unsigned(data5_i))) * 2) + (to_integer(unsigned(data2_i)));
        --// Y
            gy_p <= (to_integer(unsigned(data0_i))) + ((to_integer(unsigned(data1_i))) * 2) + (to_integer(unsigned(data2_i)));
            gy_n <= (to_integer(unsigned(data6_i))) + ((to_integer(unsigned(data7_i))) * 2) + (to_integer(unsigned(data8_i)));
        END IF; 
    END IF;
END PROCESS;

-- // process for the DATA section 

data_proc : PROCESS(clk) 
BEGIN 
    IF rising_edge(clk) THEN 
        IF (rst = '1') THEN 
        -- // X
            gx_d <= 0;
        -- // Y
            gy_d <= 0;
        else
        --// X
        gx_d <= abs(gx_p-gx_n);
        --// Y
        gy_d <= abs(gy_p-gy_n);
        END IF; 
    END IF;
END PROCESS;


sum_proc : PROCESS(clk) 
BEGIN 
    IF rising_edge(clk) THEN 
        IF (rst = '1') THEN 
            g_sum <= 0;
        else
            g_sum <= gx_d + gy_d;
        END IF; 
    END IF;
END PROCESS;

g_sum_reg <= std_logic_vector(to_signed(g_sum, g_sum_reg'length));


-- If the gradient magnitude is 60 or more, the output is set to x"FF" (255) white, it is Threshold 
out_proc : PROCESS(clk) 
BEGIN 
    IF rising_edge(clk) THEN 
        IF (rst = '1') THEN 
            grayscale_o <= (others => '0');
            count <= 0;
            --done_o <= '0';
        else
            IF (count < 3) THEN -- wait 3 clocks to send the output
                grayscale_o <= g_sum_reg(7 DOWNTO 0) WHEN (unsigned(g_sum_reg) < 60) ELSE x"FF";
                count <= count +1;
            else
                grayscale_o <= g_sum_reg(7 DOWNTO 0) WHEN (unsigned(g_sum_reg) < 60) ELSE x"FF";
                done_out <= done_i;
                count <= 0;
            END IF;
        END IF; 
    END IF;
END PROCESS;


END Arch_sobel_calc;









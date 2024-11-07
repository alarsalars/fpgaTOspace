LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.numeric_std.all;



entity grayscale_to_rgb is 
port( 
    clk : IN std_logic;
    rst : IN std_logic;
    ----------- // INPUT  /// ------
    grayscale_i : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
    done_i      : IN STD_LOGIC;
    -----------  /// OUTPUT /// ------
    red_o   : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
    green_o : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
    blue_o  : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
    done_o  : OUT STD_LOGIC
);
END Entity;

ARCHITECTURE Arch_grayscale_to_rgb  OF  grayscale_to_rgb IS 





BEGIN  



process (clk)
begin
    IF rising_edge(clk) THEN 
        IF (rst = '1') THEN 
        red_o    <= (others => '0');
        green_o  <= (others => '0');
        blue_o   <= (others => '0');
        done_o   <= '0';
        ELSE 
            If (done_i = '1') THEN 
                red_o    <= grayscale_i;
                green_o  <= grayscale_i;
                blue_o   <= grayscale_i;
                done_o <= done_i;
            end if;
        --done_o <= '0';
        end IF;
    END IF;
end process;
 
END Arch_grayscale_to_rgb;
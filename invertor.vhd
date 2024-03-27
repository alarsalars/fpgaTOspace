----------------------------------------------------------------------------------
-- Company:
-- Engineer:
--
-- Create Date: 03/21/2024 10:47:23 PM
-- Design Name:
-- Module Name: invertor - Behavioral
-- Project Name:
-- Target Devices:
-- Tool Versions:
-- Description:
--
-- Dependencies:
--
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
--
----------------------------------------------------------------------------------


library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.numeric_STD.ALL;


entity invertor is
    generic  (
        data_width      : INTEGER := 32

    );
    PORT (
        axi_clk         : IN std_logic;
        axi_reset_n     : IN std_logic;
        ---------------------------------   Slavelave ---------------
        s_axis_valid    : IN std_logic;
        s_axis_data     : IN std_logic_vector(data_width-1 DOWNTO 0);
        s_axis_ready    : OUT std_logic;
        --------------------------------    Master -----------------
        m_axis_valid    : OUT std_logic;
        m_axis_data     : OUT std_logic_vector(data_width-1 DOWNTO 0);
        m_axis_ready    : IN std_logic


    );



end invertor;




architecture Behavioral of invertor is

-- SIGNAL counut : INTEGER := 0;

SIGNAL reg_data : std_logic_vector(7 downto 0);
SIGNAL reg : UNSIGNED(7 downto 0);
begin


s_axis_ready <= m_axis_ready;

load_data_processs : PROCESS(axi_clk,axi_reset_n) IS

BEGIN
    IF rising_edge(axi_clk) THEN
        IF (axi_reset_n = '1') THEN
            m_axis_data   <= (others => '0');

        ELSE
            IF (s_axis_valid = '1' AND m_axis_ready = '1') THEN
                FOR i IN 0 to (data_width/8 - 1) LOOP
                    reg_data <= s_axis_data(i*8+7 downto i*8);
                    reg <= TO_UNSIGNED(reg_data,reg_data'length);
                --   reg <= 255 - TO_UNSIGNED(reg_data,reg_data'length);
                    m_axis_data(i*8-7 DOWNTO i*8)   <= std_logic_vector(reg);
                END LOOP;
            END IF;
        END IF;
    END IF;


END PROCESS;

assign_valid_process : PROCESS (axi_clk) IS
    Begin
        If rising_edge(axi_clk) THEN
            m_axis_valid <= s_axis_valid;
        END IF;

END PROCESS;


LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.numeric_std.all;




ENTITY rgb_to_grayscale_tb Is 

END ENTITY;



ARCHITECTURE rgy_gray_tb  OF  rgb_to_grayscale_tb IS 

COMPONENT rgb_to_grayscale IS 
    PORT (
        CLK         : IN std_logic;
        RST         : IN std_logic;
        -------- Input RGB ----
        R_cam_i     : IN std_logic_vector(7 DOWNTO 0);
        G_cam_i     : IN std_logic_vector(7 DOWNTO 0);
        B_cam_i     : IN std_logic_vector(7 DOWNTO 0);
        Done_cam_i  : IN std_logic;
        -------- OUTput grayscale  ----
        grayscale_o : OUT std_logic_vector(7 DOWNTO 0);
        done_o      : OUT std_logic

    );

END COMPONENT;

CONSTANT   clk_period  : TIME := 10 ns;

SIGNAL     CLK         : std_logic:= '0';
SIGNAL     RST         : std_logic := '0';
SIGNAL     R_cam_i     : std_logic_vector(7 DOWNTO 0);
SIGNAL     G_cam_i     : std_logic_vector(7 DOWNTO 0);
SIGNAL     B_cam_i     : std_logic_vector(7 DOWNTO 0);
SIGNAL     Done_cam_i  : std_logic;
SIGNAL     grayscale_o   : std_logic_vector(7 DOWNTO 0) := (others => '0');
SIGNAL     done_o        : std_logic           := '0';

BEGIN 


rgb_to_grayscale_inis : rgb_to_grayscale 
    PORT MAP (
        CLK          =>   CLK        ,
        RST          =>   RST        ,
        R_cam_i      =>   R_cam_i    ,
        G_cam_i      =>   G_cam_i    ,
        B_cam_i      =>   B_cam_i    ,
        Done_cam_i   =>   Done_cam_i ,
        grayscale_o  =>   grayscale_o,
        done_o       =>   done_o     


    );

clk_proc : PROCESS Is 
BEGIN 
    CLK <= '1';
    wait for clk_period/2;
    CLK <= '0';
    wait for clk_period/2;
END PROCESS;


rgb_proc : PROCESS

BEGIN 
    RST          <= '1';
    R_cam_i      <= (others => '0');
    G_cam_i      <= (others => '0');
    B_cam_i      <= (others => '0');
    Done_cam_i   <= '0';



    wait for 5 * clk_period;
    RST          <= '0';
    wait for 5* clk_period;
    R_cam_i      <= X"04";
    G_cam_i      <= X"02";
    B_cam_i      <= X"10";
    wait for 5* clk_period;
    Done_cam_i   <= '1';
    wait for 5* clk_period;
    Done_cam_i   <= '0';
    wait for 5* clk_period;

assert false report "End of simulation" severity note;

END PROCESS;
END rgy_gray_tb;

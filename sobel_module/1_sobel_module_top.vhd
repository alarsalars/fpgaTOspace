-- // --------------------------------
-- // --------- Reference Concept @HUI HU---------- 
-- // --------- Coded by Taha Alars
-- // --------------------------------  

LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.numeric_std.all;



ENTITY sobel_module_top IS 
    PORt ( 
        -------------- system ----------------------
        CLK     : IN std_logic;
        RST     : IN std_logic;
        --- RGB Data from Camera ----
        R_cam_i     : IN std_logic_vector(7 DOWNTO 0);
        G_cam_i     : IN std_logic_vector(7 DOWNTO 0);
        B_cam_i     : IN std_logic_vector(7 DOWNTO 0);
        Done_cam_i  : IN std_logic;
        --- RGB Data for HDMI ----
        R_hdmi_o     : OUT std_logic_vector(7 DOWNTO 0);
        G_hdmi_o     : OUT std_logic_vector(7 DOWNTO 0);
        B_hdmi_o     : OUT std_logic_vector(7 DOWNTO 0);
        Done_hdmi_o  : OUT std_logic


    );

END ENTITY;


ARCHITECTURE Arch_sobel_module_top  OF  sobel_module_top IS 

SIGNAL grayscale_reg :  std_logic_vector(7 DOWNTO 0);
SIGNAL done_ff       :  std_logic;

SIGNAL grayscale_reg_rgb :  std_logic_vector(7 DOWNTO 0);
SIGNAL done_ff_rgb       :  std_logic := '0';

component rgb_to_grayscale is
    port (
        -------------- system ----------------------
        CLK         : IN std_logic;
        RST         : IN std_logic;
        -------- Input RGB ----
        R_cam_i     : IN std_logic_vector(7 DOWNTO 0):= (others => '0');
        G_cam_i     : IN std_logic_vector(7 DOWNTO 0):= (others => '0');
        B_cam_i     : IN std_logic_vector(7 DOWNTO 0):= (others => '0');
        Done_cam_i  : IN std_logic;
        -------- OUTput grayscale  ----
        grayscale_o : OUT std_logic_vector(7 DOWNTO 0);
        done_o      : OUT std_logic

    );
end component;

component sobel_kernel is
    port (
        -------------- system ----------------------
        CLK         : IN std_logic;
        RST         : IN std_logic;
        ----------- // INPUT  /// ------ 
        grayscale_i : IN std_logic_vector(7 DOWNTO 0);
        done_i      : IN std_logic;               
        -----------  /// OUTPUT /// ------
        grayscale_o : OUT std_logic_vector(7 DOWNTO 0);
        done_o      : OUT std_logic


    );
end component;

component grayscale_to_rgb is
    port (
        clk         : IN std_logic;
        rst         : IN std_logic;
        ----------- // INPUT  /// ------
        grayscale_i : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
        done_i      : IN STD_LOGIC;
        -----------  /// OUTPUT /// ------
        red_o       : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
        green_o     : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
        blue_o      : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
        done_o      : OUT STD_LOGIC

    );
end component;

BEGIN 

rgb_to_grayscale_int : rgb_to_grayscale

    port map (
        CLK         => CLK,  
        RST         => RST,  
        R_cam_i     => R_cam_i   ,  
        G_cam_i     => G_cam_i   ,  
        B_cam_i     => B_cam_i   ,  
        Done_cam_i  => Done_cam_i,  
        grayscale_o => grayscale_reg,  
        done_o      => done_ff       
    

    );


sobel_kernel_int : sobel_kernel

    port map (
        CLK          => CLK,
        RST          => RST,
        grayscale_i  => grayscale_reg,
        done_i       => done_ff      ,
        grayscale_o  => grayscale_reg_rgb,
        done_o       => done_ff_rgb      
    
    
    );


grayscale_to_rgb_int : grayscale_to_rgb

    port map (
        clk          => CLK,
        rst          => RST,
        grayscale_i  => grayscale_reg_rgb,
        done_i       => done_ff_rgb      ,
        red_o        => R_hdmi_o   ,
        green_o      => G_hdmi_o   ,
        blue_o       => B_hdmi_o   ,
        done_o       => Done_hdmi_o
    
    
    );
















END Arch_sobel_module_top;
LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.numeric_std.all;



ENTITY rgb_to_grayscale IS 
    PORT (
        -------------- system ----------------------
        CLK     : IN std_logic;
        RST     : IN std_logic;
        -------- Input RGB ----
        R_cam_i     : IN std_logic_vector(7 DOWNTO 0):= (others => '0');
        G_cam_i     : IN std_logic_vector(7 DOWNTO 0):= (others => '0');
        B_cam_i     : IN std_logic_vector(7 DOWNTO 0):= (others => '0');
        Done_cam_i  : IN std_logic;
        -------- OUTput grayscale  ----
        grayscale_o : OUT std_logic_vector(7 DOWNTO 0);
        done_o      : OUT std_logic
    );

END ENTITY;


ARCHITECTURE rgb_gray  OF  rgb_to_grayscale IS 


BEGIN 


gray_proc  : PROCESS (CLK,RST) IS 

BEGIN 
    IF rising_edge(CLK) THEN 
        IF (RST = '1') THEN 
            grayscale_o <= (others => '0');
            done_o <= '0';
        ELSE 
            IF (Done_cam_i = '1') THEN 
                grayscale_o <= std_logic_vector(
                  shift_right(unsigned(R_cam_i), 2) + shift_right(unsigned(R_cam_i), 5) +
                  shift_right(unsigned(G_cam_i), 1) + shift_right(unsigned(G_cam_i), 4) +
                  shift_right(unsigned(B_cam_i), 4) + shift_right(unsigned(B_cam_i), 5)
              );    
                done_o <= '1';
            ELSE 
                grayscale_o <= (others => '0');
                done_o <= '0';
            END IF ;
        END IF;
    END IF;
END PROCESS;  

END  rgb_gray;
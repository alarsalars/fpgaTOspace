---------------------------------------------------------------
-- Title         
-- Project       
------------------
-- File          
-- Author     TAHA ALARS    
-- Email         
-- Organization  
-- Created       
------------------
-- Simulator     
-- Synthesis     
------------------
-- Description : It is a simple UART code for a quick start of understanding the main concept of UART. 
--               I could not have time to run a simulation, but it synthesised successfully, in any case, the test of the code with simulation would be very straight forward
---------------------------------------------------------------
-- Hierarchy:  





library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

entity uart_top is
    GENERIC (
    length : INTEGER := 8
    );
    port (

    -------------------------------- INPUT PORTS -------------------------------
        clk                             : IN std_logic;
        rst                             : IN std_logic;
        tx_start                        : IN std_logic;
        rx_start                        : IN std_logic;
        tx_data                         : IN std_logic_vector(7 downto 0);
        baud                            : IN std_logic_vector(16 downto 0);
     --   length                          : IN std_logic_vector(3 downto 0);    
        parity_type                     : IN std_logic;
        parity_en                       : IN std_logic;    
        stop_bit                        : IN std_logic;
    -------------------------------- OUTPUT PORTS ------------------------------
        tx_done                         : OUT std_logic;
        rx_done                         : OUT std_logic;
        tx_err                          : OUT std_logic; 
        rx_err                          : OUT std_logic; 
        rx_out                          : OUT std_logic_vector(7 downto 0)

    );
end uart_top;



ARCHITECTURE rtl OF uart_top IS


COMPONENT clk_gen 

    PORT (

        clk                 : IN  std_logic;   -- clk= 50 MHz
        rst                 : IN  std_logic;
        baud                : IN  std_logic_vector(16 downto 0);
        tx_clk              : OUT std_logic :='0';
        rx_clk              : OUT std_logic :='0'  -- 16 times fater than tx
    );
END COMPONENT;


COMPONENT uart_tx 
    GENERIC (
    length : INTEGER := 8
    );
    PORT (
        clk                 : IN std_logic;
        tx_clk              : IN  std_logic;                        
        tx_start            : IN  std_logic;                            
        rst                 : IN  std_logic;                    
        tx_data             : IN  std_logic_vector(7 DOWNTO 0);                        
     --   length              : IN  std_logic_vector(3 DOWNTO 0);   -- 5,6,7,8                     
        parity_type         : IN  std_logic;                            
        parity_en           : IN  std_logic;                            
        stop2               : IN  std_logic;                        
        tx                  : OUT std_logic;                    
        tx_done             : OUT std_logic;                        
        tx_err              : OUT std_logic  
    );
END COMPONENT;


COMPONENT uart_rx 
    GENERIC (
    length : INTEGER := 8
    );
    PORT (
        clk          : IN std_logic;
        rx_clk       : IN STD_LOGIC;                       
        rx_start     : IN STD_LOGIC;                           
        rst          : IN STD_LOGIC;                   
        rx           : IN STD_LOGIC;                       
     --   length       : IN STD_LOGIC_VECTOR(3 DOWNTO 0);                       
        parity_type  : IN STD_LOGIC;                           
        parity_en    : IN STD_LOGIC;                           
        stop2        : IN STD_LOGIC;                       
        rx_out       : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);                       
        rx_done      : OUT STD_LOGIC;                       
        rx_error     : OUT STD_LOGIC  
    );
END COMPONENT;

SIGNAL tx_clk_s                 :  std_logic :='0';
SIGNAL rx_clk_s                 :  std_logic :='0';
SIGNAL tx_rx                  :  std_logic;                    

BEGIN 


uart_clk : clk_gen
    PORT MAP(

        clk             => clk    ,       
        rst             => rst    , 
        baud            => baud   ,  
        tx_clk          => tx_clk_s ,    
        rx_clk          => rx_clk_s      

    );

uart_sender : uart_tx 
   GENERIC MAP
   (
length => length
   )
    PORT MAP(
        clk          => clk        ,
        tx_clk       => tx_clk_s     ,
        tx_start     => tx_start   ,   
        rst          => rst        ,
        tx_data      => tx_data    ,
    --    length       => length     ,
        parity_type  => parity_type,   
        parity_en    => parity_en  ,       
        stop2        => stop_bit   ,   
        tx           => tx_rx      ,   
        tx_done      => tx_done    ,
        tx_err       => tx_err          


    );

uart_reciver : uart_rx
   GENERIC MAP
   (
length => length
   )
    PORT MAP(
        clk           => clk        ,
        rx_clk        => rx_clk_s      ,   
        rx_start      => rx_start    ,   
        rst           => rst         ,
        rx            => tx_rx       ,   
    --    length        => length      ,   
        parity_type   => parity_type ,           
        parity_en     => parity_en   ,       
        stop2         => stop_bit    ,       
        rx_out        => rx_out      ,   
        rx_done       => rx_done     ,               
        rx_error      => rx_err           


    );

END ARCHITECTURE rtl;



















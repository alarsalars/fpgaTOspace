---------------------------------------------------------------
-- Title     spi TOP    
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
-- Description : it explains the simple state machine of the spi master no sclk used
--    a quick process can be used to generate sclk and send it to slave also make the state machine count on it
---------------------------------------------------------------
-- Hierarchy: 



LIBRARY IEEE;
USE IEEE.std_logic_1164.ALL;
USE IEEE.numeric_std.ALL;


ENTITY spi_top  IS
    GENERIC (
    length : INTEGER := 8
    );
    PORT (

        clk          : IN std_logic;                                  
        rst          : IN std_logic;                             
        wr           : IN std_logic;                            
        addr_m       : IN std_logic_vector(length-1 Downto 0);                              
        din_m        : IN std_logic_vector(length-1 Downto 0); 
        dout_m       : OUT std_logic_vector(length-1 Downto 0);
        done         : OUT std_logic
        din_s        : IN std_logic_vector(length-1 Downto 0); 
        dout_s       : OUT std_logic_vector(length-1 Downto 0)  

    );

END ENTITY spi_top;



COMPONENT spi_m  IS
    GENERIC (
    length : INTEGER := 8
    );
    PORT (
        clk      : IN std_logic;       ----- master domain no sclk used here                            
        rst      : IN std_logic;                             
        wr       : IN std_logic;                            
        ready    : IN std_logic;                               
        op_done  : IN std_logic;                                 
        addr     : IN std_logic_vector(length-1 Downto 0);                              
        din      : IN std_logic_vector(length-1 Downto 0);                             
        dout     : OUT std_logic_vector(length-1 Downto 0);                              
        cs       : OUT std_logic;                             
        mosi     : OUT std_logic;                              
        miso     : IN std_logic;                              
        -- sclk  : std_logic  not used
        done     : OUT std_logic                                                        
                                  
    );

END COMPONENT spi_m;


COMPONENT spi_s  IS
    GENERIC (
    length : INTEGER := 8
    );
    PORT (
        clk      : IN std_logic;       ----- slave domain no sclk used here                            
        rst      : IN std_logic;                             
        ready    : OUT std_logic;                               
        op_done  : OUT std_logic;                                 
        din      : IN std_logic_vector(length-1 Downto 0);                             
        dout     : OUT std_logic_vector(length-1 Downto 0);                              
        cs       : IN std_logic;                             
        mosi     : IN std_logic;                              
        miso     : OUT std_logic                              
        -- sclk  : std_logic  not used
                                  
    );

END COMPONENT spi_s;



ARCHITECTURE rtl OF spi_top IS

SIGNAL ready : std_logic;
SIGNAL op_done : std_logic;
SIGNAL cs, mosi, miso : std_logic;

BEGIN 


spi_master_ini : spi_m 
   GENERIC MAP
   (
length => length
   )
    PORT MAP(

        clk        => clk       ,                    
        rst        => rst       ,                
        wr         => wr        ,                
        ready      => ready     ,                
        op_done    => op_done   ,                    
        addr       => addr_m    ,                    
        din        => din_m     ,                    
        dout       => dout_m    ,                    
        cs         => cs        ,                
        mosi       => mosi      ,                
        miso       => miso      ,                
        -- sclk    =>                       
        done       => done                      

    );



    spi_slave_ini : spi_s 
    GENERIC MAP
    (
 length => length
    )
     PORT MAP(

     clk        => clk        ,                     
     rst        => rst        ,         
     ready      => ready      ,             
     op_done    => op_done    ,             
     din        => din_s      ,             
     dout       => dout_s     ,             
     cs         => cs         ,         
     mosi       => mosi       ,         
     miso       => miso                 
     -- sclk    => 
     );

END ARCHITECTURE rtl;
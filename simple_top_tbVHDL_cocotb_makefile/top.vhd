
---------------------------------------------------------------
-- Title         
-- Project       
------------------
-- File          
-- Author        
-- Email         
-- Organization  
-- Created       
------------------
-- Simulator     
-- Synthesis     
------------------
-- Description : 
---------------------------------------------------------------
-- Hierarchy: 






LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
use ieee.numeric_std.all;


ENTITY top IS
PORT (
       rst_n               : IN STD_LOGIC;                           
       clk_32MHz           : IN STD_LOGIC;                           
       clk_tb              : IN STD_LOGIC;                           
       -------------------------- Input LED signals -----------------
       led_dat             : IN STD_LOGIC_VECTOR( 7 DOWNTO 0 );      
       led_lnk             : IN STD_LOGIC_VECTOR( 7 DOWNTO 0 );      
       led_pwrgd           : IN STD_LOGIC;                           
       led_cpu_act         : IN STD_LOGIC;                           
       led_autotst         : IN STD_LOGIC;                           
       led_poe_pwrgd       : IN STD_LOGIC;                           
       --------------------------- Relays Control -------------------
       poe_bypass_relay_on : OUT STD_LOGIC;                          
       bypass_relay_on     : IN STD_LOGIC;                           
       ---------------------------- PoE Control ---------------------
       poe_ctrl_poe        : IN STD_LOGIC_VECTOR( 1 DOWNTO 0 );      
       poe_cfg_poe         : IN STD_LOGIC;                           
       -------------------------- LED Serial Link -------------------
       led_data            : OUT STD_LOGIC;                          
       led_clk             : INOUT STD_LOGIC;                        

       rec_en              : OUT STD_LOGIC
     );
END top;



ARCHITECTURE top_arch OF top IS

   --------------------------------------------------------------------------
   -- Type declaration                                                     --
   --------------------------------------------------------------------------
   TYPE TransmitState IS (
                           txIdle,
                           txWait,
                           txSample,
                           txTransmit,
                           txStop
                         );

   TYPE TransmitLineState IS (
                               txDataChange,
                               txClkRise,
                               txClkHigh,
                               txClkFall
                             );



   --------------------------------------------------------------------------
   -- Constant declaration                                                 --
   --------------------------------------------------------------------------
   CONSTANT CLK_MAX_CNT       : INTEGER := 255;
   CONSTANT CLK_RELOAD        : INTEGER := 240;
   CONSTANT TX_BUFF_SIZE      : INTEGER := 23;
   CONSTANT INTER_FRAME_TIME  : INTEGER := 200;



   --------------------------------------------------------------------------
   -- Signal declaration                                                   --
   --------------------------------------------------------------------------
   SIGNAL clk              : STD_LOGIC;                              -- Internal clock
   SIGNAL tx_buffer        : STD_LOGIC_VECTOR( TX_BUFF_SIZE - 1 DOWNTO 0 ); -- Data to transmit
   SIGNAL buffer_empty     : INTEGER RANGE 0 TO TX_BUFF_SIZE - 1;
   SIGNAL clk_counter      : INTEGER RANGE 0 TO CLK_MAX_CNT;
   SIGNAL led_pse_on       : STD_LOGIC_VECTOR( 1 DOWNTO 0 );
   SIGNAL txState          : TransmitState;
   SIGNAL txLineState      : TransmitLineState;
   SIGNAL wait_time        : INTEGER RANGE 0 TO INTER_FRAME_TIME;


   



BEGIN


   --------------------------------------------------------------------------
   -- Clock divider to slow down the clock speed                           --
   --------------------------------------------------------------------------
   CLK_DIV : PROCESS( rst_n, clk_32MHz )
   BEGIN
      IF ( rst_n = '0' ) THEN
         clk_counter <= 0;
         clk <= '0';
      ELSIF rising_edge( clk_32MHz ) THEN
         IF ( clk_counter = 0 ) THEN
            clk_counter <= CLK_RELOAD;
            clk         <= NOT( clk );
         ELSE
            clk_counter <= clk_counter - 1;
         END IF;
      END IF;
   END PROCESS CLK_DIV;



   PROCESS( rst_n, clk )
   BEGIN
      IF ( rst_n = '0' ) THEN
         buffer_empty <= TX_BUFF_SIZE - 1;
         tx_buffer <= ( OTHERS => '0' );                             -- Flush transmit buffer
         txState <= txIdle;
         txLineState <= txDataChange;
         wait_time <= INTER_FRAME_TIME;
         led_data <= '1';
         led_clk <= '1';
         rec_en  <= '0';
      ELSIF rising_edge( clk ) THEN
         CASE txState IS
            WHEN txIdle =>
               led_data <= '1';
               led_clk <= '1';
               wait_time <= INTER_FRAME_TIME;
               buffer_empty <= TX_BUFF_SIZE - 1;
               txState <= txWait;
               rec_en  <= '1';
            WHEN txWait =>
               rec_en  <= '0';
               IF ( wait_time = 0 ) THEN
                  txState <= txSample;
                  led_data <= '0';                                   -- Generate start condition
                  led_clk <= '1';
               ELSE
                  wait_time <= wait_time - 1;
                  txState <= txWait;
                  led_data <= '1';
                  led_clk <= '1';
               END IF;
            WHEN txSample =>
               tx_buffer <= led_dat( 7 DOWNTO 0 ) & led_lnk( 7 DOWNTO 0 ) &
                            led_pwrgd & led_cpu_act & led_autotst &
                            NOT( led_poe_pwrgd ) & led_pse_on( 1 DOWNTO 0 ) & '0';
               led_data <= '0';
               led_clk <= '0';                                       -- Prepare clock for first bit
               txState <= txTransmit;
               txLineState <= txDataChange;
            WHEN txTransmit =>
               CASE txLineState IS
                  WHEN txDataChange =>
                     led_data <= tx_buffer( buffer_empty );
                     led_clk <= '0';
                     txLineState <= txClkRise;
                  WHEN txClkRise =>
                     led_clk <= '1';
                     txLineState <= txClkHigh;
                  WHEN txClkHigh =>
                     led_clk <= '1';
                     txLineState <= txClkFall;
                  WHEN txClkFall =>
                     led_clk <= '0';
                     IF ( buffer_empty = 0 ) THEN
                        txState <= txStop;
                     ELSE
                        buffer_empty <= buffer_empty - 1;
                     END IF;
                     txLineState <= txDataChange;
                  WHEN OTHERS =>
                     txLineState <= txDataChange;
                     buffer_empty <= TX_BUFF_SIZE - 1;
               END CASE;
            WHEN txStop =>
               led_clk <= '1';
               led_data <= '0';
               txState <= txIdle;
            WHEN OTHERS =>
               buffer_empty <= TX_BUFF_SIZE - 1;
               tx_buffer <= ( OTHERS => '0' );
               txState <= txIdle;
               txLineState <= txDataChange;
               wait_time <= INTER_FRAME_TIME;
               ASSERT FALSE
               REPORT "Transmit FSM error"
               SEVERITY WARNING;
         END CASE;
      END IF;
   END PROCESS;

   led_pse_on( 1 ) <= '1' WHEN ( poe_ctrl_poe( 1 ) = '1' AND poe_cfg_poe = '1' ) ELSE '0';
   led_pse_on( 0 ) <= '1' WHEN ( poe_ctrl_poe( 0 ) = '1' AND poe_cfg_poe = '0' ) ELSE '0';


   BYPASS : PROCESS( rst_n, clk, bypass_relay_on )
   BEGIN
      IF ( rst_n = '0' ) THEN
         poe_bypass_relay_on <= '0';                                 -- Bypass
      ELSIF rising_edge( clk ) THEN
         IF ( bypass_relay_on = '1' ) THEN
            poe_bypass_relay_on <= '0';                              -- Bypass
         ELSE
            poe_bypass_relay_on <= '1';                              -- No bypass
         END IF;
      END IF;
   END PROCESS BYPASS;



END top_arch;


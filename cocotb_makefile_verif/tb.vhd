LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
use ieee.numeric_std.all;



ENTITY tb IS
END tb;



ARCHITECTURE arch OF tb IS

   COMPONENT top
   PORT (
          rst_n               : IN STD_LOGIC;                       
          clk_32MHz           : IN STD_LOGIC;                       
          -------------------------- Input LED signals -------------
          led_dat             : IN STD_LOGIC_VECTOR( 7 DOWNTO 0 );  
          led_lnk             : IN STD_LOGIC_VECTOR( 7 DOWNTO 0 );  
          led_pwrgd           : IN STD_LOGIC;                       
          led_cpu_act         : IN STD_LOGIC;                       
          led_autotst         : IN STD_LOGIC;                       
          led_poe_pwrgd       : IN STD_LOGIC;                       
          --------------------------- Relays Control ---------------
          poe_bypass_relay_on : OUT STD_LOGIC;                      
          bypass_relay_on     : IN STD_LOGIC;                       
          ---------------------------- PoE Control -----------------
          poe_ctrl_poe        : IN STD_LOGIC_VECTOR( 1 DOWNTO 0 );  
          poe_cfg_poe         : IN STD_LOGIC;                       
          -------------------------- LED Serial Link ---------------
          led_data            : OUT STD_LOGIC;                      
          led_clk             : INOUT STD_LOGIC;                    
          rec_en              : OUT STD_LOGIC
        );
   END COMPONENT top;






   --------------------------------------------------------------------------
   -- Constant declaration                                                 --
   --------------------------------------------------------------------------
   CONSTANT CLK_PERIOD        : TIME := 31.25 ns;                    -- External clock period
   CONSTANT RESET_DURATION    : TIME := 50 ms;                       -- Reset duration
   CONSTANT LED_PERIOD        : TIME := 21 ms;                       -- LED pulse duration (21ms to 2.7s)
   CONSTANT TX_BUFF_SIZE      : INTEGER := 23;
   --------------------------------------------------------------------------
   -- Signals declaration                                                  --
   --------------------------------------------------------------------------
   SIGNAL clk_32MHz           : STD_LOGIC := '0';                    -- External clock
   SIGNAL rst_n               : STD_LOGIC;                           -- Global reset
   SIGNAL clk_led             : STD_LOGIC := '0';                    -- Clock pulsing the LEDs

   SIGNAL led_dat             : STD_LOGIC_VECTOR( 7 DOWNTO 0 );
   SIGNAL led_lnk             : STD_LOGIC_VECTOR( 7 DOWNTO 0 );
   SIGNAL led_pwrgd           : STD_LOGIC;
   SIGNAL led_cpu_act         : STD_LOGIC;
   SIGNAL led_autotst         : STD_LOGIC;
   SIGNAL led_poe_pwrgd       : STD_LOGIC;
   SIGNAL bypass_relay_on     : STD_LOGIC;
   SIGNAL poe_ctrl_poe        : STD_LOGIC_VECTOR( 1 DOWNTO 0 );
   SIGNAL poe_cfg_poe         : STD_LOGIC;
   SIGNAL poe_bypass_relay_on : STD_LOGIC;
   SIGNAL led_data            : STD_LOGIC;
   SIGNAL led_clk             : STD_LOGIC;

   SIGNAL rec_en              : STD_LOGIC;
   SIGNAL buff_reg            : STD_LOGIC_vector( 23 DOWNTO 0 );
   
   type Val_Array is array (9 downto 0) of STD_LOGIC_VECTOR(23 downto 0);
  
   signal myArray : Val_Array := (                        
   
                              "101010101000011011111010", 
                              "100010101111011011111011", 
                              "100110110101110111111101", 
                              "010001111111111011111101", 
                              "101101001010100110111100", 
                              "100001001101010010111100", 
                              "111010100000111010111111", 
                              "011101101110000110111111", 
                              "010111010000101111111111", 
                              "100000010000010011111111");

   signal counter : natural range 0 to 9 := 0;
   BEGIN
   --------------------------------------------------------------------------
   -- External 32MHz clock generation                                      --
   --------------------------------------------------------------------------
   clk_32MHz <= NOT( clk_32MHz ) AFTER CLK_PERIOD / 2;
   clk_led   <= NOT( clk_led ) AFTER LED_PERIOD / 2;


   
   --------------------------------------------------------------------------
   -- Reset management                                                     --
   --------------------------------------------------------------------------
   rst_n <= '0', '1' AFTER RESET_DURATION;


   DUT : top
   PORT MAP (
              rst_n                 => rst_n,
              clk_32MHz             => clk_32MHz,
              ---------------------- Input LED signals ----------------------
              led_dat               => led_dat,
              led_lnk               => led_lnk,
              led_pwrgd             => led_pwrgd,
              led_cpu_act           => led_cpu_act,
              led_autotst           => led_autotst,
              led_poe_pwrgd         => led_poe_pwrgd,
              ------------------------ Relays Control -----------------------
              poe_bypass_relay_on   => poe_bypass_relay_on,
              bypass_relay_on       => bypass_relay_on,
              ------------------------ PoE Control --------------------------
              poe_ctrl_poe          => poe_ctrl_poe,
              poe_cfg_poe           => poe_cfg_poe,
              ---------------------- LED Serial Link ------------------------
              led_data              => led_data,
              led_clk               => led_clk,

              rec_en                => rec_en
            );





            
---- check the clock in the process
BUFF_LOAD: PROCESS (rst_n, clk_led)
begin
    IF ( rst_n = '0' ) THEN
        --buff_reg          <= ( OTHERS => '0' );
        counter           <= 0;
        led_dat           <= ( OTHERS => '0' );
        led_lnk           <= ( OTHERS => '0' );
        led_pwrgd         <= '0';
        led_cpu_act       <= '0';
        led_autotst       <= '0';
        led_poe_pwrgd     <= '0';
        bypass_relay_on   <= '0';
        poe_ctrl_poe      <= ( OTHERS => '0' );
        poe_cfg_poe       <= '0';
        --buff_reg          <= ( OTHERS => '0' );
    ELSIF rising_edge( clk_led ) THEN
        if counter < 10 then
            counter <= counter + 1 ;
               -- if rec_en = '1' then 
                led_dat         <=    buff_reg(23 downto 16)                ;                         
                led_lnk         <=    buff_reg(15 downto 8)                 ;       
                led_pwrgd       <=    buff_reg(7)                           ;         
                led_cpu_act     <=    buff_reg(6)                           ;           
                led_autotst     <=    buff_reg(5)                           ;           
                led_poe_pwrgd   <=    buff_reg(4)                           ;             
                bypass_relay_on  <=   buff_reg(3)                            ;         
                poe_ctrl_poe     <=   buff_reg(2 downto 1)                   ;
                poe_cfg_poe      <=   buff_reg(0)                            ;
              --  end if;
        else
            counter <= 0 ;
        end if ;
     End if;
end PROCESS BUFF_LOAD;      


update : PROCESS(rec_en)
begin
    IF  rec_en = '1' THEN
     buff_reg <= myArray(counter);
    end if;
end PROCESS update;



END arch;



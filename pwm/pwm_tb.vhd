LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.numeric_std.all;


ENTITY pwm_tb Is 
generic (
  resolution : integer range 0 to 256 := 4; -- number of duty cycles 0% 25% 50% 75% 100%, no of bits = 3
  channels    : integer range 0 to 16 := 16; -- no of bits = 4 
  clk_divider : integer range 0 to 65535 := 4 
);
END ENTITY;


ARCHITECTURE pwm_tb_arch  OF  pwm_tb IS 


COMPONENT pwm IS 
generic (
  resolution : integer range 0 to 256 := 4; -- number of duty cycles 0% 25% 50% 75% 100%, no of bits = 3
  channels    : integer range 0 to 16 := 16; -- no of bits = 4 
  clk_divider : integer range 0 to 65535 := 4 
 );

port (
  clk            : in std_logic;
  rst            : in std_logic;
  set_duty_cycle : in std_logic_vector(2 downto 0); -- set new duty cycle when update_duty is '1'   , log2(resolution+1)-1 = 2
  -- number of bits = log2(resolution+1) we put +1 so we cover all the required, for example 
  -- log2(4)= 2, which is not enough to cover 0 to 4, but log2(5) = 3, enought to cover 0 to 4
  update_duty    : in std_logic;
  set_channel    : in std_logic_vector(3 downto 0); -- update a channel of 16 with new duty cycle  , log2(channels)-1 = 3
  -- number of bits = log2(channels) we dont put +1 cuz for channels should be enough, for example 
  -- log2(16)= 4, which is not enough to cover 0 to 15.
  pwm_out        : out std_logic_vector(channels-1 downto 0)
 );


END COMPONENT;


CONSTANT   clk_period  : TIME := 10 ns;
SIGNAL clk : std_logic:= '0';
SIGNAL rst : std_logic:= '0';

SIGNAL set_duty_cycle :std_logic_vector(2 downto 0); 
SIGNAL update_duty    :std_logic;
SIGNAL set_channel    :std_logic_vector(3 downto 0);
SIGNAL pwm_out        : std_logic_vector(channels-1 downto 0);

begin 


mean_top_inis : pwm  
generic map (
    resolution   => resolution ,
    channels     => channels   ,  
    clk_divider  => clk_divider
  )
PORT MAP (
  clk             => clk         ,  
  rst             => rst          , 
  set_duty_cycle  => set_duty_cycle,
  update_duty     => update_duty   ,
  set_channel     => set_channel   ,
  pwm_out         => pwm_out       
);


clk_proc : PROCESS Is 
BEGIN 
    clk <= '1';
    wait for clk_period/2;
    clk <= '0';
    wait for clk_period/2;
END PROCESS;



mean_test_process : process 
BEGIN 
    rst             <= '1';
    update_duty     <= '0';
    set_duty_cycle  <= (others => '0') ;
    set_channel     <= (others => '0') ;

    wait for 4* clk_period;
    rst             <= '0';
    wait for 100* clk_period;
    set_duty_cycle <= "011";
    update_duty <= '1';
    set_channel <= "0111";
    wait for 50* clk_period;
    set_duty_cycle <= "001";
    update_duty <= '1';
    set_channel <= "0100";
    wait for 50* clk_period;
    set_duty_cycle <= "001";
    update_duty <= '1';
    set_channel <= "0110";
    wait for 100* clk_period;
    assert false report "Condition met, stopping simulation" severity failure;
end process;

end pwm_tb_arch;
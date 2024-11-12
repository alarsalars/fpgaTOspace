
-- Taha Alars

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;


entity pwm is 
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
end entity;


architecture behav_pwm of pwm is

signal clk_div_count  : integer range 0 to clk_divider-1 := 0; -- number of divider of the new clock for the pwm
signal pwm_duty_count : integer range 0 to resolution-1  := 0; -- count to output the pwm with the specifi duty cycle

type duty_cycle_array is array (0 to channels-1) of std_logic_vector(2 downto 0); -- store the duty cycle for each channel, 16 requiured here
signal duty_cycle       : duty_cycle_array :=  (others => "010"); -- start the channels with 50% duty cycle


begin 



clk_div_proc : process (clk ,rst) is 
begin 
  if rising_edge(clk) then 
    if rst = '1' then 
      clk_div_count <= 0;
    elsif (clk_div_count = clk_divider -1) then 
        clk_div_count <= 0;
    else 
        clk_div_count <= clk_div_count + 1 ;
    end if;
  end if;
end process;


duty_cycle_proc : process (clk ,rst) is 
begin 
  if rising_edge(clk) then 
    if rst = '1' then 
      pwm_duty_count    <= 0;  
      -- duty_cycle        <= (others => (others => '0'));
    elsif (clk_div_count = clk_divider -1) then  
      if (pwm_duty_count < resolution-1 ) then 
        pwm_duty_count <= pwm_duty_count +1;
      else 
        pwm_duty_count <= 0; 
      end if;
      if (update_duty = '1') then   
        duty_cycle(to_integer(unsigned(set_channel))) <= set_duty_cycle;
      end if; 
    end if;
  end if;
end process;

pwm_proc  : process (pwm_duty_count) is   -- output the 16 channels depends on the array of the duty cycle
begin 
      for i in 0 to CHANNELS-1 loop 
        if (pwm_duty_count < to_integer(unsigned(duty_cycle(i)))) then  
          pwm_out(i) <= '1';
        else
          pwm_out(i) <= '0';
        end if;
      end loop;
end process;


end behav_pwm;
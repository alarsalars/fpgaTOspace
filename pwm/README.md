# PWM 

This module provide more flex ip with the following parameters:

1. Resolution  : integer range 0 to 256   := 4
2. Channels    : integer range 0 to 16    := 16
3. Clk_divider : integer range 0 to 65535 := 4

The only change that we need to do if we will reconfigre the IP (generics), the bits of set_duty_cycle and set_channel need to calculate manually. 

I started all the channals with 50% duty cycle but also can be configured or updated during the test.



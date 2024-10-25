# Wishbone Slave UART (Half Duplex)

This IP is my first attempt at designing a protocol in HLS, focusing on implementing the Wishbone and UART concepts. The design has been verified using an HLS testbench, although there is room for improvement, which is not my current focus.

While I ensured thorough testing of the core functionality, not all edge cases have been covered.

Key Notes:
UART Output: The UART currently outputs 10 bits instead of the standard 8 bits. This was done intentionally to verify the concept in the testbench but should be adjusted to 8 bits in future iterations.



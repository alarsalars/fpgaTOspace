library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;


-- Package Declaration
package constants_pkg is


------------------------- move data Operation ------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------
CONSTANT MOV         : std_logic_vector(4 DOWNTO 0) := "00001";
------------------------- Arithmatic Operations ------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------
CONSTANT ADD         : std_logic_vector(4 DOWNTO 0) := "00010";
CONSTANT SUB         : std_logic_vector(4 DOWNTO 0) := "00011";
CONSTANT MULT        : std_logic_vector(4 DOWNTO 0) := "00100";
CONSTANT MOVsGPR     : std_logic_vector(4 DOWNTO 0) := "00000";  -- to Move the MSB OF MULTIPLICATION and store it in GPR
------------------------- Logic  Operations ------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------
CONSTANT L_OR         : std_logic_vector(4 DOWNTO 0) := "00101";
CONSTANT L_AND        : std_logic_vector(4 DOWNTO 0) := "00110";
CONSTANT L_XOR        : std_logic_vector(4 DOWNTO 0) := "00111";
CONSTANT L_XNOR       : std_logic_vector(4 DOWNTO 0) := "01000";
CONSTANT L_NAND       : std_logic_vector(4 DOWNTO 0) := "01001";
CONSTANT L_NOR        : std_logic_vector(4 DOWNTO 0) := "01010";
CONSTANT L_NOT        : std_logic_vector(4 DOWNTO 0) := "01011";
------------------------- Logic  Operations ------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------
constant STORE_REG   : std_logic_vector(4 downto 0) := "01100"; --- store reg in data memory
constant STORE_DIN   : std_logic_vector(4 downto 0) := "01101"; --- store din in data memory
constant STORE_IMM   : std_logic_vector(4 downto 0) := "01110"; --- store imm_data in data memory
constant SEND_DM     : std_logic_vector(4 downto 0) := "01111"; ----send data memory to dout
constant SEND_REG    : std_logic_vector(4 downto 0) := "10000"; ----send reg to dout
constant SEND_IMM    : std_logic_vector(4 downto 0) := "10001"; ----send imm to dout
------------------------- Jump and Halt  Operations ------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------
constant JUMP              : std_logic_vector(4 downto 0) := "10010";
constant JUMP_CARRY        : std_logic_vector(4 downto 0) := "10011";
constant JUMP_NO_CARRY     : std_logic_vector(4 downto 0) := "10100";
constant JUMP_SIGN         : std_logic_vector(4 downto 0) := "10101";
constant JUMP_NO_SIGN      : std_logic_vector(4 downto 0) := "10110";
constant JUMP_ZERO         : std_logic_vector(4 downto 0) := "10111";
constant JUMP_NO_ZERO      : std_logic_vector(4 downto 0) := "11000";
constant JUMP_OVERFLOW     : std_logic_vector(4 downto 0) := "11001";
constant JUMP_NO_OVERFLOW  : std_logic_vector(4 downto 0) := "11010";
constant HALT              : std_logic_vector(4 downto 0) := "11011";

 


end package constants_pkg;
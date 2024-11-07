
-------------------------------------------------------
------------ Reference Concept @ Kumar Khandagle  -------------
------------ Coded by Taha Alars
------------ VHDL 2008
-------------------------------------------------------
-- Simple Processor, consider all the inputs are ports for simpilicity purposes in real immpememntation will be buses with memory
-- Register bit layout:
        --   31    27 26    22 21    17 16   15    11 10       0
        --  +--------+--------+--------+----+----------------+
        --  | IN_Op  | Dest   | Src    |Mode|  Src2  |  Unused      --- The Mode  when '0' it takes the data from the source of bits(21-17) Or from Src2 depends on Operation
        ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        --  | IN_Op  | Dest   | Src    |Mode|  Direct Data   |      --- The Mode when '1' it takes the data from bits(15-0) 


library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use ieee.std_logic_misc.all; --- for or_reduce bitwise
use IEEE.std_logic_textio.ALL;
use ieee.std_logic_unsigned.all;
use std.textio.all;  ---- to read a file for initial data for the memeory array
library work;
use work.constants_pkg.all; -- Include the package containing constants




Entity processor_top is 
    generic (
        CONSTANT IN_REG_WIDTH : INTEGER := 32
    );
    port(
        CLK  : IN std_logic; 
        RST  : IN std_logic;
        DIN  : IN std_logic_vector(15 DOWNTO 0);
        DOUT : OUT std_logic_vector(15 DOWNTO 0)

    );
End processor_top;


architecture Behavioral of processor_top is

----------------------------------------------------------------------------------------------------------------------------------
--------------------------------------- Arrays and Function for the Memory  ---------------------- ----------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------


---------------------- Program Memory of 64 X 32 ------------------

TYPE PROGRAM_MEMORY IS ARRAY (0 TO 63) of STD_LOGIC_VECTOR(31 DOWNTO 0);

---------------------------- Function to inisitate the PROGRAM memeory with source file conatin a program ----------------

impure FUNCTION LOAD_PROGRAM(file_location : in STRING) RETURN PROGRAM_MEMORY IS 
FILE mem_file : text open read_mode is file_location;
VARIABLE read_line : line;
VARIABLE PROGRAM_DATA : PROGRAM_MEMORY;

BEGIN 
        FOR I IN 0 TO 63 loop 
                READLINE(mem_file,read_line);
                READ(read_line,PROGRAM_DATA(I)); 
        END LOOP;
RETURN PROGRAM_DATA;
END FUNCTION;

----------------------------- SOURCE FILE ---------------------------
SIGNAL inst_data : PROGRAM_MEMORY := LOAD_PROGRAM("C:\Users\TAlars\Documents\vivado_projects_tests\fpgaTOspace\simplified_processor_vhdl\data.txt");--- TODO INCLUDE THE PATH 

--------------------DATA Memory -------------------
TYPE DATA_MEMORY Is ARRAY (63 DOWNTO 0) OF STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL data_mem : DATA_MEMORY := (others => (others => '0'));

SIGNAL program_counter : INTEGER := 0;
SIGNAL delay_counter   : INTEGER := 0;
SIGNAL IN_REG          : std_logic_vector(IN_REG_WIDTH-1 DOWNTO 0);-- instruction Register 

----------------------------------------------------------------------------------------------------------------------------------
--------------------------------------- End function of the memory  ---------------------- ----------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------

---------------------------- Various Operations in IN_REG -----------------------------
SIGNAL IN_Op : std_logic_vector(4 DOWNTO 0);
SIGNAL Dest  : std_logic_vector(4 DOWNTO 0);
SIGNAL Src   : std_logic_vector(4 DOWNTO 0);
SIGNAL Src2  : std_logic_vector(4 DOWNTO 0);
SIGNAL Mode  : std_logic;
SIGNAL Data  : std_logic_vector(15 DOWNTO 0);
 
------------------------------------ Handling registers----------------------------
----------------------------------------------------------------------------------------------------------------------------------
SIGNAL MULTI_REG : std_logic_vector(31 DOWNTO 0); ----- To store the result of Multipilcation
SIGNAL ADD_REG   : std_logic_vector(16 DOWNTO 0); ----- To store the result of Addition 16 bits + 1 bit as carry
SIGNAL SGPR      : std_logic_vector(15 DOWNTO 0); ----- Special GPR To store the MSB OF MULTIPLICATION

-------------------------- General Purpose Register    ------------------------------------
----------------------------------------------------------------------------------------------------------------------------------
Type GPR_ARRAY is ARRAY (31 DOWNTO 0) OF std_logic_vector(15 DOWNTO 0);    ------ 32 X 16 ARRAY
SIGNAL GPR : GPR_ARRAY := (others => (others => '0'));


---------------------------------------  Flags ----------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------
SIGNAL ZERO_F, SIGN_F, OVERFLOW_F, CARRY_F: std_logic := '0';

---------------------------------------  Jump stop signals for FSM ----------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------
SIGNAL jump_s, stop_s : std_logic := '0';

---------------------------------------  Procedure for instructions operation ----------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------

procedure exec_op(IN_REG : IN std_logic_vector(31 DOWNTO 0); SIGNAL ZERO_F, SIGN_F, OVERFLOW_F, CARRY_F : IN std_logic; SIGNAL jump_s,stop_s : OUT std_logic; SIGNAL data_mem : INOUT DATA_MEMORY; SIGNAL DIN  : IN std_logic_vector(15 DOWNTO 0); SIGNAL DOUT  : OUT std_logic_vector(15 DOWNTO 0);   SIGNAL GPR : INOUT GPR_ARRAY;  SIGNAL MULTI_REG : INOUT std_logic_vector(31 DOWNTO 0);SIGNAL ADD_REG   : INOUT std_logic_vector(16 DOWNTO 0);SIGNAL SGPR  : INOUT std_logic_vector(15 DOWNTO 0)) IS 
------------------------- IN_REG -------------------------------------    



BEGIN 
    jump_s <= '0';
    stop_s <= '0';
 
--------------- Process to decode the operation from the IN_REG  ------------------------------

        CASE (IN_Op) IS 
            ----------------------------------------------------------- Arithmetic Operations----------------------------------------
            WHEN MOV => -- "00001" 
                IF (Mode = '1') THEN 
                    GPR(to_integer(unsigned(Dest))) <= Data;
                ELSE 
                    GPR(to_integer(unsigned(Dest))) <= GPR(to_integer(unsigned(Src)));
                END IF; 

            WHEN ADD =>  -- "00010"           -----------The extra bit that added to handle the carry and store in 17 bits REG
                IF (Mode = '1') THEN
                    ADD_REG <= (('0' & GPR(to_integer(unsigned(Src)))) + ('0' & Data));
                ELSE 
                    ADD_REG <= (('0' & GPR(to_integer(unsigned(Src)))) + ('0' & GPR(to_integer(unsigned(Src2)))));  
                END  IF;   
                GPR(to_integer(unsigned(Dest))) <= ADD_REG(15 DOWNTO 0); -- NO carry
            
            WHEN SUB =>  -- "00011"
                IF (Mode = '1') THEN
                    GPR(to_integer(unsigned(Dest))) <= GPR(to_integer(unsigned(Src))) - Data;
                ELSE 
                    GPR(to_integer(unsigned(Dest))) <= GPR(to_integer(unsigned(Src))) - GPR(to_integer(unsigned(Src2)));  
                END  IF;
                
            WHEN MULT => -- "00100"
                IF (Mode = '1') THEN
                    MULTI_REG <= GPR(to_integer(unsigned(Src))) * Data ;
                ELSE 
                    MULTI_REG <= GPR(to_integer(unsigned(Src))) * GPR(to_integer(unsigned(Src2)));  
                END  IF;   
                GPR(to_integer(unsigned(Dest))) <= MULTI_REG(15 DOWNTO 0);  
                SGPR <= MULTI_REG(31 DOWNTO 16); 
            
            WHEN MOVsGPR => -- "00000" 
                GPR(to_integer(unsigned(Dest))) <= SGPR;

            ----------------------------------------------------------- Logic Operations----------------------------------------
            WHEN L_OR =>  -- "00101"
                IF (Mode = '1') THEN
                    GPR(to_integer(unsigned(Dest))) <= GPR(to_integer(unsigned(Src))) OR Data;
                ELSE 
                    GPR(to_integer(unsigned(Dest))) <= GPR(to_integer(unsigned(Src))) OR GPR(to_integer(unsigned(Src2)));  
                END  IF;
            
                WHEN L_AND =>  -- "00110"
                IF (Mode = '1') THEN
                    GPR(to_integer(unsigned(Dest))) <= GPR(to_integer(unsigned(Src))) AND Data;
                ELSE 
                    GPR(to_integer(unsigned(Dest))) <= GPR(to_integer(unsigned(Src))) AND GPR(to_integer(unsigned(Src2)));  
                END  IF;

            WHEN L_XOR =>  -- "00111"
                IF (Mode = '1') THEN
                    GPR(to_integer(unsigned(Dest))) <= GPR(to_integer(unsigned(Src))) XOR Data;
                ELSE 
                    GPR(to_integer(unsigned(Dest))) <= GPR(to_integer(unsigned(Src))) XOR GPR(to_integer(unsigned(Src2)));  
                END  IF;

            WHEN L_XNOR =>  -- "01000"
                IF (Mode = '1') THEN
                    GPR(to_integer(unsigned(Dest))) <= GPR(to_integer(unsigned(Src))) XNOR Data;
                ELSE 
                    GPR(to_integer(unsigned(Dest))) <= GPR(to_integer(unsigned(Src))) XNOR GPR(to_integer(unsigned(Src2)));  
                END  IF;

            WHEN L_NAND =>  -- "01001"
                IF (Mode = '1') THEN
                    GPR(to_integer(unsigned(Dest))) <= GPR(to_integer(unsigned(Src))) NAND Data;
                ELSE 
                    GPR(to_integer(unsigned(Dest))) <= GPR(to_integer(unsigned(Src))) NAND GPR(to_integer(unsigned(Src2)));  
                END  IF;

            WHEN L_NOR =>  -- "01010"
                IF (Mode = '1') THEN
                    GPR(to_integer(unsigned(Dest))) <= GPR(to_integer(unsigned(Src))) NOR Data;
                ELSE 
                    GPR(to_integer(unsigned(Dest))) <= GPR(to_integer(unsigned(Src))) NOR GPR(to_integer(unsigned(Src2)));  
                END  IF;

            WHEN L_NOT =>  -- "01011"
                IF (Mode = '1') THEN
                    GPR(to_integer(unsigned(Dest))) <= NOT Data;
                ELSE 
                    GPR(to_integer(unsigned(Dest))) <= NOT GPR(to_integer(unsigned(Src2)));  
                END  IF;
            
            ----------------------------------------------------------- Data moving Operations----------------------------------------

            ----------------- Store SRC to Data memory
            WHEN STORE_REG => 
                data_mem(to_integer(unsigned(Dest))) <= GPR(to_integer(unsigned(Src)));
            ----------------- Store DIN to Data memory
            WHEN STORE_DIN => 
                data_mem(to_integer(unsigned(Dest))) <= DIN;
            ----------------- Store Data of instruction Reg  to Data memory
            WHEN STORE_IMM => 
                data_mem(to_integer(unsigned(Dest))) <= Data;
            ----------------- Send data from Data memory
            WHEN SEND_DM => 
                DOUT <= data_mem(to_integer(unsigned(Dest)));
            ----------------- Send data from GPR
            WHEN SEND_REG => 
                DOUT <= GPR(to_integer(unsigned(Dest)));
            ----------------- Send data from Instruction REG
            WHEN SEND_IMM => 
                DOUT <= Data;

        ----------------------------------------------------------- Jump and halt Operations----------------------------------------
            WHEN JUMP => 
                jump_s <= '1';
            WHEN JUMP_CARRY => 
                IF (CARRY_F = '1') THEN 
                    jump_s <= '1';
                ELSE 
                    jump_s <= '0';
                END IF ;
            WHEN JUMP_NO_CARRY => 
                IF (CARRY_F = '0') THEN 
                    jump_s <= '1';
                    ELSE 
                    jump_s <= '0';
                END IF ;
            WHEN JUMP_SIGN => 
                IF (SIGN_F = '1') THEN 
                    jump_s <= '1';
                ELSE 
                    jump_s <= '0';
                END IF ;
            WHEN JUMP_NO_SIGN => 
                IF (SIGN_F = '0') THEN 
                    jump_s <= '1';
                ELSE 
                    jump_s <= '0';
                END IF ;
            WHEN JUMP_ZERO => 
                IF (ZERO_F = '1') THEN 
                    jump_s <= '1';
                ELSE 
                    jump_s <= '0';
                END IF ;
            WHEN JUMP_NO_ZERO => 
                IF (ZERO_F = '0') THEN 
                    jump_s <= '1';
                ELSE 
                    jump_s <= '0';
                END IF ;
            WHEN JUMP_OVERFLOW => 
                IF (OVERFLOW_F = '1') THEN 
                    jump_s <= '1';
                ELSE 
                    jump_s <= '0';
                END IF ;
            WHEN JUMP_NO_OVERFLOW => 
                IF (OVERFLOW_F = '0') THEN 
                    jump_s <= '1';
                ELSE 
                    jump_s <= '0';
                END IF ;
            WHEN HALT => 
                stop_s <= '1';


 
            WHEN others   => 
                NULL;

        END CASE;



END PROCEDURE;


--------------------------------------- END Procedure  ---------------------- ----------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------------------
--------------------------------------- Conditions FLAG Function ---------------------- ----------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------

----- Impure Function to access the global parameters -------------------------------------
Impure FUNCTION flags_extract(IN_REG : IN std_logic_vector(31 DOWNTO 0);SIGNAL ADD_REG   : IN std_logic_vector(16 DOWNTO 0);SIGNAL SGPR : IN std_logic_vector(15 DOWNTO 0) ) RETURN STD_LOGIC_VECTOR IS  

    variable FLAG : std_logic_vector(3 DOWNTO 0) := "0000";   

        --       3           2        1        0  
        --  +-----------+---------+---------+-------
        --  | overflow  | zero    |  carry  | sign



    BEGIN 
        --------------- Find the Sign FLAG --------
        IF (IN_Op = MULT) THEN 
            FLAG(0) := SGPR(15); 
        ELSE 
            FLAG(0) := GPR(to_integer(unsigned(Dest)))(15);
        END IF; 

         --------------- Find the Carry FLAG --------
        IF (IN_Op = ADD) THEN 
            FLAG(1) := ADD_REG(16);
        ELSE 
            FLAG(1) := '0';
        END IF;
        --------------- Find the ZERO FLAG -------- We will use or_reduce to perfom bitwise for the GPR ----
        ----------------------------------------------------------------------------------------------------
        IF (IN_Op = MULT) THEN 
            FLAG(2) := NOT (or_reduce(SGPR) OR ( or_reduce(GPR(to_integer(unsigned(Dest)))))); --- If it is then we get one in the flag
        ELSE 
            FLAG(2) := NOT (or_reduce(GPR(to_integer(unsigned(Dest)))));
        END IF; 
        --------------- Find the OVERFLOW FLAG -------- 

        IF (IN_REG(31 downto 27) = ADD) THEN
            if(IN_REG(16) = '1') then
                FLAG(3) :=  ( ( NOT GPR(to_integer(unsigned(IN_REG(21 downto 17))))(15)) AND (NOT IN_REG(15)) AND ( GPR(to_integer(unsigned(IN_REG(26 downto 22))))(15) ) ) OR  ( ( GPR(to_integer(unsigned(IN_REG(21 downto 17))))(15)) AND (IN_REG(15)) AND ( NOT GPR(to_integer(unsigned(IN_REG(26 downto 22))))(15) ) ); 
            else
                FLAG(3) :=  ( ( NOT GPR(to_integer(unsigned(IN_REG(21 downto 17))))(15)) AND (NOT GPR(to_integer(unsigned(IN_REG(15 downto 11))))(15)) AND ( GPR(to_integer(unsigned(IN_REG(26 downto 22))))(15) ) ) OR ( (  GPR(to_integer(unsigned(IN_REG(21 downto 17))))(15)) AND ( GPR(to_integer(unsigned(IN_REG(15 downto 11))))(15)) AND ( NOT GPR(to_integer(unsigned(IN_REG(26 downto 22))))(15) ) );
            end if;
            
       ELSIF (IN_REG(31 downto 27) = SUB) then
             if(IN_REG(16) = '1') then
                FLAG(3) :=  ( ( NOT GPR(to_integer(unsigned(IN_REG(21 downto 17))))(15)) AND ( IN_REG(15)) AND ( GPR(to_integer(unsigned(IN_REG(26 downto 22))))(15) ) ) OR  ( ( GPR(to_integer(unsigned(IN_REG(21 downto 17))))(15)) AND (NOT IN_REG(15)) AND ( NOT GPR(to_integer(unsigned(IN_REG(26 downto 22))))(15) ) ); 
            else
                FLAG(3) :=  ( ( NOT GPR(to_integer(unsigned(IN_REG(21 downto 17))))(15)) AND ( GPR(to_integer(unsigned(IN_REG(15 downto 11))))(15)) AND ( GPR(to_integer(unsigned(IN_REG(26 downto 22))))(15) ) ) OR ( (  GPR(to_integer(unsigned(IN_REG(21 downto 17))))(15)) AND ( NOT GPR(to_integer(unsigned(IN_REG(15 downto 11))))(15)) AND ( NOT GPR(to_integer(unsigned(IN_REG(26 downto 22))))(15) ) );
            end if;
       ELSE
                FLAG(3) := '0';         
       
       END IF ; 

       RETURN FLAG;
END FUNCTION; 




----------------------------------  State machine -------------
TYPE FSM IS (IDLE, FETCH_IN, EXCU_IN, FLAGS_STATUS, DELAY, JUMP,HALT);

SIGNAL state, next_state : FSM := IDLE;




BEGIN 



--------------------------------- UPDATE state --------------------
state_proc : PROCESS(CLK) Is 
BEGIN 
    If(rising_edge(CLK)) THEN 
            IF (RST = '1') THEN 
                state      <= IDLE;  
            ELSE 
                state <= next_state;
            END IF;
    END IF; 
END PROCESS;



-------------------------- MAIN FSM, update next_state ---------------------

next_state_proc : PROCESS(state, delay_counter,RST ) IS  
variable FLAG_REG : std_logic_vector(3 DOWNTO 0) := "0000";
BEGIN 
    IF (RST = '1') THEN 
        next_state <= IDLE;  
        program_counter <= 0;
        delay_counter   <= 0;     
        CASE (state) IS  
                WHEN IDLE => 
                    IN_REG <= (others => '0');
                    program_counter <= 0;
                    delay_counter   <= 0;
                    next_state <= FETCH_IN;
                WHEN FETCH_IN => 
                    IN_REG <= inst_data(program_counter);  
                    IN_Op <= IN_REG(31 DOWNTO 27);
                    Dest  <= IN_REG(26 DOWNTO 22);  
                    Src   <= IN_REG(21 DOWNTO 17);    
                    Mode  <= IN_REG(16);  
                    Data  <= IN_REG(15 DOWNTO 0)   WHEN Mode = '1'  else  (others => '0');
                    Src2  <= IN_REG(15 DOWNTO 11)  WHEN Mode = '0'  else  (others => '0');
                    next_state <= EXCU_IN;
                WHEN EXCU_IN => 
                    exec_op(IN_REG, ZERO_F, SIGN_F, OVERFLOW_F, CARRY_F,jump_s,stop_s, data_mem,DIN,DOUT, GPR, MULTI_REG, ADD_REG,SGPR );
                    next_state <= FLAGS_STATUS;
                WHEN FLAGS_STATUS => 
                    FLAG_REG    := flags_extract(IN_REG, ADD_REG, SGPR );
                    ZERO_F      <= FLAG_REG(2);
                    SIGN_F      <= FLAG_REG(0);
                    OVERFLOW_F  <= FLAG_REG(3);    
                    CARRY_F     <= FLAG_REG(1); 
                    next_state  <= DELAY;
                WHEN DELAY =>  ---- Delay of 4 Clocks for each instruction in IN
                    IF (delay_counter = 4) THEN 
                        next_state <= JUMP;
                        delay_counter <= 0;
                    ELSE 
                        delay_counter <= delay_counter +1;
                        next_state  <= DELAY;
                    END IF; 
                WHEN JUMP => 
                    next_state <= HALT;
                    IF (jump_s = '1') THEN 
                        program_counter <= to_integer(unsigned(Data));
                    ELSE 
                        program_counter <= program_counter +1;
                    END IF; 
                WHEN HALT => 
                    IF (stop_s = '1') THEN 
                        next_state <= HALT;
                    ELSE 
                        next_state <= FETCH_IN;
                    END IF; 
                WHEN OTHERS => 
                    next_state <= IDLE;
        END CASE;
    END IF;  
END PROCESS; 

            





 

END Behavioral;



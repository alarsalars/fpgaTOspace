---------------------------------------------------------------
-- Title     UART Sender    
-- Project       
------------------
-- File          
-- Author    TAHA ALARS    
-- Email         
-- Organization  
-- Created       
------------------
-- Simulator     
-- SynthesIS     
------------------
-- Description : This is an AXI slave, the aim of the code only for learning the concept. In case you need to use it in hardware please, first immplement 
------------------ the testbench , you might find bugs in the code cuz I had to deal with different signals types.
----------------- IF you need urgent support please contact me.
---------------------------------------------------------------
-- Hierarchy: 




LIBRARY IEEE;
USE IEEE.std_logic_1164.ALL;
USE IEEE.numeric_std.ALL;


ENTITY axi_slave IS 
    PORT (
        --------------- Control signals clk and rst ---------------
            clk           : IN  std_logic;                            
            reset        : IN  std_logic;                    
        ---------------  write address ---------------
            awvalid       : IN  std_logic;                        
            awready       : out std_logic;                    
            awid          : IN  std_logic_vector(3 DOWNTO 0);                    
            awlen         : IN  std_logic_vector(3 DOWNTO 0);                    
            awsize        : IN  std_logic_vector(2 DOWNTO 0);                    
            awaddr        : IN  std_logic_vector(31 DOWNTO 0);                    
            awburst       : IN  std_logic_vector(1 DOWNTO 0);                    
        ---------------  write data ---------------
            wvalid        : IN  std_logic;                    
            wready        : OUT std_logic;                    
            wid           : IN  std_logic_vector(3 DOWNTO 0);                
            wdata         : IN  std_logic_vector(31 DOWNTO 0);                    
            wstrb         : IN  std_logic_vector(3 DOWNTO 0);                    
            wlast         : IN  std_logic;                    
        --------------  read address ---------------
            arready       : OUT std_logic;                    
            arid          : IN  std_logic_vector(3 DOWNTO 0);                    
            araddr        : IN  std_logic_vector(31 DOWNTO 0);                    
            arlen         : IN  std_logic_vector(3 DOWNTO 0);                    
            arsize        : IN  std_logic_vector(2 DOWNTO 0);                    
            arburst       : IN  std_logic_vector(1 DOWNTO 0);                    
            arvalid       : IN  std_logic;                    
        ---------------  read data ---------------    
            rid           : OUT std_logic_vector(3 DOWNTO 0);                
            rdata         : OUT std_logic_vector(31 DOWNTO 0);                    
            rresp         : OUT std_logic_vector(1 DOWNTO 0);                    
            rlast         : OUT std_logic;                    
            rvalid        : OUT std_logic;                    
            rready        : IN  std_logic;                    
        ---------------   write/read handshake ---------------
            bready        : IN  std_logic;                    
            bvalid        : OUT std_logic;                    
            bid           : OUT std_logic_vector(3 DOWNTO 0);                
            bresp         : OUT std_logic_vector(1 DOWNTO 0)                    

    );

END ENTITY axi_slave;

ARCHITECTURE rtl OF axi_slave IS

------- FSM TYPES ---------
TYPE STATE_fsm_aw IS (awidle , awstart , awreadys);
SIGNAL awstate, awnext_state : STATE_fsm_aw;

TYPE STATE_fsm_w IS (widle , wstart , wreadys, wvalids , waddr_dec);
SIGNAL wstate, wnext_state : STATE_fsm_w;

TYPE STATE_fsm_b IS (bidle , bdetect_last , bstart, bwait);
SIGNAL bstate, bnext_state : STATE_fsm_b;

TYPE STATE_fsm_ar IS (aridle , arstart , arreadys);
SIGNAL arstate, arnext_state : STATE_fsm_ar;

TYPE STATE_fsm_r IS (ridle , rstart , rwait, rvalids, rerror);
SIGNAL rstate, rnext_state : STATE_fsm_r;

-- IN write address fsm
SIGNAL awaddrt : std_logic_vector(31 DOWNTO 0); 

-- IN write data fsm
TYPE mem_array IS ARRAY(0 TO 128-1) OF std_logic_vector(7 DOWNTO 0);
SIGNAL mem          : mem_array := (others => (others => '0'));
SIGNAL wdatat       : std_logic_vector(31 DOWNTO 0);
SIGNAL retaddr      : std_logic_vector(31 DOWNTO 0);    
SIGNAL nextaddr     : std_logic_vector(31 DOWNTO 0);    
SIGNAL first        : std_logic;   
SIGNAL boundary     : std_logic_vector(7 DOWNTO 0);
SIGNAL wlen_count   : INTEGER;    

-- for read address
SIGNAL araddrt      : std_logic_vector(31 DOWNTO 0);

-- FOR read data---
SIGNAL rdfirst     : std_logic;
SIGNAL rdnextaddr, rdretaddr : std_logic_vector(31 DOWNTO 0);
SIGNAL len_count   : INTEGER;
SIGNAL rdboundary  : std_logic_vector(7 DOWNTO 0);


------------(write)--------------- FUNCTION to compute next address durINg FIXED burst type
Procedure data_wr_fixed(wstrb : IN std_logic_vector(3 DOWNTO 0); awaddrt : IN std_logic_vector(31 DOWNTO 0); mem_out : OUT mem_array)  IS
    Variable mem_out_buf : mem_array;
    BEGIN 
        CASE(wstrb) IS 
            WHEN    "0001" =>  ---only the first byte bits has a vaild data
                mem_out_buf(to_integer(unsigned(awaddrt))) := wdatat(7 DOWNTO 0);

            WHEN    "0010" => ---only the second byte  bits has a vaild data
                mem_out_buf(to_integer(unsigned(awaddrt))) := wdatat(15 DOWNTO 8);

            WHEN    "0011" => ---only the first byte and the second byte has a vaild data
                mem_out_buf(to_integer(unsigned(awaddrt))) := wdatat(7 DOWNTO 0);
                mem_out_buf(to_integer(unsigned(awaddrt)) + 1) := wdatat(15 DOWNTO 8);

            WHEN    "0100" => ---only the third byte has a vaild data
                mem_out_buf(to_integer(unsigned(awaddrt))) := wdatat(23 DOWNTO 16);

            WHEN    "0101" =>
                mem_out_buf(to_integer(unsigned(awaddrt))) := wdatat(7 DOWNTO 0);
                mem_out_buf(to_integer(unsigned(awaddrt)) + 1) := wdatat(23 DOWNTO 16);

            WHEN    "0110" =>
                mem_out_buf(to_integer(unsigned(awaddrt))) := wdatat(15 DOWNTO 8);
                mem_out_buf(to_integer(unsigned(awaddrt)) + 1) := wdatat(23 DOWNTO 16);

            WHEN    "0111" =>
                mem_out_buf(to_integer(unsigned(awaddrt))) := wdatat(7 DOWNTO 0);
                mem_out_buf(to_integer(unsigned(awaddrt)) + 1) := wdatat(15 DOWNTO 8);
                mem_out_buf(to_integer(unsigned(awaddrt)) + 2) := wdatat(23 DOWNTO 16);

            WHEN    "1000" =>
                mem_out_buf(to_integer(unsigned(awaddrt))) := wdatat(31 DOWNTO 24);

            WHEN    "1001" =>
                mem_out_buf(to_integer(unsigned(awaddrt))) := wdatat(7 DOWNTO 0);
                mem_out_buf(to_integer(unsigned(awaddrt)) + 1) := wdatat(31 DOWNTO 24);

            WHEN    "1010" =>
                mem_out_buf(to_integer(unsigned(awaddrt))) := wdatat(15 DOWNTO 8);
                mem_out_buf(to_integer(unsigned(awaddrt)) + 1) := wdatat(31 DOWNTO 24);

            WHEN    "1011" =>
                mem_out_buf(to_integer(unsigned(awaddrt))) := wdatat(7 DOWNTO 0);
                mem_out_buf(to_integer(unsigned(awaddrt)) + 1) := wdatat(15 DOWNTO 8);
                mem_out_buf(to_integer(unsigned(awaddrt)) + 2) := wdatat(31 DOWNTO 24);

            WHEN    "1100" =>
                mem_out_buf(to_integer(unsigned(awaddrt))) := wdatat(23 DOWNTO 16);
                mem_out_buf(to_integer(unsigned(awaddrt)) + 1) := wdatat(31 DOWNTO 24);

            WHEN    "1101" =>
                mem_out_buf(to_integer(unsigned(awaddrt))) := wdatat(7 DOWNTO 0);
                mem_out_buf(to_integer(unsigned(awaddrt)) + 1) := wdatat(23 DOWNTO 16);
                mem_out_buf(to_integer(unsigned(awaddrt)) + 2) := wdatat(31 DOWNTO 24);

            WHEN    "1110" =>
                mem_out_buf(to_integer(unsigned(awaddrt))) := wdatat(15 DOWNTO 8);
                mem_out_buf(to_integer(unsigned(awaddrt)) + 1) := wdatat(23 DOWNTO 16);
                mem_out_buf(to_integer(unsigned(awaddrt)) + 2) := wdatat(31 DOWNTO 24);

            WHEN    "1111" =>
                mem_out_buf(to_integer(unsigned(awaddrt))) := wdatat(7 DOWNTO 0);
                mem_out_buf(to_integer(unsigned(awaddrt)) + 1) := wdatat(15 DOWNTO 8);
                mem_out_buf(to_integer(unsigned(awaddrt)) + 2) := wdatat(23 DOWNTO 16);
                mem_out_buf(to_integer(unsigned(awaddrt)) + 3) := wdatat(31 DOWNTO 24);

            WHEN others =>
            null; -- Handle any other CASEs as needed

    END CASE;
    mem_out :=  mem_out_buf; -- It's unclear why the FUNCTION RETURNs awaddrt
END Procedure;

------------(write)--------------- FUNCTION to compute next address durINg INCR burst type
PROCEDURE data_wr_incr(wstrb : IN std_logic_vector(3 DOWNTO 0); awaddrt : IN std_logic_vector(31 DOWNTO 0); mem_out : OUT mem_array; addr : OUT std_logic_vector(31 DOWNTO 0) ) IS
    VARIABLE mem_out_buf : mem_array; 
    VARIABLE add_buf : std_logic_vector(31 DOWNTO 0);
    BEGIN 
        CASE(wstrb) IS 
            WHEN "0001" =>
                mem_out_buf(to_integer(unsigned(awaddrt))) := wdatat(7 DOWNTO 0);
                add_buf := std_logic_vector(unsigned(awaddrt) + 1);

            WHEN "0010" =>
                mem_out_buf(to_integer(unsigned(awaddrt))) := wdatat(15 DOWNTO 8);
                add_buf := std_logic_vector(unsigned(awaddrt) + 1);

            WHEN "0011" =>
                mem_out_buf(to_integer(unsigned(awaddrt))) := wdatat(7 DOWNTO 0);
                mem_out_buf(to_integer(unsigned(awaddrt)) + 1) := wdatat(15 DOWNTO 8);
                add_buf := std_logic_vector(unsigned(awaddrt) + 2);

            WHEN "0100" =>
                mem_out_buf(to_integer(unsigned(awaddrt))) := wdatat(23 DOWNTO 16);
                add_buf := std_logic_vector(unsigned(awaddrt) + 1);

            WHEN "0101" =>
                mem_out_buf(to_integer(unsigned(awaddrt))) := wdatat(7 DOWNTO 0);
                mem_out_buf(to_integer(unsigned(awaddrt)) + 1) := wdatat(23 DOWNTO 16);
                add_buf := std_logic_vector(unsigned(awaddrt) + 2);

            WHEN "0110" =>
                mem_out_buf(to_integer(unsigned(awaddrt))) := wdatat(15 DOWNTO 8);
                mem_out_buf(to_integer(unsigned(awaddrt)) + 1) := wdatat(23 DOWNTO 16);
                add_buf := std_logic_vector(unsigned(awaddrt) + 2);

            WHEN "0111" =>
                mem_out_buf(to_integer(unsigned(awaddrt))) := wdatat(7 DOWNTO 0);
                mem_out_buf(to_integer(unsigned(awaddrt)) + 1) := wdatat(15 DOWNTO 8);
                mem_out_buf(to_integer(unsigned(awaddrt)) + 2) := wdatat(23 DOWNTO 16);
                add_buf := std_logic_vector(unsigned(awaddrt) + 3);

            WHEN "1000" =>
                mem_out_buf(to_integer(unsigned(awaddrt))) := wdatat(31 DOWNTO 24);
                add_buf := std_logic_vector(unsigned(awaddrt) + 1);

            WHEN "1001" =>
                mem_out_buf(to_integer(unsigned(awaddrt))) := wdatat(7 DOWNTO 0);
                mem_out_buf(to_integer(unsigned(awaddrt)) + 1) := wdatat(31 DOWNTO 24);
                add_buf := std_logic_vector(unsigned(awaddrt) + 2);

            WHEN "1010" =>
                mem_out_buf(to_integer(unsigned(awaddrt))) := wdatat(15 DOWNTO 8);
                mem_out_buf(to_integer(unsigned(awaddrt)) + 1) := wdatat(31 DOWNTO 24);
                add_buf := std_logic_vector(unsigned(awaddrt) + 2);

            WHEN "1011" =>
                mem_out_buf(to_integer(unsigned(awaddrt))) := wdatat(7 DOWNTO 0);
                mem_out_buf(to_integer(unsigned(awaddrt)) + 1) := wdatat(15 DOWNTO 8);
                mem_out_buf(to_integer(unsigned(awaddrt)) + 2) := wdatat(31 DOWNTO 24);
                add_buf := std_logic_vector(unsigned(awaddrt) + 3);

            WHEN "1100" =>
                mem_out_buf(to_integer(unsigned(awaddrt))) := wdatat(23 DOWNTO 16);
                mem_out_buf(to_integer(unsigned(awaddrt)) + 1) := wdatat(31 DOWNTO 24);
                add_buf := std_logic_vector(unsigned(awaddrt) + 2);

            WHEN "1101" =>
                mem_out_buf(to_integer(unsigned(awaddrt))) := wdatat(7 DOWNTO 0);
                mem_out_buf(to_integer(unsigned(awaddrt)) + 1) := wdatat(23 DOWNTO 16);
                mem_out_buf(to_integer(unsigned(awaddrt)) + 2) := wdatat(31 DOWNTO 24);
                add_buf := std_logic_vector(unsigned(awaddrt) + 3);

            WHEN "1110" =>
                mem_out_buf(to_integer(unsigned(awaddrt))) := wdatat(15 DOWNTO 8);
                mem_out_buf(to_integer(unsigned(awaddrt)) + 1) := wdatat(23 DOWNTO 16);
                mem_out_buf(to_integer(unsigned(awaddrt)) + 2) := wdatat(31 DOWNTO 24);
                add_buf := std_logic_vector(unsigned(awaddrt) + 3);

            WHEN "1111" =>
                mem_out_buf(to_integer(unsigned(awaddrt))) := wdatat(7 DOWNTO 0);
                mem_out_buf(to_integer(unsigned(awaddrt)) + 1) := wdatat(15 DOWNTO 8);
                mem_out_buf(to_integer(unsigned(awaddrt)) + 2) := wdatat(23 DOWNTO 16);
                mem_out_buf(to_integer(unsigned(awaddrt)) + 3) := wdatat(31 DOWNTO 24);
                add_buf := std_logic_vector(unsigned(awaddrt) + 4);

            WHEN others =>
            null; -- Handle any other CASEs as needed
    END CASE;
    mem_out :=  mem_out_buf;
    addr    :=  add_buf;
END PROCEDURE;


--------------(write)----------------- FUNCTION to compute WrappINg boundary
PROCEDURE wrap_boundary(awlen : IN std_logic_vector(3 DOWNTO 0); awsize : IN std_logic_vector(2 DOWNTO 0); boundary_buf : OUT std_logic_vector(7 DOWNTO 0)) IS
    VARIABLE boundary     : INTEGER := 0 ;
BEGIN 
    CASE(awlen) IS 
        WHEN "0001" => 
            CASE(awsize) IS 
                WHEN "000" =>
                    boundary := 2 * 1; -- 2 * 1
                WHEN "001" =>
                    boundary := 2 * 2; -- 2 * 2
                WHEN "010" =>
                    boundary := 2 * 4; -- 2 * 4
                WHEN others =>
                    boundary := 0;
            END CASE;
        WHEN "0011" =>
            CASE awsize IS
                WHEN "000" =>
                    boundary := 4 * 1; -- 4 * 1
                WHEN "001" =>
                    boundary := 4 * 2; -- 4 * 2
                WHEN "010" =>
                    boundary := 4 * 4; -- 4 * 4
                WHEN others =>
                    boundary := 0;
            END CASE;
        WHEN "0111" =>
            CASE awsize IS
                WHEN "000" =>
                    boundary := 8 * 1; -- 8 * 1
                WHEN "001" =>
                    boundary := 8 * 2; -- 8 * 2
                WHEN "010" =>
                    boundary := 8 * 4; -- 8 * 4
                WHEN others =>
                    boundary := 0;
            END CASE;
        WHEN "1111" =>
            CASE awsize IS
                WHEN "000" =>
                    boundary := 16 * 1; -- 16 * 1
                WHEN "001" =>
                    boundary := 16 * 2; -- 16 * 2
                WHEN "010" =>
                    boundary := 16 * 4; -- 16 * 4
                WHEN others =>
                    boundary := 0;
            END CASE;
        WHEN others =>
            boundary := 0;
    END CASE;
        boundary_buf := std_logic_vector(to_unsigned(boundary, boundary_buf'length));
        
END PROCEDURE;

--------------(write)----------------- FUNCTION to compute WrappINg boundary

PROCEDURE data_wr_wrap(wdatat  : IN std_logic_vector(31 DOWNTO 0); wstrb : IN STD_LOGIC_VECTOR(3 DOWNTO 0); awaddrt : IN STD_LOGIC_VECTOR(31 DOWNTO 0); wboundary : IN STD_LOGIC_VECTOR(7 DOWNTO 0) ; mem_out : OUT mem_array; addr : OUT std_logic_vector(31 DOWNTO 0)  ) IS
    VARIABLE addr1, addr2, addr3, addr4 : INTEGER;
    VARIABLE mem_out_buf     : mem_array;
    VARIABLE awaddrt_buf     : INTEGER;
    VARIABLE wboundary_buf   : INTEGER;

BEGIN 
    awaddrt_buf     := to_integer(unsigned(awaddrt));
    wboundary_buf   := to_integer(unsigned(wboundary));
    CASE(wstrb) IS 
        WHEN  "0001" =>
                mem_out_buf(awaddrt_buf) := wdatat(7 DOWNTO 0);
                addr1 := (awaddrt_buf + 1 - wboundary_buf) WHEN (((awaddrt_buf + 1) MOD wboundary_buf) = 0) ELSE awaddrt_buf + 1;
                addr   := std_logic_vector(to_unsigned(addr1, addr'length)); 
         
        WHEN  "0010" =>
                mem_out_buf(awaddrt_buf) := wdatat(15 DOWNTO 8);
                addr1 := (awaddrt_buf + 1 - wboundary_buf) WHEN (((awaddrt_buf + 1) MOD wboundary_buf) = 0) ELSE awaddrt_buf + 1;
                addr := std_logic_vector(to_unsigned(addr1, addr'length)); 

        WHEN  "0011" =>
                mem_out_buf(awaddrt_buf) := wdatat(7 DOWNTO 0);
                addr1 := (awaddrt_buf + 1 - wboundary_buf) WHEN (((awaddrt_buf + 1) MOD wboundary_buf) = 0) ELSE awaddrt_buf + 1;

                mem_out_buf(addr1) := wdatat(15 DOWNTO 8);
                addr2 := (addr1 + 1 - wboundary_buf) WHEN (((addr1 + 1) MOD wboundary_buf) = 0) ELSE addr1 + 1;
                addr := std_logic_vector(to_unsigned(addr2, addr'length)); 


        WHEN  "0100" =>
                mem_out_buf(awaddrt_buf) := wdatat(23 DOWNTO 16);
                addr1 := (awaddrt_buf + 1 - wboundary_buf) WHEN (((awaddrt_buf + 1) MOD wboundary_buf) = 0) ELSE awaddrt_buf + 1;
                addr := std_logic_vector(to_unsigned(addr1, addr'length)); 


        WHEN  "0101" =>
                mem_out_buf(awaddrt_buf) := wdatat(7 DOWNTO 0);
                addr1 := (awaddrt_buf + 1 - wboundary_buf) WHEN (((awaddrt_buf + 1) MOD wboundary_buf) = 0) ELSE awaddrt_buf + 1;

                mem_out_buf(addr1) := wdatat(23 DOWNTO 16);
                addr2 := (addr1 + 1 - wboundary_buf) WHEN (((addr1 + 1) MOD wboundary_buf) = 0) ELSE addr1 + 1;
                addr := std_logic_vector(to_unsigned(addr2, addr'length)); 


        WHEN  "0110" =>
                mem_out_buf(awaddrt_buf) := wdatat(15 DOWNTO 8);
                addr1 := (awaddrt_buf + 1 - wboundary_buf) WHEN (((awaddrt_buf + 1) MOD wboundary_buf) = 0) ELSE awaddrt_buf + 1;

                mem_out_buf(addr1) := wdatat(23 DOWNTO 16);
                addr2 := (addr1 + 1 - wboundary_buf) WHEN (((addr1 + 1) MOD wboundary_buf) = 0) ELSE addr1 + 1;
                addr := std_logic_vector(to_unsigned(addr2, addr'length)); 


        WHEN  "0111" =>
                mem_out_buf(awaddrt_buf) := wdatat(7 DOWNTO 0);
                addr1 := (awaddrt_buf + 1 - wboundary_buf) WHEN (((awaddrt_buf + 1) MOD wboundary_buf) = 0) ELSE awaddrt_buf + 1;

                mem_out_buf(addr1) := wdatat(15 DOWNTO 8);
                addr2 := (addr1 + 1 - wboundary_buf) WHEN (((addr1 + 1) MOD wboundary_buf) = 0) ELSE addr1 + 1;

                mem_out_buf(addr2) := wdatat(23 DOWNTO 16);
                addr3 := (addr2 + 1 - wboundary_buf) WHEN (((addr2 + 1) MOD wboundary_buf) = 0) ELSE addr2 + 1;
                addr := std_logic_vector(to_unsigned(addr3, addr'length)); 

        WHEN  "1000" =>
                mem_out_buf(awaddrt_buf) := wdatat(31 DOWNTO 24);
                addr1 := (awaddrt_buf + 1 - wboundary_buf) WHEN (((awaddrt_buf + 1) MOD wboundary_buf) = 0) ELSE awaddrt_buf + 1;
                addr := std_logic_vector(to_unsigned(addr1, addr'length));              

        WHEN  "1001" =>
                mem_out_buf(awaddrt_buf) := wdatat(7 DOWNTO 0);
                addr1 := (awaddrt_buf + 1 - wboundary_buf) WHEN (((awaddrt_buf + 1) MOD wboundary_buf) = 0) ELSE awaddrt_buf + 1;

                mem_out_buf(addr1) := wdatat(31 DOWNTO 24);
                addr2 := (addr1 + 1 - wboundary_buf) WHEN (((addr1 + 1) MOD wboundary_buf) = 0) ELSE addr1 + 1;
                addr := std_logic_vector(to_unsigned(addr2, addr'length));             

        WHEN  "1010" =>
                mem_out_buf(awaddrt_buf) := wdatat(15 DOWNTO 8);
                addr1 := (awaddrt_buf + 1 - wboundary_buf) WHEN (((awaddrt_buf + 1) MOD wboundary_buf) = 0) ELSE awaddrt_buf + 1;

                mem_out_buf(addr1) := wdatat(31 DOWNTO 24);
                addr2 := (addr1 + 1 - wboundary_buf) WHEN (((addr1 + 1) MOD wboundary_buf) = 0) ELSE addr1 + 1;
                addr := std_logic_vector(to_unsigned(addr2, addr'length));   


        WHEN  "1011" =>
                mem_out_buf(awaddrt_buf) := wdatat(7 DOWNTO 0);
                addr1 := (awaddrt_buf + 1 - wboundary_buf) WHEN (((awaddrt_buf + 1) MOD wboundary_buf) = 0) ELSE awaddrt_buf + 1;

                mem_out_buf(addr1) := wdatat(15 DOWNTO 8);
                addr2 := (addr1 + 1 - wboundary_buf) WHEN (((addr1 + 1) MOD wboundary_buf) = 0) ELSE addr1 + 1;

                mem_out_buf(addr2) := wdatat(31 DOWNTO 24);
                addr3 := (addr2 + 1 - wboundary_buf) WHEN (((addr2 + 1) MOD wboundary_buf) = 0) ELSE addr2 + 1;
                addr := std_logic_vector(to_unsigned(addr3, addr'length)); 

        WHEN  "1100" =>
                mem_out_buf(awaddrt_buf) := wdatat(23 DOWNTO 16);
                addr1 := (awaddrt_buf + 1 - wboundary_buf) WHEN (((awaddrt_buf + 1) MOD wboundary_buf) = 0) ELSE awaddrt_buf + 1;

                mem_out_buf(addr1) := wdatat(31 DOWNTO 24);
                addr2 := (addr1 + 1 - wboundary_buf) WHEN (((addr1 + 1) MOD wboundary_buf) = 0) ELSE addr1 + 1;
                addr := std_logic_vector(to_unsigned(addr2, addr'length));                 


        WHEN  "1101" =>
                mem_out_buf(awaddrt_buf) := wdatat(7 DOWNTO 0);
                addr1 := (awaddrt_buf + 1 - wboundary_buf) WHEN (((awaddrt_buf + 1) MOD wboundary_buf) = 0) ELSE awaddrt_buf + 1;

                mem_out_buf(addr1) := wdatat(23 DOWNTO 16);
                addr2 := (addr1 + 1 - wboundary_buf) WHEN (((addr1 + 1) MOD wboundary_buf) = 0) ELSE addr1 + 1;

                mem_out_buf(addr2) := wdatat(31 DOWNTO 24);
                addr3 := (addr2 + 1 - wboundary_buf) WHEN (((addr2 + 1) MOD wboundary_buf) = 0) ELSE addr2 + 1;
                addr := std_logic_vector(to_unsigned(addr3, addr'length)); 

        WHEN  "1110" =>
                mem_out_buf(awaddrt_buf) := wdatat(15 DOWNTO 8);
                addr1 := (awaddrt_buf + 1 - wboundary_buf) WHEN (((awaddrt_buf + 1) MOD wboundary_buf) = 0) ELSE awaddrt_buf + 1;

                mem_out_buf(addr1) := wdatat(23 DOWNTO 16);
                addr2 := (addr1 + 1 - wboundary_buf) WHEN (((addr1 + 1) MOD wboundary_buf) = 0) ELSE addr1 + 1;

                mem_out_buf(addr2) := wdatat(31 DOWNTO 24);
                addr3 := (addr2 + 1 - wboundary_buf) WHEN (((addr2 + 1) MOD wboundary_buf) = 0) ELSE addr2 + 1;
                addr := std_logic_vector(to_unsigned(addr3, addr'length)); 

        WHEN  "1111" =>
                mem_out_buf(awaddrt_buf) := wdatat(7 DOWNTO 0);
                addr1 := (awaddrt_buf + 1 - wboundary_buf) WHEN (((awaddrt_buf + 1) MOD wboundary_buf) = 0) ELSE awaddrt_buf + 1;

                mem_out_buf(addr1) := wdatat(15 DOWNTO 8);
                addr2 := (addr1 + 1 - wboundary_buf) WHEN (((addr1 + 1) MOD wboundary_buf) = 0) ELSE addr1 + 1;

                mem_out_buf(addr2) := wdatat(23 DOWNTO 16);
                addr3 := (addr2 + 1 - wboundary_buf) WHEN (((addr2 + 1) MOD wboundary_buf) = 0) ELSE addr2 + 1;

                mem_out_buf(addr3) := wdatat(31 DOWNTO 24);
                addr4 := (addr3 + 1 - wboundary_buf) WHEN (((addr3 + 1) MOD wboundary_buf) = 0) ELSE addr3 + 1;
                addr := std_logic_vector(to_unsigned(addr4, addr'length)); 
                
        WHEN OTHERS => 
            NULL;
        END CASE;
        mem_out := mem_out_buf;
END PROCEDURE;


--------------(read)----------------- FUNCTION to compute Fixed Mode
PROCEDURE read_data_fixed(addr : IN std_logic_vector(31 DOWNTO 0); arsize : IN  std_logic_vector(2 DOWNTO 0); mem : IN mem_array ; rdata : OUT std_logic_vector(31 DOWNTO 0))  IS
    VARIABLE addr_buf   : INTEGER;
BEGIN
    addr_buf := to_integer(unsigned(addr));
    CASE arsize is
        WHEN "000" =>
            rdata(7 DOWNTO 0) := mem(addr_buf);
        WHEN "001" =>
            rdata(7 DOWNTO 0)   := mem(addr_buf);
            rdata(15 DOWNTO 8)  := mem(addr_buf + 1);
        WHEN "010" =>
            rdata(7 DOWNTO 0)   := mem(addr_buf);
            rdata(15 DOWNTO 8)  := mem(addr_buf + 1);
            rdata(23 DOWNTO 16) := mem(addr_buf + 2);
            rdata(31 DOWNTO 24) := mem(addr_buf + 3);
        WHEN OTHERS =>
            NULL;
    END CASE;
END PROCEDURE read_data_fixed;

--------------(read)----------------- FUNCTION to compute increament Mode
PROCEDURE read_data_incr(addr : IN std_logic_vector(31 DOWNTO 0); arsize : IN std_logic_vector(2 DOWNTO 0); mem : IN mem_array ; rdata : OUT std_logic_vector(31 DOWNTO 0); nextaddr : OUT std_logic_vector(31 DOWNTO 0)) IS
VARIABLE addr_buf   : INTEGER;
BEGIN
    addr_buf   := to_integer(unsigned(addr));
    CASE arsize is
        WHEN "000" =>
            rdata(7 DOWNTO 0) := mem(addr_buf);
            addr_buf := addr_buf + 1;
            nextaddr := std_logic_vector(to_unsigned(addr_buf, addr'length));
        
        WHEN "001" =>
            rdata(7 DOWNTO 0) := mem(addr_buf);
            rdata(15 DOWNTO 8) := mem(addr_buf + 1);
            addr_buf := addr_buf + 2;
            nextaddr := std_logic_vector(to_unsigned(addr_buf, addr'length));
        
        WHEN "010" =>
            rdata(7 DOWNTO 0) := mem(addr_buf);
            rdata(15 DOWNTO 8) := mem(addr_buf + 1);
            rdata(23 DOWNTO 16) := mem(addr_buf + 2);
            rdata(31 DOWNTO 24) := mem(addr_buf + 3);
            addr_buf := addr_buf + 4;
            nextaddr := std_logic_vector(to_unsigned(addr_buf, addr'length));

        WHEN OTHERS =>
            NULL;
    end CASE;
END PROCEDURE;

--------------(read)----------------- FUNCTION to compute wrap Mode
PROCEDURE read_data_wrap(addr : IN std_logic_vector(31 DOWNTO 0); rsize : IN std_logic_vector(2 DOWNTO 0); rboundary : IN std_logic_vector(7 DOWNTO 0); mem : IN mem_array ; rdata : OUT std_logic_vector(31 DOWNTO 0); nextaddr : OUT std_logic_vector(31 DOWNTO 0))   IS 
        VARIABLE addr1, addr2, addr3, addr4 : INTEGER;
        VARIABLE addr_buf        : INTEGER;
        VARIABLE rboundary_buf   : INTEGER;
    BEGIN   
    addr_buf         := to_integer(unsigned(addr));
    rboundary_buf    := to_integer(unsigned(rboundary));
        CASE(rsize) IS 
            WHEN "000" =>
                rdata(7 DOWNTO 0) := mem(addr_buf);
                addr1 := (addr_buf +  1 - rboundary_buf) WHEN (((addr_buf + 1) MOD rboundary_buf) = 0) ELSE (addr_buf +  1); 
                nextaddr := std_logic_vector(to_unsigned(addr1, addr'length));

            WHEN "001" => 
                rdata(7 DOWNTO 0) := mem(addr_buf);
                addr1 := (addr_buf +  1 - rboundary_buf) WHEN (((addr_buf + 1) MOD rboundary_buf) = 0) ELSE (addr_buf +  1);

                rdata(15 DOWNTO 8) := mem(addr1);
                addr2 := (addr1 + 1 - rboundary_buf) WHEN (((addr1 +1) MOD rboundary_buf) = 0) ELSE (addr1 + 1);
                nextaddr := std_logic_vector(to_unsigned(addr2, addr'length));

            WHEN "010" => 
                rdata(7 DOWNTO 0) := mem(addr_buf);
                addr1 := (addr_buf +  1 - rboundary_buf) WHEN (((addr_buf + 1) MOD rboundary_buf) = 0) ELSE (addr_buf +  1);

                rdata(15 DOWNTO 8) := mem(addr1);
                addr2 := (addr1 + 1 - rboundary_buf) WHEN (((addr1 +1) MOD rboundary_buf) = 0) ELSE (addr1 + 1);

                rdata(23 DOWNTO 16) := mem(addr2);
                addr3 := (addr2 + 1 - rboundary_buf) WHEN (((addr2 +1) MOD rboundary_buf) = 0) ELSE (addr2 + 1);

                rdata(15 DOWNTO 8) := mem(addr3);
                addr4 := (addr3 + 1 - rboundary_buf) WHEN (((addr3 +1) MOD rboundary_buf) = 0) ELSE (addr3 + 1);
                nextaddr := std_logic_vector(to_unsigned(addr4, addr'length));
        
         WHEN OTHERS =>
            NULL;   
        END CASE; 
END PROCEDURE;
            






BEGIN 

----------- FSM for write  process -----------
reset_process : PROCESS(clk, reset)
    BEGIN 
        IF (reset = '0' ) THEN
            awstate <= awidle;
            wstate  <= widle; 
            bstate  <= bidle; 
        ELSE 
            awstate <= awnext_state;
            wstate  <= wnext_state;
            bstate  <= bnext_state;
        END IF;
END PROCESS;


--------- fsm for writtINg address ----------

write_addr_proc : PROCESS(awstate)
    BEGIN 
        CASE(awstate) IS 
            WHEN awidle => 
                awready <= '0';
                awnext_state <= awstart;
            
            WHEN(awstart) => 
                IF (awvalid ='1') THEN 
                    awnext_state <= awreadys;
                    awaddrt      <= awaddr;  --- load the address
                ELSE 
                    awnext_state <= awstart;
                END IF;
            
            WHEN(awreadys) => 
                awready <= '1';
                IF(wstate = wreadys) THEN    ------stay here until write IS ready
                    awnext_state <= awidle;
                ELSE 
                    awnext_state <= awreadys;
                END IF;
        END CASE;
END PROCESS;



------------------- fsm writtINg data ----------------

write_data_proc : PROCESS(wstate)
Variable boundary    : std_logic_vector(7 DOWNTO 0);
Variable mem_st      : mem_array;
Variable retaddr_st  : std_logic_vector(31 DOWNTO 0);
BEGIN 
    mem_st := mem;
    CASE(wstate) IS 
        WHEN widle => 
            wready <= '0';
            wnext_state <= wstart;
            first <= '0';
            wlen_count <= 0;
        
        WHEN wstart =>
            IF(wvalid = '1') THEN
                wnext_state <= waddr_dec;
                wdatat <= wdata;
            ELSE 
                wnext_state <= wstart;
            END IF;
        
        WHEN waddr_dec => 
            wnext_state <= wreadys;
            IF(first = '0') THEN 
                nextaddr    <= awaddr;
                first       <= '1';
                wlen_count  <= 0;
            ELSIF(wlen_count < (to_integer(unsigned(awlen)) + 1)) THEN 
                nextaddr     <= retaddr;
            ELSE 
                nextaddr <= awaddr;
            END IF;

        WHEN wreadys => 
            IF(wlast = '1') THEN 
                wnext_state <= widle;
                wready <= '0';
                wlen_count <= 0;
                first <= '0';
            ELSIF(wlen_count < (to_integer(unsigned(awlen)) + 1)) THEN 
                wnext_state <= wvalids;
                wready <= '1';
            ELSE 
                wnext_state <= wreadys;
            END IF; 

            CASE(awburst) IS 
                WHEN "00" => --- Fixed Mode
                    data_wr_fixed(wstrb, awaddr,mem_st);
                    retaddr <= awaddr;
                WHEN "01" => ----- INcr mode
                    data_wr_incr(wstrb,nextaddr,mem_st,retaddr_st);  
                    retaddr <= retaddr_st;
                WHEN "10" =>  ------- wrappINg
                    wrap_boundary(awlen, awsize, boundary);
                    data_wr_wrap(wdatat, wstrb, nextaddr, boundary,mem_st, retaddr_st);
                    retaddr <= retaddr_st;
                WHEN "11" =>
                    NULL;
            END CASE;

        WHEN wvalids => 
            wready <= '0';
            wnext_state <= wstart;
            IF (wlen_count < (to_integer(unsigned(awlen)) + 1)) THEN 
                wlen_count <= wlen_count + 1;
            ELSE 
                wlen_count <= wlen_count;
            END IF;
    END CASE;
END PROCESS;


-------------------------- fsm for the handshake
res_proc : PROCESS(bstate)
BEGIN 
    CASE(bstate) IS 
        WHEN bidle => 
            bid     <= (others => '0');
            bresp   <= (others => '0'); 
            bvalid  <= '0';
            bnext_state <= bdetect_last;
        
        WHEN bdetect_last =>
            IF(wlast = '1') THEN 
                bnext_state <= bstart;  
            ELSE 
                bnext_state <= bdetect_last;
            END IF;

        WHEN bstart =>
            bid <= awid;
            bvalid <= '1';
            bnext_state <= bwait;
            IF ((awaddr < "10000000") AND (awsize <= "010")) THEN ---- Met cond
                bresp <= "00";
            ELSIF (awsize > "010") THEN ---  slave
                bresp <= "10";
            ELSE 
                bresp <= "11";  --- NO slave
            END IF;
        
        WHEN bwait =>
            IF(bready = '1') THEN --- ready to recive
                bnext_state <= bidle;
            ELSE 
                bnext_state <= bwait;
            END IF;
    END CASE;
END PROCESS;

--------------------------- FSM for update read processes --------------

 read_proc : PROCESS (clk,reset)    
 BEGIN 
    IF (risINg_edge(clk) OR (fallINg_edge(reset))) THEN 
        arstate <= aridle;
        rstate  <= ridle; 
    ELSE 
        arstate <= arnext_state;
        rstate  <= rnext_state;     
    END IF;
END PROCESS;


---------------- FSM for read address proc ---------------
add_read : PROCESS(arstate) 
BEGIN   
    CASE(arstate) IS 
        WHEN aridle =>
            arready <= '0';
            arnext_state <= arstart;
        
        WHEN arstart => 
            IF(arvalid <= '1') THEN 
                arnext_state <= arreadys;
                araddrt <= araddr; 
            ELSE 
                arnext_state <= arstart;  
            END IF;

        WHEN arreadys => 
            arnext_state <= aridle;
            arready <= '1';
    END CASE;
END PROCESS;


read_data_process : PROCESS(rstate)
Variable rdboundary     : std_logic_vector(7 DOWNTO 0);
Variable rdretaddr      : std_logic_vector(31 DOWNTO 0);
Variable rdata_st       : std_logic_vector(31 DOWNTO 0);
BEGIN 
        CASE(rstate) IS 
            WHEN ridle => 
                rid        <= (others => '0');
                rdfirst    <= '0';
                rdata      <= (others => '0');
                rdata_st   := (others => '0');
                rresp      <= (others => '0');
                rlast      <= '0';
                rvalid     <= '0';
                len_count  <= 0;
                IF (rvalid = '1') THEN 
                    rnext_state <= rstart;
                ELSE
                    rnext_state <= ridle;
                END IF;


            WHEN rstart => 
                IF ((araddrt < "10000000") AND (arsize <= "010")) THEN 
                    rid <= arid;
                    rvalid <='1';
                    rnext_state <= rwait;
                    rresp <= "00";

                        CASE(arburst) IS 
                            WHEN "00" => 
                                IF(rdfirst = '0') THEN
                                    rdnextaddr <= araddr;
                                    rdfirst <= '1';
                                    len_count <= 0;
                                ELSIF(len_count /= (to_integer(unsigned(arlen))  + 1)) THEN 
                                    rdnextaddr <= araddr;
                                END IF;
                                read_data_fixed(araddrt, arsize,mem,rdata_st);
                                rdata <= rdata_st;
                            WHEN "01" =>
                                IF(rdfirst = '0') THEN 
                                    rdnextaddr  <= araddr;
                                    rdfirst     <= '1';
                                    len_count   <= 0;
                                ELSIF (len_count /= (to_integer(unsigned(arlen))  + 1)) THEN 
                                    rdnextaddr <= rdretaddr;
                                END IF;
                                read_data_incr(rdnextaddr, arsize,mem,rdata_st,rdretaddr);
                                rdata <= rdata_st;
                            WHEN "10" => 
                                IF(rdfirst = '0') THEN 
                                    rdnextaddr  <= araddr;
                                    rdfirst     <= '1';
                                    len_count   <= 0;
                                ELSIF (len_count /= (to_integer(unsigned(arlen))  + 1)) THEN 
                                    rdnextaddr <= rdretaddr;    
                                END IF;
                                wrap_boundary(arlen, arsize,rdboundary);
                                read_data_wrap(rdnextaddr, arsize, rdboundary,mem,rdata_st,rdretaddr);
                                rdata <= rdata_st;
                            WHEN OTHERS =>
                                NULL;
                        END CASE;
                    
                ELSIF ((araddr >= "10000000") AND (arsize <= "010")) THEN
                    rresp  <= "11";  
                    rvalid <= '1';
                    rnext_state <= rerror; 

                ELSIF(arsize > "010") THEN
                    rresp <= "10";
                    rvalid <='1';
                    rnext_state <= rerror;
                END IF;
            
            WHEN rwait =>
                rvalid <= '0';
                IF (rready ='1') THEN 
                    rnext_state <= rvalids;
                ELSE 
                    rnext_state <= rwait;
                END IF;

            WHEN rvalids => 
                len_count <= len_count +1;
                IF(len_count = (to_integer(unsigned(arlen))  + 1)) THEN 
                    rnext_state <= ridle;
                    rlast <= '1';
                ELSE 
                    rnext_state <= rstart;
                    rlast <= '0';
                END  IF;
            WHEN rerror => 
                rvalid <= '0';
                IF(len_count < (to_integer(unsigned(arlen)))) THEN 
                    IF (arready = '1') THEN 
                        rnext_state <= rstart;
                        len_count <= len_count +1;
                    END IF;
                ELSE 
                    rlast <= '1';
                    rnext_state <= ridle;
                    len_count <= 0;
                END IF;

            WHEN OTHERS =>
                NULL;
        END CASE;
END PROCESS;
END ARCHITECTURE rtl;       







                






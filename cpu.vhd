library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;
use ieee.math_real.uniform;
use ieee.math_real.floor;
use ieee.std_logic_arith.all;

entity cpu is
    port( 
        clk : in std_logic;
        reset : in std_logic;
        wr_en : out std_logic;
        dr : in std_logic_vector( 7 downto 0); -- Data from the memory
        dw : out std_logic_vector( 7 downto 0); -- Data to the memory
        addr : out std_logic_vector( 7 downto 0); -- Memory address
        pc_out : out std_logic_vector( 7 downto 0); -- Program counter value
        accu_out : out std_logic_vector( 7 downto 0) -- Accumulator value
    );
end cpu;

architecture fsm of cpu is

    component multiplier is
    port (  clk, reset : in std_logic;
            multiplicand, multiplier : in integer;
            product: out integer);
    end component;
    -- op-codes
    constant LDA : std_logic_vector( 3 downto 0) := "0001";         -- 1
    constant STA : std_logic_vector( 3 downto 0) := "0010";         -- 2
    constant ADD : std_logic_vector( 3 downto 0) := "0011";         -- 3
    constant JNC : std_logic_vector( 3 downto 0) := "0100";         -- 4
    constant JMP : std_logic_vector( 3 downto 0) := "0101";         -- 5
    constant SUB : std_logic_vector( 3 downto 0) := "0110";         -- 6
    constant SHL_0 : std_logic_vector( 3 downto 0) := "0111";       -- 7
    constant SHR_0 : std_logic_vector( 3 downto 0) := "1000";       -- 8
    constant MUL : std_logic_vector( 3 downto 0) := "1001";         -- 9
    constant PRG : std_logic_vector( 3 downto 0) := "1010";         -- A
    constant JSR : std_logic_vector( 3 downto 0) := "1011";         -- B
    constant JFA : std_logic_vector( 3 downto 0) := "1100";         -- C
    constant LDV : std_logic_vector( 3 downto 0) := "1101";         -- D
    constant one : std_logic_vector( 7 downto 0) := "00000001";
    -- FSM states
    type state_t is ( load_opcode, LDA_1, STA_1, ADD_1, JNC_1, JMP_1, SUB_1, SHL_1, SHR_1, MUL_1, PRG_1, JSR_1, JFA_1, LDV_1); -- List of states in the CPU FSM
    -- Signals used for debugging
    signal state_watch : state_t;
    -- CPU registers
    signal accu : std_logic_vector( 7 downto 0) := "00000000" ; --Accumulator
    signal accu_carry : std_logic_vector( 8 downto 0) := "000000000" ; --Accumulator with carry
    signal op_code : std_logic_vector( 3 downto 0) := "0000" ; -- Current op-code
    signal pc : std_logic_vector( 7 downto 0) := "00000000" ; -- Program counter
    signal counter : std_logic_vector( 7 downto 0) := "00000000" ; -- Counter
    signal carry_flag: std_logic := '0';
    
    signal reset_mult: std_logic := '0';
    signal multiplicand, multiplier_b, product: integer;
        
begin -- fsm
    mult4:entity work.multiplier(mixed)
    port map(clk=>clk, reset=>reset_mult, multiplicand=>multiplicand, multiplier=>multiplier_b,product=>product);

    -- Accumulator and program counter value outputs
    accu_out <= accu;
    pc_out <= pc;
    fsm_proc : process ( clk, reset)
            variable state : state_t := load_opcode;
            variable rnd_num : real;
            variable seed1 : positive;
            variable seed2 : positive;
        begin -- process fsm_proc
        if accu_carry(8) = '1' then
            carry_flag <= '1';
        end if;
        if ( reset = '1') then -- Asynchronous reset
            -- output and variable initialisation
            wr_en <= '0';
            dw <= ( others => '0');
            addr <= ( others => '0');
            op_code <= ( others => '0');
            accu <= ( others => '0');
            accu_carry <= ( others => '0');
            carry_flag <= '0';
            pc <= ( others => '0');
            counter <= ( others => '0');
            state := load_opcode;
        elsif rising_edge( clk) then -- Synchronous FSM
            state_watch <= state;
            case state is
            when load_opcode =>
                op_code <= dr(3 downto 0); -- Load the op-code
                pc <= pc + one; -- Increment the program counter
                addr <= pc + one; -- Memory address pointed to PC
                -- Op-code determines the next state:
                case dr (3 downto 0) is
                    when LDA => state := LDA_1;
                    when STA => state := STA_1;
                    when ADD => state := ADD_1;
                    when JNC => state := JNC_1;
                    when JMP => state := JMP_1;
                    when SUB => state := SUB_1;
                    when SHL_0 => state := SHL_1;
                    when SHR_0 => state := SHR_1;
                    when MUL => state := MUL_1;
                    when PRG => state := PRG_1;
                    when JSR => state := JSR_1;
                    when JFA => state := JFA_1;
                    when LDV => state := LDV_1;
                    when others => state := load_opcode;
                end case; -- opcode decoder
            -- Op-code behaviors here:
            when LDA_1 => -- Load accumulator from memory address
                wr_en <= '0';
                accu <= dr;
                pc <= pc + one;
                addr <= pc + one;
                state := load_opcode;
            when STA_1 => -- Store accumulator to memory address
                if counter = "00000001" then
                    wr_en <= '0';
                    counter <= "00000000";
                    pc <= pc + one;
                    addr <= pc + one;
                    state := load_opcode;
                else
                    wr_en <= '1';
                    dw <= accu;
                    addr <= dr;
                    counter <= counter + one;
                end if;
            when ADD_1 => -- Add contents at addr to accumulator
                wr_en <= '0';
                if counter = "00001000" then
                    accu_carry <= ('0'&accu) + ('0'&dr);
                    accu <= accu + dr;
                    counter <= "00000000";
                    pc <= pc + one;
                    addr <= pc + one;
                    state := load_opcode;
                elsif counter = "00000000" then
                    addr <= dr;
                    counter <= counter + one;
                else
                    counter <= counter + one;
                end if;
            when JNC_1 => -- Jump to addr if carry flag is not set
                wr_en <= '0';
                pc <= pc + one;
                addr <= pc + one;
                if carry_flag='0' then
                    pc <= dr;
                    addr <= dr;
                else
                    carry_flag <= '0';
                    accu_carry(8) <= '0';
                end if;
                state := load_opcode;
            when JMP_1 => -- Jump to addr
                wr_en <= '0';
                pc <= dr;
                addr <= dr;
                state := load_opcode;
            when SUB_1 => -- Subtracts contents at addr from accumulator
                wr_en <= '0';
                if counter = "00001000" then
                    accu_carry <= ('0'&accu) - ('0'&dr);
                    accu <= accu - dr;
                    counter <= "00000000";
                    pc <= pc + one;
                    addr <= pc + one;
                    state := load_opcode;
                elsif counter = "00000000" then
                    addr <= dr;
                    counter <= counter + one;
                else
                    counter <= counter + one;
                end if;
            when SHL_1 => -- Shifts accumulator left by 1
                wr_en <= '0';
                accu <= accu(6 downto 0) & '0';
                pc <= pc + one;
                addr <= pc + one;
                state := load_opcode;
            when SHR_1 => -- Shift accumulator right by 1
                wr_en <= '0';
                accu <= '0' & accu(7 downto 1);
                pc <= pc + one;
                addr <= pc + one;
                state := load_opcode;
            when MUL_1 => -- Multiplies contents at addr with accumulator
                wr_en <= '0';
                if counter = "00001000" then
                    --accu <= accu(3 downto 0) * dr(3 downto 0);
                    accu <= conv_std_logic_vector(product, 8);
                    reset_mult <= '0';
                    counter <= "00000000";
                    pc <= pc + one;
                    addr <= pc + one;
                    state := load_opcode;
                elsif counter = "00000000" then
                    addr <= dr;   
                    counter <= counter + one; 
                elsif counter = "00000001" then
                    reset_mult <= '1';
                    multiplicand <= conv_integer(accu);        
                    multiplier_b <= conv_integer(dr);                     
                    counter <= counter + one;   
                elsif counter = "00000010" then
                    reset_mult <= '0';
                    counter <= counter + one; 
                else
                    counter <= counter + one;
                end if;
            when PRG_1 => -- Load accumulator with pseudo-random
                wr_en <= '0';
                seed1 := 0;
                seed2 := 1;
                if counter = "00000100" then
                    counter <= "00000000";
                    uniform(seed1, seed2, rnd_num);
                    accu <= conv_std_logic_vector(integer(floor(rnd_num * 256.0)), 8);
                    pc <= pc + one;
                    addr <= pc + one;
                    state := load_opcode;
                else
                    counter <= counter + one;
                end if;               
            when JSR_1 => -- Store next address to operate when doen, jump to subroutine at address value.
                wr_en <= '1';
                addr <= "00011111";
                if counter = "00000010" then
                    wr_en <= '0';
                    counter <= "00000000";
                    pc <= pc;
                    addr <= pc;
                    state := load_opcode;
                elsif counter = "00000001" then
                    counter <= counter + one;
                else
                    pc <= dr;
                    dw <= pc+one;
                    counter <= counter + one;
                end if;
            when JFA_1 => -- Jumps to address stored in accumulator
                wr_en <= '0';
                pc <= accu;
                addr <= accu;
                state := load_opcode;
            when LDV_1 =>
                wr_en <= '0';
                if counter = "00000010" then
                    counter <= "00000000";
                    pc <= pc + 1;
                    addr <= pc + 1;
                    state := load_opcode;
                elsif counter = "00000001" then
                    counter <= counter + 1;
                    accu <= dr;
                else
                    addr <= dr;
                    counter <= counter + 1;
                end if;
            end case; -- state
        end if; -- rising_edge(clk)
    end process fsm_proc;
end fsm;

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

entity procram is
    port(
        A : in std_logic_vector(7 downto 0);
        DI : in std_logic_vector(7 downto 0);
        RESET : in std_logic;
        WR_EN : in std_logic;
        CLK : in std_logic;
        DO : out std_logic_vector(7 downto 0)
    );
end procram;

architecture sim of procram is
    -- 16-word blocks of RAM and ROM memory
    type mem_array is array (0 to 15) of std_logic_vector(7 downto 0);
    signal ram_data: mem_array := (others => x"00");
    signal rom_data: mem_array :=
--    (x"0b",x"04",x"05",x"00",x"0a",x"00",x"06",x"01",
--    x"07",x"00",x"09",x"02",x"0d",x"1F",x"0c",x"00");
--    (x"01",x"07",x"03",x"0a",x"02",x"10",x"04",x"02",
--    x"05",x"00",x"09",x"00",x"00",x"00",x"00",x"00");
(x"01",x"05",x"09",x"0F",x"01",x"0A",x"09",x"0F",
 x"00",x"00",x"00",x"00",x"00",x"00",x"00",x"0C");

begin
    process(clk, WR_EN, RESET, A)
        variable address : integer := 0;
    begin
        address := to_integer(unsigned(a));
        if reset = '1' then
            ram_data <= (others => x"00");
        elsif rising_edge(clk) then
            if ((WR_EN='1') and (address>15) and (address<32)) then
                ram_data (address-16) <= DI; -- Write to RAM address
            end if;
        end if;
        if (address>31) then
            DO <= (others => '0');
        elsif (address>15) then
            DO <= ram_data(address-16); -- Read from RAM address
        else
            DO <= rom_data(address); -- Read from ROM address
        end if;
    end process;
END sim;


library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;

entity disp4 is
    Port (
        clk: in std_logic;
        disp_in : in std_logic_vector(15 downto 0);
        an : out std_logic_vector (3 downto 0);
        CA, CB, CC, CD, CE, CF, CG : out std_logic
    );
end disp4;

architecture Behavioral of disp4 is

procedure display_digit
    (signal digit: in std_logic_vector (3 downto 0);
    signal A, B, C, D, E, F, G : out std_logic) is
begin
    case digit is
        when "0000" => -- 0
            A <= '0'; B <= '0'; C <= '0'; D <= '0';
            E <= '0'; F <= '0'; G <= '1'; -- 0
        when "0001" => -- 1
            A <= '1'; B <= '0'; C <= '0'; D <= '1';
            E <= '1'; F <= '1'; G <= '1'; -- 1
        when "0010" => -- 02
            A <= '0'; B <= '0'; C <= '1'; D <= '0';
            E <= '0'; F <= '1'; G <= '0'; -- 2
        when "0011" => -- 03
            A <= '0'; B <= '0'; C <= '0'; D <= '0';
            E <= '1'; F <= '1'; G <= '0'; -- 3
        when "0100" => -- 04
            A <= '1'; B <= '0'; C <= '0'; D <= '1';
            E <= '1'; F <= '0'; G <= '0'; -- 4
        when "0101" => -- 05
            A <= '0'; B <= '1'; C <= '0'; D <= '0';
            E <= '1'; F <= '0'; G <= '0'; -- 5
        when "0110" => -- 06
            A <= '0'; B <= '1'; C <= '0'; D <= '0';
            E <= '0'; F <= '0'; G <= '0'; -- 6
        when "0111" => -- 07
            A <= '0'; B <= '0'; C <= '0'; D <= '1';
            E <= '1'; F <= '1'; G <= '1'; -- 7
        when "1000" => -- 08
            A <= '0'; B <= '0'; C <= '0'; D <= '0';
            E <= '0'; F <= '0'; G <= '0'; -- 8
        when "1001" => -- 09
            A <= '0'; B <= '0'; C <= '0'; D <= '0';
            E <= '1'; F <= '0'; G <= '0'; -- 9
        when "1010" => -- A
            A <= '0'; B <= '0'; C <= '0'; D <= '1';
            E <= '0'; F <= '0'; G <= '0';
        when "1011" => -- B
            A <= '1'; B <= '1'; C <= '0'; D <= '0';
            E <= '0'; F <= '0'; G <= '0';
        when "1100" => -- C
            A <= '0'; B <= '1'; C <= '1'; D <= '0';
            E <= '0'; F <= '0'; G <= '1';
        when "1101" => -- D
            A <= '1'; B <= '0'; C <= '0'; D <= '0';
            E <= '0'; F <= '1'; G <= '0';
        when "1110" => -- E
            A <= '0'; B <= '1'; C <= '1'; D <= '0';
            E <= '0'; F <= '0'; G <= '0';
        when "1111" => -- F
            A <= '0'; B <= '1'; C <= '1'; D <= '1';
            E <= '0'; F <= '0'; G <= '0';
        when others => null;
    end case;
end display_digit;

type digit_array is array (3 downto 0) of std_logic_vector (3 downto 0);
signal digit: digit_array;
begin
    gen0 : for i in 0 to 3 generate
        digit(i) <= disp_in (((4*i)+3) downto (4*i));
    end generate;
    selector: process (clk, digit)
        variable counter : integer := 0;
        variable place : integer := 0;
    begin
        if rising_edge(clk) then
            counter := counter + 1;
            if (counter > 50000) then
                counter := 0;
                place := place + 1;
                if place > 3 then
                    place := 0;
                end if;
            end if;
        end if;
        -- select digit to display
        if (place = 0) then
            an <= "1110";
            display_digit(digit(0),CA,CB,CC,CD,CE,CF,CG);
        elsif (place = 1) then
            an <= "1101";
            display_digit(digit(1),CA,CB,CC,CD,CE,CF,CG);
        elsif (place = 2) then
            an <= "1011";
            display_digit(digit(2),CA,CB,CC,CD,CE,CF,CG);
        else
            an <= "0111";
            display_digit(digit(3),CA,CB,CC,CD,CE,CF,CG);
        end if;
    end process selector;
end Behavioral;

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;

entity cpu_top is
    port( 
        clk : in std_logic; -- cpu clock
        clk50 : in std_logic; -- display clock (50 MHz)
        reset : in std_logic;
        an : out std_logic_vector(3 downto 0); -- Anode for 7-segment display
        CA, CB, CC, CD, CE, CF, CG : out std_logic; -- 7-segment display cathodes
        dr_out, dw_out, addr_out, pc, accu : out std_logic_vector (7 downto 0) -- Waveform output
    );
end cpu_top;

architecture rtl of cpu_top is
    signal wr_en: std_logic := '1';
    signal dr : std_logic_vector( 7 downto 0) := (others => '0'); --Data from the memory
    signal dw : std_logic_vector( 7 downto 0) := (others => '0'); --Data to the memory
    signal addr : std_logic_vector( 7 downto 0) := (others => '0'); --Memory address
    signal pc_out : std_logic_vector( 7 downto 0); -- Program counter value
    signal accu_out : std_logic_vector( 7 downto 0); -- Accumulator value
    signal output : std_logic_vector (15 downto 0);
    
    component procram
        port(
            A : in std_logic_vector(7 downto 0);
            DI : in std_logic_vector(7 downto 0);
            RESET : in std_logic;
            WR_EN : in std_logic;
            CLK : in std_logic;
            DO : out std_logic_vector(7 downto 0)
        );
    end component;
    
    component cpu
        port(
            clk : in std_logic;
            reset : in std_logic;
            wr_en : out std_logic;
            dr : in std_logic_vector( 7 downto 0); -- Data from the memory
            dw : out std_logic_vector( 7 downto 0); -- Data to the memory
            addr : out std_logic_vector( 7 downto 0); -- Memory address
            pc_out : out std_logic_vector( 7 downto 0); -- Program counter value
            accu_out : out std_logic_vector( 7 downto 0) -- Accumulator value
        );
    end component;

    component disp4
        Port (
            clk: in std_logic;
            disp_in : in std_logic_vector(15 downto 0);
            an : out std_logic_vector (3 downto 0);
            CA, CB, CC, CD, CE, CF, CG : out std_logic
        );
    end component;
    
begin -- rtl
    output <= pc_out & accu_out;
    dr_out <= dr;
    dw_out <= dw;
    addr_out <= addr;
    pc <= pc_out;
    accu <= accu_out;
    cpu_inst: cpu
        port map(
            clk => clk,
            reset => reset,
            wr_en => wr_en,
            dr => dr,
            dw => dw,
            addr => addr,
            pc_out => pc_out,
            accu_out => accu_out
        );
        
    mem_inst: procram
        port map(
            A => addr,
            DI => dw,
            RESET => reset,
            WR_EN => wr_en,
            CLK => clk,
            DO => dr
        );
        
    display : disp4
        port map( clk50, output, an, CA, CB, CC, CD, CE, CF, CG);
end rtl;
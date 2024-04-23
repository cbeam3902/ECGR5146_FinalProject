library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;
entity mux is
    port (
        a, b, s : in std_logic;
        y: out std_logic
    );
end entity mux;

architecture struct of mux is
signal temp1, temp2, s_not: std_logic;
begin
and_gate1: entity work.and2(basic)
port map (a, s_not, temp1);
not_gate: entity work.not1(basic)
port map (s, s_not);
and_gate2: entity work.and2(basic)
port map (b, s, temp2);
or_gate: entity work.or2(basic)
port map (temp1, temp2, y);
end architecture struct;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;
entity reg is
    port (
        d : in integer;
        q : out integer;
        en, reset, clk: in std_logic 
    );
end entity reg;

architecture struct of reg is
    signal d_temp, q_temp, l_temp, mux_out: std_logic_vector(7 downto 0);
    signal clk_not: std_logic;
begin
    d_temp <= conv_std_logic_vector(d, 8);
    
    not_gate: entity work.not1(basic)
    port map (clk, clk_not);
    
    mux_bit0: entity work.mux(struct)
    port map(d_temp(0), '0', reset, mux_out(0));
    mux_bit1: entity work.mux(struct)
    port map(d_temp(1), '0', reset, mux_out(1));
    mux_bit2: entity work.mux(struct)
    port map(d_temp(2), '0', reset, mux_out(2));
    mux_bit3: entity work.mux(struct)
    port map(d_temp(3), '0', reset, mux_out(3));
    mux_bit4: entity work.mux(struct)
    port map(d_temp(4), '0', reset, mux_out(4));
    mux_bit5: entity work.mux(struct)
    port map(d_temp(5), '0', reset, mux_out(5));
    mux_bit6: entity work.mux(struct)
    port map(d_temp(6), '0', reset, mux_out(6));
    mux_bit7: entity work.mux(struct)
    port map(d_temp(7), '0', reset, mux_out(7));
    
    reg41: entity work.reg4(struct)
    port map (mux_out(0), mux_out(1), mux_out(2), mux_out(3), en, clk_not, l_temp(0), l_temp(1), l_temp(2), l_temp(3));
    reg42: entity work.reg4(struct)
    port map (mux_out(4), mux_out(5), mux_out(6), mux_out(7), en, clk_not, l_temp(4), l_temp(5), l_temp(6), l_temp(7));
    reg41_out: entity work.reg4(struct)
    port map (l_temp(0), l_temp(1), l_temp(2), l_temp(3), en, clk, q_temp(0), q_temp(1), q_temp(2), q_temp(3));
    reg42_out: entity work.reg4(struct)
    port map (l_temp(4), l_temp(5), l_temp(6), l_temp(7), en, clk, q_temp(4), q_temp(5), q_temp(6), q_temp(7));
    
    q <= conv_integer(q_temp);
end architecture struct;


library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;
entity shift_reg is
    port (
        d : in integer;
        q : out std_logic;
        load, clk: in std_logic
    );
end entity shift_reg;

architecture struct of shift_reg is
    signal d_temp, temp, mux_out, latch_out: std_logic_vector(3 downto 0);
    signal clk_not: std_logic;
begin
    d_temp <= conv_std_logic_vector(d, 4);
    
    not_gate: entity work.not1(basic)
    port map(clk, clk_not);
    
    mux_bit0: entity work.mux(struct)
    port map(temp(1), d_temp(0), load, mux_out(0));
    mux_bit1: entity work.mux(struct)
    port map(temp(2), d_temp(1), load, mux_out(1));
    mux_bit2: entity work.mux(struct)
    port map(temp(3), d_temp(2), load, mux_out(2));
    mux_bit3: entity work.mux(struct)
    port map('0', d_temp(3), load, mux_out(3));
    
    reg1: entity work.reg4(struct)
    port map (mux_out(0), mux_out(1), mux_out(2), mux_out(3), '1', clk_not, latch_out(0), latch_out(1), latch_out(2), latch_out(3));
    reg2: entity work.reg4(struct)
    port map (latch_out(0), latch_out(1), latch_out(2), latch_out(3), '1', clk, temp(0), temp(1), temp(2), temp(3));
    
    q <= temp(0);
end architecture struct;


library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;
entity shift_adder is
    port (
        addend, augend: in integer;
        sum: out integer;
        add_load, add_control, clk: in std_logic
    );
end entity shift_adder;

architecture struct of shift_adder is
    signal addend_temp: std_logic_vector(3 downto 0);
    signal augend_temp, sum_temp, l_temp, q, temp, mux_out: std_logic_vector(7 downto 0);
    signal clk_not: std_logic; 
begin
    addend_temp <= conv_std_logic_vector(addend, 4);
    augend_temp <= conv_std_logic_vector(augend, 8);
    
    not_gate: entity work.not1(basic)
    port map (clk, clk_not);
    
    mux_bit0: entity work.mux(struct)
    port map('0', addend_temp(0), add_load, mux_out(0));
    mux_bit1: entity work.mux(struct)
    port map(q(0), addend_temp(1), add_load, mux_out(1));
    mux_bit2: entity work.mux(struct)
    port map(q(1), addend_temp(2), add_load, mux_out(2));
    mux_bit3: entity work.mux(struct)
    port map(q(2), addend_temp(3), add_load, mux_out(3));
    mux_bit4: entity work.mux(struct)
    port map(q(3), '0', add_load, mux_out(4));
    mux_bit5: entity work.mux(struct)
    port map(q(4), '0', add_load, mux_out(5));
    mux_bit6: entity work.mux(struct)
    port map(q(5), '0', add_load, mux_out(6));
    mux_bit7: entity work.mux(struct)
    port map('0', '0', add_load, mux_out(7));
    
    reg41: entity work.reg4(struct)
    port map (mux_out(0), mux_out(1), mux_out(2), mux_out(3), '1', clk_not, l_temp(0), l_temp(1), l_temp(2), l_temp(3));
    reg42: entity work.reg4(struct)
    port map (mux_out(4), mux_out(5), mux_out(6), mux_out(7), '1', clk_not, l_temp(4), l_temp(5), l_temp(6), l_temp(7));
    reg41_out: entity work.reg4(struct)
    port map (l_temp(0), l_temp(1), l_temp(2), l_temp(3), '1', clk, q(0), q(1), q(2), q(3));
    reg42_out: entity work.reg4(struct)
    port map (l_temp(4), l_temp(5), l_temp(6), l_temp(7), '1', clk, q(4), q(5), q(6), q(7));
    
    and_gate0: entity work.and2(basic)
    port map (q(0), add_control, temp(0));
    and_gate1: entity work.and2(basic)
    port map (q(1), add_control, temp(1));
    and_gate2: entity work.and2(basic)
    port map (q(2), add_control, temp(2));
    and_gate3: entity work.and2(basic)
    port map (q(3), add_control, temp(3));
    and_gate4: entity work.and2(basic)
    port map (q(4), add_control, temp(4));
    and_gate5: entity work.and2(basic)
    port map (q(5), add_control, temp(5));
    and_gate6: entity work.and2(basic)
    port map (q(6), add_control, temp(6));
    and_gate7: entity work.and2(basic)
    port map (q(7), add_control, temp(7));
    
    adder: entity work.eightbitadder(struct)
    port map (temp, augend_temp, sum_temp, open);
    
    sum <= conv_integer(sum_temp);
end architecture struct;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;

entity multiplier is
    port ( 
        clk, reset : in std_logic;
        multiplicand, multiplier : in integer;
        product : out integer 
    );
end entity multiplier;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;
entity multiplier_behavior is
    port ( 
        clk, reset : in std_logic;
        multiplicand, multiplier : in integer;
        product : out integer 
    );
end entity multiplier_behavior;


architecture behav of multiplier_behavior is
begin
product <= multiplicand * multiplier;
end architecture behav;

architecture mixed of multiplier is
    signal arith_control, result_en, mult_bit, mult_load : std_logic;
    signal partial_product, full_product : integer;
    
begin
    
    shift_reg_component: entity work.shift_reg(struct)
    port map (multiplier, mult_bit, mult_load, clk);
    
    shift_adder_component: entity work.shift_adder(struct)
    port map (multiplicand, full_product, partial_product, mult_load, mult_bit, clk);
    
    reg_component: entity work.reg(struct)
    port map (partial_product, full_product, result_en, reset, clk);
    
    control_section: process is
    variable counter: integer := 0;
    variable flag: integer := 8;
    begin
        if(reset = '1') then
            counter := 0;
            result_en <= '1';
        end if;
        if (clk = '1' and counter = 0) then
            counter := counter + 1;
            result_en <= '1';
            mult_load <= '1';
        elsif (clk = '1' and counter = 1) then
            counter := counter + 1;
            mult_load <= '0' ;--after 10ns;
        elsif (clk = '1' and counter < flag) then
            counter := counter + 1;
            mult_load <= '0';
        end if;
        if (counter = flag) then
            result_en <= '0';
        end if;
        wait on clk;
    end process control_section;
    product <= full_product;
end architecture mixed;

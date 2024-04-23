library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;

entity reg4 is

port ( d0, d1, d2, d3, en, clk : in std_logic;

q0, q1, q2, q3 : out std_logic );

end entity reg4;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;
entity d_latch is
port ( d, clk : in std_logic; q : out std_logic );
end entity d_latch;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;
entity and2 is
port ( a, b : in std_logic; y : out std_logic );
end entity and2;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;
entity not1 is
port ( a : in std_logic; y : out std_logic );
end entity not1;


architecture behav of reg4 is

begin

storage : process is
variable stored_d0, stored_d1, stored_d2, stored_d3 : std_logic;

begin
if en = '1' and clk = '1' then
stored_d0 := d0;
stored_d1 := d1;
stored_d2 := d2;
stored_d3 := d3;
end if;

q0 <= stored_d0;
q1 <= stored_d1;
q2 <= stored_d2;
q3 <= stored_d3;
wait on d0, d1, d2, d3, en, clk;

end process storage;
end architecture behav;


architecture basic of d_latch is
begin
latch_behavior : process is
begin
if clk = '1' then
q <= d;
end if;
wait on clk, d;
end process latch_behavior;
end architecture basic;

architecture basic of and2 is
begin
and2_behavior : process is
begin
y <= a and b;
wait on a, b;
end process and2_behavior;
end architecture basic;

architecture basic of not1 is
begin
not1_behavior : process is
begin
y <= not a;
wait on a;
end process not1_behavior;
end architecture basic;

architecture struct of reg4 is
signal int_clk : std_logic;
begin
bit0 : entity work.d_latch(basic)
port map ( d0, int_clk, q0 );
bit1 : entity work.d_latch(basic)
port map ( d1, int_clk, q1 );
bit2 : entity work.d_latch(basic)
port map ( d2, int_clk, q2 );
bit3 : entity work.d_latch(basic)
port map ( d3, int_clk, q3 );
gate : entity work.and2(basic)
port map ( en, clk, int_clk );
end architecture struct;

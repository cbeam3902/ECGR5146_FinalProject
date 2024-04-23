library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;

entity xor2 is 
port (a, b: in std_logic; y: out std_logic);
end entity xor2;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;

entity and2 is
port (a, b: in std_logic; y: out std_logic);
end entity and2;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;
entity or2 is
port (a, b: in std_logic; y: out std_logic);
end entity or2;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;
entity halfadder is
port (a, b: in std_logic; y, cout: out std_logic);
end entity halfadder;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;
entity fulladder is
port (a, b, cin: in std_logic; y, cout: out std_logic);
end entity fulladder;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;
entity fourbitadder is
port (a0, a1, a2, a3, b0, b1, b2, b3 : in std_logic; y0, y1, y2, y3, cout: out std_logic);
end entity fourbitadder;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;
entity eightbitadder is
port (a, b : in std_logic_vector(7 downto 0); y: out std_logic_vector(7 downto 0); cout: out std_logic);
end entity eightbitadder;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;
entity fourbitadder_behavior is 
port (a0, a1, a2, a3, b0, b1, b2, b3 : in std_logic; y0, y1, y2, y3, cout: out std_logic);
end entity fourbitadder_behavior;

architecture Behavior of fourbitadder_behavior is
begin
fourbitadder_behavior_behavior: process is
variable c0, c1, c2: std_logic;
begin
y0 <= a0 xor b0;
c0 := a0 and b0;
y1 <= a1 xor b1 xor c0;
c1 := (a1 and b1) or (c0 and (a1 xor b1));
y2 <= a2 xor b2 xor c1;
c2 := (a2 and b2) or (c1 and (a2 xor b2));
y3 <= a3 xor b3 xor c2;
cout <= (a3 and b3) or (c2 and (a3 xor b3));
wait on a0,b0,a1,b1,a2,b2,a3,b3;
end process fourbitadder_behavior_behavior;
end architecture Behavior;

architecture basic of and2 is
begin
and2_behavior : process is
begin
y <= a and b;
wait on a, b;
end process and2_behavior;
end architecture basic;

architecture basic of xor2 is
begin
xor2_behavior : process is
begin
y <= a xor b;
wait on a, b;
end process xor2_behavior;
end architecture basic;

architecture basic of or2 is
begin
or2_behavior : process is
begin
y <= a or b;
wait on a, b;
end process or2_behavior;
end architecture basic;

architecture struct of halfadder is
begin
out_bit: entity work.xor2(basic)
port map (a, b, y);
carry_bit: entity work.and2(basic)
port map (a, b, cout);
end architecture struct;

architecture struct of fulladder is
signal temp0, temp1, temp2: std_logic;
begin
xor_gate1: entity work.xor2(basic)
port map (a,b,temp0);
out_bit: entity work.xor2(basic)
port map (temp0, cin, y);
and_gate1: entity work.and2(basic)
port map (a,b,temp1);
and_gate2: entity work.and2(basic)
port map (temp0,cin,temp2);
carry_bit: entity work.or2(basic)
port map (temp1, temp2, cout); 
end architecture struct;

architecture struct of fourbitadder is
signal temp0, temp1, temp2: std_logic;
begin
ha: entity work.halfadder(struct)
port map (a0,b0,y0,temp0);
fa1: entity work.fulladder(struct)
port map (a1, b1, temp0, y1, temp1);
fa2: entity work.fulladder(struct)
port map (a2, b2, temp1, y2, temp2);
fa3: entity work.fulladder(struct)
port map (a3, b3, temp2, y3, cout);
end architecture struct;

architecture struct of eightbitadder is
signal temp: std_logic_vector(6 downto 0);
begin
ha: entity work.halfadder(struct)
port map (a(0),b(0),y(0),temp(0));
fa1: entity work.fulladder(struct)
port map (a(1), b(1), temp(0), y(1), temp(1));
fa2: entity work.fulladder(struct)
port map (a(2), b(2), temp(1), y(2), temp(2));
fa3: entity work.fulladder(struct)
port map (a(3), b(3), temp(2), y(3), temp(3));
fa4: entity work.fulladder(struct)
port map (a(4), b(4), temp(3), y(4), temp(4));
fa5: entity work.fulladder(struct)
port map (a(5), b(5), temp(4), y(5), temp(5));
fa6: entity work.fulladder(struct)
port map (a(6), b(6), temp(5), y(6), temp(6));
fa7: entity work.fulladder(struct)
port map (a(7), b(7), temp(6), y(7), cout);
end architecture struct;
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity cpu_tb is
end cpu_tb;

architecture Behavioral of cpu_tb is
component cpu_top is
    port( 
        clk : in std_logic; -- cpu clock
        clk50 : in std_logic; -- display clock (50 MHz)
        reset : in std_logic;
        an : out std_logic_vector(3 downto 0); -- Anode for 7-segment display
        CA, CB, CC, CD, CE, CF, CG : out std_logic; -- 7-segment display cathodes
        dr_out, dw_out, addr_out, pc, accu : out std_logic_vector (7 downto 0)
    );
end component;

signal clk, clk50, reset, CA, CB, CC, CD, CE, CF, CG: std_logic;
signal an : std_logic_vector(3 downto 0);
signal dr_out, dw_out, addr_out, pc, accu : std_logic_vector(7 downto 0);
begin

UUT: cpu_top port map(
    clk => clk,
    clk50 => clk50,
    reset => reset,
    an => an,
    CA => CA,
    CB => CB,
    CC => CC,
    CD => CD,
    CE => CE,
    CF => CF,
    CG => CG,
    dr_out => dr_out,
    dw_out => dw_out,
    addr_out => addr_out,
    pc => pc,
    accu => accu
);

process1: process begin
    reset <= '0';
    clk <= '0';
    wait for 5ns;
    clk <= '1';
    wait for 5 ns;
end process;

process2: process begin
    clk50 <= '0';
    wait for 250 ns;
    clk50 <= '1';
    wait for 250 ns;
end process;

end Behavioral;

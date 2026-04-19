ValveControl7Steer is the code to use with relay port
Work with arduino nano
Output are pins D4 and D5
they are always working you need to put extra switch on the relay cable or modify the code

Plug in blade control port

Edit INO for selecting deadband and propo/onoff mode

Troubleshooting sensor jitter (A3 / MCP OUT):
- If the original blade-height sensor is very old, unstable voltage from that sensor can directly cause MCP OUT jitter.
- Replacing a worn or noisy sensor often improves stability significantly.
- Before replacing, check connector corrosion, ground quality, and 5V reference stability.
- After replacement, calibrate voltage range and re-check function button behavior (D8/D10/D11).

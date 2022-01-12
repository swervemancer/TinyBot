*Bang-Bang Control*
- If input is less than the setpoint, output is set to 100%. 
- Only two states: on and off.

*Implementation*
input, output, voltage

if (output < setpoint) then: voltage = 100;
else if (output >= setpoint) then: voltage = 0;

- Note: rough pseudocode, but should be simple enough to understand
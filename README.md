# Arduino-Transmitter-for-ELRS
Arduino based RC transmitter for ELRS external TX Module

There are many amazing Expreslrs handset. But I want a simple arduino based transmitter for my DIY 2.4G TX module.

I successfully made a TX with arduino nano. 

I replaced the control board in a DEVO 7 with arduino nano and DIY 2.4G elrs module, after some wiring and codeing and it works.

Make sure to uncheck uart_reverse when upload firmware to TX module, because the board do not support reverse uart. 

Otherwise you need to use a digital revese cycle for example a NOT gate SN74LS06N.

I need more test to see if it works stable.

This is still a working process project.

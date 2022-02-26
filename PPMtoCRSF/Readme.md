PPM to ELRS conveter

Use simpleTX PCB to make a ppm to ELRS converter

Plug PPM to D2 (switch 4 in pcb) and GND

This code is highly expermental, and unstable.
Sometimes the decoded channel signal will jump 

Make sure your transmitter output ppm correctly, otherwise the TX will receive noise singal, and transmit unexpected singal.

Use this arduino library:

https://github.com/dimag0g/PPM-reader

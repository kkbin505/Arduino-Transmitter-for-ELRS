# Arduino-Transmitter-for-ELRS
Arduino based RC transmitter for ELRS external TX Module

There are many amazing Expreslrs handset. But I want a simple arduino based transmitter for my DIY 2.4G TX module.

I successfully made a TX with arduino nano. 

https://youtu.be/PrfSfnqcBgk

I replaced the control board in a DEVO 7 with arduino nano and DIY 2.4G elrs module, after some wiring and codeing and it works.

Make sure to uncheck uart_reverse when upload firmware to TX module, because the board do not support reverse uart. 

Otherwise you need to use a digital revese cycle for example a NOT gate SN74LS06N.

I need more test to see if it works stable.

This is still a working process project. 

![IMG_20211231_230305](https://user-images.githubusercontent.com/43392862/147845208-0726187d-0374-496a-80d1-303791e30d3f.jpg)
![IMG_20211228_215915](https://user-images.githubusercontent.com/43392862/147845211-86a539c5-958d-44ce-be10-5ccce836f60c.jpg)
![IMG_20211227_221222](https://user-images.githubusercontent.com/43392862/147845217-37778ccd-7a42-4e84-9291-ddd86a3ed9e5.jpg)
![IMG_20211227_213607](https://user-images.githubusercontent.com/43392862/147845218-07ae3f93-578e-45b4-b0da-48cd59fea7bb.jpg)


Some code are based on  Pawel's amazing work on SBUS generate code.

https://quadmeup.com/generate-s-bus-with-arduino-in-a-simple-way/

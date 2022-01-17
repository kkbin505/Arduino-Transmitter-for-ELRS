# Simple TX

This is an Arduino based RC transmitter TX.
Features:
- Support ExpressLRS 2.4G external TX module
- Serial communicatin （Up to 250Hz packet rate）
- 4 analog channel
- 4 AUX channel
- M7 hall Gimbal

Due to the hardware limitation of the ATMega328p, the follow functions are not supported:
* The 328p do not support inverted UART (which OpenTX use), there are 2 options:
	1. Tun  UART_INVERTED off.
	2. Adding a digital invert IC (I tested with SN74LS06N)
** Telemetray is off, 328p do not support half duplex.
*** The 328p only support 115K baud, theoretical maxmum packet rate is 250Hz (115200/256bit per second, 4ms per frame). RC data apcket is 32byte=256bit.

The uart logic voltage of ESP32 is 3.3V, my ATMega328p is 5V. However mine TX work fine, it seems ESP32 UART is 5V tolerance.

#### Tips
Default RC channel order is AETR, you can change channel order whatever you like, by changing the RC channel order.
Current gimbal calibratin method is complex (but you need to do only once):
- Enter debug mode by delete "//" before #define DEBUG
- Flash with arduino IDE
- Open serialmonitor change baud rate to 115200
- Record max and min RC channel value and write in map() function (reverse order if neede)
   example:map(Aileron_value,893,223,RC_CHANNEL_MIN,RC_CHANNEL_MAX)
- Add "//" back before #define DEBUG
- Flash with arduino IDE

There are many amazing Expreslrs handset. But I want a simple arduino based transmitter for my DIY 2.4G external TX module.

https://youtu.be/PrfSfnqcBgk

I replaced the control board in a DEVO 7 with arduino nano and DIY 2.4G Elrs module, after some wiring and codeing and it works.

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

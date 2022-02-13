
This Transmitter is inspired by cruwallero:
https://github.com/cruwaller/elrs_handset

### Upload STL File

![微信图片_20220213205906](https://user-images.githubusercontent.com/43392862/153756463-16c5e99b-b1b6-4f23-9fea-cefb4bb9df04.jpg)

### Update Log V0.9.2
Update cmd to elrs 2.0 (power setting commond of 2.0 is different from 1.0)

Re write gimbal calibation method, add center stick offset in config file


### Update Log V0.9.1
Add 2 pre settings for power and package rate:


Now you can enter setting 1 and 2 by gesutre:

Setting 1:
//250Hz 25mW

Hold AILERON left and ELEVATOR down at power up.

Setting 2:
//150Hz 100mW

Hold AILERON right and ELEVATOR up at power up.

Center stick will keep your previous setting.

# Simple TX

This is an Arduino based RC transmitter TX.
Features:
- Support ExpressLRS 2.4G external TX module
- Serial communicatin （Up to 250Hz packet rate）
- 4 analog channel
- 4 AUX channel
- M7 hall Gimbal

![微信图片_20220123195009](https://user-images.githubusercontent.com/43392862/150677087-87a9d849-5e43-41d2-8526-66564b54dd8e.jpg)

Due to the hardware limitation of the ATMega328p, the follow functions are not supported:

* The 328p do not support inverted UART (which OpenTX use), there are 2 options:
	1. Tun  UART_INVERTED off.
	2. Adding a digital invert IC (I tested with SN74LS06N)

** Telemetray is off, 328p do not support half duplex.

*** The 328p only support 115K baud, theoretical maxmum packet rate is 250Hz (115200/320bit per second, 4ms per frame). RC data apcket is 32byte=320bit(plus 1 start bit and 1 end bit).

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

### Add PCB V0.9


I replace the control board with my PCB and system.



![微信图片_20220123195005](https://user-images.githubusercontent.com/43392862/150677082-80ab29c4-f2e4-475e-a4b3-db004aeacba5.jpg)




### Below is old prototype

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

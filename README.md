# Simple TX

This is an Arduino based RC transmitter TX.

![微信图片_20220213205906](https://user-images.githubusercontent.com/43392862/153756463-16c5e99b-b1b6-4f23-9fea-cefb4bb9df04.jpg)

# Features:
- Support ExpressLRS 2.4G external TX module
- Up to 500Hz packet rate
- 4 analog channel
- 4 AUX channel

This project includes code, stl file for transmitter shell and pcb board, you can easily diy your own rc transmitter from arduino development in a very simple way.

I have tested for 6 month since the first prototype (mode from an old Devo7), it is stable for me.

 * Simple TX is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY. 

I use superslim module in the shell, but any esp32 based tx module should be working.

It was tested for ELRS 1.0 and 2.0.




### The development was moved to VS code and platformio

The old version will was saved in the v0.93 branch

New feature will be most likely happen in the main branch

video:

https://www.youtube.com/watch?v=BPLFkZUoV28

### 2022/6/3 Optimize Calibration

1. Add center stick calibration
2. Optimze channel reverse
	use 4 values to handdle reverse in config.h
	
// Define Reverse 1 = reverse, 0 = normal
	
Is_Aileron_Reverse  =1;
	
Is_Elevator_Reverse =1;
	
Is_Throttle_Reverse =0;
	
Is_Rudder_Reverse   =0;

### 2022/5/29 Calibration added

Calibration function added by erstec in Platformio_test_branch, thanks

1. enter debug mode by uncomment #define DEBUG
2. flash firmware to arduino nano
4. open serial monitor at 115200 baud, center stick and keep for 5 seconds
5. move stick full range within 20 seconds untill you see "Calibration Done"
6. comment //#define DEBUG and flash firmware to arduino nano
7. test in bf configurator
8. done

### 2022/4/6 Test branch

platformio


### Update Log 2022/4/3

Add Test branch for ppm output


### Update Log 2022/3/26

Update STL File：

Solve gimbal & switch interference

Add battery tray

![ASSEMBLE](https://user-images.githubusercontent.com/43392862/160218062-a254729a-f0f7-474e-8428-6ba044f421b5.jpg)



### Update Log 2022/3/20



Update PPM to ELRS library, siginal is more stable, still need more test


https://github.com/kkbin505/Arduino-Transmitter-for-ELRS/tree/main/PPMtoCRSF



### Update Log V0.9.3
Increase baud rate to 400K

*** In 400k baud, theoretical maxmum packet rate is 500Hz (400000/320bit per second, 1.6ms per frame). RC data pcket is 32byte=320bit(plus 1 start bit and 1 end bit).

Now simpleTX is supporting 500Hz

Add 3rd pre seeting for 500Hz 25mW

Gestrue:
Hold AILERON left and ELEVATOR up at power up.

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


Due to the hardware limitation of the ATMega328p, the follow functions are not supported:

* The 328p do not support inverted UART (which OpenTX use), there are 2 options:
	1. Tun  UART_INVERTED off.
	2. Adding a digital invert IC (I tested with SN74LS06N)

** Telemetray is off, 328p do not support half duplex.



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


This Transmitter is inspired by cruwallero:
https://github.com/cruwaller/elrs_handset

Some code are based on  Pawel's amazing work on SBUS generate code.

https://quadmeup.com/generate-s-bus-with-arduino-in-a-simple-way/

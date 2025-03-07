# Simple TX

现已增加中文介绍，详情请看Wiki页面

This is an Arduino based RC transmitter TX. I want to design a RC transmitter (soft and hardware) that is simple to build, and use arduino board wich I am familler with, and it should supoort ELRS.

![微信图片_20220213205906](https://user-images.githubusercontent.com/43392862/153756463-16c5e99b-b1b6-4f23-9fea-cefb4bb9df04.jpg)


video:

https://www.youtube.com/watch?v=BPLFkZUoV28

# Features:
- Support ExpressLRS 2.4G external TX module
- Up to 500Hz packet rate
- 4 analog channel
- 4 AUX channel

This project includes code, stl file for transmitter shell and pcb board, you can easily diy your own rc transmitter from arduino development board in a very simple way.

I have tested for 6 month since the first prototype (mod from an old Devo7), it works pretty well.

 * Simple TX is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY. 

I use superslim module in the shell, but any esp32 based tx module should be working.

It was tested for ELRS 1.0 and 2.0.

### Update:

### 2025/03/07 Update STL file to V2

1. Redesign battery bay

2. Add 2 additional switch postion, now support up to 10 channels

3. Add buzzer hold

This version is slight thicker then V1, because I enlarged the battery tray to support bigger batteries. And I redesigned the battery tray cover, now it is clip on.

![SimpleTX_V2](https://github.com/user-attachments/assets/0ff9ad77-3488-408c-a4b3-d2a0daaffb8a)

![SimpleTX_V2_Battery](https://github.com/user-attachments/assets/8c68900a-a139-49c6-9f36-e9c2819744a5)


![battery_cover](https://github.com/user-attachments/assets/800b2b06-b9ff-4771-8fd6-f5cfa7a5a56c)



### 2025/03/04 Release ELRS 3.0 support

Merge 3.0 support to main

Fix buzzer issue

Change channel 6 to 3 postion AUX



### 2025/2/12 Add support for ELRS 3.0
Thanks to the great work doing by dbloemhard now the SimpleTX support 3.0, I merged the update into the main branch.
You can change pocket rate, power and tune on/off dynamic power by gesutre.
His repo:
https://github.com/dbloemhard/Arduino-Transmitter-for-ELRS3.x


### 2022/11/5 Move IDE to VSCODE and Arduino Extension

Add buzzer support on pin 6

The PIO becomes unusable dude to unstalbe network, so I switch back to arduino extension, still in VSCODE, because the original Arduino IDE is low efficient.

The code still works in Arduino IDE, you can compile in both enviroment

The PIO will be saved in a seprate branch and will unliked be updated

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


### 1. Project Intro:

### 1.1 Hardware (PCB)

In PCB folder, you can find the PCB and BOM, the PCB is designed in Easyeda and files in Gerber type. I orderd my PCB use the Gerber file from JLCpcb.

![EasyEDA(标准版) - 免费、易用、强大的在线电路设计软件 - Google Chrome 2022_1_23 19_47_16](https://user-images.githubusercontent.com/43392862/150677345-1feb8e57-7f06-45fc-b877-a0eb61f51f26.png)

The PCB is v0.9 means it is not complete, but it works well at least for me. 

The version includes 4 digital switch, but I forget to wire out the LED pint, so I use one channel for LED only 3 digital available. 

The analogy channel support 3.3v gimbal (hall and poentional meter both works), the pin use 1.25 6p plug, it is the same pin order as most frsky gimbal (M7 for example). If you use orther gimbal pay attention to the pin order. I find jumper gimbal reversed GND(red) and 3.3V(black).

I powered my board with 2S battery, the arduino nano have a decent LDO （ASM1117）, and the ELRS module also powerd from 2S.

### 1.2 Firmware

All source codes are in SimpleTX folder.

It is writen in arduino frame use VScode, but arduino IDE should also work.

### 1.3 CAD-Model

3 versions of transmitter shell are available.

1 is a remix from https://github.com/cruwaller/elrs_handset

![DSC00981](https://user-images.githubusercontent.com/43392862/206710142-ffb84e65-c567-4101-8fe2-d68fe21cae29.JPG)

I removed 2 switch, and modify switch in front side from Switches to Tango2 like switch

![DSC00982](https://user-images.githubusercontent.com/43392862/206710187-ea65c351-b675-4ccc-97e1-455140d3e338.JPG)

Front_Shell_REV-A_M7_kkbin_modify.stl

2 is my design, it is compatibal with jumper TX 12 hall gimbal, very compact.

![DSC00988](https://user-images.githubusercontent.com/43392862/206710049-68ed9d0f-3812-43b5-ade6-776c0b12b786.JPG)


SimpleTX_Top_v15.stl

3 is for X9 lite gimbal, it feels too big for me.


!!! Do not solder the buzzer, need to add triode in next version, I burned my board.
![buzzer](https://user-images.githubusercontent.com/43392862/202906574-64efb654-1023-4be7-9c3d-3a5a70b9f1aa.jpg)


This Transmitter is inspired by cruwallero:
https://github.com/cruwaller/elrs_handset

Some code are based on  Pawel's amazing work on SBUS generate code.

https://quadmeup.com/generate-s-bus-with-arduino-in-a-simple-way/

Add PCB V0.9

BOM :
1. Frsky M7 Gimbal *2
2. Arduino Nano
3. XH2.54 3P *6
4. PH2.0 2P
5. 1.25 6P
6. 2 Position Switch *5
7. LED *1
8. 100 ohm resister 
9. 10K 0805 resister
10. 20K 0805 resister
11. UFL to SMA connector
12. 2.4G SMA antenna
13. ExpressLRS 2.4G TX module
14. 2S battery
15. PCB https://oshwlab.com/kkbinmail/arduino_transmitter

Build order:

1. Print shell from:https://github.com/cruwaller/elrs_handset/tree/master/revision_A/shell
2. Order PCB from JLB PCB
3. Sold 10K and 20K resiter to PCB
4. Sold 1.25 6P male header
5. Sold Arduino Nano
6. Sold XH2.45 (Double check the pole of the battery header, make sure ground is to ground)
7. Sold PH2.0
8. Down load firmware
9. Build and Flash to Arduino Nano with IDE
10. Flash TX module use ExpressLRS un-check UART_INVERTOR & OpenTX_SYNC
11. Assemble switch to shell
12. Assemble M7 Gimbal
13. Assemble SMA connector
14. Connect everything



<img width="526" alt="SCHEAM" src="https://user-images.githubusercontent.com/43392862/151694978-880b1bac-8fd5-4ba7-b7fa-85947317776f.png">

![EasyEDA(标准版) - 免费、易用、强大的在线电路设计软件 - Google Chrome 2022_1_23 19_47_16](https://user-images.githubusercontent.com/43392862/150677345-1feb8e57-7f06-45fc-b877-a0eb61f51f26.png)

![微信图片_20220123194957](https://user-images.githubusercontent.com/43392862/150677353-b007ecf4-92b2-4e32-adc0-9507a2886bab.jpg)

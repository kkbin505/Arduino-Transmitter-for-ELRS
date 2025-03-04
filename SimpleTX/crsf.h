#include <Arduino.h>
#include <stdint.h>

/*
 * This file is part of Simple TX
 *
 * Simple TX is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Simple TX is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

// Basic setup
#define CRSF_MAX_CHANNEL        16
#define CRSF_FRAME_SIZE_MAX     64
// Device address & type
#define RADIO_ADDRESS           0xEA
// #define ADDR_MODULE             0xEE  //  Crossfire transmitter
#define TYPE_CHANNELS           0x16

// Define AUX channel input limite
#define CRSF_DIGITAL_CHANNEL_MIN 172
#define CRSF_DIGITAL_CHANNEL_MAX 1811
#define CRSF_DIGITAL_CHANNEL_MID 992

// internal crsf variables
#define CRSF_TIME_NEEDED_PER_FRAME_US   1100 // 700 ms + 400 ms for potential ad-hoc request
//#define SERIAL_BAUDRATE                 115200 //low baud for Arduino Nano , the TX module will auto detect baud. 115200/400000
//#define CRSF_TIME_BETWEEN_FRAMES_US     4000 // 4 ms 250Hz
#define SERIAL_BAUDRATE                 400000
#define CRSF_TIME_BETWEEN_FRAMES_US     1666 // 1.6 ms up to 500Hz
#define CRSF_PAYLOAD_OFFSET             offsetof(crsfFrameDef_t, type)
#define CRSF_MSP_RX_BUF_SIZE            128
#define CRSF_MSP_TX_BUF_SIZE            128
#define CRSF_PAYLOAD_SIZE_MAX           60
#define CRSF_PACKET_LENGTH              22
#define CRSF_PACKET_SIZE                26
#define CRSF_FRAME_LENGTH               24 // length of type + payload + crc
#define CRSF_CMD_PACKET_SIZE            8

// ELRS command
#define ELRS_ADDRESS                    0xEE
#define ELRS_PKT_RATE_COMMAND           0x01
#define ELRS_TLM_RATIO_COMMAND          0x02
#define ELRS_SWITCH_MODE_COMMAND        0x03
#define ELRS_MODEL_MATCH_COMMAND        0x04
#define ELRS_POWER_COMMAND              0x06
#define ELRS_DYNAMIC_POWER_COMMAND      0x07
#define ELRS_BLE_JOYSTIC_COMMAND        0x11
#define ELRS_WIFI_COMMAND               0x0F
#define ELRS_BIND_COMMAND               0x11
#define ELRS_START_COMMAND              0x04
#define TYPE_SETTINGS_WRITE             0x2D
#define ADDR_RADIO                      0xEA //  Radio Transmitter
#define port                            Serial

class CRSF {
public:
    void begin(void);
    void crsfPrepareDataPacket(uint8_t packet[], int16_t channels[]);
    void crsfPrepareCmdPacket(uint8_t packetCmd[], uint8_t command, uint8_t value);
    void CrsfWritePacket(uint8_t packet[], uint8_t packetLength);
};

/* ESP32 Team900
https://github.com/danxdz/simpleTx_esp32/blob/master/src/Simple_TX.cpp

// buildElrsPacket(crsfCmdPacket,X,3);
// 0 : ELRS status request => ??
// 1 : Set Lua [Packet Rate]= 0 - 25Hz / 1 - 50Hz / 2 - 100Hz / 3 - 200Hz
// 2 : Set Lua [Telem Ratio]= 0 - off / 1 - 1:128 / 2 - 1:64 / 3 - 1:32 / 4 - 1:16 / 5 - 1:8 / 6 - 1:4 / 7 - 1:2
// 3 : Set Lua [Switch Mode]=0 -> Hybrid;Wide
// 4 : Set Lua [Model Match]=0 -> Off;On
// 5 : Set Lua [TX Power]=0 - 10mW / 1 - 25mW
// 6 : Set Lua [Max Power]=0 - 10mW / 1 - 25mW *dont force to change, but change after reboot if last power was greater
// 7 : Set Lua [Dynamic]=0 -> Off;On;AUX9;AUX10;AUX11;AUX12 -> * @ ttgo screen
// 8 : Set Lua [VTX Administrator]=0
// 9 : Set Lua [Band]=0 -> Off;A;B;E;F;R;L
// 10: Set Lua [Channel]=0 -> 1;2;3;4;5;6;7;8
// 11: Set Lua [Pwr Lvl]=0 -> -;1;2;3;4;5;6;7;8
// 12: Set Lua [Pitmode]=0 -> Off;On
// 13: Set Lua [Send VTx]=0 sending response for [Send VTx] chunk=0 step=2
// 14: Set Lua [WiFi Connectivity]=0
// 15: Set Lua [Enable WiFi]=0 sending response for [Enable WiFi] chunk=0 step=0
// 16: Set Lua [Enable Rx WiFi]=0 sending response for [Enable Rx WiFi] chunk=0 step=2
// 17: Set Lua [BLE Joystick]=0 sending response for [BLE Joystick] chunk=0 step=0
//     Set Lua [BLE Joystick]=1 sending response for [BLE Joystick] chunk=0 step=3
//     Set Lua [BLE Joystick]=2 sending response for [BLE Joystick] chunk=0 step=3
//     Set Lua [BLE Joystick]=3 sending response for [BLE Joystick] chunk=0 step=3
//     Set Lua [BLE Joystick]=4 to enable
//                hwTimer stop
//                Set Lua [TX Power]=0
//                hwTimer interval: 5000
//                Adjusted max packet size 22-22
//                Starting BLE Joystick!
// 19: Set Lua [Bad/Good]=0
// 20: Set Lua [2.1.0 EU868]=0 =1 ?? get

0:Packet Rate:0:9:0:4:4;25Hz(-123dBm):50Hz(-120dBm):100Hz(-117dBm):100Hz Full(-112dBm):200Hz(-112dBm): :: OPT
1:Telem Ratio:0:9:0:9:0;Std:Off:1:128:1:64:1:32:1:16:1:8:1:4:1:2:Race: :: OPT
2:Switch Mode:0:9:0:1:0;Wide:Hybrid: :: OPT
3:Model Match:0:9:0:1:0;Off:On: :: OPT
4:TX Power (50mW):0:11:0:0:0 :: MainMenuItem 
5:Max Power:5:9:0:2:2;10:25:50: :: OPT
6:Dynamic:5:9:0:5:0;Off:Dyn:AUX9:AUX10:AUX11:AUX12: :: OPT
7:VTX Administrator:0:11:0:0:0 :: MainMenuItem
8:Band:8:9:0:6:0;Off:A:B:E:F:R:L: :: OPT
9:Channel:8:9:0:7:0;1:2:3:4:5:6:7:8: :: OPT
10:Pwr Lvl:8:9:0:8:0;-:1:2:3:4:5:6:7:8: :: OPT
11:Pitmode:8:9:0:8:79;Off:On:AUX1�:AUX1�:AUX2�:AUX2�:AUX3�:AUX3�:AUX  Pitmode: :: OPT
12:Send VTx:8:13:0:0:0 :: CMD
13:WiFi Connectivity:0:11:0:0:0 :: MainMenuItem
14:Enable WiFi:14:13:0:0:0 :: CMD
15:Enable Rx WiFi:14:13:0:0:0 :: CMD
16:BLE Joystick:0:13:0:0:0 :: CMD
17:Bind:0:13:0:0:0 :: CMD
18:Bad/Good:0:12:0:0:0 :: INFO
19:3.1.2 EU868:0:12:0:0:0 :: INFO

as menu:

Main Menu
|- Packet Rate (OPT)
   |- 25Hz(-123dBm)
   |- 50Hz(-120dBm)
   |- 100Hz(-117dBm)
   |- 100Hz Full(-112dBm)
   |- 200Hz(-112dBm)
|- Telem Ratio (OPT)
   |- Std
   |- Off
   |- 1:128
   |- 1:64
   |- 1:32
   |- 1:16
   |- 1:8
   |- 1:4
   |- 1:2
   |- Race
|- Switch Mode (OPT)
   |- Wide
   |- Hybrid
|- Model Match (OPT)
   |- Off
   |- On
|- TX Power (50mW)
|  |- Max Power (OPT)
   |  |- 10
   |  |- 25
   |  |- 50
   |- Dynamic (OPT)
   |  |- Off
   |  |- Dyn
   |  |- AUX9
   |  |- AUX10
   |  |- AUX11
   |  |- AUX12
|- VTX Administrator
|  |- Band (OPT)
   |  |- Off
   |  |- A
   |  |- B
   |  |- E
   |  |- F
   |  |- R
   |  |- L
   |- Channel (OPT)
   |  |- 1
   |  |- 2
   |  |- 3
   |  |- 4
   |  |- 5
   |  |- 6
   |  |- 7
   |  |- 8
   |- Pwr Lvl (OPT)
   |  |- -
   |  |- 1
   |  |- 2
   |  |- 3
   |  |- 4
   |  |- 5
   |  |- 6
   |  |- 7
   |  |- 8
   |- Pitmode (OPT)
      |- Off
      |- On
|- Send VTx
|- WiFi Connectivity
|  |- Enable WiFi
   |- Enable Rx WiFi
|- BLE Joystick
|- Bind
|- Bad/Good
|- 3.1.2 EU868



*/

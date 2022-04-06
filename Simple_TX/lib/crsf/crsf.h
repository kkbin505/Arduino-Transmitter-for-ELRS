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
#define CRSF_MAX_CHANNEL 16
#define CRSF_FRAME_SIZE_MAX 64
 // Device address & type
#define RADIO_ADDRESS                  0xEA
//#define ADDR_MODULE                    0xEE  //  Crossfire transmitter
#define TYPE_CHANNELS                  0x16


//Define AUX channel input limite
#define CRSF_DIGITAL_CHANNEL_MIN 172
#define CRSF_DIGITAL_CHANNEL_MAX 1811

// internal crsf variables
#define CRSF_TIME_NEEDED_PER_FRAME_US   1100 // 700 ms + 400 ms for potential ad-hoc request
//#define SERIAL_BAUDRATE 115200 //low baud for Arduino Nano , the TX module will auto detect baud. 115200/400000
//#define CRSF_TIME_BETWEEN_FRAMES_US     4000 // 4 ms 250Hz
#define SERIAL_BAUDRATE 400000 
#define CRSF_TIME_BETWEEN_FRAMES_US     1666 // 1.6 ms 500Hz
#define CRSF_PAYLOAD_OFFSET offsetof(crsfFrameDef_t, type)
#define CRSF_MSP_RX_BUF_SIZE 128
#define CRSF_MSP_TX_BUF_SIZE 128
#define CRSF_PAYLOAD_SIZE_MAX   60
#define CRSF_PACKET_LENGTH 22
#define CRSF_PACKET_SIZE  26
#define CRSF_FRAME_LENGTH 24;   // length of type + payload + crc
#define CRSF_CMD_PACKET_SIZE  8

// ELRS command
#define ELRS_ADDRESS                   0xEE
#define ELRS_BIND_COMMAND              0xFF
#define ELRS_WIFI_COMMAND              0xFE
#define ELRS_PKT_RATE_COMMAND          0x01
#define ELRS_TLM_RATIO_COMMAND         0x02
#define ELRS_SWITCH_MODE_COMMAND       0x03
#define ELRS_MODEL_MATCH_COMMAND       0x04
#define ELRS_POWER_COMMAND             0x06
#define ELRS_BLE_JOYSTIC_COMMAND       17
#define TYPE_SETTINGS_WRITE            0x2D
#define ADDR_RADIO                     0xEA  //  Radio Transmitter
#define port Serial

class CRSF
{
        public:
                void begin(void);
                void crsfPrepareDataPacket(uint8_t packet[], int channels[]);
                void crsfPrepareCmdPacket(uint8_t packetCmd[],uint8_t command, uint8_t value);
                void CrsfWritePacket(uint8_t packet[],uint8_t packetLength);




};


/* ESP32 Team900
https://github.com/danxdz/simpleTx_esp32/blob/master/src/Simple_TX.ino
        // buildElrsPacket(crsfCmdPacket,X,3);
        // 0 : ELRS status request => ??
        // 1 : Set Lua [Packet Rate]= 0 - 50Hz / 1 - 150Hz / 3 - 250Hz
        // 2 : Set Lua [Telem Ratio]= 0 - off / 1 - 1:128 / 2 - 1:64 / 3 - 1:32 / 4 - 1:16 / 5 - 1:8 / 6 - 1:4 / 7 - 1:2
        // 3 : Set Lua [Switch Mode]=0 -> Hybrid;Wide 
        // 4 : Set Lua [Model Match]=0 -> Off;On
        // 5 : Set Lua [TX Power]=0 - 10mW / 1 - 25mW / 2 - 50mW /3 - 100mW/4 - 250mW
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
        // 19: Set Lua [Bad/Good]=0
        // 20: Set Lua [2.1.0 EU868]=0 =1 ?? get 
*/

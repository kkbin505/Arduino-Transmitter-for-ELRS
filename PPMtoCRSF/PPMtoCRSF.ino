/* 
-------------------------------------------------------------------------
 Simple Arduino trasmisster (PPM to ELRS)
 Decode PPM to CRSF protocol
 Arduino Nano
 ELRS 2.4G TX moduel
 Custom PCB from JLCPCB
 https://github.com/kkbin505/Arduino-Transmitter-for-ELRS
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

//Use this lib https://github.com/DzikuVx/PPMReader
#include "PPMReader.h"
//Setup ppm read on pin 2, 

PPMReader ppmReader(2, 0, false);

byte channelAmount = 8;

//default channel order is  AETR
#define TAER

#define RADIO_ADDRESS                  0xEA
#define ADDR_MODULE                    0xEE  //  Crossfire transmitter
#define TYPE_CHANNELS                  0x16

// internal crsf variables
#define CRSF_CHANNEL_MIN 172
#define CRSF_CHANNEL_MID 991
#define CRSF_CHANNEL_MAX 1811
#define CRSF_TIME_NEEDED_PER_FRAME_US   1100 // 700 ms + 400 ms for potential ad-hoc request
#define CRSF_TIME_BETWEEN_FRAMES_US     4000 // 4 ms 250Hz
#define CRSF_PAYLOAD_OFFSET offsetof(crsfFrameDef_t, type)
#define CRSF_MAX_CHANNEL 16
#define CRSF_FRAME_SIZE_MAX 64
#define SERIAL_BAUDRATE 400000
#define CRSF_MSP_RX_BUF_SIZE 128
#define CRSF_MSP_TX_BUF_SIZE 128
#define CRSF_PAYLOAD_SIZE_MAX   60
#define CRSF_PACKET_LENGTH 22
#define CRSF_PACKET_SIZE  26
#define CRSF_FRAME_LENGTH 24;   // length of type + payload + crc

uint8_t crsfPacket[CRSF_PACKET_SIZE];
int rcChannels[CRSF_MAX_CHANNEL];
uint32_t crsfTime = 0;

#ifdef TAER
enum chan_order{
    THROTTLE, 
    AILERON,
    ELEVATOR,
    RUDDER,
    AUX1,  // (CH5)  ARM switch for Expresslrs
    AUX2,  // (CH6)  angel / airmode change
    AUX3,  // (CH7)  flip after crash
    AUX4,  // (CH8) 
    AUX5,  // (CH9) 
    AUX6,  // (CH10) 
    AUX7,  // (CH11)
    AUX8,  // (CH12)
};
#else
enum chan_order{
    AILERON,
    ELEVATOR,
    THROTTLE, 
    RUDDER,
    AUX1,  // (CH5)  ARM switch for Expresslrs
    AUX2,  // (CH6)  angel / airmode change
    AUX3,  // (CH7)  flip after crash
    AUX4,  // (CH8) 
    AUX5,  // (CH9) 
    AUX6,  // (CH10) 
    AUX7,  // (CH11)
    AUX8,  // (CH12)
};
#endif

void setup()
{
  /*
  TCCR1A = 0;  //reset timer1
  TCCR1B = 0;
  TCCR1B |= (1 << CS11);  //set timer1 to increment every 0,5 us or 1us on 8MHz
  */
  for (uint8_t i = 0; i < CRSF_MAX_CHANNEL; i++) {
        rcChannels[i] = CRSF_CHANNEL_MID;
    }
    rcChannels[THROTTLE] = CRSF_CHANNEL_MIN; // Throttle
 
    delay(1000);
    Serial.begin(SERIAL_BAUDRATE);
  
    //Serial.begin(115200);
    //Serial.println("ready");
}

void loop()
{
  uint32_t currentMicros = micros();

 if(ppmReader.isReceiving()==true){
  int count;
  while(ppmReader.get(count) != 0){  //print out the servo values
     int value = ppmReader.get(count);;
     rcChannels[count]=map(value,1000,2000,CRSF_CHANNEL_MIN,CRSF_CHANNEL_MAX);
    //Serial.print(ppmReader.get(count));
    //Serial.print("  ");
    count++;
  }
 }
  //set fail safe value
else{
    rcChannels[THROTTLE] = CRSF_CHANNEL_MIN; // 
    rcChannels[AILERON] = CRSF_CHANNEL_MID; // 
    rcChannels[ELEVATOR] = CRSF_CHANNEL_MID; // 
    rcChannels[RUDDER] = CRSF_CHANNEL_MID; // 
    rcChannels[AUX1] = CRSF_CHANNEL_MIN; // ARM Low
  }
  
    if (currentMicros > crsfTime) {
        crsfPreparePacket(crsfPacket, rcChannels);
        Serial.write(crsfPacket, CRSF_PACKET_SIZE);            
        crsfTime = currentMicros + CRSF_TIME_BETWEEN_FRAMES_US;
    }

}

// crc implementation from CRSF protocol document rev7
static u8 crsf_crc8tab[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9};

u8 crsf_crc8(const u8 *ptr, u8 len) {
    u8 crc = 0;
    for (u8 i=0; i < len; i++) {
        crc = crsf_crc8tab[crc ^ *ptr++];
    }
    return crc;
}

void crsfPreparePacket(uint8_t packet[], int channels[]){

    static int output[CRSF_MAX_CHANNEL] = {0};
    const uint8_t crc = crsf_crc8(&packet[2], CRSF_PACKET_SIZE-3);
    /*
     * Map 1000-2000 with middle at 1500 chanel values to
     * 173-1811 with middle at 992 S.BUS protocol requires
     */
    for (uint8_t i = 0; i < CRSF_MAX_CHANNEL; i++) {
        output[i] = channels[i];
    }    
   /*    
       Serial.print("crsf  "); 
       Serial.print(output[0]); 
       Serial.print("  i"); 
       Serial.print(output[1]); 
       Serial.print("  i"); 
       Serial.print(output[2]); 
       Serial.print("  i"); 
       Serial.print(output[3]); 
       Serial.println(); */

    // packet[0] = UART_SYNC; //Header
    packet[0] = ADDR_MODULE; //Header
    packet[1] = 24;   // length of type (24) + payload + crc
    packet[2] = TYPE_CHANNELS;
    packet[3] = (uint8_t) (channels[0] & 0x07FF);
    packet[4] = (uint8_t) ((channels[0] & 0x07FF)>>8 | (channels[1] & 0x07FF)<<3);
    packet[5] = (uint8_t) ((channels[1] & 0x07FF)>>5 | (channels[2] & 0x07FF)<<6);
    packet[6] = (uint8_t) ((channels[2] & 0x07FF)>>2);
    packet[7] = (uint8_t) ((channels[2] & 0x07FF)>>10 | (channels[3] & 0x07FF)<<1);
    packet[8] = (uint8_t) ((channels[3] & 0x07FF)>>7 | (channels[4] & 0x07FF)<<4);
    packet[9] = (uint8_t) ((channels[4] & 0x07FF)>>4 | (channels[5] & 0x07FF)<<7);
    packet[10] = (uint8_t) ((channels[5] & 0x07FF)>>1);
    packet[11] = (uint8_t) ((channels[5] & 0x07FF)>>9 | (channels[6] & 0x07FF)<<2);
    packet[12] = (uint8_t) ((channels[6] & 0x07FF)>>6 | (channels[7] & 0x07FF)<<5);
    packet[13] = (uint8_t) ((channels[7] & 0x07FF)>>3);
    packet[14] = (uint8_t) ((channels[8] & 0x07FF));
    packet[15] = (uint8_t) ((channels[8] & 0x07FF)>>8 | (channels[9] & 0x07FF)<<3);
    packet[16] = (uint8_t) ((channels[9] & 0x07FF)>>5 | (channels[10] & 0x07FF)<<6);  
    packet[17] = (uint8_t) ((channels[10] & 0x07FF)>>2);
    packet[18] = (uint8_t) ((channels[10] & 0x07FF)>>10 | (channels[11] & 0x07FF)<<1);
    packet[19] = (uint8_t) ((channels[11] & 0x07FF)>>7 | (channels[12] & 0x07FF)<<4);
    packet[20] = (uint8_t) ((channels[12] & 0x07FF)>>4  | (channels[13] & 0x07FF)<<7);
    packet[21] = (uint8_t) ((channels[13] & 0x07FF)>>1);
    packet[22] = (uint8_t) ((channels[13] & 0x07FF)>>9  | (channels[14] & 0x07FF)<<2);
    packet[23] = (uint8_t) ((channels[14] & 0x07FF)>>6  | (channels[15] & 0x07FF)<<5);
    packet[24] = (uint8_t) ((channels[15] & 0x07FF)>>3);
    
    packet[25] = crsf_crc8(&packet[2], CRSF_PACKET_SIZE-3); //CRC


}

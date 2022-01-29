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

/* 
 =======================================================================================================
 Simple TX CONFIG OPTIONS (comment out unneeded options)
 =======================================================================================================
 */

 // Define RC input Offset
int Aileron_OFFSET = 0;        // values read from the pot 
int Elevator_OFFSET  = 0; 
int Throttle_OFFSET =0;
int Rudder_OFFSET  = 0; 

//IO setup
//pins that used for the Joystick
const int analogInPinAileron = A4;
const int analogInPinElevator = A3; 
const int analogInPinThrottle = A2;
const int analogInPinRudder = A1; 
const int VOLTAGE_READ_PIN = A0; 

//pins that used for the switch
const int DIGITAL_PIN_SWITCH_ARM = 4;  // Arm switch
const int DIGITAL_PIN_SWITCH_AUX2 = 3;  // 
const int DIGITAL_PIN_SWITCH_AUX3 = 5;  // 
const int DIGITAL_PIN_SWITCH_AUX4 = 6;  // 

//pins that used for output
const int DIGITAL_PIN_LED = 5;  // 
const int DIGITAL_PIN_BUZZER = 7;  // 

//----- Voltage monitoring -------------------------
#define VOLTAGE_READS 10 //get average of VOLTAGE_READS readings

 // Define battery warning voltage
 const float WARNING_VOLTAGE=7.4; //2S Lipo

 // Define Commond for start Up Setting
 #define RC_MIN_COMMAND  480
 #define RC_MAX_COMMAND  1500

// from https://github.com/DeviationTX/deviation/pull/1009/ ELRS menu implement in deviation TX
/*static uint8_t  currentPktRate =1; //  "250Hz", "150Hz", "50Hz"
  //                                      1        3       5      
static uint8_t  currentPower =1 ;//  "10mW", "25mW", "50mW", "100mW", "250mW"
  //                                   0     1         2        3        4   
static uint8_t currentTlmRatio =0 ;
static uint8_t currentBind = 0;
static uint8_t currentWiFi = 0;
static uint8_t getParamsCounter = 0;
static uint8_t currentFrequency = 6; //2.4G
*/
//2 Default Settings
#define SETTING_1_PktRate 1  //250Hz
#define SETTING_1_Power 2    //50mW

#define SETTING_2_PktRate 3  //150Hz
#define SETTING_2_Power 3    //100wW

enum chan_order{
    AILERON,
    ELEVATOR,
    THROTTLE, 
    RUDDER,
    AUX1,  // (CH5)  led light, or 3 pos. rate on CX-10, H7, or inverted flight on H101
    AUX2,  // (CH6)  flip control
    AUX3,  // (CH7)  still camera (snapshot)
    AUX4,  // (CH8)  video camera
    AUX5,  // (CH9)  headless
    AUX6,  // (CH10) calibrate Y (V2x2), pitch trim (H7), RTH (Bayang, H20), 360deg flip mode (H8-3D, H22)
    AUX7,  // (CH11) calibrate X (V2x2), roll trim (H7), emergency stop (Bayang, Silverware)
    AUX8,  // (CH12) Reset / Rebind
};

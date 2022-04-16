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

//#define USE_M7

/*
 =======================================================================================================
 Simple TX CONFIG OPTIONS (comment out unneeded options)
 =======================================================================================================
 */
// Define RC input limite
/*
#define RC_CHANNEL_MIN 172
#define RC_CHANNEL_MID 991
#define RC_CHANNEL_MAX 1811
*/

#define ANALOG_CUTOFF 150 // cut off lower and upper end to avoid un-symmetric joystick range in trade off resolution

// Define RC input Offset
#ifdef USE_M7
// M7
int Aileron_OFFSET    = -46;
int Elevator_OFFSET   = 57;
int Throttle_OFFSET   = 0;
int Rudder_OFFSET     = -18;
#else
// stat war
int Aileron_OFFSET    = -8;
int Elevator_OFFSET   = 8;
int Throttle_OFFSET   = 0;
int Rudder_OFFSET     = 5;
#endif

// IO setup
// pins that used for the Joystick
const int analogInPinAileron = A4;
const int analogInPinElevator = A3;
const int analogInPinThrottle = A2;
const int analogInPinRudder = A1;
const int VOLTAGE_READ_PIN = A0;

// pins that used for the switch
const int DIGITAL_PIN_SWITCH_ARM = 4;  // Arm switch
const int DIGITAL_PIN_SWITCH_AUX2 = 3; //
// const int DIGITAL_PIN_SWITCH_AUX3 = 2;  //
// const int DIGITAL_PIN_SWITCH_AUX4 = 5;  //

// pins that used for output
const int DIGITAL_PIN_LED = 5;    // in pcb v0.9 led is reused from AUX4 (remember to add 300om resistor in led)
const int DIGITAL_PIN_BUZZER = 7; // do not use in pcb v0.9

//----- Voltage monitoring -------------------------
// Define battery warning voltage
const float WARNING_VOLTAGE = 7.4; // 2S Lipo

// Define Commond for start Up Setting
#define RC_MIN_COMMAND 600
#define RC_MAX_COMMAND 1400

// from https://github.com/DeviationTX/deviation/pull/1009/ ELRS menu implement in deviation TX
/*static uint8_t  currentPktRate =1; //  "250Hz", "150Hz", "50Hz"
  //                                       1        3       5
static uint8_t  currentPower =1 ;//  "10mW", "25mW", "50mW", "100mW", "250mW"
  //                                   0     1         2        3        4
*/
// ELRS 2.0:
//  1 : Set Lua [Packet Rate]= 0 - 50Hz / 1 - 150Hz / 3 - 250Hz
//  2 : Set Lua [Telem Ratio]= 0 - off / 1 - 1:128 / 2 - 1:64 / 3 - 1:32 / 4 - 1:16 / 5 - 1:8 / 6 - 1:4 / 7 - 1:2
//  3 : Set Lua [Switch Mode]=0 -> Hybrid;Wide
//  4 : Set Lua [Model Match]=0 -> Off;On
//  5 : Set Lua [TX Power]=0 - 10mW / 1 - 25mW / 2 - 50mW /3 - 100mW/4 - 250mW
// 3 Default Settings
#define SETTING_1_PktRate 2 // 250Hz
#define SETTING_1_Power 1   // 25mW

#define SETTING_2_PktRate 1 // 150Hz
#define SETTING_2_Power 3   // 100mW

#define SETTING_3_PktRate 3 // 500Hz
#define SETTING_3_Power 1   // 25mW

enum chan_order
{
    AILERON,
    ELEVATOR,
    THROTTLE,
    RUDDER,
    AUX1, // (CH5)  ARM switch for Expresslrs
    AUX2, // (CH6)  angel / airmode change
    AUX3, // (CH7)  flip after crash
    AUX4, // (CH8)
    AUX5, // (CH9)
    AUX6, // (CH10)
    AUX7, // (CH11)
    AUX8, // (CH12)
};

#define PPMOUTPUT
//-------------------- PPM Config-------------------------
#define CHANNEL_NUMBER 12          // set the number of chanels
#define CHANNEL_DEFAULT_VALUE 1500 // set the default servo value
#define FRAME_LENGTH 22500         // set the PPM frame length in microseconds (1ms = 1000µs)
#define PULSE_LENGTH 300           // set the pulse length
#define onState 1                  // set polarity of the pulses: 1 is positive, 0 is negative
#define ppmPin 2                   // set PPM signal output pin on the arduino

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
// Define RC input limite
/*
#define RC_CHANNEL_MIN 172
#define RC_CHANNEL_MID 991
#define RC_CHANNEL_MAX 1811
*/

#define ANALOG_CUTOFF 150  //cut off lower and upper end to avoid un-symmetric joystick range in trade off resolution

 // Define RC input Offset
int Aileron_OFFSET = -8;        
int Elevator_OFFSET  = 18; 
int Throttle_OFFSET =0;
int Rudder_OFFSET  = 5; 

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
const int DIGITAL_PIN_SWITCH_AUX3 = 2;  // 
//const int DIGITAL_PIN_SWITCH_AUX4 = 5;  // 

//pins that used for output
const int DIGITAL_PIN_LED = 5;  // in pcb v0.9 led is reused from AUX4 (remember to add 300om resistor in led)
const int DIGITAL_PIN_BUZZER = 7;  // do not use in pcb v0.9

//----- Voltage monitoring -------------------------
// Define battery warning voltage
 const float WARNING_VOLTAGE=7.4; //2S Lipo

 // Define Commond for start Up Setting
 #define RC_MIN_COMMAND  600
 #define RC_MAX_COMMAND  1400



//[Packet Rate]= 0 - 50Hz / 1 - 150Hz / 3 - 250Hz 
//[TX Power]=0 - 10mW / 1 - 25mW / 2 - 50mW /3 - 100mW/4 - 250mW
//3 Default Settings

#define SETTING_1_PktRate 2  //250Hz 
#define SETTING_1_Power 1    //25mW

#define SETTING_2_PktRate 1  //150Hz
#define SETTING_2_Power 3    //100mW

#define SETTING_3_PktRate 3  //500Hz
#define SETTING_3_Power 1    //25mW

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

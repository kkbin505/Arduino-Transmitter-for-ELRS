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

//pins that used for output
const int DIGITAL_PIN_LED = 5;  // 
const int DIGITAL_PIN_BUZZER = 7;  // 

//----- Voltage monitoring -------------------------
#define VOLTAGE_READS 10 //get average of VOLTAGE_READS readings

 // Define battery warning voltage
 const float WARNING_VOLTAGE=7.2; //2S Lipo

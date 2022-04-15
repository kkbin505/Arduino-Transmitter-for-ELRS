/*
// Simple Arduino trasmisster
// Arduino Nano
// ELRS 2.4G TX moduel
// Custom PCB from JLCPCB
// const float codeVersion = 0.92; // Software revision
// https://github.com/kkbin505/Arduino-Transmitter-for-ELRS

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
 BUILD OPTIONS (comment out unneeded options)
 =======================================================================================================
 */
//#define DEBUG // if not commented out, Serial.print() is active! For debugging only!!

#include <Arduino.h>

#include "config.h"
#include "crsf.h"
#include "led.h"

int Aileron_value = 0; // values read from the pot
int Elevator_value = 0;
int Throttle_value = 0;
int Rudder_value = 0;

int loopCount = 0; // for ELRS seeting

int Arm = 0; // switch values read from the digital pin
int AUX2_value = 0;
int AUX3_value = 0;
int AUX4_value = 0;

float batteryVoltage;

int currentPktRate = 0;
int currentPower = 0;
int currentSetting = 0;

uint8_t crsfPacket[CRSF_PACKET_SIZE];
uint8_t crsfCmdPacket[CRSF_CMD_PACKET_SIZE];
int rcChannels[CRSF_MAX_CHANNEL];
uint32_t crsfTime = 0;

CRSF crsfClass;

void selectSetting() {
    // startup stick commands (protocol selection / renew transmitter ID)

    if (rcChannels[AILERON] < RC_MIN_COMMAND && rcChannels[ELEVATOR] < RC_MIN_COMMAND) { // Elevator down + aileron left
        currentPktRate = SETTING_1_PktRate;
        currentPower = SETTING_1_Power;
        currentSetting = 1;
    } else if (rcChannels[AILERON] > RC_MAX_COMMAND && rcChannels[ELEVATOR] > RC_MAX_COMMAND) { // Elevator up + aileron right
        currentPktRate = SETTING_2_PktRate;
        currentPower = SETTING_2_Power;
        currentSetting = 2;
    } else if (rcChannels[AILERON] < RC_MIN_COMMAND && rcChannels[ELEVATOR] > RC_MAX_COMMAND) { // Elevator up + aileron right
        currentPktRate = SETTING_3_PktRate;
        currentPower = SETTING_3_Power;
        currentSetting = 3;
    } else {
        currentSetting = 0;
    }
}

void setup() {
    // inialize rc data
    for (uint8_t i = 0; i < CRSF_MAX_CHANNEL; i++) {
        rcChannels[i] = CRSF_DIGITAL_CHANNEL_MIN;
    }

    analogReference(EXTERNAL);
    pinMode(DIGITAL_PIN_SWITCH_ARM, INPUT_PULLUP);
    pinMode(DIGITAL_PIN_SWITCH_AUX2, INPUT_PULLUP);
    // pinMode(DIGITAL_PIN_SWITCH_AUX3, INPUT_PULLUP);
    // pinMode(DIGITAL_PIN_SWITCH_AUX4, INPUT_PULLUP);
    pinMode(DIGITAL_PIN_LED, OUTPUT);    // LED
    pinMode(DIGITAL_PIN_BUZZER, OUTPUT); // LED
    // digitalWrite(DIGITAL_PIN_BUZZER, LOW);
    // inialize voltage:
    batteryVoltage = 0.0;
#ifdef PPMOUTPUT
    pinMode(ppmPin, OUTPUT);
    digitalWrite(ppmPin, !onState); // set the PPM signal pin to the default state (off)
    cli();
    TCCR1A = 0; // set entire TCCR1 register to 0
    TCCR1B = 0;

    OCR1A = 100;             // compare match register, change this
    TCCR1B |= (1 << WGM12);  // turn on CTC mode
    TCCR1B |= (1 << CS11);   // 8 prescaler: 0,5 microseconds at 16mhz
    TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
    sei();
#endif

    delay(1000); // Give enough time for uploda firmware 1 second
#ifdef DEBUG
    Serial.begin(115200);
#else
    crsfClass.begin();
#endif
    digitalWrite(DIGITAL_PIN_LED, HIGH); // LED ON
}

void loop() {
    uint32_t currentMicros = micros();

    // Read Voltage
    batteryVoltage = analogRead(VOLTAGE_READ_PIN) / 103.0f; // 98.5

    if (batteryVoltage < WARNING_VOLTAGE) {
        slowBlinkLED(DIGITAL_PIN_LED);
        // fastBlinkLED(DIGITAL_PIN_LED);
    }

    /*
     * Handel analogy input
     */
    // constrain to avoid overflow
    /*  M7
    Aileron_value = constrain(analogRead(analogInPinAileron)+Aileron_OFFSET,ANALOG_CUTOFF,1023-ANALOG_CUTOFF);
    Elevator_value= constrain(analogRead(analogInPinElevator)+Elevator_OFFSET,ANALOG_CUTOFF,1023-ANALOG_CUTOFF);
    Throttle_value=constrain(analogRead(analogInPinThrottle)+Throttle_OFFSET,ANALOG_CUTOFF,1023-ANALOG_CUTOFF);
    Rudder_value = constrain(analogRead(analogInPinRudder)+Rudder_OFFSET,ANALOG_CUTOFF,1023-ANALOG_CUTOFF);
    rcChannels[AILERON] = map(Aileron_value,1023-ANALOG_CUTOFF,ANALOG_CUTOFF,CRSF_DIGITAL_CHANNEL_MIN,CRSF_DIGITAL_CHANNEL_MAX); //reverse
    rcChannels[ELEVATOR] = map(Elevator_value,ANALOG_CUTOFF,1023-ANALOG_CUTOFF,CRSF_DIGITAL_CHANNEL_MIN,CRSF_DIGITAL_CHANNEL_MAX); //reverse
    rcChannels[THROTTLE] = map(Throttle_value,1023-ANALOG_CUTOFF,ANALOG_CUTOFF,CRSF_DIGITAL_CHANNEL_MIN,CRSF_DIGITAL_CHANNEL_MAX);//reverse
    rcChannels[RUDDER] = map(Rudder_value ,ANALOG_CUTOFF,1023-ANALOG_CUTOFF,CRSF_DIGITAL_CHANNEL_MIN,CRSF_DIGITAL_CHANNEL_MAX);
    */

    /* Star  war */
    Aileron_value = constrain(analogRead(analogInPinAileron) + Aileron_OFFSET, ANALOG_CUTOFF, 1023 - ANALOG_CUTOFF);
    Elevator_value = constrain(analogRead(analogInPinElevator) + Elevator_OFFSET, ANALOG_CUTOFF, 1023 - ANALOG_CUTOFF);
    Throttle_value = constrain(analogRead(analogInPinThrottle) + Throttle_OFFSET, ANALOG_CUTOFF, 1023 - ANALOG_CUTOFF);
    Rudder_value = constrain(analogRead(analogInPinRudder) + Rudder_OFFSET, ANALOG_CUTOFF, 1023 - ANALOG_CUTOFF);
    rcChannels[AILERON] = map(Aileron_value, 1023 - ANALOG_CUTOFF, ANALOG_CUTOFF, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX);   // reverse
    rcChannels[ELEVATOR] = map(Elevator_value, 1023 - ANALOG_CUTOFF, ANALOG_CUTOFF, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX); // reverse
    rcChannels[THROTTLE] = map(Throttle_value, 1023 - ANALOG_CUTOFF, ANALOG_CUTOFF, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX); // reverse
    rcChannels[RUDDER] = map(Rudder_value, ANALOG_CUTOFF, 1023 - ANALOG_CUTOFF, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX);

    /*
     * Handel digital input
     */
    Arm = digitalRead(DIGITAL_PIN_SWITCH_ARM);
    AUX2_value = digitalRead(DIGITAL_PIN_SWITCH_AUX2);
    //    AUX3_value = digitalRead(DIGITAL_PIN_SWITCH_AUX3);
    // AUX4_value = digitalRead(DIGITAL_PIN_SWITCH_AUX4);// reuse for LED

    // Aux 1 Arm Channel
    if (Arm == 0)
    {
        rcChannels[AUX1] = CRSF_DIGITAL_CHANNEL_MIN;
    }
    else
    {
        rcChannels[AUX1] = CRSF_DIGITAL_CHANNEL_MAX;
    }

    // Aux 2 Channel
    if (AUX2_value == 0)
    {
        rcChannels[AUX2] = CRSF_DIGITAL_CHANNEL_MIN;
    }
    else
    {
        rcChannels[AUX2] = CRSF_DIGITAL_CHANNEL_MAX;
    }

/*  Aux 3
    if(AUX3_value==0){
      rcChannels[AUX3] =CRSF_DIGITAL_CHANNEL_MIN;
    }else{
      rcChannels[AUX3] =CRSF_DIGITAL_CHANNEL_MAX;
    }
*/
    //Additional switch add here.
/*
    if(AUX4_value==0){
      rcChannels[AUX4] =CRSF_DIGITAL_CHANNEL_MIN;
    }else if(AUX4_value==1){
      rcChannels[AUX4] =CRSF_DIGITAL_CHANNEL_MAX;
    }
*/

    selectSetting();

    if (currentMicros > crsfTime) {
        if (loopCount <= 500) { // repeat 500 packets to build connection to TX module
            // Build commond packet
            crsfClass.crsfPrepareDataPacket(crsfPacket, rcChannels);
            crsfClass.CrsfWritePacket(crsfPacket, CRSF_PACKET_SIZE);
            loopCount++;
        }

        if (loopCount > 500 && loopCount <= 505) { // repeat 500 packets to build connection to TX module
            // Build commond packet
            if (currentSetting > 0) {
                crsfClass.crsfPrepareCmdPacket(crsfCmdPacket, ELRS_PKT_RATE_COMMAND, currentPktRate);
                // buildElrsPacket(crsfCmdPacket,ELRS_WIFI_COMMAND,0x01);
                crsfClass.CrsfWritePacket(crsfCmdPacket, CRSF_CMD_PACKET_SIZE);
            }
            loopCount++;
        } else if (loopCount > 505 && loopCount < 510) { // repeat 10 packets to avoid bad packet
            if (currentSetting > 0) {
                crsfClass.crsfPrepareCmdPacket(crsfCmdPacket, ELRS_POWER_COMMAND, currentPower);
                // buildElrsPacket(crsfCmdPacket,ELRS_WIFI_COMMAND,0x01);
                crsfClass.CrsfWritePacket(crsfCmdPacket, CRSF_CMD_PACKET_SIZE);
            }
            loopCount++;
        } else {
            crsfClass.crsfPrepareDataPacket(crsfPacket, rcChannels);
// For gimal calibation only
#ifdef DEBUG
            Serial.print("A_");
            Serial.print(Aileron_value);
            Serial.print("_");
            Serial.print(rcChannels[0]);
            Serial.print(";E_");
            Serial.print(Elevator_value);
            Serial.print("_");
            Serial.print(rcChannels[1]);
            Serial.print(";T_");
            Serial.print(Throttle_value);
            Serial.print("_");
            Serial.print(rcChannels[2]);
            Serial.print("_R_");
            Serial.print(Rudder_value);
            Serial.print("_");
            Serial.print(rcChannels[3]);
            Serial.print(";Arm_");
            Serial.print(Arm);
            Serial.print(";AUX2_");
            Serial.print(AUX2_value);
            Serial.print(";AUX3_");
            Serial.print(AUX3_value);
            Serial.print("_BatteryVoltage:");
            Serial.print(batteryVoltage);
            Serial.print("_CurrentSetting:");
            Serial.print(currentSetting);
            Serial.println();
            delay(500);
#else
            crsfClass.CrsfWritePacket(crsfPacket, CRSF_PACKET_SIZE);
#endif
        }
        crsfTime = currentMicros + CRSF_TIME_BETWEEN_FRAMES_US;
    }
}

ISR(TIMER1_COMPA_vect) { // leave this alone
    static boolean state = true;

    TCNT1 = 0;

    if (state) { // start pulse
        digitalWrite(ppmPin, onState);
        OCR1A = PULSE_LENGTH * 2;
        state = false;
    } else { // end pulse and calculate when to start the next pulse
        static byte cur_chan_numb;
        static unsigned int calc_rest;

        digitalWrite(ppmPin, !onState);
        state = true;

        if (cur_chan_numb >= CHANNEL_NUMBER) {
            cur_chan_numb = 0;
            calc_rest = calc_rest + PULSE_LENGTH; //
            OCR1A = (FRAME_LENGTH - calc_rest) * 2;
            calc_rest = 0;
        } else {
            OCR1A = (map(rcChannels[cur_chan_numb], CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX, 1000, 2000) - PULSE_LENGTH) * 2;
            calc_rest = calc_rest + map(rcChannels[cur_chan_numb], CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX, 1000, 2000);
            cur_chan_numb++;
        }
    }
}

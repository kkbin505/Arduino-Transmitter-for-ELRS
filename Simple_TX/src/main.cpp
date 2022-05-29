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

#include <Arduino.h>

#include "EEPROM.h"

#include "config.h"
#include "crsf.h"
#include "led.h"

//#define DEBUG // if not commented out, Serial.print() is active! For debugging only!!

int Aileron_value = 0; // values read from the pot
int Elevator_value = 0;
int Throttle_value = 0;
int Rudder_value = 0;

int loopCount = 0; // for ELRS seeting

int AUX1_Arm = 0; // switch values read from the digital pin
int AUX2_value = 0;
int AUX3_value = 0;
int AUX4_value = 0;

float batteryVoltage;

int currentPktRate = 0;
int currentPower = 0;
int currentSetting = 0;

uint32_t currentMillis = 0;

uint8_t crsfPacket[CRSF_PACKET_SIZE];
uint8_t crsfCmdPacket[CRSF_CMD_PACKET_SIZE];
int16_t rcChannels[CRSF_MAX_CHANNEL];
uint32_t crsfTime = 0;

CRSF crsfClass;

bool calStatus=false;

// -----------------------------------------------------------------------------------------------------
// Calibration

#define CALIB_MARK      0x55
#define CALIB_MARK_ADDR 0x00
#define CALIB_VAL_ADDR  CALIB_MARK_ADDR + 1

#define CALIB_CNT       5       // times of switch on/off
#define CALIB_TMO       20000    // ms
int     cal_reset = 0 ;

struct CalibValues {
    int aileronMin;
    int aileronMax;
    int aileronCenter;
    int elevatorMin;
    int elevatorMax;
    int elevatorCenter;
    int thrMin;
    int thrMax;
    int rudderMin;
    int rudderMax;
    int rudderCenter;
};

CalibValues calValues;

bool calibrationPresent() {
    return EEPROM.read(CALIB_MARK_ADDR) == CALIB_MARK;
}

void calibrationSave() {
    EEPROM.update(CALIB_MARK_ADDR, CALIB_MARK);
    EEPROM.put(CALIB_VAL_ADDR, calValues);
}

void calibrationLoad() {
    EEPROM.get(CALIB_VAL_ADDR, calValues);
}

void calibrationReset() {
    EEPROM.put(CALIB_MARK_ADDR, (uint8_t)0xFF);
    calValues = {
        .aileronMin     = ANALOG_CUTOFF,
        .aileronMax     = 1023 - ANALOG_CUTOFF,
        .aileronCenter  = 988,
        .elevatorMin    = ANALOG_CUTOFF,
        .elevatorMax    = 1023 - ANALOG_CUTOFF,
        .elevatorCenter = 988,
        .thrMin         = ANALOG_CUTOFF,
        .thrMax         = 1023 - ANALOG_CUTOFF,
        .rudderMin      = ANALOG_CUTOFF,
        .rudderMax      = 1023 - ANALOG_CUTOFF,
        .rudderCenter    = 988,
    };
    EEPROM.put(CALIB_VAL_ADDR, calValues);
}

uint8_t aux2cnt = 0;

unsigned long calibrationTimerStart;


//Flash LED and beep for (times)
void calibrationChirp(uint8_t times) {
    
    digitalWrite(DIGITAL_PIN_LED, HIGH);
    delay(1000);
    digitalWrite(DIGITAL_PIN_LED, LOW);
    delay(1000);

    for (uint8_t i = 0; i < times; i++) {
        digitalWrite(DIGITAL_PIN_BUZZER, HIGH);
        digitalWrite(DIGITAL_PIN_LED, HIGH);
        delay(100);
        digitalWrite(DIGITAL_PIN_BUZZER, LOW);
        digitalWrite(DIGITAL_PIN_LED, LOW);
        delay(100);
    }
    digitalWrite(DIGITAL_PIN_LED, LOW);
}

void calibrationCount(int aux1, int aux2) {
    static uint8_t preAux2 = 0;

    // Don't do anything if ARMED
    if (aux1 != 0) {
        aux2cnt = 0;
        preAux2 = 0;
        return;
    }

    else{
        // Start timer window // Arm = DISARMED
        if (aux2cnt == 0) {
            calibrationTimerStart = millis();
        }

        // Timeout
        if (calibrationTimerStart + CALIB_TMO > millis()) {
            aux2cnt = 0;
            preAux2 = 0;
            return;
        }

        // Analyze AUX2 switch, count only one side of the switch
        if (aux2 == 1 && preAux2 == 0) {
            aux2cnt++;
            preAux2 = 1;
        } else if (aux2 == 0 && preAux2 == 1) {
            preAux2 = 0;
        }
    }
}

bool calibrationRequested() {
    //return aux2cnt > CALIB_CNT;
    return calStatus;
}

bool calibrationProcess() {
    aux2cnt = 0;

    //calibrationChirp(2);    // start calibration

    // Reset variables to "centers"

    while(cal_reset<1){
    const int centerValue = (1023 - ANALOG_CUTOFF - ANALOG_CUTOFF) / 2;
    calValues.aileronMin    = centerValue;
    calValues.aileronMax    = centerValue;
    calValues.elevatorMin   = centerValue;
    calValues.elevatorMax   = centerValue;
    calValues.thrMin        = centerValue;
    calValues.thrMax        = centerValue;
    calValues.rudderMin     = centerValue;
    calValues.rudderMax     = centerValue;
    cal_reset++;
    }

    currentMillis = millis();


    // 15 seconds for moving sticks
    if (currentMillis < CALIB_TMO){
    //while ((calibrationTimerStart + CALIB_TMO ) < millis()) {
        // A Min-Max
        int val = analogRead(analogInPinAileron);
        if (val < calValues.aileronMin) {
            calValues.aileronMin = val;
        } 
        if (val > calValues.aileronMax) {
            calValues.aileronMax = val;
        }
        // E Min-Max
        val = analogRead(analogInPinElevator);
        if (val < calValues.elevatorMin) {
            calValues.elevatorMin = val;
        } 
        if (val > calValues.elevatorMax) {
            calValues.elevatorMax = val;
        }

        // T Min-Max
        val = analogRead(analogInPinThrottle);
        if (val < calValues.thrMin) {
            calValues.thrMin = val;
        } 
        if (val > calValues.thrMax) {
            calValues.thrMax = val;
        }

        // R Min-Max
        val = analogRead(analogInPinRudder);
        if (val < calValues.rudderMin) {
            calValues.rudderMin = val;
        } 
        if (val > calValues.rudderMax) {
            calValues.rudderMax = val;
        }

        // Double beep and blink every 500ms
        if (millis() % 500 == 0) {
           // calibrationChirp(2);
        }
        Serial.print("Aileron_Min:");
        Serial.print(calValues.aileronMin);
        Serial.print(" Max:");
        Serial.print(calValues.aileronMax);
        Serial.print(" ElevatorMin:");
        Serial.print(calValues.elevatorMin);
        Serial.print(" Max:");
        Serial.print(calValues.elevatorMax);
        Serial.print(" RudderMin:");
        Serial.print(calValues.rudderMin);
        Serial.print(" Max:");
        Serial.print(calValues.rudderMax);
        Serial.println();


    }
    else {
        Serial.println("Calibration Done");  
        calibrationSave();
        calStatus = false;
    }
    return true;

}

void calibrationRun(int aux1, int aux2) {
    if (calibrationRequested()) {
        //calibrationReset();
        if (calibrationProcess()) {
            //calibrationSave();
            //calibrationChirp(3);    // ok
           // Serial.println("Calibration OK");
        } else {
            //calibrationReset();
           // calibrationChirp(10);   // error
            Serial.println("Calibration error!");
        }
    } else {
        calibrationCount(aux1, aux2);
    }
}

// -----------------------------------------------------------------------------------------------------

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

void setup()
{
    // inialize rc data
    for (uint8_t i = 0; i < CRSF_MAX_CHANNEL; i++) {
        rcChannels[i] = CRSF_DIGITAL_CHANNEL_MIN;
    }

    analogReference(EXTERNAL);
    pinMode(DIGITAL_PIN_SWITCH_ARM, INPUT_PULLUP);
    pinMode(DIGITAL_PIN_SWITCH_AUX2, INPUT_PULLUP);
    pinMode(DIGITAL_PIN_SWITCH_AUX3, INPUT_PULLUP);
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
        //calibrationReset();
        calStatus = true;
    #else
        crsfClass.begin();
    #endif

    digitalWrite(DIGITAL_PIN_LED, HIGH); // LED ON

    //calibrationReset();
    calibrationLoad();

    #ifdef DEBUG
        Serial.println("Start Calibration");  
    #endif
}

void loop()
{
    #ifdef DEBUG
        if(calStatus ){
            calibrationRun(AUX1_Arm, AUX2_value);
        }
    #endif

    uint32_t currentMicros = micros();

    // Read Voltage
    batteryVoltage = analogRead(VOLTAGE_READ_PIN) / 103.0f; // 98.5

    if (batteryVoltage < WARNING_VOLTAGE) {
        blinkLED(DIGITAL_PIN_LED, 1000);
        // fastBlinkLED(DIGITAL_PIN_LED, 300);
    }

    /*
     * Handle analogy input
     */
    // constrain to avoid overflow
    Aileron_value  = analogRead(analogInPinAileron)  + Aileron_OFFSET;
    Elevator_value = analogRead(analogInPinElevator) + Elevator_OFFSET;
    Throttle_value = analogRead(analogInPinThrottle) + Throttle_OFFSET;
    Rudder_value   = analogRead(analogInPinRudder)   + Rudder_OFFSET;
    // rcChannels[AILERON] = map(Aileron_value, 1023 - ANALOG_CUTOFF, ANALOG_CUTOFF, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX);   // reverse
    // rcChannels[ELEVATOR] = map(Elevator_value, 1023 - ANALOG_CUTOFF, ANALOG_CUTOFF, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX); // reverse
    // rcChannels[THROTTLE] = map(Throttle_value, 1023 - ANALOG_CUTOFF, ANALOG_CUTOFF, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX); // reverse
    // rcChannels[RUDDER] = map(Rudder_value, ANALOG_CUTOFF, 1023 - ANALOG_CUTOFF, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX);
    rcChannels[AILERON]   = constrain(map(Aileron_value,    calValues.aileronMax,   calValues.aileronMin,   CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX), CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX); // reverse
    rcChannels[ELEVATOR]  = constrain(map(Elevator_value,   calValues.elevatorMax,  calValues.elevatorMin,  CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX), CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX); // reverse
    rcChannels[THROTTLE]  = constrain(map(Throttle_value,   calValues.thrMax,       calValues.thrMin,       CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX), CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX); // reverse
    rcChannels[RUDDER]    = constrain(map(Rudder_value,     calValues.rudderMin,    calValues.rudderMax,    CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX), CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX);

    /*
     * Handel digital input
     */
    AUX1_Arm = digitalRead(DIGITAL_PIN_SWITCH_ARM);
    AUX2_value = digitalRead(DIGITAL_PIN_SWITCH_AUX2);
    AUX3_value = digitalRead(DIGITAL_PIN_SWITCH_AUX3);
    // AUX4_value = digitalRead(DIGITAL_PIN_SWITCH_AUX4);// reuse for LED

    // Aux Channels
    rcChannels[AUX1] = (AUX1_Arm == 0) ? CRSF_DIGITAL_CHANNEL_MIN : CRSF_DIGITAL_CHANNEL_MAX;
    rcChannels[AUX2] = (AUX2_value == 0) ? CRSF_DIGITAL_CHANNEL_MIN : CRSF_DIGITAL_CHANNEL_MAX;
    rcChannels[AUX3] = (AUX3_value == 0) ? CRSF_DIGITAL_CHANNEL_MIN : CRSF_DIGITAL_CHANNEL_MAX;
    // rcChannels[AUX4] = (AUX4_value == 0) ? CRSF_DIGITAL_CHANNEL_MIN : CRSF_DIGITAL_CHANNEL_MAX;

    selectSetting();

    if (currentMicros > crsfTime) {
        #ifdef DEBUG
            //Serial.println("test");
        #else
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
                crsfClass.CrsfWritePacket(crsfPacket, CRSF_PACKET_SIZE);

            }
        #endif
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

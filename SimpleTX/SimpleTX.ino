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
#include "tone.h"

//#define DEBUG // if not commented out, Serial.print() is active! For debugging only!!
//#define GIMBAL_CALIBRATION // if not commented out, Serial.print() is active! For debugging only!!

int Aileron_value = 0; // values read from the pot
int Elevator_value = 0;
int Throttle_value = 0;
int Rudder_value = 0;
int previous_throttle = 191;

int loopCount = 0; // for ELRS seting

int AUX1_Arm = 0; // switch values read from the digital pin
int AUX2_value = 0;
int AUX3_value = 0;
int AUX4_value = 0;
float batteryVoltage;
int currentPktRate = 0;
int currentPower = 0;
int currentSetting = 0;
int currentDynamic = 0;
int stickMoved = 0;
int stickInt = 0;
uint32_t stickMovedMillis = 0;
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
#define CALIB_CENT_TMO  5000    //ms
#define CALIB_TMO       20000   // ms
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
    calValues.aileronMin     = centerValue;
    calValues.aileronMax     = centerValue;
    calValues.aileronCenter  = centerValue;
    calValues.elevatorMin    = centerValue;
    calValues.elevatorMax    = centerValue;
    calValues.elevatorCenter = centerValue;
    calValues.thrMin         = centerValue;
    calValues.thrMax         = centerValue;
    calValues.rudderMin      = centerValue;
    calValues.rudderMax      = centerValue;
    calValues.rudderCenter   = centerValue;
    cal_reset++;
    }

    currentMillis = millis();

    if (currentMillis <CALIB_CENT_TMO){
        // A Center
        int val = analogRead(analogInPinAileron);
        calValues.aileronCenter = val;
        
        // E Center
        val = analogRead(analogInPinElevator);
        calValues.elevatorCenter = val;

        // R Center
        val = analogRead(analogInPinRudder);
        calValues.rudderCenter = val;
    
     Serial.print("Aileron_Min:");
        Serial.print("Center Stick: AilerCenter:");
        Serial.print(calValues.aileronCenter);
        Serial.print(" ElevatorCenter:");
        Serial.print(calValues.elevatorCenter);
        Serial.print(" RudderCenter:");
        Serial.print(calValues.rudderCenter);
        Serial.println();
        
    }
    // 15 seconds for moving sticks
    else if (currentMillis > CALIB_CENT_TMO && currentMillis < CALIB_TMO){
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
        Serial.print("Mover stick full range: Aileron_Min:");
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
        Serial.print(" ThrottleMin:");
        Serial.print(calValues.thrMin);
        Serial.print(" Max:");
        Serial.print(calValues.thrMax);
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

<<<<<<< Updated upstream
    if (rcChannels[AILERON] < RC_MIN_COMMAND && rcChannels[ELEVATOR] < RC_MIN_COMMAND) { // Elevator down + aileron left
=======
    if (rcChannels[AILERON] < RC_MIN_COMMAND && rcChannels[ELEVATOR] > RC_MAX_COMMAND) { // Elevator up + aileron right
>>>>>>> Stashed changes
        currentPktRate = SETTING_1_PktRate;
        currentPower = SETTING_1_Power;
        currentDynamic = SETTING_1_Dynamic;
        currentSetting = 1;
    } else if (rcChannels[AILERON] > RC_MAX_COMMAND && rcChannels[ELEVATOR] > RC_MAX_COMMAND) { // Elevator up + aileron left
        currentPktRate = SETTING_2_PktRate;
        currentPower = SETTING_2_Power;
        currentDynamic = SETTING_2_Dynamic;
        currentSetting = 2;
<<<<<<< Updated upstream
    } else if (rcChannels[AILERON] < RC_MIN_COMMAND && rcChannels[ELEVATOR] > RC_MAX_COMMAND) { // Elevator up + aileron right
        currentPktRate = SETTING_3_PktRate;
        currentPower = SETTING_3_Power;
        currentSetting = 3;
=======
    } else if (rcChannels[AILERON] < RC_MIN_COMMAND && rcChannels[ELEVATOR] < RC_MIN_COMMAND) { // Elevator down + aileron right
        currentSetting = 3; // Bind to RX
    } else if (rcChannels[AILERON] > RC_MAX_COMMAND && rcChannels[ELEVATOR] < RC_MIN_COMMAND) { // Elevator down + aileron left
        currentSetting = 4;  // Start TX Wifi
>>>>>>> Stashed changes
    } else {
        currentSetting = 0;
    }
}

bool checkStickMove(){
    // check if stick moved, warring after 10 minutes
    if(abs(previous_throttle - rcChannels[THROTTLE]) < 30){
        stickMoved = 0;
        //Serial.println(abs(previous_throttle - rcChannels[THROTTLE]));
    }else{
        previous_throttle = rcChannels[THROTTLE];
        stickMovedMillis = millis();
        stickMoved = 1;
    }

    if (millis() - stickMovedMillis > STICK_ALARM_TIME){
       // Serial.println((millis() - stickMovedMillis));
        return true;
    }else{
        return false;
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

    #else
        crsfClass.begin();
    #endif

    #ifdef GIMBAL_CALIBRATION
        Serial.begin(115200);
        //calibrationReset();
        calStatus = true;
        Serial.println("Start Calibration"); 
    #endif

    digitalWrite(DIGITAL_PIN_LED, HIGH); // LED ON
    //digitalWrite(DIGITAL_PIN_BUZZER, HIGH); // BUZZER OFF

    //calibrationReset();
    calibrationLoad();

}

void loop()
{
    #ifdef GIMBAL_CALIBRATION
        if(calStatus ){
            calibrationRun(AUX1_Arm, AUX2_value);
        }
    #endif

    uint32_t currentMicros = micros();

    // Read Voltage
    batteryVoltage = analogRead(VOLTAGE_READ_PIN) / 103.0f; // 98.5

    if (batteryVoltage < WARNING_VOLTAGE && batteryVoltage >= BEEPING_VOLTAGE) {
        blinkLED(DIGITAL_PIN_LED, 1000);
    }else if(batteryVoltage < BEEPING_VOLTAGE){
        blinkLED(DIGITAL_PIN_LED, 300);
        playingTones(2);
    }

    if (checkStickMove() == true){
        blinkLED(DIGITAL_PIN_LED, 100);
        playingTones(5);
    }

    /*
     * Handle analogy input
     */
    // constrain to avoid overflow
    int analogVal = analogRead(analogInPinAileron);
    if (analogVal <= calValues.aileronCenter){
        Aileron_value = map(analogVal,calValues.aileronMin,   calValues.aileronCenter, ADC_MIN, ADC_MID);
    }
    else{
        Aileron_value = map(analogVal,calValues.aileronCenter,   calValues.aileronMax, ADC_MID+1, ADC_MAX);
    }
    
    analogVal = analogRead(analogInPinElevator);
    if (analogVal <= calValues.elevatorCenter){
        Elevator_value = map(analogVal,calValues.elevatorMin,   calValues.elevatorCenter, ADC_MIN, ADC_MID);
    }
    else{
        Elevator_value = map(analogVal,calValues.elevatorCenter,   calValues.elevatorMax, ADC_MID+1, ADC_MAX);
    }

    analogVal = analogRead(analogInPinThrottle);
    Throttle_value = map(analogVal, calValues.thrMax, calValues.thrMin, ADC_MIN, ADC_MAX);

    analogVal = analogRead(analogInPinRudder);
    if (analogVal <= calValues.rudderCenter){
        Rudder_value = map(analogVal,calValues.rudderMin,   calValues.rudderCenter, ADC_MIN, ADC_MID);
    }
    else{
        Rudder_value = map(analogVal,calValues.rudderCenter,   calValues.rudderMax, ADC_MID+1, ADC_MAX);
    }
    
    //Constrain value to avoid overflow
    Aileron_value  = constrain(Aileron_value,  ADC_MIN, ADC_MAX); 
    Elevator_value = constrain(Elevator_value, ADC_MIN, ADC_MAX); 
    Throttle_value = constrain(Throttle_value, ADC_MIN, ADC_MAX); 
    Rudder_value   = constrain(Rudder_value,   ADC_MIN, ADC_MAX); 

    //Handdle reverse
    if (Is_Aileron_Reverse == 1){
        Aileron_value  = 1023-Aileron_value;
    }
    if (Is_Elevator_Reverse == 1){
        Elevator_value = 1023-Elevator_value;
    }
    if (Is_Throttle_Reverse == 1){
        Throttle_value = 1023-Throttle_value;
    }
    if (Is_Rudder_Reverse == 1){
        Rudder_value   = 1023-Rudder_value;
    }
    // rcChannels[AILERON] = map(Aileron_value, 1023 - ANALOG_CUTOFF, ANALOG_CUTOFF, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX);   // reverse
    // rcChannels[ELEVATOR] = map(Elevator_value, 1023 - ANALOG_CUTOFF, ANALOG_CUTOFF, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX); // reverse
    // rcChannels[THROTTLE] = map(Throttle_value, 1023 - ANALOG_CUTOFF, ANALOG_CUTOFF, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX); // reverse
    // rcChannels[RUDDER] = map(Rudder_value, ANALOG_CUTOFF, 1023 - ANALOG_CUTOFF, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX);
    rcChannels[AILERON]   = map(Aileron_value,  ADC_MIN, ADC_MAX, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX); 
    rcChannels[ELEVATOR]  = map(Elevator_value, ADC_MIN, ADC_MAX, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX);
    rcChannels[THROTTLE]  = map(Throttle_value, ADC_MIN, ADC_MAX, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX);
    rcChannels[RUDDER]    = map(Rudder_value,   ADC_MIN, ADC_MAX, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX);

    /*
     * Handel digital input
     */
    AUX1_Arm = digitalRead(DIGITAL_PIN_SWITCH_ARM);
    AUX2_value = digitalRead(DIGITAL_PIN_SWITCH_AUX2);
    AUX3_value = digitalRead(DIGITAL_PIN_SWITCH_AUX3);
    // AUX4_value = digitalRead(DIGITAL_PIN_SWITCH_AUX4);// reuse for LED

    // Aux Channels
    rcChannels[AUX1] = (AUX1_Arm == 1)   ? CRSF_DIGITAL_CHANNEL_MIN : CRSF_DIGITAL_CHANNEL_MAX;
    rcChannels[AUX2] = (AUX2_value == 1) ? CRSF_DIGITAL_CHANNEL_MIN : CRSF_DIGITAL_CHANNEL_MAX;
    rcChannels[AUX3] = (AUX3_value == 0) ? CRSF_DIGITAL_CHANNEL_MIN : CRSF_DIGITAL_CHANNEL_MAX;
<<<<<<< Updated upstream
    // rcChannels[AUX4] = (AUX4_value == 0) ? CRSF_DIGITAL_CHANNEL_MIN : CRSF_DIGITAL_CHANNEL_MAX;
=======
    rcChannels[AUX4] = (AUX4_value == 0) ? CRSF_DIGITAL_CHANNEL_MIN : CRSF_DIGITAL_CHANNEL_MAX;
>>>>>>> Stashed changes

    if(stickInt=0){
        previous_throttle=rcChannels[THROTTLE];
        stickInt=1;
    }
    selectSetting();

    if (currentMicros > crsfTime) {
        #ifdef DEBUG
            //Serial.println("DEBUG");
        Serial.print(" AILERON:");
        Serial.print(rcChannels[AILERON]);
        Serial.print(" ELEVATOR:");
        Serial.print(rcChannels[ELEVATOR] );
        Serial.print(" THROTTLE:");
        Serial.print(rcChannels[THROTTLE]);
        Serial.print(" RUDDER:");
        Serial.print(rcChannels[RUDDER] );
        Serial.print(" stickstatus:");
        Serial.print(stickMoved);
        Serial.print(" previous_throttle:");
        Serial.print(previous_throttle);
        Serial.println();

        #else
            if (loopCount <= 500) { // repeat 500 packets to build connection to TX module
                // Build commond packet
                crsfClass.crsfPrepareDataPacket(crsfPacket, rcChannels);
                crsfClass.CrsfWritePacket(crsfPacket, CRSF_PACKET_SIZE);
                loopCount++;
            }

            if (loopCount > 500 && loopCount <= 505) { // repeat 5 packets to avoid bad packet, change rate setting
                // Build commond packet
                if (currentSetting > 0) {
                    crsfClass.crsfPrepareCmdPacket(crsfCmdPacket, ELRS_PKT_RATE_COMMAND, currentPktRate);
                    // buildElrsPacket(crsfCmdPacket,ELRS_WIFI_COMMAND,0x01);
                    crsfClass.CrsfWritePacket(crsfCmdPacket, CRSF_CMD_PACKET_SIZE);
<<<<<<< Updated upstream
=======
                } else if (currentSetting == 3) {
                    crsfClass.crsfPrepareCmdPacket(crsfCmdPacket, ELRS_BIND_COMMAND, ELRS_START_COMMAND);
                    crsfClass.CrsfWritePacket(crsfCmdPacket, CRSF_CMD_PACKET_SIZE);
                } else if (currentSetting == 4) {
                    crsfClass.crsfPrepareCmdPacket(crsfCmdPacket, ELRS_WIFI_COMMAND, ELRS_START_COMMAND);
                    crsfClass.CrsfWritePacket(crsfCmdPacket, CRSF_CMD_PACKET_SIZE);
>>>>>>> Stashed changes
                }
                loopCount++;
            } else if (loopCount > 505 && loopCount < 510) { // repeat 10 packets to avoid bad packet, change TX power level
                if (currentSetting > 0) {
                    crsfClass.crsfPrepareCmdPacket(crsfCmdPacket, ELRS_POWER_COMMAND, currentPower);
                    // buildElrsPacket(crsfCmdPacket,ELRS_WIFI_COMMAND,0x01);
                    crsfClass.CrsfWritePacket(crsfCmdPacket, CRSF_CMD_PACKET_SIZE);
                }
                loopCount++;
            } else if (loopCount > 510 && loopCount <= 515) { // repeat 5 packets to avoid bad packet, change TX dynamic power setting
                if (currentSetting == 1 || currentSetting == 2) {
                    crsfClass.crsfPrepareCmdPacket(crsfCmdPacket, ELRS_DYNAMIC_POWER_COMMAND, currentDynamic);
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

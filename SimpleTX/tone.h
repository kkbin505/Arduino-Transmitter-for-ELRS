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
//----- global variables for tones generation --------------
unsigned long previousToneMillis = 0;
bool started=false;

// ----------------------------------------------------------------------------
void playingTones(uint8_t timeSeconde) {
    unsigned long currentMillis = millis();
#ifdef PASSIVE_BUZZER
    if ( !rtttl::isPlaying()) {
      if ( !started ) {  // 
        if (timeSeconde == 2) { // Low Battery
          // The following two tones mimick the old Nokia 3310 battery warning
          if (currentMillis - previousToneMillis <= 100) {
              tone(DIGITAL_PIN_BUZZER,1568,100);
          }else if(currentMillis - previousToneMillis > 100 && currentMillis - previousToneMillis <= 300){
              tone(DIGITAL_PIN_BUZZER,784,100);
          }else if(currentMillis - previousToneMillis > 300 && currentMillis - previousToneMillis <= 5000){
              noTone(DIGITAL_PIN_BUZZER);
          }else{
              previousToneMillis = currentMillis;     // save the last time buzzer play tone
              noTone(DIGITAL_PIN_BUZZER);
          }
          // Tones did not work for the low battery warning
          // const char * LOW_BATT_WARNING = "nokia:d=4,o=5,b=140:16g6,8g,16p,16g6,8g,16p,16g6,8g";
          // rtttl::begin(DIGITAL_PIN_BUZZER, LOW_BATT_WARNING);
        } 
        else if (timeSeconde == 5) { // Stick move timeout
          rtttl::begin(DIGITAL_PIN_BUZZER, STICK_MOVE_WARNING);
          started = true;
          previousToneMillis = currentMillis;     // save the last time buzzer play tone
        }
      }
      else {
        if (currentMillis - previousToneMillis > 5000) {
          started = false;
        }
      }
    }

#else 
    //ACTIVE_BUZZER
    if (currentMillis - previousToneMillis <= timeSeconde*100) {
        analogWrite(DIGITAL_PIN_BUZZER, 128);
    }else if(currentMillis - previousToneMillis > timeSeconde*100 && currentMillis - previousToneMillis <= timeSeconde*200){
        analogWrite(DIGITAL_PIN_BUZZER, 0);
    }else if(currentMillis - previousToneMillis > timeSeconde*200 && currentMillis - previousToneMillis <= timeSeconde*300){
        analogWrite(DIGITAL_PIN_BUZZER, 128);
    }else if(currentMillis - previousToneMillis > timeSeconde*300 && currentMillis - previousToneMillis <= 5000){
        analogWrite(DIGITAL_PIN_BUZZER, 0);
    }else{
        previousToneMillis = currentMillis;     // save the last time buzzer play tone
        analogWrite(DIGITAL_PIN_BUZZER, 0);
    }
 #endif   
    
}


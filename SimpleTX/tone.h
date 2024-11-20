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


//#define BUZZERFREQ 1000

//----- global variables for tones generation --------------
uint8_t playSound = 0;

bool curDurIndex = 1;


unsigned long previousToneMillis = 0;

// ----------------------------------------------------------------------------
void playingTones(uint8_t timeSeconde) {
    unsigned long currentMillis = millis();

        if (currentMillis - previousToneMillis <= timeSeconde*100) {

            playSound ^= 1;    
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
    
    
}

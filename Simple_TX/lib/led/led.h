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

uint8_t ledState = LOW;
unsigned long previousMillis = 0;

void blinkLED(int ledPin, uint16_t blinkRate) {
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= blinkRate) {
        previousMillis = currentMillis;     // save the last time you blinked the LED
        ledState ^= 1;                      // if the LED is off turn it on and vice-versa
        digitalWrite(ledPin, ledState);
    }
}

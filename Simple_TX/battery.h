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




 uint8_t readVoltage() {
    int voltageA = 0;
    float voltageB;

    analogRead(VOLTAGE_READ_PIN); // first fake read to improve further readings accuracy (as suggested by Nicola Gorghetto)

    for (uint8_t i = 0; i < VOLTAGE_READS; i++) {
        voltageA += analogRead(VOLTAGE_READ_PIN);
    }

    voltageA = voltageA/VOLTAGE_READS; // average of VOLTAGE_READS readings
    voltageB=voltageA/33;
    return voltageB;
}

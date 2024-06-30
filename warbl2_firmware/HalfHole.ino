/*
    Copyright (C) 2024 Gianluca Barbaro barbaro.it

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see http://www.gnu.org/licenses/
*/


//Initializes runtime parameters
void hh_init() {
    halfHole.currentHoleSettings = 99; //disabled
    resetHalfHoleConfig();
}

//Resets saved Half Hole Calibration values
void resetHalfHoleConfig() {
    halfHole.buffer = HALF_HOLE_BUFFER_SIZE;
    for (byte i = 0; i < TONEHOLE_SENSOR_NUMBER; i++) {
        halfHole.enabled[i] = false;
    }
    halfHole.enabled[THUMB_HOLE] = true;
    halfHole.enabled[R3_HOLE] = true;
    halfHole.enabled[R4_HOLE] = true;
    
}

//Loads Half Hole Detection parameters
void  loadHalfHoleConfig() {
    halfHole.buffer = readEEPROM(EEPROM_HALF_HOLE_BUFFER_SIZE);
    for (byte i = 0; i < TONEHOLE_SENSOR_NUMBER; i++) {
        halfHole.enabled[i] = readEEPROM(EEPROM_HALF_HOLE_ENABLED + i);
    }
}
//Saves Half Hole Detection parameters
void  saveHalfHoleConfig() {
    writeEEPROM(EEPROM_HALF_HOLE_BUFFER_SIZE, halfHole.buffer);
    for (byte i = 0; i < TONEHOLE_SENSOR_NUMBER; i++) {
        writeEEPROM(EEPROM_HALF_HOLE_ENABLED + i, halfHole.enabled[i]);
    }
}

//Always call UpperBound first, to calculate correction
uint16_t getHalfHoleLowerBound(byte hole) {
    return getHalfHoleUpperBound(hole) - HALF_HOLE_WINDOW_SIZE;
}

uint16_t getHalfHoleUpperBound(byte hole) {
    if (!halfHole.enabled[hole]) {
        return 1024;
    }
    // return toneholeCovered[hole] - HOLE_OPEN_OFFSET + HALF_HOLE_UPPER_OFFSET + halfHole.correction*HALF_HOLE_CALIB_OFFSET;
    return toneholeCovered[hole] - (HALF_HOLE_UPPER_OFFSET - halfHole.correction*HALF_HOLE_CALIB_OFFSET);
}

//returns if the selected hole is half covered
bool isHalfHole(int hole) {
    if (!halfHole.enabled[hole]) return false; //disabled

    bool result = toneholeRead[hole] <  getHalfHoleUpperBound(hole) && toneholeRead[hole] > getHalfHoleLowerBound(hole);
    #ifdef DEBUG_HH

    if (hole == THUMB_HOLE && result) {
        Serial.print(getHalfHoleLowerBound(hole));
        Serial.print(" -> ");
        Serial.print(toneholeRead[hole]);
        Serial.print(" <- C: ");
        Serial.print(toneholeCovered[hole]);
        Serial.print(" - UB");
        Serial.println(getHalfHoleUpperBound(hole));
    } 
    #endif

    return result;
}


//Calculates the baseline moving average for readings in the current time window and applies an eventual correction factor to half hole detection
void baselineUpdate() {
    byte counter = 0;
    baselineCurrentAverage = 0;
    for (byte i = 1; i<TONEHOLE_SENSOR_NUMBER; i++) {

        if (toneholeBaselineCurrent[i] >= 0 && toneholeBaselineCurrent[i] <= maxBaseline)  { //LPF
            baselineCurrentAverage += toneholeBaselineCurrent[i];
            counter++;
        }
        toneholeBaselineCurrent[i] = 1024; //Resets for next run

        //Updates baseline too with previous average - Useful for PB
        toneholeBaseline[i] = ((float) toneholeBaseline[i] * baselinePreviousAverage/baselineAverage);
    }
    if (counter > 0 ) { //Found usable values
        //Calculates current (quasi)autoregressive moving average - baseLine Average. It measures the current ambient light, more or less
        baselineCurrentAverage = (BASELINE_AVRG_SPEED*((baselineCurrentAverage / (float) counter)) + (1.0-BASELINE_AVRG_SPEED)*baselinePreviousAverage);
        
        float diff = baselineCurrentAverage  - baselineAverage;
        diff = sqrt(abs(diff))*diff/diff ;
        halfHole.correction = diff;

        #ifdef DEBUG_HH
        Serial.print("halfHole.correction: ");
        Serial.println(halfHole.correction);
        #endif
        
        baselinePreviousAverage = baselineCurrentAverage;
        // if (communicationMode) {
        //     sendIntValue(MIDI_SEND_BASELINE_CURRENT_AVERAGE, diff * BASELINE_MACRO_FACTOR);
        // }
    }
}

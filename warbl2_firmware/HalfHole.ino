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


//Always call UpperBound first, to calculate correction
uint16_t getHalfHoleLowerBound(byte hole) {
    // return getHalfHoleUpperBound(hole) - HALF_HOLE_WINDOW_SIZE;
    return toneholeCovered[hole] * (1.0f - HALF_HOLE_LOW_WINDOW_PERC) + halfHole.correction*HALF_HOLE_CALIB_OFFSET;

}

uint16_t getHalfHoleUpperBound(byte hole) {
    if (!isHalfHoleEnabled(hole)) {
        return 1024;
    }
    // return toneholeCovered[hole] - HOLE_OPEN_OFFSET + HALF_HOLE_UPPER_OFFSET + halfHole.correction*HALF_HOLE_CALIB_OFFSET;
    // return toneholeCovered[hole] - (HALF_HOLE_UPPER_OFFSET - halfHole.correction*HALF_HOLE_CALIB_OFFSET);
    return toneholeCovered[hole] * (1.0f - HALF_HOLE_HIGH_WINDOW_PERC) + halfHole.correction*HALF_HOLE_CALIB_OFFSET;
}

//returns id Half hole is enabled on a hole
bool isHalfHoleEnabled(int hole) {
    switch (hole) {
        case THUMB_HOLE:
            return switches[mode][HALF_HOLE_THUMB_ENABLED];
        case R3_HOLE:
            return switches[mode][HALF_HOLE_R3_ENABLED];
        case R4_HOLE:
            return switches[mode][HALF_HOLE_R4_ENABLED];
        default:
            return false;
    }
}
//returns if the selected hole is half covered
bool isHalfHole(int hole) {
    if (!isHalfHoleEnabled(hole)) return false; //disabled

    bool result = toneholeRead[hole] <  getHalfHoleUpperBound(hole) && toneholeRead[hole] > getHalfHoleLowerBound(hole);
#if DEBUG_HH_

    if (result) {
        Serial.print(getHalfHoleLowerBound(hole));
        Serial.print(" <- ");
        Serial.print(toneholeRead[hole]);
        Serial.print(" -> ");
        Serial.print(getHalfHoleLowerBound(hole));
        Serial.print(" - Max: ");
        Serial.println(toneholeCovered[hole]);
    } 
#endif

    return result;
}

//Calculate the current note shift based on HH settings and current fingering
int8_t getHalfHoleShift(unsigned int fingerPattern) {

    uint8_t result = 0;
    uint8_t tempCovered = fingerPattern >> 1; //To store finger holes only

    for (byte i = 0; i < TONEHOLE_SENSOR_NUMBER; i++) {
        if (isHalfHoleEnabled(i)) {
            switch (i) {

                case R3_HOLE: {
                    if ( (tempCovered & 0x7F) == 0b1111110 //To capture base position only, ignore thumb
                        && bitRead(fingerPattern, HALF_HOLE_BIT_OFFSET+i) == 1 //set by getFingers()
                        ) { 
                        result += 1;   
                    }
                    continue;
                }

                case R4_HOLE: {
                    if ((tempCovered & 0x7F) == 0b1111111 //To capture base position only, ignore thumb
                        && bitRead(fingerPattern, HALF_HOLE_BIT_OFFSET+i) == 1) { //set by getFingers()
                        result += 1;   
                    }
                    continue;
                }

                case THUMB_HOLE: {

                    result += 12 * bitRead(fingerPattern, HALF_HOLE_BIT_OFFSET);
                    continue;
                }
                default:
                    continue;
            }
        }
    }
    
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

#ifdef DEBUG_AUTO_CALIB
        Serial.print("halfHole.correction: ");
        Serial.print(halfHole.correction);
        Serial.print(" - thumbhole, window: ");
        Serial.print(getHalfHoleLowerBound(THUMB_HOLE));
        Serial.print(" <-> ");
        Serial.print(getHalfHoleUpperBound(THUMB_HOLE));
        Serial.print(", calibration: ");
        Serial.println(toneholeCovered[THUMB_HOLE]);
#endif
        
        baselinePreviousAverage = baselineCurrentAverage;
        // if (communicationMode) {
        //     sendIntValue(MIDI_SEND_BASELINE_CURRENT_AVERAGE, diff * BASELINE_MACRO_FACTOR);
        // }
    }
}

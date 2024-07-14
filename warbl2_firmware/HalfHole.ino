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




/* Returns the setpoint for the lower window (below which the hole is considered open)
* Always call UpperBound first, to calculate correction
*/
uint16_t getHalfHoleLowerBound(byte hole) {

#if BASELINE_AUTO_CALIBRATION
    return toneholeCovered[hole] * (1.0f - hh.lowWindowPerc) + hh.correction*HALF_HOLE_CALIB_OFFSET;
#else
    return toneholeCovered[hole] * (1.0f - hh.lowWindowPerc);
#endif

}



/*
* Returns the setpoint for the upper window (above which the hole is considered closed)
*/
uint16_t getHalfHoleUpperBound(byte hole) {
    if (!isHalfHoleEnabled(hole)) {
        return 1024;
    }
#if BASELINE_AUTO_CALIBRATION
    return toneholeCovered[hole] * (1.0f - hh.highWindowPerc) + hh.correction*HALF_HOLE_CALIB_OFFSET;
#else
    return toneholeCovered[hole] * (1.0f - hh.highWindowPerc);
#endif
}




/* 
* Returns if Half hole is enabled on a hole
*/
bool isHalfHoleEnabled(int hole) {

    return bitRead(hh.halfHoleSelector, hole) == 1;

}





/*
* Debounces half holes with hysteresis
* it works on holeCovered (current readings after getFingers())
*/
void debounceHalfHole() {

    for (byte i = 0; i < TONEHOLE_SENSOR_NUMBER; i++) {

        byte currentStatus = holeStatus(i); //Based on current sensor readings

        if (!isHalfHoleEnabled(i)) { //"Regular" holes
            hh.prevHoleStatus[i] = currentStatus;
            continue;  //we do nothing
        }

        // byte bitOffset = getHalfHoleBitOffset(i); //High bit for signaling half hole
        uint16_t readDelta = 0;
        bool confirmed = true;

        //When exiting half-hole, we modify the half-hole window by a small quantity, to make the passage less subject to oscillations
        if (currentStatus != hh.prevHoleStatus[i] && hh.prevHoleStatus[i] == HOLE_STATUS_HALF) { //From Half
            if (currentStatus == HOLE_STATUS_OPEN) { //to open
                readDelta = abs(getHalfHoleLowerBound(i) - toneholeRead[i]);
            } else { //to closed
                readDelta = abs(getHalfHoleUpperBound(i) - toneholeRead[i]);
            }
            confirmed = readDelta > HALF_HOLE_HYSTERESIS;
        }
        
        bool halfHoleNow = (currentStatus == HOLE_STATUS_HALF) || !confirmed; //The exit from HalfHole must be confirmed

        if (halfHoleNow) {
            if (i == THUMB_HOLE) {
                bitWrite(currentFP.fp.holes, i, switches[mode][HALF_HOLE_THUMB_INVERT]); //This decides the base note to be raised by an octave
            } else {
                bitWrite(currentFP.fp.holes, i, 1); //Otherwise Half-hole is always considered closed
            }
            bitWrite(currentFP.fp.halfHoles, i, 1); //To trigger fingering change

        } else {
            bitWrite(currentFP.fp.holes, i, currentStatus);  //Confirm open or closed
            bitWrite(currentFP.fp.halfHoles, i, 0);    //No half hole bit set
        }

#if DEBUG_HH
        if (currentStatus != hh.prevHoleStatus[i]) {

            Serial.print(millis());
            Serial.print(" --- Hole ");
            Serial.print(i);
        
            Serial.print(" Status: ");
            printFingering(currentFP);

            if (readDelta == 0) { //Entering Half
                Serial.print(" Entering: ");
            } else {
                Serial.print(" Exiting - delta: ");
                Serial.print(readDelta);
                Serial.print(" - ");
            }
            Serial.print(hh.prevHoleStatus[i]);
            Serial.print("->");
            Serial.print(currentStatus);
            Serial.println(" ---");

            Serial.println("");
        }
#endif
        hh.prevHoleStatus[i] = currentStatus; //Store for next debounce iteration
    }
}




/* 
 * Returns status for the hole. 
 * If finger pattern is provided, it bases its return value on it
 * otherwise, it is based on current sensor readings
 */
byte holeStatus(byte hole, fingering_pattern_union_t fingerPattern) {
    
    bool halfHoleEnabled = isHalfHoleEnabled(hole);

    if (fingerPattern.holeCovered < 0xFFFFFFFF) { //analyze fingerPattern - see prototype declaration in HalfHole.h
        byte bitStatus = bitRead(fingerPattern.holeCovered, hole);
        if (halfHoleEnabled) {
            if (bitRead(fingerPattern.fp.halfHoles, hole)) {
                return HOLE_STATUS_HALF;
            }
        } 

        return bitStatus;

    } else { //analyze current sensor readings
        if (halfHoleEnabled) {
            uint16_t lowWindowPoint = getHalfHoleLowerBound(hole);
            uint16_t highWindowPoint = getHalfHoleUpperBound(hole);
            if (toneholeRead[hole] <  highWindowPoint && toneholeRead[hole] > lowWindowPoint) {
#if DEBUG_HH && DEBUG_VERBOSE && DEBUG_HH_VERBOSE
                Serial.print(hole);
                Serial.print(": ");
                Serial.print(getHalfHoleLowerBound(hole));
                Serial.print(" <- ");
                Serial.print(toneholeRead[hole]);
                Serial.print(" -> ");
                Serial.print(getHalfHoleUpperBound(hole));
                Serial.print(" - Max: ");
                Serial.println(toneholeCovered[hole]);
#endif
                return HOLE_STATUS_HALF;
            }
            if (toneholeRead[hole] >= highWindowPoint) {
                return HOLE_STATUS_CLOSED;
            } else if (toneholeRead[hole] <= lowWindowPoint) {
                return HOLE_STATUS_OPEN;
            }
        }

        // "Regular" tonehole - it's the same as in getFingers()
        if (toneholeRead[hole] > (toneholeCovered[hole] - HOLE_COVERED_OFFSET)) {
            return HOLE_STATUS_CLOSED;
        } else if (toneholeRead[hole] <= (toneholeCovered[hole] - HOLE_OPEN_OFFSET)) {
            return HOLE_STATUS_OPEN;
        } else {  // the "hole uncovered" reading is a little less then the "hole covered" reading, to prevent oscillations. In case, we just read the bit in current holeCovered
            return holeStatus(hole, currentFP);
        }
    }
}




/*
* Returns if the selected hole is half covered
*/
bool isHalfHole(int hole) {
    return holeStatus(hole) == HOLE_STATUS_HALF; //Based on current sensor readings
}



/*
* Calculates the current note shift based on HH settings and current fingering
*/
int8_t getHalfHoleShift(fingering_pattern_union_t fingerPattern) {

    uint8_t result = 0;
    uint16_t tempCovered = fingerPattern.fp.holes >> 1; //To store finger holes only

    for (byte i = 0; i < TONEHOLE_SENSOR_NUMBER; i++) {
        if (isHalfHoleEnabled(i)) {
            switch (i) {

                case R4_HOLE: {
                    if ((tempCovered & 0x7F) == 0b1111111 //To capture base position only, ignore thumb
                        && bitRead(fingerPattern.fp.halfHoles, i) == 1) { //set by debouceHalfHole()
                        result += 1;   
                    }
                    continue;
                }

                case R3_HOLE: {
                    if ( (tempCovered & 0x7F) == 0b1111110 //To capture base position only, ignore thumb
                        && bitRead(fingerPattern.fp.halfHoles, i) == 1 //set by debouceHalfHole()
                        ) { 
                        result += 1;   
                    }
                    continue;
                }
                case R2_HOLE: {
                    if ((tempCovered & 0x7F) == 0b1111100 //To capture base position only, ignore thumb
                        && bitRead(fingerPattern.fp.halfHoles, i) == 1) { //set by debouceHalfHole()
                        result += 1;   
                    }
                    continue;
                }
                case R1_HOLE: {
                    if ((tempCovered & 0x7F) == 0b1111000 //To capture base position only, ignore thumb
                        && bitRead(fingerPattern.fp.halfHoles, i) == 1) { //set by debouceHalfHole()
                        result += 1;   
                    }
                    continue;
                }
                case L3_HOLE: {
                    if ((tempCovered & 0x7F) == 0b1110000 //To capture base position only, ignore thumb
                        && bitRead(fingerPattern.fp.halfHoles, i) == 1) { //set by debouceHalfHole()
                        result += 1;   
                    }
                    continue;
                }
                case L2_HOLE: {
                    if ((tempCovered & 0x7F) == 0b1100000 //To capture base position only, ignore thumb
                        && bitRead(fingerPattern.fp.halfHoles, i) == 1) { //set by debouceHalfHole()
                        result += 1;   
                    }
                    continue;
                }
                case L1_HOLE: {
                    if ((tempCovered & 0x7F) == 0b1000000 //To capture base position only, ignore thumb
                        && bitRead(fingerPattern.fp.halfHoles, i) == 1) { //set by debouceHalfHole()
                        result += 1;   
                    }
                    continue;
                }
                case THUMB_HOLE: {
                    result += 12 * bitRead(fingerPattern.fp.halfHoles, i);
                    continue;
                }

                default:
                    continue;
            }
        }
    }
    
    return result;
}




#if BASELINE_AUTO_CALIBRATION
/*
* Inits baseline calibration data for auto calibration
*/
void baselineInit() {
#if DEBUG_AUTO_CALIB
        Serial.println("BL init. ");
#endif
    ac.baselineAverage = 0;
    for (byte i = 0; i < TONEHOLE_SENSOR_NUMBER; i++) { 

#if DEBUG_AUTO_CALIB
        Serial.print("hole: ");
        Serial.print(i);
        Serial.print(" value: ");
        Serial.println(toneholeBaseline[i]);
#endif
        ac.baselineAverage += toneholeBaseline[i];    //Calculates baseline Average

        if (toneholeBaseline[i] > ac.maxBaseline) { 
            ac.maxBaseline = toneholeBaseline[i];
        }
    }
    ac.baselineAverage = (ac.baselineAverage / (float)(TONEHOLE_SENSOR_NUMBER-1));

#if DEBUG_AUTO_CALIB
        Serial.print("BL init end. Average: ");
        Serial.print(ac.baselineAverage);
        Serial.print(" maxBaseline: ");
        Serial.println(ac.maxBaseline);
#endif
}



/*
* Calculates the baseline moving average for readings in the current time window and applies an eventual correction factor to half hole detection
*/
void baselineUpdate() {
    byte counter = 0;
    ac.baselineCurrentAverage = 0;
    for (byte i = 0; i<TONEHOLE_SENSOR_NUMBER; i++) {

        if (ac.toneholeBaselineCurrent[i] >= 0 && ac.toneholeBaselineCurrent[i] <= ac.maxBaseline)  { //LPF
#if DEBUG_AUTO_CALIB
        Serial.print("hole: ");
        Serial.print(i);
        Serial.print(" value: ");
        Serial.println(ac.toneholeBaselineCurrent[i]);
#endif
            ac.baselineCurrentAverage += ac.toneholeBaselineCurrent[i];
            counter++;
        }
        ac.toneholeBaselineCurrent[i] = 1024; //Resets for next run

        //Updates baseline too with previous average - Useful for PB
        // toneholeBaseline[i] = ((float) toneholeBaseline[i] * ac.baselinePreviousAverage/ac.baselineAverage);
    }
    if (counter > 0 ) { //Found usable values
        //Calculates current (quasi)autoregressive moving average - baseLine Average. It measures the current ambient light, more or less
        ac.baselineCurrentAverage = (BASELINE_AVRG_SPEED*((ac.baselineCurrentAverage / (float) counter)) + (1.0-BASELINE_AVRG_SPEED)*ac.baselinePreviousAverage);
        
        float diff = ac.baselineCurrentAverage  - ac.baselineAverage;
        hh.correction = sqrt(abs(diff)); //We don't want a linear reaction
        if (diff < 0)  hh.correction *= -1;

#if DEBUG_AUTO_CALIB
        Serial.print("BL - Average: ");
        Serial.print(ac.baselineAverage);
        Serial.print(" - CurrentAverage: ");
        Serial.print(ac.baselineCurrentAverage);       
        Serial.print(" - diff: ");
        Serial.print(diff);       
        Serial.print(" - hh.correction: ");
        Serial.print(hh.correction);
        Serial.print(" - thumbhole, BL: ");
        Serial.print(toneholeBaseline[THUMB_HOLE]);
        Serial.print(" - window: ");
        Serial.print(getHalfHoleLowerBound(THUMB_HOLE));
        Serial.print(" <-> ");
        Serial.print(getHalfHoleUpperBound(THUMB_HOLE));
        Serial.print(", covered: ");
        Serial.println(toneholeCovered[THUMB_HOLE]);
#endif
        
        ac.baselinePreviousAverage = ac.baselineCurrentAverage;

        //In the config tool, this value che be used to show current light conditions with respect to light conditions at the time of calibration
        //the relative code from config Tool has been removed
        if (communicationMode) {
            byte lightCondition = 1; //Within baseline initial calibration
            if (hh.correction < -0.10) lightCondition = 0; //Darker
            else if (hh.correction > 1.0) lightCondition = 2; //Lighter

            // sendMIDICouplet(MIDI_SEND_LIGHT_CONDITION, lightCondition); // Send current correction.
        }
    }
}
#endif

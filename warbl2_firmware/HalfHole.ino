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

    return toneholeCovered[hole] * (1.0f - hh.lowWindowPerc);

}



/*
* Returns the setpoint for the upper window (above which the hole is considered closed)
*/
uint16_t getHalfHoleUpperBound(byte hole) {
    if (!isHalfHoleEnabled(hole)) {
        return 1024;
    }

    return toneholeCovered[hole] * (1.0f - hh.highWindowPerc);
}




/* 
* Returns if Half hole is enabled on a hole
*/
bool isHalfHoleEnabled(int hole) {

    if (hole == BELL_HOLE) return false;

    return bitRead(hh.halfHoleSelector, hole) == 1;

}





/*
* Debounces half holes with hysteresis
* it works on holeCovered (current readings after getFingers())
*/
void debounceHalfHole() {

    for (byte i = R4_HOLE; i <= THUMB_HOLE; i++) {

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
                bitWrite(currentFP.fp.holeCovered, i, switches[mode][HALF_HOLE_THUMB_INVERT]); //This decides the base note to be raised by an octave
            } else {
                bitWrite(currentFP.fp.holeCovered, i, 1); //Otherwise Half-hole is always considered closed
            }
            bitWrite(currentFP.fp.halfHoles, i, 1); //To trigger fingering change

        } else {
            bitWrite(currentFP.fp.holeCovered, i, currentStatus);  //Confirm open or closed
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
 * Returns open/closed base status for the hole, base in toneholeRead
 * it is the one previuosly used in getFingers
 * now we call it from three different places
 */
byte holeBaseStatus(byte hole) {
    
    if (toneholeRead[hole] > (toneholeCovered[hole] - HOLE_COVERED_OFFSET)) {
        return HOLE_STATUS_CLOSED;
    } else if (toneholeRead[hole] <= (toneholeCovered[hole] - HOLE_OPEN_OFFSET)) {
        return HOLE_STATUS_OPEN;
    } else {  // the "hole uncovered" reading is a little less then the "hole covered" reading, to prevent oscillations. In this case, we return a N.D. status
        return HOLE_STATUS_ND;
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
        byte status = holeBaseStatus(hole);
        if (status < HOLE_STATUS_HALF) {
            return status;
        } else { //In case of HOLE_STATUS_ND, we just read the bit in current holeCovered
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
    uint16_t tempCovered = fingerPattern.fp.holeCovered >> 1; //To store finger holes only

    for (byte i = R4_HOLE; i <= THUMB_HOLE; i++) {
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



/*
* Inits auto calibration data 
*/
void autoCalibrationInit() {
    for (byte i = R4_HOLE; i <= THUMB_HOLE; i++) { 
        ac.currentAverage[i] = toneholeCovered[i];
        ac.previousAverage[i] = ac.currentAverage[i];
    }
}

/*
* Calculates the baseline moving average for readings in the current time window and applies an eventual correction factor to half hole detection
*/
void calibrationUpdate() {

#if DEBUG_AUTO_CALIB
    Serial.print("---");
    Serial.println(millis());
#endif

    xSemaphoreTake(ac.mutex, portMAX_DELAY);

    for (byte i = R4_HOLE; i <= THUMB_HOLE; i++) {

        if (ac.toneholeCoveredSampleCounter[i] >= AUTO_CALIB_MIN_SAMPLES && ac.toneholeCoveredCurrentSum[i] > 0) {
            float currentAverage = ac.toneholeCoveredCurrentSum[i] / (float) ac.toneholeCoveredSampleCounter[i];
            
            //Calculates current (quasi)autoregressive moving average.
            ac.currentAverage[i] = (AUTO_CALIB_AVRG_SPEED*currentAverage + (1.0f-AUTO_CALIB_AVRG_SPEED)*ac.previousAverage[i]);
        

#if DEBUG_AUTO_CALIB
            float diff = ac.currentAverage[i]  - toneholeCovered[i];
            Serial.print(" hole: ");
            Serial.print(i);
            Serial.print(" - samples: ");
            Serial.print(ac.toneholeCoveredSampleCounter[i]);
            Serial.print(" - ");
            Serial.print(toneholeCovered[i]);
            Serial.print(" -> ");
            Serial.print(currentAverage);
            Serial.print(" - ");
            Serial.print(ac.currentAverage[i]);
            Serial.print(" d: ");
            Serial.println(diff);
#endif
            toneholeCovered[i] = ac.currentAverage[i];

            ac.toneholeCoveredCurrentSum[i] = 0; //Resets for next run
            ac.toneholeCoveredSampleCounter[i] = 0;  //Resets for next run
        }
    }
    xSemaphoreGive(ac.mutex);
}


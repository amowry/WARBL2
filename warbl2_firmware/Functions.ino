


// Debug
void printStuff(void) {

    //Serial.println(thumbHalfHole);
    //Serial.println(ED[mode][HALFHOLE_FINGERRATE]);
    //Serial.println("");

    //static float CPUtemp = readCPUTemperature(); // If needed for something like calibrating sensors. Can also use IMU temp. The CPU is in the middle of the PCB and the IMU is near the mouthpiece.

    //This can be used to print out the currently selected fingering chart (all 256 possible values) when button 1 is clicked.
    /*
    if (digitalRead(4) == 0) {
        for (unsigned int i = 0; i < 256; i++) {
            Serial.print(getNote(i << 1));
            Serial.print(" ");
        }
        delay(5000);
    } 
*/
}








// Determine how long to sleep each time through the loop.
byte calculateDelayTime(void) {

    byte delayCorrection = (millis() - wakeTime);  // Include a correction factor to reduce jitter if something else in the loop or the radio interrupt has eaten some time.
                                                   // The main thing that adds jitter to the loop is sending lots of data over BLE, i.e. pitchbend, CC, etc. (though it's typically not noticeable). Being connected to the Config Tool is also a significant source of jitter.
                                                   // With USB MIDI only there is virtually no jitter with a loop period of 2 ms.
    //if (delayCorrection > 3) { Serial.println(delayCorrection); }  // Print the amount of time that other things have consumed.

    byte delayTime = 3;

    if (connIntvl == 0) {  // Use a 2 ms sleep instead if we are only using USB MIDI.
        delayTime = 2;
    }

    if (delayCorrection < delayTime) {  // If we haven't used up too much time since the last time through the loop, we can sleep for a bit.
        delayTime = delayTime - delayCorrection;
        return delayTime;
    }
    return 0;
}








// Read the pressure sensor and get latest tone hole readings from the ATmega.
void getSensors(void) {

    analogPressure.update();  //  Read the pressure sensor now while the ATmega is still asleep and the board is very quiet.
    twelveBitPressure = analogPressure.getRawValue();
    smoothed_pressure = analogPressure.getValue();  // Use an adaptively smoothed 12-bit reading to map to CC, aftertouch, poly.
    sensorValue = twelveBitPressure >> 2;           // Reduce the reading to stable 10 bits for state machine.

    // Receive tone hole readings from ATmega32U4. The transfer takes ~ 125 us.
    byte toneholePacked[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
    digitalWrite(2, LOW);   // CS -- Wake up ATmega.
    delayMicroseconds(10);  // Give it time to wake up.
    SPI.transfer(0);        // We don't receive anything useful back from the first transfer.
    for (byte i = 0; i < 12; i++) {
        toneholePacked[i] = SPI.transfer(i + 1);
    }
    digitalWrite(2, HIGH);  //CS
    SPI.endTransaction();

    // Unpack the readings from bytes to ints.
    for (byte i = 0; i < 9; i++) {
        toneholeRead[i] = toneholePacked[i];  // Unpack lower 8 bits.
    }

    for (byte i = 0; i < 4; i++) {  // Unpack the upper 2 bits of holes 0-3.
        toneholePacked[9] = toneholePacked[9] & 0b11111111, BIN;
        toneholeRead[i] = toneholeRead[i] | (toneholePacked[9] << 2) & 0b1100000000;
        toneholePacked[9] = toneholePacked[9] << 2;
    }

    for (byte i = 4; i < 8; i++) {  // Unpack the upper 2 bits of holes 4-7.
        toneholePacked[10] = toneholePacked[10] & 0b11111111, BIN;
        toneholeRead[i] = toneholeRead[i] | (toneholePacked[10] << 2) & 0b1100000000;
        toneholePacked[10] = toneholePacked[10] << 2;
    }

    toneholeRead[8] = toneholeRead[8] | (toneholePacked[11] << 8);  // Unpack the upper 2 bits of hole 8.



    for (byte i = 0; i < 9; i++) {
        if (calibration == 0) {  // If we're not calibrating, compensate for baseline sensor offset (the stored sensor reading with the hole completely uncovered).
            toneholeRead[i] = toneholeRead[i] - toneholeBaseline[i];
        }
        if (toneholeRead[i] < 0) {  // In rare cases the adjusted readings might end up being negative.
            toneholeRead[i] = 0;
        }
    }
}









void readIMU(void) {

    // Note: Gyro is turned off by default to save power unless using roll/pitch/yaw, elevation register, or sticks mode (see loadPrefs()).

    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    sox.getEvent(&accel, &gyro, &temp);

    rawGyroX = gyro.gyro.x;
    rawGyroY = gyro.gyro.y;
    rawGyroZ = gyro.gyro.z;
    accelX = accel.acceleration.x;
    accelY = accel.acceleration.y;
    accelZ = accel.acceleration.z;
    IMUtemp = temp.temperature;

    // Calibrate gyro
    gyroX = rawGyroX - gyroXCalibration;
    gyroY = rawGyroY - gyroYCalibration;
    gyroZ = rawGyroZ - gyroZCalibration;

    float deltat = sfusion.deltatUpdate();
    deltat = constrain(deltat, 0.001f, 0.01f);

    // Serial.println(deltat, 4);

    sfusion.MahonyUpdate(gyroX, gyroY, gyroZ, accelX, accelY, accelZ, deltat);

    // Pitch and roll are swapped due to PCB sensor orientation.
    roll = sfusion.getPitchRadians();
    pitch = sfusion.getRollRadians();
    yaw = sfusion.getYawRadians();


#if 0
    float * quat = sfusion.getQuat();
    for (int i=0; i < 4; ++i) {
        Serial.print(quat[i], 4);
        Serial.print(", ");
    }
    Serial.println(" 1.1, -1.1");
#endif


    currYaw = yaw;  // Needs to be the unadjusted value

    yaw += yawOffset;
    if (yaw > PI) yaw -= TWO_PI;
    else if (yaw < -PI) yaw += TWO_PI;

    // Adjust pitch so it makes more sense for way warbl is held, shift it 180 deg
    pitch += PI;
    if (pitch > PI) pitch -= TWO_PI;

    // Invert all three.
    roll = -roll;
    pitch = -pitch;
    yaw = -yaw;

    roll = roll * RAD_TO_DEG;
    pitch = pitch * RAD_TO_DEG;
    yaw = yaw * RAD_TO_DEG;



#if 1
    // Integrate gyroY without accelerometer to get roll in the local frame (around the long axis of the WARBL regardless of orientation). This seems more useful/intuitive than the "roll" Euler angle.
    static float rollLocal = roll;  // Initialize using global frame.
    static float correctionFactor;
    const byte correctionRate = 40;  // How quickly we correct to the gravity vector when roll and rollLocal have opposite signs.
    static float prevYaw;
    static float prevRoll;

    if ((signbit(yaw - prevYaw) == signbit(roll - prevRoll) && pitch <= 0) || (signbit(yaw - prevYaw) != signbit(roll - prevRoll) && pitch > 0) || (abs(pitch) >= 85)) {  // Only integrate gyro if yaw is changing in the same direction as roll (or at a steep pitch angle). This helps minimize the influence of yaw on rollLocal. This could be improved.
        rollLocal += ((gyroY * RAD_TO_DEG) * deltat);                                                                                                                     // Integrate gyro Y axis.
    }
    prevYaw = yaw;
    prevRoll = roll;

    if (signbit(rollLocal) != signbit(roll)) {  // If roll and rollLocal have opposite signs, nudge rollLocal towards zero, aligning the zero crossing point with gravity.
        correctionFactor = rollLocal / correctionRate;

    }

    else {  // No correction if they have the same sign.
        correctionFactor = 0;
    }
    if (abs(pitch) >= 85) {  // Correcting for gravity gets sketchy if pitch is near vertical, so don't apply a correction factor then.
        correctionFactor = 0;
    }

    rollLocal -= correctionFactor;
    roll = rollLocal;
#endif


    // Drumstick mode: WARBL2 must be held by the USB end, with the button side up. No note off messages are sent.
    if (IMUsettings[mode][STICKS_MODE]) {
        static bool armed = 0;
        static float maxGyro;
        static byte LEDCounter = 0;  // Time how long the LED is on.

        if (LEDCounter > 0) {  // Count up if the LED is on.
            LEDCounter++;
        };

        if (LEDCounter > 5) {  // Turn off LED after a bit.
            analogWrite(LEDpins[GREEN_LED], 0);
            analogWrite(LEDpins[BLUE_LED], 0);

            LEDCounter = 0;
        }

        if (gyroX > 0.5f) {
            armed = true;                              // Detect forward rotation above a threshold to prepare for a hit.
            if (gyroX > maxGyro) { maxGyro = gyroX; }  // Find the fastest X rotation, to use for hit velocity.
            else if (maxGyro > 0) {
                maxGyro -= 0.7f;  // Gradually reduce the velocity analog if the rotation is slowing. This adjusts the velocity and/or prevents a hit if you start out with a fast swing but then slow it before the rebound occurs.
            }
            if (maxGyro <= 0) {
                armed = false;
            }
        }
        if (gyroX < 0 && armed == true) {                                    // A hit occurs if we are armed and the X gyro goes negative (indicating a slight rebound).
            byte hitVelocity = 12 * (constrain(maxGyro + 1, 1.0f, 10.58f));  // Scale the velocity up to 0-127.
                                                                             // Just in case
            sendMIDI(NOTE_ON, mainMidiChannel, yawOutput, hitVelocity);      // Use yawOutput for MIDI note.
            analogWrite(LEDpins[GREEN_LED], hitVelocity << 3);               // Fire teal LED to indicate a hit, with brightness based on note velocity.
            analogWrite(LEDpins[BLUE_LED], hitVelocity << 1);
            LEDCounter = 1;  // Start counting to turn the LED off again.
            armed = false;   // Don't allow another hit until we've passed the threshold again.
            maxGyro = 0;
            powerDownTimer = pitchBendTimer;  // Reset the powerDown timer because we're sending notes.
        }
    }
}








// Calibrate the gyroscope when the command is received from the Config Tool.
void calibrateIMU() {

    sox.setGyroDataRate(LSM6DS_RATE_208_HZ);  // Make sure the gyro is on.
    delay(50);                                // Give it a bit of time.
    readIMU();                                // Get a reading in case we haven't been reading it.
    delay(50);                                // Give it a bit of time.
    readIMU();                                // Another reading seems to be necessary after turning the gyro on.

    gyroXCalibration = rawGyroX;
    gyroYCalibration = rawGyroY;
    gyroZCalibration = rawGyroZ;

    putEEPROM(EEPROM_XGYRO_CALIB_START, gyroXCalibration);  // Put the current readings into EEPROM.
    putEEPROM(EEPROM_YGYRO_CALIB_START, gyroYCalibration);
    putEEPROM(EEPROM_ZGYRO_CALIB_START, gyroZCalibration);
    writeEEPROM(EEPROM_XGYRO_CALIB_SAVED, 3);  // Remember that we have saved a calibration.
}









// Reset heading 0 to current heading
void centerIMU() {
    yawOffset = -currYaw;
}








// Map IMU to CC and send
void sendIMU() {

    static byte prevRollCC = 0;
    static byte prevPitchCC = 0;
    static byte prevYawCC = 0;

    // The min and max settings from the Config Tool range from 0-36 and are scaled up to the maximum range of angles for each DOF.

    if (IMUsettings[mode][SEND_ROLL]) {

        byte lowerConstraint;
        byte upperConstraint;

        if (IMUsettings[mode][ROLL_OUTPUT_MIN] > IMUsettings[mode][ROLL_OUTPUT_MAX]) {  // Flip the constraints if the lower output is greater than the upper output, so the output can be inverted.
            lowerConstraint = IMUsettings[mode][ROLL_OUTPUT_MAX];
            upperConstraint = IMUsettings[mode][ROLL_OUTPUT_MIN];
        }

        else {
            lowerConstraint = IMUsettings[mode][ROLL_OUTPUT_MIN];
            upperConstraint = IMUsettings[mode][ROLL_OUTPUT_MAX];
        }

        byte rollOutput = constrain(map((roll + 90) * 1000, IMUsettings[mode][ROLL_INPUT_MIN] * 5000, IMUsettings[mode][ROLL_INPUT_MAX] * 5000, IMUsettings[mode][ROLL_OUTPUT_MIN], IMUsettings[mode][ROLL_OUTPUT_MAX]), lowerConstraint, upperConstraint);

        if (prevRollCC != rollOutput) {
            sendMIDI(CONTROL_CHANGE, IMUsettings[mode][ROLL_CC_CHANNEL], IMUsettings[mode][ROLL_CC_NUMBER], rollOutput);
            prevRollCC = rollOutput;
        }
    }



    if (IMUsettings[mode][SEND_PITCH]) {

        byte lowerConstraint;
        byte upperConstraint;

        if (IMUsettings[mode][PITCH_OUTPUT_MIN] > IMUsettings[mode][PITCH_OUTPUT_MAX]) {  // Flip the constraints if the lower output is greater than the upper output, so the output can be inverted.
            lowerConstraint = IMUsettings[mode][PITCH_OUTPUT_MAX];
            upperConstraint = IMUsettings[mode][PITCH_OUTPUT_MIN];
        }

        else {
            lowerConstraint = IMUsettings[mode][PITCH_OUTPUT_MIN];
            upperConstraint = IMUsettings[mode][PITCH_OUTPUT_MAX];
        }

        byte pitchOutput = constrain(map((pitch + 90) * 1000, IMUsettings[mode][PITCH_INPUT_MIN] * 5000, IMUsettings[mode][PITCH_INPUT_MAX] * 5000, IMUsettings[mode][PITCH_OUTPUT_MIN], IMUsettings[mode][PITCH_OUTPUT_MAX]), lowerConstraint, upperConstraint);

        if (prevPitchCC != pitchOutput) {
            sendMIDI(CONTROL_CHANGE, IMUsettings[mode][PITCH_CC_CHANNEL], IMUsettings[mode][PITCH_CC_NUMBER], pitchOutput);
            prevPitchCC = pitchOutput;
        }
    }



    if (IMUsettings[mode][SEND_YAW] || IMUsettings[mode][STICKS_MODE]) {

        byte lowerConstraint;
        byte upperConstraint;

        if (IMUsettings[mode][YAW_OUTPUT_MIN] > IMUsettings[mode][YAW_OUTPUT_MAX]) {  // Flip the constraints if the lower output is greater than the upper output, so the output can be inverted.
            lowerConstraint = IMUsettings[mode][YAW_OUTPUT_MAX];
            upperConstraint = IMUsettings[mode][YAW_OUTPUT_MIN];
        }

        else {
            lowerConstraint = IMUsettings[mode][YAW_OUTPUT_MIN];
            upperConstraint = IMUsettings[mode][YAW_OUTPUT_MAX];
        }

        yawOutput = constrain(map((yaw + 180) * 1000, IMUsettings[mode][YAW_INPUT_MIN] * 10000, IMUsettings[mode][YAW_INPUT_MAX] * 10000, IMUsettings[mode][YAW_OUTPUT_MIN], IMUsettings[mode][YAW_OUTPUT_MAX]), lowerConstraint, upperConstraint);

        if (prevYawCC != yawOutput && IMUsettings[mode][SEND_YAW]) {
            sendMIDI(CONTROL_CHANGE, IMUsettings[mode][YAW_CC_CHANNEL], IMUsettings[mode][YAW_CC_NUMBER], yawOutput);
            prevYawCC = yawOutput;
        }
    }
}









// Use the Y accelerometer axis with gravity removed for vibrato.
void shakeForVibrato() {

    static float accelFilteredOld;
    const float timeConstant = 0.1f;
    float accelFiltered = timeConstant * accelY + (1.0f - timeConstant) * accelFilteredOld;  // Low-pass filter to isolate gravity from the Y accelerometer axis.
    accelFilteredOld = accelFiltered;
    float highPassY = accelY - accelFiltered;  // Subtract gravity to high-pass Y.

    static float accelFilteredBOld;
    const float timeConstant2 = 0.5f;
    float accelFilteredB = timeConstant2 * highPassY + (1.0f - timeConstant2) * accelFilteredBOld;  // A second low pass filter to minimize spikes from tapping the tone holes.
    float lastFilteredB = accelFilteredBOld;
    accelFilteredBOld = accelFilteredB;

    //accelFilteredB = highPassY;  // (Don't) temporarily eliminate this lowpass to see if speeds up response noticeably.

    const float shakeBendDepth = 4.0f * IMUsettings[mode][Y_PITCHBEND_DEPTH] / 100.0f;                   // Adjust the vibrato depth range based on the Config Tool setting.
    const float shakeModCCDepth = 64.0f * IMUsettings[mode][Y_SHAKE_MOD_CC_DEPTH] / 100.0f;              // Adjust the pressure CC out depth range based on the Config Tool setting.
    const float shakeModChanPressDepth = 64.0f * IMUsettings[mode][Y_SHAKE_MOD_CHPRESS_DEPTH] / 100.0f;  // Adjust the pressure ChanPress out depth range based on the Config Tool setting.
    const float shakeModKeyPressDepth = 64.0f * IMUsettings[mode][Y_SHAKE_MOD_KEYPRESS_DEPTH] / 100.0f;  // Adjust the pressure KeyPress out depth range based on the Config Tool setting.
    const float kShakeStartThresh = 0.5f;
    const float kShakeFinishThresh = 0.35f;
    const long kShakeFinishTimeMs = 400;
    const long ktapFilterTimeMs = 12;  // ms window for further filtering out taps on the tone holes
    static bool shakeActive = false;
    static bool tapFilterActive = false;
    static long lastThreshExceedTime = 0;
    static long tapFilterStartTime = 0;
    static bool preTrigger = false;
    const float kShakeGestureThresh = 20.0f;  // Theshold for shake gesture
    unsigned long nowtime = millis();


    if (abs(accelFilteredB) > kShakeGestureThresh) {  // This is for the shake gesture "button" assignment.
        shakeDetected = true;
    } else {
        shakeDetected = false;
    }

    bool doShake = (IMUsettings[mode][Y_SHAKE_PITCHBEND] || IMUsettings[mode][Y_SHAKE_MOD_CC] || IMUsettings[mode][Y_SHAKE_MOD_CHPRESS] || IMUsettings[mode][Y_SHAKE_MOD_KEYPRESS]);
    if (doShake) {  // Do this part only if one of the shake mod options is turned on.

        shakeVibrato = 0;
        shakePressureCCMod = 0;
        shakePressureChanPressMod = 0;
        shakePressureKeyPressMod = 0;

        if (tapFilterActive == false && abs(accelFilteredB) > kShakeStartThresh) {
            tapFilterActive = true;
            tapFilterStartTime = nowtime;
        }

        if ((nowtime - tapFilterStartTime) < ktapFilterTimeMs) {  // Return if we haven't waited long enough after crossing the shake threshold, for further filtering brief tone hole taps.
            return;
        }

        if (!preTrigger && abs(accelFilteredB) > kShakeStartThresh) {
            preTrigger = true;
        }

        if (!shakeActive && preTrigger  // Start shake vib only when doing a zero crossing after threshold has been triggered.
            // Comment out next line to test without the zero crossing requirement
            //&& ((lastFilteredB <= 0.0f && accelFilteredB > 0.0f) || (lastFilteredB > 0.0f && accelFilteredB <= 0.0f))
        ) {
            shakeActive = true;
            lastThreshExceedTime = nowtime;
            // Serial.println("Start ShakeVib");
        } else if (abs(accelFilteredB) > kShakeFinishThresh) {
            lastThreshExceedTime = nowtime;
        }

        if ((shakeActive || tapFilterActive) && (nowtime - lastThreshExceedTime) > kShakeFinishTimeMs) {
            // Stop shake vibrato.
            shakeActive = false;
            tapFilterActive = false;
            preTrigger = false;
        }


        if (shakeActive) {  // Normalize and clip, +/-15 input seems to be reasonably realistic max accel while still having it in the mouth!
            const float basenormshake = constrain(accelFilteredB * 0.06666f, -1.0f, 1.0f);
            float normshake = basenormshake;

            if (IMUsettings[mode][Y_PITCHBEND_MODE] == Y_PITCHBEND_MODE_UPDOWN) {
                normshake *= -1.0f;  // reverse phase
            } else if (IMUsettings[mode][Y_PITCHBEND_MODE] == Y_PITCHBEND_MODE_DOWNONLY) {
                normshake = constrain(normshake, -1.0f, 0.0f);
            } else if (IMUsettings[mode][Y_PITCHBEND_MODE] == Y_PITCHBEND_MODE_UPONLY) {
                normshake = constrain(-1.0f * normshake, 0.0f, 1.0f);
            } else if (IMUsettings[mode][Y_PITCHBEND_MODE] == Y_PITCHBEND_MODE_DOWNONLY_INV) {
                normshake = constrain(-1.0f * normshake, -1.0f, 0.0f);
            } else if (IMUsettings[mode][Y_PITCHBEND_MODE] == Y_PITCHBEND_MODE_UPONLY_INV) {
                normshake = constrain(normshake, 0.0f, 1.0f);
            }

            if (IMUsettings[mode][Y_SHAKE_PITCHBEND]) {
                shakeVibrato = (int)(normshake * shakeBendDepth * pitchBendPerSemi);
            }

            if (IMUsettings[mode][Y_SHAKE_MOD_CC]) {
                float normmodshake = basenormshake;
                if (IMUsettings[mode][Y_SHAKE_MOD_CC_MODE] == Y_PITCHBEND_MODE_UPDOWN) {
                    normmodshake *= -1.0f;  // reverse phase
                } else if (IMUsettings[mode][Y_SHAKE_MOD_CC_MODE] == Y_PITCHBEND_MODE_DOWNONLY) {
                    normmodshake = constrain(normmodshake, -1.0f, 0.0f);
                } else if (IMUsettings[mode][Y_SHAKE_MOD_CC_MODE] == Y_PITCHBEND_MODE_UPONLY) {
                    normmodshake = constrain(-1.0f * normmodshake, 0.0f, 1.0f);
                } else if (IMUsettings[mode][Y_SHAKE_MOD_CC_MODE] == Y_PITCHBEND_MODE_DOWNONLY_INV) {
                    normmodshake = constrain(-1.0f * normmodshake, -1.0f, 0.0f);
                } else if (IMUsettings[mode][Y_SHAKE_MOD_CC_MODE] == Y_PITCHBEND_MODE_UPONLY_INV) {
                    normmodshake = constrain(normmodshake, 0.0f, 1.0f);
                }
                shakePressureCCMod = (int)(normmodshake * shakeModCCDepth);
            }
            if (IMUsettings[mode][Y_SHAKE_MOD_CHPRESS]) {
                float normmodshake = basenormshake;
                if (IMUsettings[mode][Y_SHAKE_MOD_CHPRESS_MODE] == Y_PITCHBEND_MODE_UPDOWN) {
                    normmodshake *= -1.0f;  // reverse phase
                } else if (IMUsettings[mode][Y_SHAKE_MOD_CHPRESS_MODE] == Y_PITCHBEND_MODE_DOWNONLY) {
                    normmodshake = constrain(normmodshake, -1.0f, 0.0f);
                } else if (IMUsettings[mode][Y_SHAKE_MOD_CHPRESS_MODE] == Y_PITCHBEND_MODE_UPONLY) {
                    normmodshake = constrain(-1.0f * normmodshake, 0.0f, 1.0f);
                } else if (IMUsettings[mode][Y_SHAKE_MOD_CHPRESS_MODE] == Y_PITCHBEND_MODE_DOWNONLY_INV) {
                    normmodshake = constrain(-1.0f * normmodshake, -1.0f, 0.0f);
                } else if (IMUsettings[mode][Y_SHAKE_MOD_CHPRESS_MODE] == Y_PITCHBEND_MODE_UPONLY_INV) {
                    normmodshake = constrain(normmodshake, 0.0f, 1.0f);
                }
                shakePressureChanPressMod = (normmodshake * shakeModChanPressDepth);
            }
            if (IMUsettings[mode][Y_SHAKE_MOD_KEYPRESS]) {
                float normmodshake = basenormshake;
                if (IMUsettings[mode][Y_SHAKE_MOD_KEYPRESS_MODE] == Y_PITCHBEND_MODE_UPDOWN) {
                    normmodshake *= -1.0f;  // reverse phase
                } else if (IMUsettings[mode][Y_SHAKE_MOD_KEYPRESS_MODE] == Y_PITCHBEND_MODE_DOWNONLY) {
                    normmodshake = constrain(normmodshake, -1.0f, 0.0f);
                } else if (IMUsettings[mode][Y_SHAKE_MOD_KEYPRESS_MODE] == Y_PITCHBEND_MODE_UPONLY) {
                    normmodshake = constrain(-1.0f * normmodshake, 0.0f, 1.0f);
                } else if (IMUsettings[mode][Y_SHAKE_MOD_KEYPRESS_MODE] == Y_PITCHBEND_MODE_DOWNONLY_INV) {
                    normmodshake = constrain(1.0f * normmodshake, -1.0f, 0.0f);
                } else if (IMUsettings[mode][Y_SHAKE_MOD_KEYPRESS_MODE] == Y_PITCHBEND_MODE_UPONLY_INV) {
                    normmodshake = constrain(normmodshake, 0.0f, 1.0f);
                }
                shakePressureKeyPressMod = (int)(normmodshake * shakeModKeyPressDepth);
            }
        }
    }
}









// Return number of registers to jump based on IMU pitch (elevation).
int pitchRegister() {

    for (byte i = 1; i < IMUsettings[mode][PITCH_REGISTER_NUMBER] + 1; i++) {
        if (pitch < pitchRegisterBounds[i] || (i == IMUsettings[mode][PITCH_REGISTER_NUMBER] && pitch >= pitchRegisterBounds[i])) {  // See if IMU pitch is within the bounds for each register.
            return i - 1;
        }
    }

    return 0;
}









// Read incoming messages
void readMIDI(void) {

    MIDI.read();  // Read any new USBMIDI messages.

    if (Bluefruit.connected()) {        // Don't read if we aren't connected to BLE.
        if (blemidi.notifyEnabled()) {  // ...and ready to receive messages.
            BLEMIDI.read();             // Read new BLEMIDI messages.
        }
    }
}









// Monitor the status of the 3 buttons. The integrating debouncing algorithm is taken from debounce.c, written by Kenneth A. Kuhn:http://www.kennethkuhn.com/electronics/debounce.c
// NOTE: Button array is zero-indexed, so "button 1" in all documentation is button 0 here.
void checkButtons() {

    static byte integrator[] = { 0, 0, 0 };                // When this reaches MAXIMUM, a button press is registered. When it reaches 0, a release is registered.
    static bool prevOutput[] = { 0, 0, 0 };                // Previous state of button.
    bool buttonUsed = 0;                                   // Flag any button activity, so we know to handle it.
    static unsigned int longPressCounter[] = { 0, 0, 0 };  // For counting how many readings each button has been held, to indicate a long button press

    for (byte j = 0; j < 3; j++) {

        //20230629 MrMep - Timer for doubleclick
        if (waitingSecondClick[j] && doubleClickTimer++ > DOUBLE_CLICK_WAIT_INTERVAL) {  //We were waiting for a second click, but timer has exipired
            waitingSecondClick[j] = false;
        }

        if (digitalRead(buttons[j]) == 0) {  // If the button reads low, reduce the integrator by 1
            if (integrator[j] > 0) {
                integrator[j]--;
            }
        } else if (integrator[j] < MAXIMUM) {  // If the button reads high, increase the integrator by 1
            integrator[j]++;
        }


        if (integrator[j] == 0) {  // The button is pressed.
            pressed[j] = 1;        // We make the output the inverse of the input so that a pressed button reads as a "1".
            buttonUsed = 1;

            if (prevOutput[j] == 0 && !longPressUsed[j]) {
                justPressed[j] = 1;  // The button has just been pressed
            }

            else {
                justPressed[j] = 0;
            }

            if (prevOutput[j] == 1) {  // Increase a counter so we know when a button has been held for a long press.
                longPressCounter[j]++;
            }
        }


        else if (integrator[j] >= MAXIMUM) {  // The button is not pressed.
            pressed[j] = 0;
            integrator[j] = MAXIMUM;  // Defensive code if integrator got corrupted

            if (prevOutput[j] == 1 && !longPressUsed[j]) {
                released[j] = 1;  // The button has just been released.
                buttonUsed = 1;
            }

            longPress[j] = 0;
            longPressUsed[j] = 0;  // If a button is not pressed, reset the flag that tells us it's been used for a long press.
            longPressCounter[j] = 0;
        }


        if (longPressCounter[j] > 300 && !longPressUsed[j]) {  // If the counter gets to a certain level, it's a long press.
            longPress[j] = 1;
            longPressCounter[j] = 0;
            buttonUsed = 1;
        }

        prevOutput[j] = pressed[j];  // Keep track of state for next time around.
    }

    if (millis() < 1000) {  // Ignore button 3 for the first bit after powerup in case it was only being used to power on the device. ToDo?: Make it so the first release after startup isn't registered because if you hold 3 longer than 1 second it will still register.
        pressed[2] = 0;
        released[2] = 0;
    }

    if (buttonUsed) {
        handleButtons();  // If a button had been used, process the command. We only do this when we need to, so we're not wasting time.
    }

    buttonUsed = 0;  // Now that we've handled any important button activity, clear the flag until there's been new activity.
}









// Determine which holes are covered.
void getFingers() {


    for (byte i = 0; i < 9; i++) {
        const int caloffset = calibration > 0 ? (int)((toneholeCovered[i] - toneholeBaseline[i]) * 0.2f) : 0;  // only need to apply this offset during calibration itself
        if ((toneholeRead[i]) > (toneholeCovered[i] - toneholeBaseline[i] - caloffset)) {
            bitWrite(holeCovered, i, 1);  // Use the tonehole readings to decide which holes are covered
        } else if ((toneholeRead[i]) <= (toneholeCovered[i] - toneholeBaseline[i] - caloffset - 4)) {
            bitWrite(holeCovered, i, 0);  // Decide which holes are uncovered -- the "hole uncovered" reading is a little less then the "hole covered" reading, to prevent oscillations.
        }
    }
}










// This should be called right after a new hole state change, before sending out any new note.
bool isMaybeInTransition() {
    // Look at all finger hole states to see if there might be other
    // fingers in motion (partial hole covered) which means this
    // might be a multi-finger note transition and we may want to wait
    // a bit to see if the pending fingers land before triggering the new note.
    int pendingHoleCovered = holeCovered;
    unsigned long now = millis();

    for (byte i = 0; i < 9; i++) {
        int thresh = senseDistance;  //  ((toneholeCovered[i] - 50) - senseDistance) / 2;
        if (toneholeRead[i] > thresh) {
            if (bitRead(holeCovered, i) != 1) {
                bitWrite(pendingHoleCovered, i, 1);
            }
        }
    }

    if (pendingHoleCovered != holeCovered) {
        // See if the pending is a new note.
        byte tempNewNote = getNote(holeCovered);
        byte pendingNote = getNote(pendingHoleCovered);
        if (pendingNote != 127 && pendingNote != tempNewNote) {
#if DEBUG_TRANSITION_FILTER
            Serial.print(now);
            Serial.print(" : Maybe: ");
            Serial.print(pendingNote);
            Serial.print(" wait on: ");
            Serial.println(tempNewNote);
#endif
            return true;
        }
    }

    return false;
}









// Detect changes in fingering. Contributions by Louis Barman and Jesse Chappell
void debounceFingerHoles() {

    static unsigned long debounceTimer;
    unsigned long now = millis();
    static bool timing;

    if (prevHoleCovered != holeCovered) {
        prevHoleCovered = holeCovered;
        debounceTimer = now;
        timing = 1;
        if (transientFilterDelay > 0 && isMaybeInTransition()) {
            transitionFilter = transientFilterDelay;  // ms timeout for transition to fail out
        } else {
            transitionFilter = 0;
        }
    }

    long debounceDelta = now - debounceTimer;

    if (transitionFilter > 0 && (debounceDelta >= transitionFilter || !isMaybeInTransition())) {  // Reset it if necessary.
#if DEBUG_TRANSITION_FILTER
        Serial.print(now);
        Serial.println(" canceltrans");
#endif
        transitionFilter = 0;
    }

    if (debounceDelta >= transitionFilter
        && timing == 1) {  // The fingering pattern has changed.
        timing = 0;
        fingersChanged = 1;
        fingeringChangeTimer = millis();     // Start timing after the fingering pattern has changed.
        tempNewNote = getNote(holeCovered);  // Get the next MIDI note from the fingering pattern if it has changed. 3us.
        sendToConfig(true, false);           // Put the new pattern into a queue to be sent later so that it's not sent during the same connection interval as a new note (to decrease BLE payload size).

        if (tempNewNote != 127 && newNote != tempNewNote) {  // If a new note has been triggered (127 can be used as a "blank" position that has no effect),
            if (pitchBendMode != kPitchBendNone) {
                holeLatched = holeCovered;  // Remember the pattern that triggered it (it will be used later for vibrato).
                for (byte i = 0; i < 9; i++) {
                    iPitchBend[i] = 0;  // Reset pitchbend.
                    pitchBendOn[i] = 0;
                }
            }

#if DEBUG_TRANSITION_FILTER
            Serial.print(now);
            Serial.print(" : Commit: ");
            Serial.println(tempNewNote);
#endif

            newNote = tempNewNote;
            if (ED[mode][HALFHOLE_PITCHBEND] && ED[mode][HALFHOLE_USE_MIDI_NOTE]) {  // If we're halfholing to change the MIDI note, we need to calculate pitchbend again right before sending a note to avoid sending two notes rapidfire.
                calculateAndSendPitchbend();
            }
            getState();  // Get state again if the note has changed.
        }
    }
}









// Send the finger pattern and pressure to the Configuration Tool after a delay to prevent sending during the same connection interval as a new MIDI note.
void sendToConfig(bool newPattern, bool newPressure) {

    static bool patternChanged = false;
    static bool pressureChanged = false;
    static unsigned long patternSendTimer;
    static unsigned long pressureSendTimer;

    unsigned long nowtime = millis();

    if (communicationMode) {
        if (newPattern && patternChanged == false) {  // If the fingering pattern has changed, start a timer.
            patternChanged = true;
            patternSendTimer = nowtime;
        }

        if (newPressure && pressureChanged == false) {  // If the pressure has changed, start a timer.
            pressureChanged = true;
            pressureSendTimer = nowtime;
        }

        if (patternChanged && (nowtime - patternSendTimer) > 25) {                              // If some time has past, send the new pattern to the Config Tool.
            sendMIDICouplet(MIDI_CC_114, holeCovered >> 7, MIDI_CC_115, lowByte(holeCovered));  // Because it's MIDI we have to send it in two 7-bit chunks.
            patternChanged = false;
        }

        if (pressureChanged && (nowtime - pressureSendTimer) > 25) {  // If some time has past, send the new pressure to the Config Tool.
            sendMIDICouplet(MIDI_CC_116, sensorValue & 0x7F, MIDI_CC_118, sensorValue >> 7);
            pressureChanged = false;
        }
    }
}








// Return a MIDI note number (0-127) based on the current fingering.
byte getNote(unsigned int fingerPattern) {
    byte ret = 127;  // Default (blank position)

    uint8_t tempCovered = fingerPattern >> 1;  // Bitshift once to ignore bell sensor reading.

    // Read the MIDI note for the current fingering (all charts except the custom ones).
    if (modeSelector[mode] < kWARBL2Custom1) {
        ret = charts[modeSelector[mode]][tempCovered];
    } else {
        ret = WARBL2CustomChart[tempCovered];  // Otherwise read from the currently selected custom chart.
    }

    // For whistle and uilleann also read the vibrato flag for the current fingering.
    if (modeSelector[mode] == kModeWhistle || modeSelector[mode] == kModeChromatic) {
        vibratoEnable = whistleVibrato[tempCovered];
    }

    if (modeSelector[mode] == kModeUilleann || modeSelector[mode] == kModeUilleannStandard) {
        vibratoEnable = uilleannVibrato[tempCovered];
    }

    if ((modeSelector[mode] == kModeNorthumbrian && ret == 63) || (breathMode != kPressureBell && holeCovered == 0b111111111))  // Play silence if all holes incuding bell sensor are covered, or if we're in Northumbrian mode and all top sense and thumb are covered. That simulates the closed pipe.
    {
        ret = 0;  // Silence
    }

    return ret;
}









// Add up any transposition based on key and register.
void getShift() {

    byte pitchShift;
    byte totalHalfHoleShift = 0;

    for (byte i = 0; i < 9; i++) {
        if (halfHoleShift[i] == true) {  // See if any hole contributes to halfHole MIDI note shift.
            totalHalfHoleShift = -1;
        }
    }

    if (IMUsettings[mode][PITCH_REGISTER] == true) {
        pitchShift = pitchRegister();
    }

    shift = ((octaveShift * 12) + noteShift + (pitchShift * 12) + totalHalfHoleShift);  // Adjust for key and octave shift.

    // Overblow if allowed.
    if (newState == TOP_REGISTER && !(modeSelector[mode] == kModeEVI || (modeSelector[mode] == kModeSax && newNote < 58) || (modeSelector[mode] == kModeSaxBasic && newNote < 70) || (modeSelector[mode] == kModeRecorder && newNote < 74)) && !(newNote == 62 && (modeSelector[mode] == kModeUilleann || modeSelector[mode] == kModeUilleannStandard))) {  // If overblowing (except EVI, sax and recorder in the lower register, and low D with uilleann fingering, which can't overblow)
        shift = shift + ED[mode][OVERBLOW_SEMITONES];                                                                                                                                                                                                                                                                                                       // Add a register jump to the transposition if overblowing.
        if (modeSelector[mode] == kModeKaval) {                                                                                                                                                                                                                                                                                                             // Kaval only plays a fifth higher in the second register.
            shift = shift - 5;
        }
    }
    // Use the bell sensor to control register if desired.
    if (breathMode == kPressureBell && modeSelector[mode] != kModeUilleann && modeSelector[mode] != kModeUilleannStandard) {
        if (bitRead(holeCovered, 0) == switches[mode][INVERT]) {
            shift = shift + ED[mode][OVERBLOW_SEMITONES];
            if (modeSelector[mode] == kModeKaval) {
                shift = shift - 5;
            }
        }
    }

    // ToDo: Are there any others that don't use the thumb that can be added here?
    // If we're using the left thumb to control the regiser with a fingering patern that doesn't normally use the thumb
    else if ((breathMode == kPressureThumb && (modeSelector[mode] == kWARBL2Custom1 || modeSelector[mode] == kWARBL2Custom2 || modeSelector[mode] == kWARBL2Custom3 || modeSelector[mode] == kWARBL2Custom4 || modeSelector[mode] == kModeWhistle || modeSelector[mode] == kModeChromatic || modeSelector[mode] == kModeNAF))) {
        byte thumbShift = getThumbHalfHoleShift();                    // Number of registers shifted by thumb
        shift = shift + (thumbShift * ED[mode][OVERBLOW_SEMITONES]);  // Add an octave jump to the transposition if necessary.
    }
}









// Get the number of registers shifted by the thumb.
byte getThumbHalfHoleShift() {

    /* Thumb halfhole function (table from MrPep)
_________________________________________________________________________________________________
|   Invert Thumb/Bell	|   Half Invert Thumb |  1st octave	|   2nd octave  |   3rd octave  |
_________________________________________________________________________________________________
|         off	        |         off	        |     closed	|      open	    |      half     |
|         on	        |         off	        |      open	  |     closed	  |      half     |
|         off	        |         on	        |     closed	|      half	    |      open     |
|         on	        |         on	        |      open	  |      half	    |     closed    |
_________________________________________________________________________________________________
*/
    const byte lookup[4][3] = { { 1, 2, 3 }, { 1, 3, 2 }, { 2, 1, 3 }, { 3, 1, 2 } };       // Lookup table for thumb halfhole functionality.
    byte combinedSwitches = switches[mode][INVERT] << 1 | ED[mode][HALFHOLE_INVERT_THUMB];  // Append the invert switches for the first dimension o fthe lookup table.
    byte thumbPosition = thumbHalfHole ? 2 : 1 - bitRead(holeCovered, 8);                   // Second dimension is thumb position: 0 closed, 1 half, 2 open

    if (!(ED[mode][HALFHOLE_PITCHBEND] && bitRead(ED[mode][HALFHOLE_HOLES_HIGH4BITS], 3))) {  // First handle register contribution by the thumb if we're not using it for half-holing.
        if (bitRead(holeCovered, 8) == switches[mode][INVERT]) {
            return 1;
        } else {
            return 0;
        }
    }

    else {  // If thumb halfhole is being used
        return lookup[combinedSwitches][thumbPosition] - 1;
    }
}









// Use IMU elevation angle to prevent overblowing from changing the current register (allow finer control of dynamics within the current register).
void getRegisterHold() {

    if (enableRegisterHold) {

        // settings (to be moved to Config Tool):
        const byte registerHoldMode = 4;      // 1 = hold both registers (both tilt zones), 2 = hold low register only (both tilt zones), 3 = hold high register only (both tilt zones), 4 = hold low register with low tilt zone and high register with high tilt zone.
        const float lowerHoldAngle = -60.0f;  // Elevation angle below which the register will be locked (can be set at -90 for no lower lock zone).
        const float upperHoldAngle = 0.0f;    // Elevation angle above which the register will be locked (can be set at 90 for no upper lock zone).

        bool inTiltZone = false;

        if ((registerHoldMode == 1 || (registerHoldMode < 4 && registerHoldMode == newState)) && (pitch < lowerHoldAngle || pitch > upperHoldAngle)) {  // Modes 1-3
            inTiltZone = true;
        }

        if (registerHoldMode == 4 && ((pitch < lowerHoldAngle && newState == 2) || (pitch > upperHoldAngle && newState == 3))) {  // Mode 4
            inTiltZone = true;
        }

        if (inTiltZone) {
            if (!registerHold && newState != SILENCE) {
                registerHold = true;
                heldRegister = newState;
            }
        } else if ((registerHoldMode < 4 && pitch >= lowerHoldAngle && pitch <= upperHoldAngle) || (registerHoldMode == 4 && ((pitch >= lowerHoldAngle && heldRegister == 2) || (pitch <= upperHoldAngle && heldRegister == 3)))) {  // Reset if we've exited the hold zones.
            registerHold = false;
        }
    }
}








// State machine that models the way that a tinwhistle etc. begins sounding and jumps octaves in response to breath pressure.
// The current jump/drop behavior is from Louis Barman
void getState() {

    byte scalePosition;  // ScalePosition is used to tell where we are on the scale, because higher notes are more difficult to overblow.
    unsigned int tempHoleCovered = holeCovered;
    bitSet(tempHoleCovered, 8);                                  // Ignore thumb hole.
    scalePosition = findleftmostunsetbit(tempHoleCovered) + 62;  // Use the highest open hole to calculate.
    if (scalePosition > 69) {
        scalePosition = 70;
    }

    if (ED[mode][DRONES_CONTROL_MODE] == 3) {  // Use pressure to control drones if that option has been selected. There's a small amount of hysteresis added.

        if (!dronesOn && sensorValue > 5 + (ED[mode][DRONES_PRESSURE_HIGH_BYTE] << 7 | ED[mode][DRONES_PRESSURE_LOW_BYTE])) {
            startDrones();
        }

        else if (dronesOn && sensorValue < (ED[mode][DRONES_PRESSURE_HIGH_BYTE] << 7 | ED[mode][DRONES_PRESSURE_LOW_BYTE])) {
            stopDrones();
        }
    }

    upperBound = (sensorThreshold[1] + ((scalePosition - 60) * multiplier));  // Calculate the threshold between state 2 (bottom register) and state 3 (top register). This will also be used to calculate expression.

    newState = currentState;

    if (sensorValue <= sensorThreshold[0]) {
        newState = SILENCE;
        holdoffActive = false;  // No need to wait for jump/drop if we've already crossed the threshold for silence
    } else if (sensorValue > sensorThreshold[0] + SILENCE_HYSTERESIS) {
        if (currentState == SILENCE) {
            newState = BOTTOM_REGISTER;
        }


        if (breathMode == kPressureBreath) {  // If overblowing is enabled
            upperBoundHigh = calcHysteresis(upperBound, true);
            upperBoundLow = calcHysteresis(upperBound, false);
            if (sensorValue > upperBoundHigh) {
                newState = TOP_REGISTER;
                holdoffActive = false;
            } else if (sensorValue <= upperBoundLow) {
                newState = BOTTOM_REGISTER;
            }

            // Wait to decide about jump or drop if necessary.
            if (currentState == SILENCE && newState == BOTTOM_REGISTER) {
                newState = delayStateChange(JUMP, sensorValue, upperBoundHigh);
            } else if (currentState == TOP_REGISTER && newState == BOTTOM_REGISTER && (millis() - fingeringChangeTimer) > 20) {  // Only delay for drop if the note has been playing for a bit. This fixes erroneous high-register notes.
                newState = delayStateChange(DROP, sensorValue, upperBoundLow);
            }
        }
    }

    if (registerHold && newState != SILENCE) {  // If register is locked, only allow silence or the held register.
        newState = heldRegister;
    }

    currentState = newState;

    if (switches[mode][SEND_VELOCITY]) {  // If we're sending NoteOn velocity based on pressure,
        if (prevState == SILENCE && newState != SILENCE) {
            velocityDelayTimer = millis();  // reset the delay timer used for calculating velocity when a note is turned on after silence.
        }
        prevState = newState;
    }
}







// Delay the overblow state until either it has timed out or the pressure has leveled off.
byte delayStateChange(byte jumpDrop, int pressure, int upper) {
    static unsigned long holdOffTimer;
    unsigned long now = millis();
    bool exitEarly = false;
    int rateChange;

    if (!holdoffActive) {  // Start our timer if we haven't already.
        holdoffActive = true;
        holdOffTimer = now;
        rateChangeIdx = 0;
        previousPressure = 0;
    }

    if ((jumpDrop == JUMP && (now - holdOffTimer) < jumpTime) || (jumpDrop == DROP && (now - holdOffTimer) < dropTime)) {  // If we haven't paused long enough, check the pressure rate change to see if it has leveled off.


        rateChange = pressureRateChange(pressure);

        if (rateChange != 2000) {  // Make sure it's valid
            if (jumpDrop == JUMP && rateChange <= 0) {
                exitEarly = true;
            } else if (jumpDrop == DROP && rateChange >= 0) {
                exitEarly = true;
            }
        }
    }

    // If we've paused long enough or the pressure has stabilized, go to the bottom register
    else {
        holdoffActive = false;
        return BOTTOM_REGISTER;
    }

    if (exitEarly) {
        holdoffActive = false;
        return BOTTOM_REGISTER;
    }

    return currentState;  // Stay in the current state if we haven't waited the total time and the pressure hasn't yet leveled off.
}








// Calculate the rate of pressure change to see if the slope has reversed
int pressureRateChange(int pressure) {
    int rateChange = 2000;  // If not valid
    if (rateChangeIdx == 0) {
        rateChangeIdx = 1;
        previousPressure = pressure;
    } else {
        rateChange = pressure - previousPressure;
        previousPressure = pressure;
    }

    return rateChange;
}








// Calculate the upper boundary for the register when hysteresis is applied, from Louis Barman.
int calcHysteresis(int currentUpperBound, bool high) {
    if (hysteresis == 0) {
        return currentUpperBound;
    }
    int range = currentUpperBound - sensorThreshold[0];
    int newUpperBound;
    if (high) {
        newUpperBound = currentUpperBound + (range * hysteresis) / 400;
    } else {
        newUpperBound = currentUpperBound - (range * hysteresis * 3) / 400;
    }

    return newUpperBound;
}









inline float curveValToExponent(int val) {
    // takes 0 -> 127 and returns curve exponent
    // which is between 0.25 -> 0 -> 4.0
    float retval = 1.0f;
    val -= 64;
    if (val >= 0) {
        retval = ((3.0f * val / 63.0f) + 1.0f);
    } else {
        retval = (0.75f * (64.0f + val) / 64.0f + 0.25f);
    }
    return retval;
}








// Calculate the bend in a low and high pressure range segments, with a stable (no-bend) range between.
void getExpression() {

    int lowPressureMin = (ED[mode][EXPRESSION_MIN] * 9) + 100;
    int lowPressureMax = (ED[mode][EXPRESSION_MIN_HIGH] * 9) + 100;
    int highPressureMin = (ED[mode][EXPRESSION_MAX_LOW] * 9) + 100;
    int highPressureMax = (ED[mode][EXPRESSION_MAX] * 9) + 100;
    int centsLowOffset = 2 * (ED[mode][EXPRESSION_OUT_LOW_CENTS] - 64);
    int centsHighOffset = 2 * (ED[mode][EXPRESSION_OUT_HIGH_CENTS] - 64);
    float lowCurveExp = curveValToExponent(ED[mode][EXPRESSION_CURVE_LOW]);
    float highCurveExp = curveValToExponent(ED[mode][EXPRESSION_CURVE_HIGH]);
    bool doClamp = ED[mode][EXPRESSION_OUT_CLAMP] > 0;

    // Fixed non-override options or overblow mode
    if (!switches[mode][OVERRIDE] || (breathMode == kPressureBreath)) {
        if (newState == TOP_REGISTER) {
            lowPressureMin = upperBoundLow;                                                  // sensorThreshold[0];  // Otherwise use boundaries based on the pressure range of the current register.
            highPressureMax = upperBoundLow + ((upperBoundHigh - sensorThreshold[0]) >> 1);  // Get the register boundary taking hysteresis into consideration.
        } else {
            lowPressureMin = sensorThreshold[0];  // Otherwise use boundaries based on the pressure range of the current register.
            if (breathMode == kPressureBreath) {
                highPressureMax = upperBoundHigh;
            } else {
                int centerPressure = (ED[mode][EXPRESSION_FIXED_CENTER_PRESSURE] * 9) + 100;
                highPressureMax = 2 * (centerPressure - lowPressureMin) + lowPressureMin;
            }
        }

        // Make them meet in the middle.
        lowPressureMax = highPressureMin = ((highPressureMax + lowPressureMin) >> 1);

        // May need to play with these - XXX
        centsLowOffset = -20 * ED[mode][EXPRESSION_DEPTH];  // -20 cents at least
        centsHighOffset = ((breathMode == kPressureBreath) ? 10 : 20) * ED[mode][EXPRESSION_DEPTH];
        doClamp = true;
        lowCurveExp = 1.0f;
        highCurveExp = 1.0f;
    }

    float centsOffset = 0.0f;
    float ratio = 0.0f;
    int lowRangeSpan = max(lowPressureMax - lowPressureMin, 1);
    int highRangeSpan = max(highPressureMax - highPressureMin, 1);
    if (sensorValue <= lowPressureMax) {  // Pressure is in the lower range.
        ratio = min(((lowPressureMax - sensorValue)) / (float)(lowRangeSpan), 1.0f);
        if (lowCurveExp != 1.0f) {
            ratio = powf(ratio, lowCurveExp);
        }
        centsOffset = centsLowOffset * ratio;
    } else if (sensorValue >= highPressureMin) {  // Pressure is in the higher range.
        ratio = max(((sensorValue - highPressureMin)) / (float)(highRangeSpan), 0.0f);
        if (highCurveExp != 1.0f) {
            ratio = powf(ratio, highCurveExp);
        }
        if (doClamp) {
            ratio = min(ratio, 1.0f);
        }
        centsOffset = centsHighOffset * ratio;
    } else {
        // In the stable range
        centsOffset = 0;
    }

    expression = (int)(0.01f * centsOffset * pitchBendPerSemi);  // Expression is in raw pitch bend units.
}








// Calculate the bend in a low and high IMU range segments, with a stable (no-bend) range between.
void getIMUpitchbend() {

    IMUpitchbend = 0;  // Reset.

    for (byte i = 0; i < 3; i++) {
        if (IMUsettings[mode][MAP_ROLL_TO_PITCHBEND + i]) {  // Calculate pitchbend separately for roll, pitch, and yaw if necessary.
            int IMUvalue;
            if (i == 0) {
                IMUvalue = ((constrain(roll, -90, 90) + 90) * 5) + 100;  // Constrain roll to 180 degrees and map to same range as pressure sensor, for re-using expression calculations.
            }
            if (i == 1) {
                IMUvalue = ((constrain(pitch, -90, 90) + 90) * 5) + 100;  // Pitch
            }
            if (i == 2) {
                IMUvalue = ((constrain(yaw, -180, 180) + 180) * 2.5f) + 100;  // Yaw (full 360 degree range)
            }
            int lowIMUmin = (IMUsettings[mode][IMU_ROLL_PITCH_MIN + (i * 9)] * 25) + 100;  // Values received from the Config Tool range from 0-36 and we scale them up to 900 here to make it easy to re-use the calculations from the expression override function above.
            int lowIMUmax = (IMUsettings[mode][IMU_ROLL_PITCH_MIN_HIGH + (i * 9)] * 25) + 100;
            int highIMUmin = (IMUsettings[mode][IMU_ROLL_PITCH_MAX_LOW + (i * 9)] * 25) + 100;
            int highIMUmax = (IMUsettings[mode][IMU_ROLL_PITCH_MAX + (i * 9)] * 25) + 100;
            int centsLowOffset = 2 * (IMUsettings[mode][IMU_ROLL_PITCH_OUT_LOW_CENTS + (i * 9)] - 64);
            int centsHighOffset = 2 * (IMUsettings[mode][IMU_ROLL_PITCH_OUT_HIGH_CENTS + (i * 9)] - 64);
            float lowCurveExp = curveValToExponent(IMUsettings[mode][IMU_ROLL_PITCH_CURVE_LOW + (i * 9)]);
            float highCurveExp = curveValToExponent(IMUsettings[mode][IMU_ROLL_PITCH_CURVE_HIGH + (i * 9)]);
            bool doClamp = IMUsettings[mode][IMU_ROLL_PITCH_OUT_CLAMP + (i * 9)] > 0;
            float centsOffset = 0.0f;
            float ratio = 0.0f;
            int lowRangeSpan = max(lowIMUmax - lowIMUmin, 1);
            int highRangeSpan = max(highIMUmax - highIMUmin, 1);
            if (IMUvalue <= lowIMUmax) {  // IMU is in the lower range.
                ratio = ((lowIMUmax - IMUvalue)) / (float)(lowRangeSpan);
                if (lowCurveExp != 1.0f) {
                    ratio = powf(ratio, lowCurveExp);
                }
                if (doClamp) {
                    ratio = min(ratio, 1.0f);
                }
                centsOffset = centsLowOffset * ratio;
            } else if (IMUvalue >= highIMUmin) {  // IMU is in the higher range.
                ratio = max(((IMUvalue - highIMUmin)) / (float)(highRangeSpan), 0.0f);
                if (highCurveExp != 1.0f) {
                    ratio = powf(ratio, highCurveExp);
                }
                if (doClamp) {
                    ratio = min(ratio, 1.0f);
                }
                centsOffset = centsHighOffset * ratio;
            } else {
                // In the stable range
                centsOffset = 0;
            }

            IMUpitchbend += (int)(0.01f * centsOffset * pitchBendPerSemi);  // Add in contribution by roll, pitch, yaw. In raw pitch bend units.
        }
    }
}









// For a specific hole, return the number of half-steps interval it would be from the current note with hole-covered state.
int findStepsOffsetFor(int hole) {
    unsigned int closedHolePattern = holeCovered;
    bitSet(closedHolePattern, hole);  // Figure out what the fingering pattern would be if we closed the hole.
    int stepsOffset = getNote(closedHolePattern) - newNote;
    return stepsOffset;
}










// Custom pitchbend algorithms, tin whistle and uilleann by Michael Eskin
void handleCustomPitchBend() {
    for (byte i = 0; i < 9; i++) {  // Reset all holes
        iPitchBend[i] = 0;
    }

    if (pitchBendMode == kPitchBendSlideVibrato || pitchBendMode == kPitchBendLegatoSlideVibrato || ED[mode][HALFHOLE_PITCHBEND]) {  // Calculate slide and halfhole if necessary.
        getSlide();
    }

    if (pitchBendMode != kPitchBendNone) {  // If halfhole pitchbend is enabled but no other pitchbend is, we don't need to calculate any vibrato.

        // This method only cares if 2 or 3 are slideholes.
        int slideHoleIndex = iPitchBend[2] != 0 ? 2 : iPitchBend[3] != 0 ? 3
                                                                         : 0;

        if (modeSelector[mode] != kModeGHB && modeSelector[mode] != kModeNorthumbrian) {  // Only used for whistle and uilleann
            if (vibratoEnable == 1) {                                                     // If it's a vibrato fingering pattern
                if (iPitchBend[2] == 0) {
                    iPitchBend[2] = adjvibdepth;  // Just assign max vibrato depth to a hole that isn't being used for sliding (it doesn't matter which hole, it's just so it will be added in later).
                    iPitchBend[3] = 0;
                } else if (iPitchBend[3] == 0) {
                    iPitchBend[3] = adjvibdepth;
                    iPitchBend[2] = 0;
                }
            }


            if (vibratoEnable == 2) {  // Used for whistle and uilleann, indicates that it's a pattern where lowering finger 2 or 3 partway would trigger progressive vibrato.

                if (modeSelector[mode] == kModeWhistle || modeSelector[mode] == kModeChromatic) {
                    for (byte i = 2; i < 4; i++) {
                        if ((toneholeRead[i] > senseDistance) && (bitRead(holeCovered, i) != 1 && (i != slideHoleIndex))) {  // If the hole is contributing, bend down.
                            iPitchBend[i] = (int)((toneholeRead[i] - senseDistance) * vibratoScale[i]);
                        } else if (i != slideHoleIndex) {
                            iPitchBend[i] = 0;
                        }
                    }
                    if (slideHoleIndex != 2 && slideHoleIndex != 3 && iPitchBend[2] + iPitchBend[3] > adjvibdepth) {
                        iPitchBend[2] = adjvibdepth;  // Cap at max vibrato depth if they combine to add up to more than that (just set one to max and the other to zero).
                        iPitchBend[3] = 0;
                    }
                }

                else if (modeSelector[mode] == kModeUilleann || modeSelector[mode] == kModeUilleannStandard) {

                    if ((holeCovered & 0b100000000) == 0) {  // If the back-D is open, and the vibrato hole completely open, max the pitch bend.
                        if (bitRead(holeCovered, 3) == 1) {
                            iPitchBend[3] = 0;
                        } else {  // Otherwise, bend down proportional to distance
                            if (toneholeRead[3] > senseDistance) {
                                iPitchBend[3] = adjvibdepth - (((toneholeRead[3] - senseDistance) * vibratoScale[3]));
                            } else {
                                iPitchBend[3] = adjvibdepth;
                            }
                        }
                    } else {

                        if ((toneholeRead[3] > senseDistance) && (bitRead(holeCovered, 3) != 1) && 3 != slideHoleIndex) {
                            iPitchBend[3] = (int)((toneholeRead[3] - senseDistance) * vibratoScale[3]);
                        }

                        else if ((toneholeRead[3] < senseDistance) || (bitRead(holeCovered, 3) == 1)) {
                            iPitchBend[3] = 0;  // If the finger is removed or the hole is fully covered, there's no pitchbend contributed by that hole.
                        }
                    }
                }
            }
        }


        else if (modeSelector[mode] == kModeGHB || modeSelector[mode] == kModeNorthumbrian) {  // This one is designed for closed fingering patterns, so raising a finger sharpens the note.
            for (byte i = 2; i < 4; i++) {                                                     // Use holes 2 and 3 for vibrato.
                if (i != slideHoleIndex || (holeCovered & 0b100000000) == 0) {
                    static unsigned int testNote;                        // The hypothetical note that would be played if a finger were lowered all the way.
                    if (bitRead(holeCovered, i) != 1) {                  // If the hole is not fully covered
                        if (fingersChanged) {                            // If the fingering pattern has changed
                            testNote = getNote(bitSet(holeCovered, i));  // Check to see what the new note would be.
                            fingersChanged = 0;
                        }
                        if (testNote == newNote) {  // If the hole is uncovered and covering the hole wouldn't change the current note (or the left thumb hole is uncovered, because that case isn't included in the fingering chart).
                            if (toneholeRead[i] > senseDistance) {
                                iPitchBend[i] = 0 - (int)((max(toneholeCovered[i] - toneholeBaseline[i] - toneholeRead[i], 0) * vibratoScale[i]));  // Bend up, yielding a negative pitchbend value.
                            } else {
                                iPitchBend[i] = 0 - adjvibdepth;  // If the hole is totally uncovered, max the pitchbend.
                            }
                        }
                    } else {                // If the hole is covered
                        iPitchBend[i] = 0;  // Reset the pitchbend to 0
                    }
                }
            }
            if ((((iPitchBend[2] + iPitchBend[3]) * -1) > adjvibdepth) && ((slideHoleIndex != 2 && slideHoleIndex != 3) || (holeCovered & 0b100000000) == 0)) {  // Cap at vibrato depth if more than one hole is contributing and they add to up to more than the vibrato depth.
                iPitchBend[2] = 0 - adjvibdepth;                                                                                                                 // Assign max vibrato depth to a hole that isn't being used for sliding.
                iPitchBend[3] = 0;
            }
        }
    }
}









// Andrew's version of vibrato
void handlePitchBend() {
    for (byte i = 0; i < 9; i++) {  // Reset
        iPitchBend[i] = 0;
    }

    if (pitchBendMode == kPitchBendSlideVibrato || pitchBendMode == kPitchBendLegatoSlideVibrato || ED[mode][HALFHOLE_PITCHBEND]) {  // Calculate slide and halfhole if necessary.
        getSlide();
    }

    if (pitchBendMode != kPitchBendNone) {  // If halfhole pitchbend is enabled but no other pitchbend is, we don't need to calculate any vibrato.

        for (byte i = 0; i < 9; i++) {

            if (bitRead(holeLatched, i) == 1 && toneholeRead[i] < senseDistance) {
                (bitWrite(holeLatched, i, 0));  // We "unlatch" (enable for vibrato) a hole if it was covered when the note was triggered but now the finger has been completely removed.
            }

            if (bitRead(vibratoHoles, i) == 1 && bitRead(holeLatched, i) == 0  // If this is a vibrato hole and we're in a mode that uses vibrato, and the hole is unlatched
                && (iPitchBend[i] == 0)) {                                     // ...and not already being used for slide or halfhole
                if (toneholeRead[i] > senseDistance) {
                    if (bitRead(holeCovered, i) != 1) {
                        iPitchBend[i] = (int)(((toneholeRead[i] - senseDistance) * vibratoScale[i]));  //bend downward
                        pitchBendOn[i] = 1;
                    }
                } else {
                    pitchBendOn[i] = 0;
                    if (bitRead(holeCovered, i) == 1) {
                        iPitchBend[i] = 0;
                    }
                }

                if (bitRead(holeCovered, i) == 1) {
                    iPitchBend[i] = adjvibdepth;  // Set vibrato to max downward bend if a hole was being used to bend down and now is covered
                }
            }
        }
    }
}









// Calculate slide pitchBend, to be added with vibrato.
void getSlide() {

    byte halfHoleEnabled = (ED[mode][HALFHOLE_HOLES_HIGH4BITS] << 4) | ED[mode][HALFHOLE_HOLES_LOW4BITS];

    for (byte i = 0; i < 9; i++) {
        if (toneholeRead[i] > senseDistance && bitRead(holeCovered, i) != 1 && transitionFilter == 0) {  // Does transitionFilter really need to be zero here? AM
            const int offsetLimit = constrain(ED[mode][SLIDE_LIMIT_MAX], 0, midiBendRange);

            int offsetSteps = findStepsOffsetFor(i);
            int trueOffsetSteps = offsetSteps;

            if (pitchBendModeSelector[mode] == kPitchBendSlideVibrato && offsetSteps < -offsetLimit) {  // Added by AM 5/24 to make the slide behavior more like that of the original WARBL.
                offsetSteps = -offsetLimit;
            }
            if (breathMode == kPressureThumb && i == 8 && (bitRead(halfHoleEnabled, 7) == 1) && bitRead(holeCovered, 8) != 1) {  // If we're using the thumb for register shift, go ahead and detect that now.
                getHalfholePitchbend(i);
            }

            else if (offsetSteps != 0 && offsetSteps <= offsetLimit && offsetSteps >= -offsetLimit) {
                if (!(ED[mode][HALFHOLE_PITCHBEND] && ED[mode][HALFHOLE_USE_MIDI_NOTE]) && !(ED[mode][HALFHOLE_PITCHBEND] && trueOffsetSteps == -2 && (bitRead(halfHoleEnabled, i - 1) == 1))) {  // Calculate slide normally if halfhole doesn't apply (if we're using half-holing and sending MIDI notes instead of pitch, we also don't use sliding on any holes).
                    if (pitchBendModeSelector[mode] == kPitchBendSlideVibrato || pitchBendModeSelector[mode] == kPitchBendLegatoSlideVibrato) {
                        iPitchBend[i] = ((((int)((toneholeRead[i] - senseDistance) * toneholeScale[i])) * -offsetSteps));  // Scale.
                    }
                } else if (trueOffsetSteps == -2) {  // Calculate halfhole pitchbend if all the conditions for this hole are met.
                    getHalfholePitchbend(i);
                }
            } else {
                iPitchBend[i] = 0;
                snapped[i] = false;
                halfHoleShift[i] = false;
                if (i == 8) { thumbHalfHole = false; }
            }
        } else {
            iPitchBend[i] = 0;
            snapped[i] = false;
            halfHoleShift[i] = false;
            if (i == 8) { thumbHalfHole = false; }
        }
    }
}









// Snap pitchbend to a semitone and calculate slide to smoothly integrate.
void getHalfholePitchbend(byte i) {

    int heightOffset = ED[mode][HALFHOLE_HEIGHT_OFFSET];   // (0-100) Height offset below (0-50) or above (51-100)  the "natural" semitone point where the halfhole region is centered.
    int width = ED[mode][HALFHOLE_WIDTH];                  // The size of the halfhole region (%). Lower values require more accurate finger placement but leave more room for sliding (and smoother transitions from sliding to semitone).
    int fingerRate = ED[mode][HALFHOLE_FINGERRATE] * 1.5;  // 0-127. Only used if not using slide too. The finger movement rate (in normalized sensor counts per reading) below which we'll snap to the semitone. Has the effect of a transient filter but uses finger rate rather than elapsed time so we only need to take two readings to calulate it.
    const int hysteresis = 3;                              // Hysteresis for the target region
    bool inTargetRegion;                                   // Whether the finger is in the assigned halfhole region
    const int offsetSteps = -2;                            // This is always true because there is a full step drop for the holes we use for halfholing.
    static int prevToneholeRead[9];                        // For calculating rate of finger movement
    bool MIDInoteShifted = false;

    float change = (abs(sqrt(float(toneholeRead[i])) - sqrt(float(prevToneholeRead[i])))) * 30;  // Absolute rate of finger movement, linearized. Typically ranges from 0-100.

    prevToneholeRead[i] = toneholeRead[i];
    int center = ((toneholeCovered[i] - senseDistance) >> 1) + senseDistance;  // Center sensor value for current slide hole (midpoint of sensor readings)

    heightOffset = ((heightOffset - 50) * center) / 50;  // Convert offset to a positive or negative sensor value.
    width = (width * (center + heightOffset)) / 100;     // Convert width to a sensor value.

    // Determine if the finger is in the target region.
    if ((pitchBendModeSelector[mode] == kPitchBendSlideVibrato || pitchBendModeSelector[mode] == kPitchBendLegatoSlideVibrato) && !ED[mode][HALFHOLE_USE_MIDI_NOTE] && !(kPressureThumb && i == 8)) {
        if (abs(toneholeRead[i] - center + heightOffset) < width) {  // If we're using slide, there's a "not halfhole" space both above and below the halfhole region.
            inTargetRegion = true;

        } else if (abs(toneholeRead[i] - center + heightOffset) > width + hysteresis) {  // Use hysteresis to exit target region to avoid oscillations when we're not also using slide.
            inTargetRegion = false;
        }
    } else if (toneholeRead[i] > (center - heightOffset - width)) {  // If we're not using sliding, if we're using MIDI notes for pitchbend, or using the thumb for register, the halfhole region extends all the way down to the fully-covered hole.
        inTargetRegion = true;
    } else if (toneholeRead[i] < (center - heightOffset - width - hysteresis)) {
        inTargetRegion = false;
    }

    if ((change < fingerRate || fingerRate == 127 || pitchBendModeSelector[mode] == kPitchBendSlideVibrato || pitchBendModeSelector[mode] == kPitchBendLegatoSlideVibrato) && inTargetRegion) {  // Snap to halfhole if the finger is moving slowly enough (or we're using slide) and it is within the defined region.
        snapped[i] = true;                                                                                                                                                                       // Snapped to semitone.
    }

    if (!inTargetRegion) {
        snapped[i] = false;
        if (i == 8 && breathMode == kPressureThumb) {  // If we're using the thumb for register control
            thumbHalfHole = false;
        }
        if (halfHoleShift[i] == true) {
            halfHoleShift[i] = false;
        }
    }

    if (snapped[i] == true) {
        if (i == 8 && breathMode == kPressureThumb) {  // If we're using the thumb for register control and it's in the halfhole region...
            thumbHalfHole = true;                      // Shift the register.
        } else if (!ED[mode][HALFHOLE_USE_MIDI_NOTE]) {
            iPitchBend[i] = pitchBendPerSemi;  // Snap to semitone if not using MIDI note instead.
        } else {
            halfHoleShift[i] = true;  // Or shift MIDI note.
        }
    }

    // Calculate slide here if we're not snapped to semitone. We calculate the slide in two portions, converging on the edges of the target region at one semitone. This gives a smooth transition from sliding to semitone.
    if (!snapped[i] && !ED[mode][HALFHOLE_USE_MIDI_NOTE] && (pitchBendModeSelector[mode] == kPitchBendSlideVibrato || pitchBendModeSelector[mode] == kPitchBendLegatoSlideVibrato) && !(i == 8 && breathMode == kPressureThumb)) {
        if (toneholeRead[i] < (center - heightOffset)) {                                                                                                      // The sensor value is lower than the target region.
            float tempToneholeScale = (((16383.0f / midiBendRange)) / max((center - heightOffset - width) - toneholeBaseline[i] - senseDistance, 1) / 4.0f);  // We need to recalculate the tonehole scaling factor based on whether we are above the region or below it.
            iPitchBend[i] = ((((int)((toneholeRead[i] - senseDistance) * tempToneholeScale)) * -offsetSteps));
        } else {  // The sensor value is higher than the target region.
            float tempToneholeScale = (((16383.0f / midiBendRange)) / max(toneholeCovered[i] - (center - heightOffset + width), 1) / 4.0f);
            iPitchBend[i] = pitchBendPerSemi + ((((int)((toneholeRead[i] - (center - heightOffset + width)) * tempToneholeScale)) * -offsetSteps));
        }
    }
}










void calculateAndSendPitchbend() {

    bool pitchBendUsed = false;

    if (ED[mode][EXPRESSION_ON] && !switches[mode][BAGLESS]) {
        getExpression();  // If using pitchbend expression, calculate pitchbend based on pressure reading.
        pitchBendUsed = true;
    }
    if (IMUsettings[mode][MAP_ROLL_TO_PITCHBEND] || IMUsettings[mode][MAP_ELEVATION_TO_PITCHBEND] || IMUsettings[mode][MAP_YAW_TO_PITCHBEND]) {
        getIMUpitchbend();
        pitchBendUsed = true;
    }
    if (pitchBendMode != kPitchBendNone || ED[mode][HALFHOLE_PITCHBEND]) {
        if (!customEnabled) {
            handlePitchBend();
        } else {
            handleCustomPitchBend();
        }
        pitchBendUsed = true;
    }
    shakeForVibrato();  // This always needs to be called to detect the shake "button" gesture.
    if (IMUsettings[mode][Y_SHAKE_PITCHBEND] || IMUsettings[mode][Y_SHAKE_MOD_CC] || IMUsettings[mode][Y_SHAKE_MOD_CHPRESS] || IMUsettings[mode][Y_SHAKE_MOD_KEYPRESS]) {
        pitchBendUsed = true;
    }
    if (pitchBendUsed) {  // Send pitchbend if necessary.
        sendPitchbend();
    }
}










void sendPitchbend() {


    static int prevRawPitchBend = 0;

    pitchBend = 0;  // Reset the overall pitchbend in preparation for adding up the contributions from all the toneholes.
    for (byte i = 0; i < 9; i++) {
        pitchBend = pitchBend + iPitchBend[i];
    }

    int noteshift = 0;
    if (noteon && pitchBendModeSelector[mode] == kPitchBendLegatoSlideVibrato) {
        noteshift = (notePlaying - shift) - newNote;
        pitchBend += (int)(noteshift * pitchBendPerSemi);

        // we are sending PB at a higher rate in this mode, so we can average the current with the last
        // to provide the smallest amount of smoothing to the raw note value which avoids artifacts
        // in some destination sound sources

        const int avgcnt = 2;
        if (resetBendFilter) {
            // don't average in this case (happens around a note on)
            prevRawPitchBend = pitchBend;
            resetBendFilter = false;
        } else {
            int avgbend = (prevRawPitchBend + pitchBend) >> 1;
            prevRawPitchBend = pitchBend;

            pitchBend = avgbend;
        }
    }

    pitchBend = 8192 - pitchBend + expression + shakeVibrato + IMUpitchbend;  // Add up all sources of pitchbend.

    if (pitchBend < 0) {
        pitchBend = 0;
    } else if (pitchBend > 16383) {
        pitchBend = 16383;
    }

    if (prevPitchBend != pitchBend) {

        if (noteon) {

            sendMIDI(PITCH_BEND, mainMidiChannel, pitchBend & 0x7F, pitchBend >> 7);
            prevPitchBend = pitchBend;
        }
    }
}









// Send MIDI NoteOn/NoteOff events when necessary.
void sendNote() {
    const int velDelayMs = switches[mode][SEND_AFTERTOUCH] != 0 ? 3 : 16;  // Keep this minimal to avoid latency if also sending aftertouch, but enough to get a good reading, otherwise use longer

    if (        // Several conditions to tell if we need to turn on a new note.
      (!noteon  // If there wasn't any note playing or the current note is different than the previous one
       || (pitchBendModeSelector[mode] != kPitchBendLegatoSlideVibrato && newNote != (notePlaying - shift))
       || (pitchBendModeSelector[mode] == kPitchBendLegatoSlideVibrato && abs(newNote - (notePlaying - shift)) > midiBendRange - 1))
      && newNote != 0                                                                                 // And the MIDI note is not 0 (with a custom chart a MIDI note of 0 can be used as a silent position, so don't play the note).
      && ((newState > 1 && !switches[mode][BAGLESS]) || (switches[mode][BAGLESS] && play)) &&         // And the state machine has determined that a note should be playing, or we're in bagless mode and the sound is turned on
      !(switches[mode][SEND_VELOCITY] && !noteon && ((millis() - velocityDelayTimer) < velDelayMs)))  // And not waiting for the pressure to rise to calculate note on velocity if we're transitioning from not having any note playing.
    {

        int notewason = noteon;
        int notewasplaying = notePlaying;


        // If this is a fresh/tongued note calculate pressure now to get the freshest initial velocity/pressure
        if (!notewason) {
            if (ED[mode][SEND_PRESSURE]) {
                calculatePressure(0);
            }
            if (switches[mode][SEND_VELOCITY]) {
                calculatePressure(1);
            }
            if (switches[mode][SEND_AFTERTOUCH] & 1) {
                calculatePressure(2);
            }
            if (switches[mode][SEND_AFTERTOUCH] & 2) {
                calculatePressure(3);
            }

            if (IMUsettings[mode][AUTOCENTER_YAW] == true && (millis() - autoCenterYawTimer) > (IMUsettings[mode][AUTOCENTER_YAW_INTERVAL] * 250)) {  // Recenter yaw when we send a new note if there has been enough silence.
                centerIMU();
            }
        }


        if (notewason && !switches[mode][LEGATO]) {  // Send prior noteoff now if legato is selected.
            sendMIDI(NOTE_OFF, mainMidiChannel, notePlaying, 64);
            notewason = 0;
        }


        if (WARBL2settings[MIDI_DESTINATION] == 0 || connIntvl == 0) {                                                         // Only send here if not connected to BLE (to reduce jitter). I can't detect much difference, if any. (AM)
            if (ED[mode][SEND_PRESSURE] == 1 || switches[mode][SEND_AFTERTOUCH] != 0 || switches[mode][SEND_VELOCITY] == 1) {  // Need to send pressure prior to note, in case we are using it for velocity.
                sendPressure(true);
            }
        }

        // Set it now so that send pitchbend will operate correctly.
        noteon = 1;  // Keep track of the fact that there's a note turned on.
        notePlaying = newNote + shift;
        resetBendFilter = true;

        // Send pitch bend immediately prior to note if necessary.
        if (switches[mode][IMMEDIATE_PB]) {
            calculateAndSendPitchbend();
        }

        sendMIDI(NOTE_ON, mainMidiChannel, newNote + shift, velocity);  // Send the new note.

        if (notewason) {
            // Turn off the previous note after turning on the new one (if it wasn't already done above).
            // We do it after to signal to synths that the notes are legato (not all synths will respond to this).
            sendMIDI(NOTE_OFF, mainMidiChannel, notewasplaying, 64);
        }

        pitchBendTimer = millis();  // For some reason it sounds best if we don't send pitchbend right away after starting a new note.
        noteOnTimestamp = millis();
        powerDownTimer = pitchBendTimer;  // Reset the powerDown timer because we're actively sending notes.

        prevNote = newNote;

        if (ED[mode][DRONES_CONTROL_MODE] == 2 && !dronesOn) {  // Start drones if drones are being controlled with chanter on/off.
            startDrones();
        }
    }


    if (noteon) {  // Several conditions to turn a note off
        if (
          ((newState == SILENCE && !switches[mode][BAGLESS]) || newNote == 0 || (switches[mode][BAGLESS] && !play)) ||  // If the state drops to 1 (off) or we're in bagless mode and the sound has been turned off.
          (modeSelector[mode] == kModeNorthumbrian && newNote == 63) ||                                                 // Or closed Northumbrian pipe.
          (breathMode != kPressureBell && holeCovered == 0b111111111)) {                                                // Or completely closed pipe with any fingering chart.
            sendMIDI(NOTE_OFF, mainMidiChannel, notePlaying, 64);                                                       // Turn the note off if the breath pressure drops or the bell sensor is covered and all the finger holes are covered.
                                                                                                                        // Keep track.

            if (IMUsettings[mode][AUTOCENTER_YAW] == true) {  // Reset the autocenter yaw timer.
                autoCenterYawTimer = millis();
            }

            // Not sure if this is necessary here because it won't have an effect if there's no note playing(?) (AM)
            //sendPressure(true);

            noteon = 0;

            if (ED[mode][DRONES_CONTROL_MODE] == 2 && dronesOn) {  // Stop drones if drones are being controlled with chanter on/off
                stopDrones();
            }
        }
    }
}










// Blink any LED according to blinkNumber[].
void blink() {

    static unsigned long ledTimer[3];

    for (byte i = 0; i < 3; i++) {
        if (blinkNumber[i] > 0) {
            if ((millis() - ledTimer[i]) >= 200) {
                ledTimer[i] = millis();

                if (LEDon[i]) {
                    analogWrite(LEDpins[i], 0);
                    blinkNumber[i]--;
                    LEDon[i] = 0;
                    return;
                }

                else {
                    analogWrite(LEDpins[i], 1023);
                    LEDon[i] = 1;
                }
            }
        }
    }
}










// Pulse LED with 10-bit sine wave.
void pulse() {

    static bool prevPulseState[] = { false, false, false };
    static float in[] = { 4.712f, 4.712f, 4.712f };
    float out[3];
    const float rate = 0.005f;  // Smaller constant makes pulse speed slower.


    for (byte i = 0; i < 3; i++) {
        if (pulseLED[i] == true) {
            prevPulseState[i] = true;
            in[i] = in[i] + rate;
            if (in[i] > 10.995f)
                in[i] = 4.712f;
            out[i] = sin(in[i]) * 511.5f + 511.5f;
            analogWrite(LEDpins[i], out[i]);
            in[i] = in[i] + rate * out[i] / 1023;  // Modify sine wave to spend less time at higher brightness (human eye can't detect changes in higher values as well).
        } else if (prevPulseState[i] == true) {
            prevPulseState[i] = false;
            analogWrite(LEDpins[i], 0);  // If pulsing was just turned off, write 0.
            in[i] = 4.712f;              // Reset so we'll start from 0 bext time pulse is turned on.
        }
    }
}







//Incoming message from USB
void handleControlChangeFromUSB(byte channel, byte number, byte value) {
    handleControlChange(MIDI_SOURCE_USB, channel, number, value);
}
//Incoming message from BLE
void handleControlChangeFromBLE(byte channel, byte number, byte value) {
    handleControlChange(MIDI_SOURCE_BLE, channel, number, value);
}

// Check for and handle incoming MIDI messages from the WARBL Configuration Tool.
void handleControlChange(byte source, byte channel, byte number, byte value) {
    //Serial.println(channel);
    //Serial.println(number);
    //Serial.println(value);
    //Serial.println("");

    if (number < 120) {  // Chrome sends CC 121 and 123 on all channels when it connects, so ignore these.

        if ((channel) == MIDI_CONFIG_TOOL_CHANNEL) {  // If we're on channel 7, we may be receiving messages from the configuration tool.
            powerDownTimer = millis();                // Reset the powerDown timer because we've heard from the Config Tool.
            blinkNumber[GREEN_LED] = 1;               // Blink once, indicating a received message. Some commands below will change this to three (or zero) blinks.


            /////// CC 102
            if (number == MIDI_CC_102) {                                               // Many settings are controlled by a value in CC 102 (always channel 7).
                if (value >= MIDI_CALIB_MSGS_START && value <= MIDI_CALIB_MSGS_END) {  // Handle sensor calibration commands from the configuration tool.
                    int hindex = (value >> 1) - 1;
                    if ((value & 1) == 0) {
                        toneholeCovered[hindex] -= 5;
                        if ((toneholeCovered[hindex] - toneholeBaseline[hindex] - 4) < 5) {  //if the tonehole calibration gets set too low so that it would never register as being uncovered, send a message to the configuration tool.
                            sendMIDI(MIDI_CC_102_MSG, (MIDI_MAX_CALIB_MSGS_START + (hindex)));
                        }
                    } else {
                        toneholeCovered[((value + 1) >> 1) - 1] += 5;
                    }
                }

                if (value == MIDI_SAVE_CALIB) {  // Save calibration if directed.
                    saveCalibration();
                    blinkNumber[GREEN_LED] = 3;
                }

                else if (value == MIDI_START_CALIB) {  // Begin auto-calibration if directed.
                    blinkNumber[GREEN_LED] = 0;
                    calibration = 1;
                }

                else if (value == MIDI_ENTER_COMM_MODE) {  // When communication is established, send all current settings to tool.
                    communicationMode = 1;
                    communicationModeSource = source;
#if DEBUG_CONFIG_TOOL
                    Serial.print("Entering CommMode from ");
                    Serial.println(communicationModeSource);
#endif
                    sendSettings();
                }

                else if (value == MIDI_EXIT_COMM_MODE) {  // Turn off communication mode.
                    //If comm mode was activated on the same source, it turns it off
                    if (communicationModeSource == source) {
                        communicationMode = 0;
                        communicationModeSource = MIDI_SOURCE_NONE;
                    }
#if DEBUG_CONFIG_TOOL
                    Serial.print("Exiting CommMode from ");
                    Serial.println(source);
#endif
                }


                for (byte i = 0; i < 3; i++) {  // Update the three selected fingering patterns if prompted by the tool.
                    if (value == MIDI_FINGERING_PATTERN_MODE_START + i) {
                        fingeringReceiveMode = i;
                    }
                }

                if ((value >= MIDI_FINGERING_PATTERN_START && value <= MIDI_FINGERING_PATTERN_END) || (value >= MIDI_CUST_FINGERING_PATTERN_START && value <= MIDI_CUST_FINGERING_PATTERN_END)) {
                    modeSelector[fingeringReceiveMode] = value - MIDI_FINGERING_PATTERN_START;
                    loadPrefs();
                }


                for (byte i = 0; i < 3; i++) {  // Update current mode (instrument) if directed.
                    if (value == MIDI_CURRENT_MODE_START + i) {
                        mode = i;
                        play = 0;
                        loadPrefs();     // Load the correct user settings based on current instrument.
                        sendSettings();  // Send settings for new mode to tool.
                        blinkNumber[GREEN_LED] = abs(mode) + 1;
                    }
                }

                for (byte i = 0; i < 4; i++) {  // Update current pitchbend mode if directed.
                    if (value == MIDI_PB_MODE_START + i) {
                        pitchBendModeSelector[mode] = i;
                        loadPrefs();
                        blinkNumber[GREEN_LED] = abs(pitchBendMode) + 1;
                    }
                }

                for (byte i = 0; i < 5; i++) {  // Update current breath mode if directed.
                    if (value == MIDI_BREATH_MODE_START + i) {
                        breathModeSelector[mode] = i;
                        loadPrefs();  // Load the correct user settings based on current instrument.
                        blinkNumber[GREEN_LED] = abs(breathMode) + 1;
                    }
                }

                for (byte i = 0; i < kGESTURESnVariables; i++) {  // Update button receive mode (this indicates the row in the button settings for which the next received byte will be).
                    if (value == MIDI_GESTURE_START + i) {
                        buttonReceiveMode = i;
                        blinkNumber[GREEN_LED] = 0;
                    }
                }

                for (byte i = 0; i < kGESTURESnVariables; i++) {  // Update button configuration
                    if (buttonReceiveMode == i) {
                        for (byte k = 0; k < 5; k++) {  // Update column 1 (MIDI action).
                            if (value == 112 + k) {
                                buttonPrefs[mode][i][1] = k;
                            }
                        }
                    }
                }

                for (byte i = 0; i < 3; i++) {  // Update momentary
                    if (buttonReceiveMode == i) {
                        if (value == MIDI_MOMENTARY_OFF) {
                            momentary[mode][i] = 0;
                            noteOnOffToggle[i] = 0;
                        } else if (value == MIDI_MOMENTARY_ON) {
                            momentary[mode][i] = 1;
                            noteOnOffToggle[i] = 0;
                        }
                    }
                }

                if (value == MIDI_DEFAULT_MODE_START) {  // Set current Instrument as default and save default to settings.
                    defaultMode = mode;
                    writeEEPROM(EEPROM_DEFAULT_MODE, defaultMode);
                }


                if (value == MIDI_SAVE_AS_DEFAULTS_CURRENT) {  // Save settings as the defaults for the current instrument
                    saveSettings(mode);
                    blinkNumber[GREEN_LED] = 3;
                }


                else if (value == MIDI_SAVE_AS_DEFAULTS_ALL) {  // Save settings as the defaults for all instruments
                    for (byte k = 0; k < 3; k++) {
                        saveSettings(k);
                    }
                    loadFingering();
                    loadSettingsForAllModes();
                    loadPrefs();
                    blinkNumber[GREEN_LED] = 3;

                }

                else if (value == MIDI_RESTORE_FACTORY) {  // Restore all factory settings
                    restoreFactorySettings();
                    blinkNumber[GREEN_LED] = 3;
                }
            }




            /////// CC 103
            else if (number == MIDI_CC_103) {
                senseDistanceSelector[mode] = value;
                loadPrefs();
            }

            else if (number == MIDI_CC_117) {
                unsigned long v = value * 8191UL / 100;
                vibratoDepthSelector[mode] = v;  // Scale vibrato depth in cents up to pitchbend range of 0-8191.
                loadPrefs();
            }


            for (byte i = 0; i < 3; i++) {  // Update noteshift.
                if (number == MIDI_CC_111 + i) {
                    if (value == MIDI_STICKS_MODE) {
                        sticksModeTimer = millis();         // We will be toggling hidden "sticks" mode, if "autocalibrate bell sensor only" is clicked within 10 seconds.
                        prevKey = noteShiftSelector[mode];  // Remember the current key because we'll need to reset it if we're entering or exiting sticks mode.
                    }
                    if (value < 50) {
                        noteShiftSelector[i] = value;
                    } else {
                        noteShiftSelector[i] = -127 + value;
                    }
                    loadPrefs();
                }
            }



            /////// CC 104
            if (number == MIDI_CC_104) {  // Update receive mode, used for advanced pressure range sliders, switches, and expression and drones panel settings (this indicates the variable for which the next received byte on CC 105 will be).
                pressureReceiveMode = value;
            }



            /////// CC 105
            else if (number == MIDI_CC_105) {

                if (pressureReceiveMode <= MIDI_PRESS_SELECT_VARS_END) {
                    pressureSelector[mode][pressureReceiveMode - 1] = value;  // Advanced pressure values
                    loadPrefs();
                }

                else if (pressureReceiveMode <= MIDI_ED_VARS_END) {
                    ED[mode][pressureReceiveMode - MIDI_ED_VARS_START] = value;  // Expression and drones settings
                    loadPrefs();
                }

                else if (pressureReceiveMode == MIDI_LEARNED_PRESS_LSB) {
                    LSBlearnedPressure = value;

                }

                else if (pressureReceiveMode == MIDI_LEARNED_PRESS_MSB) {
                    learnedPressureSelector[mode] = (value << 7) | LSBlearnedPressure;
                    loadPrefs();
                }


                else if (pressureReceiveMode <= MIDI_SWITCHES_VARS_END) {
                    switches[mode][pressureReceiveMode - MIDI_SWITCHES_VARS_START] = value;  // Switches in the slide/vibrato and register control panels.
                    loadPrefs();
                }

                else if (pressureReceiveMode == MIDI_BEND_RANGE) {
                    midiBendRangeSelector[mode] = value;
                    loadPrefs();
                }

                else if (pressureReceiveMode == MIDI_MIDI_CHANNEL) {
                    midiChannelSelector[mode] = value;
                    loadPrefs();
                }

                else if (pressureReceiveMode <= MIDI_ED_VARS2_END) {
                    ED[mode][pressureReceiveMode - MIDI_ED_VARS2_OFFSET] = value;  // More expression and drones settings.
                    loadPrefs();
                }

                else if (pressureReceiveMode >= MIDI_CC_109_OFFSET && pressureReceiveMode < (kIMUnVariables + MIDI_CC_109_OFFSET)) {
                    IMUsettings[mode][pressureReceiveMode - MIDI_CC_109_OFFSET] = value;  // IMU settings
                    loadPrefs();
                }

                else if (pressureReceiveMode >= MIDI_CUSTOM_CHARTS_OFFSET_START && pressureReceiveMode <= MIDI_CUSTOM_CHARTS_OFFSET_END) {  // WARBL2 Custom fingering charts
                    blinkNumber[GREEN_LED] = 0;
                    WARBL2CustomChart[WARBL2CustomChartReceiveByte] = value;  // Put the value in the custom chart.
                    WARBL2CustomChartReceiveByte++;                           // Increment the location.

                    if (WARBL2CustomChartReceiveByte == 256) {
                        WARBL2CustomChartReceiveByte = 0;  // Reset for next time;
                        for (int i = 0; i < 256; i++) {    // Write the chart to EEPROM.
                            writeEEPROM((EEPROM_CUSTOM_FINGERING_START + (256 * (pressureReceiveMode - MIDI_CUSTOM_CHARTS_OFFSET_START))) + i, WARBL2CustomChart[i]);
                        }
                        blinkNumber[GREEN_LED] = 3;
                        sendMIDI(MIDI_CUSTOM_CHARTS_RCVD);  // Indicate success.
                        loadPrefs();
                    }
                }
            }



            /////// CC 109
            if ((number == MIDI_CC_109 && value < kIMUnVariables)
                || (number == MIDI_CC_109 && value >= MIDI_CUSTOM_CHARTS_START && value <= MIDI_CUSTOM_CHARTS_END)) {  // Indicates that value for IMUsettings variable will be sent on CC 105.
                pressureReceiveMode = value + MIDI_CC_109_OFFSET;                                                      // Add to the value because lower pressureReceiveModes are used for other variables.
                blinkNumber[GREEN_LED] = 0;
            }


            /////// CC 106
            if (number == MIDI_CC_106 && value > MIDI_ACTION_MIDI_CHANNEL_END) {

                if (value >= MIDI_ENA_VIBRATO_HOLES_START && value <= MIDI_ENA_VIBRATO_HOLES_END) {  // Update enabled vibrato holes for "universal" vibrato.
                    bitSet(vibratoHolesSelector[mode], value - MIDI_ENA_VIBRATO_HOLES_START);
                    loadPrefs();
                }

                else if (value >= MIDI_DIS_VIBRATO_HOLES_START && value <= MIDI_DIS_VIBRATO_HOLES_END) {
                    bitClear(vibratoHolesSelector[mode], value - MIDI_DIS_VIBRATO_HOLES_START);
                    loadPrefs();
                }

                else if (value == MIDI_STARTUP_CALIB) {
                    useLearnedPressureSelector[mode] = 0;
                    loadPrefs();
                }

                else if (value == MIDI_USE_LEARNED_CALIB) {
                    useLearnedPressureSelector[mode] = 1;
                    loadPrefs();
                }

                else if (value == MIDI_LEARN_INITIAL_NOTE_PRESS) {
                    learnedPressureSelector[mode] = sensorValue;
                    sendMIDICouplet(MIDI_SEND_LEARNED_PRESSURE_LSB, learnedPressureSelector[mode] & 0x7F);  // Send LSB of learned pressure.
                    sendMIDICouplet(MIDI_SEND_LEARNED_PRESSURE_MSB, learnedPressureSelector[mode] >> 7);    // Send MSB of learned pressure.
                    loadPrefs();
                }

                else if (value == MIDI_CALIB_BELL_SENSOR) {  // Autocalibrate bell sensor only, or turn on stick mode using "hidden" Config Tool sequence.
                    blinkNumber[GREEN_LED] = 0;
                    if ((millis() - sticksModeTimer) < 10000) {  // Hidden way to turn on sticks mode.
                        IMUsettings[mode][STICKS_MODE] = !IMUsettings[mode][STICKS_MODE];
                        if (IMUsettings[mode][STICKS_MODE] == true) {
                            blinkNumber[GREEN_LED] = 3;
                        } else {
                            blinkNumber[GREEN_LED] = 1;
                        }
                        noteShiftSelector[mode] = prevKey;  // Reset the key to the previous value because it was only changed to toggle sticksMode.
                        sendMIDI(MIDI_SEND_CC, (MIDI_CC_111 + mode), noteShiftSelector[mode]);
                        loadPrefs();
                        return;
                    }
                    calibration = 2;

                }


                else if (value == MIDI_LEARN_DRONES_PRESSURE) {
                    int tempPressure = sensorValue;
                    ED[mode][DRONES_PRESSURE_LOW_BYTE] = tempPressure & 0x7F;
                    ED[mode][DRONES_PRESSURE_HIGH_BYTE] = tempPressure >> 7;

                    sendMIDICouplet(MIDI_SEND_DRONES_PRESSURE_LSB, ED[mode][DRONES_PRESSURE_LOW_BYTE]);   // Send LSB of learned drones pressure
                    sendMIDICouplet(MIDI_SEND_DRONES_PRESSURE_MSB, ED[mode][DRONES_PRESSURE_HIGH_BYTE]);  // Send MSB of learned drones pressure
                }


                else if (value == MIDI_SAVE_CALIB_AS_FACTORY) {                                      // Save current sensor calibration as factory calibration
                    for (int i = EEPROM_BASELINE_CALIB_START; i < EEPROM_SENSOR_CALIB_START; i++) {  // Save baseline calibration as factory baseline
                        writeEEPROM(i + EEPROM_FACTORY_SETTINGS_START, readEEPROM(i));
                    }
                    for (byte i = EEPROM_SENSOR_CALIB_START; i <= EEPROM_SENSOR_CALIB_SAVED; i++) {
                        writeEEPROM(i + EEPROM_FACTORY_SETTINGS_START, readEEPROM(i));
                    }
                    for (int i = EEPROM_XGYRO_CALIB_START; i < EEPROM_RESERVED_TESTING; i++) {  // Save gyroscope calibration as factory calibration
                        writeEEPROM(i + EEPROM_FACTORY_SETTINGS_START, readEEPROM(i));
                    }
                    blinkNumber[GREEN_LED] = 2;
                }

                else if (value == MIDI_CALIB_IMU) {
                    calibrateIMU();
                }

                else if (value >= MIDI_WARBL2_SETTINGS_START && value < (MIDI_WARBL2_SETTINGS_START + kWARBL2SETTINGSnVariables)) {
                    WARBL2settingsReceiveMode = value - MIDI_WARBL2_SETTINGS_START;
                }

                else if (value == MIDI_CENTER_YAW) {  // Recenter IMU heading based on current
                    centerIMU();
                }

                else if (value == MIDI_RESET_PITCH_EXPRESSION) {  // Recenter IMU heading based on current
                    resetExpressionOverrideDefaults();
                }

                else if (value >= MIDI_BUTTON_ACTIONS_START) {
                    for (byte i = 0; i < kGESTURESnVariables; i++) {  // Update button configuration
                        if (buttonReceiveMode == i) {

                            for (byte j = 0; j < 27; j++) {  // Update column 0 (action).
                                if (value == MIDI_BUTTON_ACTIONS_START + j) {
                                    buttonPrefs[mode][i][0] = j;
                                }
                            }
                        }
                    }
                }
            }




            /////// CC 119
            if (number == MIDI_CC_119) {
                WARBL2settings[WARBL2settingsReceiveMode] = value;
                for (byte r = 0; r < kWARBL2SETTINGSnVariables; r++) {  // Save the WARBL2settings array each time it is changed by the Config Tool because it is independent of mode.
                    writeEEPROM(EEPROM_WARBL2_SETTINGS_START + r, WARBL2settings[r]);
                }
            }



            for (byte i = 0; i < 8; i++) {  // Update channel, byte 2, byte 3 for MIDI message for button MIDI command for row i
                if (buttonReceiveMode == i) {
                    if (number == MIDI_CC_106 && value <= MIDI_ACTION_MIDI_CHANNEL_END) {
                        buttonPrefs[mode][i][2] = value;
                    } else if (number == MIDI_CC_107) {
                        buttonPrefs[mode][i][3] = value;
                    } else if (number == MIDI_CC_108) {
                        buttonPrefs[mode][i][4] = value;
                    }
                }
            }
        }
    }  // End of ignore CCs 121, 123
}









// Mouthpiece sip detection for triggering user-defined actions (same as button actions).
void detectSip() {

    const byte sipThreshold = 60;  // Pressure value must be this much below the ambient pressure to trigger a sip. The max negative pressure is around 75, so the threshold must be less than that to work. There's not much to work with.
    const byte hysteresis = 5;

    static bool sip = false;
    static byte integrator = 0;  // Keeps track of number of high pressure readings
    static byte debounce = 20;   // Number of high pressure readings required to turn sip off

    if (sensorValue < (sensorCalibration - sipThreshold) && !sip) {  //Immediately trigger sip if we've crossed the threshold.
        performAction(8);
        sip = true;
    }

    if ((sensorValue >= (sensorCalibration - sipThreshold + hysteresis)) && sip) {  // Debounce turning off sip.
        integrator++;
    }

    if (integrator >= debounce) {
        integrator = 0;
        sip = false;
    }
}









// IMU shake detection for triggering user-defined actions.
// Note that shake duration and threshold can be changed in setuo().
void detectShake() {

    static bool shake = false;
    static byte integrator = 0;  // Keeps track of number of non-shake events
    static byte debounce = 50;   // Number of non-shake events required to turn shake off

    if (shakeDetected) {  //Immediately trigger shake if detected by the IMU
        if (!shake) {
            performAction(9);
            shake = true;
        }
    }

    else if (shake) {  // Debounce turning off shake.
        integrator++;
    }

    if (integrator >= debounce) {
        integrator = 0;
        shake = false;
    }
}










// Interpret button presses. If the button is being used for momentary MIDI messages we ignore other actions with that button (except "secret" actions involving the toneholes).
void handleButtons() {
    // First, some housekeeping

    if (shiftState == 1 && released[1] == 1) {  // If button 1 was only being used along with another button, we clear the just-released flag for button 1 so it doesn't trigger another control change.
        released[1] = 0;
        //buttonUsed = 0;  // Clear the button activity flag, so we won't handle them again until there's been new button activity.
        shiftState = 0;
    }


    // Then, a few hard-coded actions that can't be changed by the configuration tool:
    //_______________________________________________________________________________

    if (justPressed[0] && !pressed[2] && !pressed[1]) {
        if (ED[mode][DRONES_CONTROL_MODE] == 1) {
            if (holeCovered >> 1 == 0b00001000) {  // Turn drones on/off if button 0 is pressed and fingering pattern is 0 0001000.
                justPressed[0] = 0;
                specialPressUsed[0] = 1;
                if (!dronesOn) {
                    startDrones();
                } else {
                    stopDrones();
                }
            }
        }

        if (switches[mode][SECRET]) {
            if (holeCovered >> 1 == 0b00010000) {  // Change pitchbend mode if button 0 is pressed and fingering pattern is 0 0000010.
                justPressed[0] = 0;
                specialPressUsed[0] = 1;
                changePitchBend();
            }

            else if (holeCovered >> 1 == 0b00000010) {  // Change instrument if button 0 is pressed and fingering pattern is 0 0000001.
                justPressed[0] = 0;
                specialPressUsed[0] = 1;
                changeInstrument();
            }
        }
    }


    // Now the button actions that can be changed with the Configuration Tool.
    //_______________________________________________________________________________


    for (byte i = 0; i < 3; i++) {


        if (released[i] && (momentary[mode][i] || (pressed[0] + pressed[1] + pressed[2] == 0))) {  // Do action for a button release ("click") NOTE: button array is zero-indexed, so "button 1" in all documentation is button 0 here (same for others).
            if (!specialPressUsed[i]) {
                //20240629 MrMep DoubleClick handling
                if (switches[mode][BUTTON_DOUBLE_CLICK] && !momentary[mode][i]) {  //Double click is active on buttons, and this button is not in momentary
                    if (waitingSecondClick[i]) {                                   //We already had a first click
                        waitingSecondClick[i] = false;
                        if (doubleClickTimer < DOUBLE_CLICK_WAIT_INTERVAL) {  //Timer has not expired yet, we had second clic
                            performAction(i);
                        }     //The else is managed above in checkButtons()
                    } else {  //This is the first click, activate timer
                        waitingSecondClick[i] = true;
                        doubleClickTimer = 0;
                    }
                } else {
                    performAction(i);
                }  // We ignore it if the button was just used for a hard-coded command involving a combination of fingerholes.
            }
            released[i] = 0;
            specialPressUsed[i] = 0;
        }


        if (longPress[i] && (pressed[0] + pressed[1] + pressed[2] == 1) && !momentary[mode][i]) {  // Do action for long press, assuming no other button is pressed.

            performAction(5 + i);
            longPressUsed[i] = 1;
            longPress[i] = 0;
            //longPressCounter[i] = 0;
        }


        // Presses of individual buttons (as opposed to releases) are special cases used only if we're using buttons to send MIDI on/off messages and "momentary" is selected. We'll handle these in a separate function.
        if (justPressed[i]) {
            justPressed[i] = 0;
            handleMomentary(i);  //do action for button press.
        }
    }


    if (pressed[1]) {
        if (released[0] && !momentary[mode][0]) {  // Do action for button 1 held and button 0 released
            released[0] = 0;
            shiftState = 1;
            performAction(3);
        }

        if (released[2] && !momentary[mode][1]) {  // Do action for button 1 held and button 2 released
            released[2] = 0;
            shiftState = 1;
            performAction(4);
        }
    }
}







// Perform desired action in response to buttons
void performAction(byte action) {

    if (communicationMode) {
        sendMIDICouplet(MIDI_SEND_BUTTON_ACTION, action);
    }

    switch (buttonPrefs[mode][action][0]) {

        case NO_ACTION:
            break;

        case SEND_MIDI_MESSAGE:

            if (buttonPrefs[mode][action][1] == 0) {
                if (noteOnOffToggle[action] == 0) {
                    sendMIDI(NOTE_ON, buttonPrefs[mode][action][2], buttonPrefs[mode][action][3], buttonPrefs[mode][action][4]);
                    noteOnOffToggle[action] = 1;
                } else if (noteOnOffToggle[action] == 1) {
                    sendMIDI(NOTE_OFF, buttonPrefs[mode][action][2], buttonPrefs[mode][action][3], buttonPrefs[mode][action][4]);
                    noteOnOffToggle[action] = 0;
                }
                blinkNumber[GREEN_LED] = 1;
            }


            if (buttonPrefs[mode][action][1] == 1) {
                if (!momentary[mode][action]) {
                    sendMIDI(CONTROL_CHANGE, buttonPrefs[mode][action][2], buttonPrefs[mode][action][3], buttonPrefs[mode][action][4]);
                } else {
                    sendMIDI(CONTROL_CHANGE, buttonPrefs[mode][action][2], buttonPrefs[mode][action][3], 0);  // If momentary is turned on, when a button is released we send a CC of 0. This allows temporarily turning on CC "switches" like CC 64-69.
                }
                blinkNumber[GREEN_LED] = 1;
            }

            if (buttonPrefs[mode][action][1] == 2) {
                sendMIDI(PROGRAM_CHANGE, buttonPrefs[mode][action][2], buttonPrefs[mode][action][3]);
                blinkNumber[GREEN_LED] = 1;
            }

            if (buttonPrefs[mode][action][1] == 3) {  // Increase program change
                if (program < 127) {
                    program++;
                } else {
                    program = 0;
                }
                sendMIDI(PROGRAM_CHANGE, buttonPrefs[mode][action][2], program);
                blinkNumber[GREEN_LED] = 1;
            }

            if (buttonPrefs[mode][action][1] == 4) {  // Decrease program change
                if (program > 0) {
                    program--;
                } else {
                    program = 127;
                }
                sendMIDI(PROGRAM_CHANGE, buttonPrefs[mode][action][2], program);
                blinkNumber[GREEN_LED] = 1;
            }

            break;

        case CHANGE_PITCHBEND_MODE:
            changePitchBend();
            break;

        case CHANGE_INSTRUMENT:
            changeInstrument();
            break;

        case PLAY_STOP:
            play = !play;  // Bagless mode
            blinkNumber[GREEN_LED] = 1;
            break;

        case OCTAVE_SHIFT_UP:
            if (!momentary[mode][action]) {  // Shift up unless we're in momentary mode, otherwise shift down.
                octaveShiftUp();
                blinkNumber[GREEN_LED] = abs(octaveShift);
            } else {
                octaveShiftDown();
            }
            break;

        case OCTAVE_SHIFT_DOWN:
            if (!momentary[mode][action]) {  // Shift down unless we're in momentary mode, otherwise shift up.
                octaveShiftDown();
                blinkNumber[GREEN_LED] = abs(octaveShift);
            } else {
                octaveShiftUp();
            }
            break;

        case MIDI_PANIC:
            for (byte i = 1; i < 17; i++) {
                sendMIDI(CONTROL_CHANGE, i, MIDI_CC_123, 0);
                dronesOn = 0;  // Remember that drones are off, because MIDI panic will have most likely turned them off in all apps.
            }
            blinkNumber[GREEN_LED] = 1;
            break;

        case CHANGE_REGISTER_CONTROL_MODE:
            breathModeSelector[mode]++;
            if (breathModeSelector[mode] == kPressureNModes) {
                breathModeSelector[mode] = kPressureSingle;
            }
            loadPrefs();
            play = 0;
            blinkNumber[GREEN_LED] = abs(breathMode) + 1;
            if (communicationMode) {
                sendMIDI(MIDI_CC_102_MSG, MIDI_BREATH_MODE_START + breathMode);  // Send current breathMode
            }
            break;


        case DRONES_ON_OFF:
            blinkNumber[GREEN_LED] = 1;
            if (!dronesOn) {
                startDrones();
            } else {
                stopDrones();
            }
            break;


        case SEMI_SHIFT_UP:
            if (!momentary[mode][action]) {
                noteShift++;  // Shift up if we're not in momentary mode
                blinkNumber[GREEN_LED] = 1;
            } else {
                noteShift--;  // Shift down if we're in momentary mode, because the button is being released and a previous press has shifted up.
            }
            break;


        case SEMI_SHIFT_DOWN:
            if (!momentary[mode][action]) {
                noteShift--;  // Shift down if we're not in momentary mode
                blinkNumber[GREEN_LED] = 1;
            } else {
                noteShift++;  // Shift up if we're in momentary mode, because the button is being released and a previous press has shifted down.
            }
            break;


        case AUTOCALIBRATE:
            calibration = 1;
            break;


        case POWER_DOWN:
            powerDown(false);
            break;


        case RECENTER_YAW:
            centerIMU();
            blinkNumber[GREEN_LED] = 1;
            break;


        case SHOW_BATTERY_LEVEL:
            {
                byte level = (battLevel + 5) / 10;
                if (level > 0) {
                    blinkNumber[GREEN_LED] = level;  // Blink LED in teal 0-10 times based on battery percentage.
                    blinkNumber[BLUE_LED] = level;
                } else {
                    blinkNumber[RED_LED] = 1;
                }
                break;
            }

        default:
            return;
    }
}







void octaveShiftUp() {
    if (octaveShift < 3) {
        octaveShiftSelector[mode]++;  // Adjust octave shift up, within reason
        octaveShift = octaveShiftSelector[mode];
    }
}







void octaveShiftDown() {
    if (octaveShift > -4) {
        octaveShiftSelector[mode]--;
        octaveShift = octaveShiftSelector[mode];
    }
}








// Cycle through pitchbend modes
void changePitchBend() {
    pitchBendModeSelector[mode]++;
    if (pitchBendModeSelector[mode] == kPitchBendNModes) {
        pitchBendModeSelector[mode] = kPitchBendSlideVibrato;
    }
    loadPrefs();
    blinkNumber[GREEN_LED] = abs(pitchBendMode) + 1;
    if (communicationMode) {
        sendMIDI(MIDI_CC_102_MSG, MIDI_PB_MODE_START + pitchBendMode);  //send current pitchbend mode to configuration tool.
    }
}








// Cycle through instruments
void changeInstrument() {
    mode++;  //set instrument
    if (mode == 3) {
        mode = 0;
    }
    play = 0;
    loadPrefs();  // Load the correct user settings based on current instrument.
    blinkNumber[GREEN_LED] = abs(mode) + 1;
    if (communicationMode) {
        sendSettings();  // Tell communications tool to switch mode and send all settings for current instrument.
    }
}








void handleMomentary(byte button) {
    if (momentary[mode][button]) {
        if (buttonPrefs[mode][button][0] == 1 && buttonPrefs[mode][button][1] == 0) {  // Handle momentary press if we're sending a MIDI message
            sendMIDI(NOTE_ON, buttonPrefs[mode][button][2], buttonPrefs[mode][button][3], buttonPrefs[mode][button][4]);
            noteOnOffToggle[button] = 1;
        }

        if (buttonPrefs[mode][button][0] == 1 && buttonPrefs[mode][button][1] == 1) {
            sendMIDI(CONTROL_CHANGE, buttonPrefs[mode][button][2], buttonPrefs[mode][button][3], buttonPrefs[mode][button][4]);
        }

        // Handle presses for shifting the octave or semitone up or down
        if (buttonPrefs[mode][button][0] == 5) {
            octaveShiftUp();
        }

        if (buttonPrefs[mode][button][0] == 6) {
            octaveShiftDown();
        }

        if (buttonPrefs[mode][button][0] == 10) {
            noteShift++;
        }

        if (buttonPrefs[mode][button][0] == 11) {
            noteShift--;
        }
    }
}








void startDrones() {
    dronesOn = 1;
    switch (ED[mode][DRONES_ON_COMMAND]) {
        case 0:
            sendMIDI(NOTE_ON, ED[mode][DRONES_ON_CHANNEL], ED[mode][DRONES_ON_BYTE2], ED[mode][DRONES_ON_BYTE3]);
            break;
        case 1:
            sendMIDI(NOTE_OFF, ED[mode][DRONES_ON_CHANNEL], ED[mode][DRONES_ON_BYTE2], ED[mode][DRONES_ON_BYTE3]);
            break;
        case 2:
            sendMIDI(CONTROL_CHANGE, ED[mode][DRONES_ON_CHANNEL], ED[mode][DRONES_ON_BYTE2], ED[mode][DRONES_ON_BYTE3]);
            break;
    }
}








void stopDrones() {
    dronesOn = 0;
    switch (ED[mode][DRONES_OFF_COMMAND]) {
        case 0:
            sendMIDI(NOTE_ON, ED[mode][DRONES_OFF_CHANNEL], ED[mode][DRONES_OFF_BYTE2], ED[mode][DRONES_OFF_BYTE3]);
            break;
        case 1:
            sendMIDI(NOTE_OFF, ED[mode][DRONES_OFF_CHANNEL], ED[mode][DRONES_OFF_BYTE2], ED[mode][DRONES_OFF_BYTE3]);
            break;
        case 2:
            sendMIDI(CONTROL_CHANGE, ED[mode][DRONES_OFF_CHANNEL], ED[mode][DRONES_OFF_BYTE2], ED[mode][DRONES_OFF_BYTE3]);
            break;
    }
}








// Find leftmost unset bit, used for finding the uppermost uncovered hole when reading from some fingering charts, and for determining the slidehole.
byte findleftmostunsetbit(uint16_t n) {
    if ((n & (n + 1)) == 0) {
        return 127;  // If number contains all 0s then return 127
    }

    int pos = 0;
    for (int temp = n, count = 0; temp > 0; temp >>= 1, count++)  // Find position of leftmost unset bit.

        if ((temp & 1) == 0)
            pos = count;

    return pos;
}









// This is used the first time the software is run, to copy all the default settings to EEPROM.
void saveFactorySettings() {
    for (byte i = 0; i < 3; i++) {  // Save all the current settings for all three instruments.
        mode = i;
        saveSettings(i);
    }

    writeEEPROM(EEPROM_DEFAULT_MODE, defaultMode);  // Save default mode

    for (byte r = 0; r < kWARBL2SETTINGSnVariables; r++) {  // Save the WARBL2settings array
        writeEEPROM(EEPROM_WARBL2_SETTINGS_START + r, WARBL2settings[r]);
    }

    writeEEPROM(EEPROM_EST_RUNTIME_START, highByte(fullRunTime));  // The initial estimate of the total run time available on a full charge (minutes)
    writeEEPROM(EEPROM_EST_RUNTIME_START + 1, lowByte(fullRunTime));

    writeEEPROM(EEPROM_RUNTIME_START, highByte(prevRunTime));  // The elapsed run time on the currrent charge (minutes)--from the "factory" we set this to an estimated number of minutes because the battery charge state is unknown.
    writeEEPROM(EEPROM_RUNTIME_START + 1, lowByte(prevRunTime));

    writeEEPROM(EEPROM_SETTINGS_SAVED, 3);  // Indicates settings have been saved.

    for (int i = 39; i < 1975; i++) {                                   // Then we read every byte in EEPROM from 39 to 1974 (Doesn't include sensor calibrations because we don't want to overwrite factory calibrations).
        writeEEPROM(EEPROM_FACTORY_SETTINGS_START + i, readEEPROM(i));  // And rewrite them from 2039 to 3974. Then they'll be available to restore later if necessary.
    }

    blinkNumber[GREEN_LED] = 3;
}








// Restore original settings from EEPROM
void restoreFactorySettings() {
    for (int i = 1; i < 1987; i++) {  // Read factory settings and rewrite to the normal settings locations.
        writeEEPROM(i, readEEPROM(EEPROM_FACTORY_SETTINGS_START + i));
    }


    loadFingering();  // Load the newly restored settings
    loadCalibration();
    loadSettingsForAllModes();
    loadPrefs();
    communicationMode = 1;  // We are connected to the Config Tool because that's what initiated restoring settings.
    sendSettings();         // Send the new settings.
}








// Send all settings for current instrument to the WARBL Configuration Tool. New variables should be added at the end to maintain backweard compatability with settings import/export in the Config Tool.
void sendSettings() {

    sendMIDI(MIDI_CC_110_MSG, VERSION);  //Send the firmware version.

    for (byte i = 0; i < 3; i++) {
        sendMIDICouplet(MIDI_CC_102, MIDI_FINGERING_PATTERN_MODE_START + i, MIDI_CC_102, MIDI_FINGERING_PATTERN_START + modeSelector[i]);  //Send the fingering pattern for instrument i.

        if (noteShiftSelector[i] >= 0) {
            sendMIDI(MIDI_SEND_CC, MIDI_CC_111 + i, noteShiftSelector[i]);
        }  // Send noteShift, with a transformation for sending negative values over MIDI.
        else {
            sendMIDI(MIDI_SEND_CC, MIDI_CC_111 + i, noteShiftSelector[i] + 127);
        }
    }

    sendMIDI(MIDI_CC_102_MSG, MIDI_CURRENT_MODE_START + mode);         // Send current instrument.
    sendMIDI(MIDI_CC_102_MSG, MIDI_DEFAULT_MODE_START + defaultMode);  // Send default instrument.

    sendMIDI(MIDI_CC_103_MSG, senseDistance);  // Send sense distance

    sendMIDI(MIDI_CC_117_MSG, vibratoDepth * 100UL / 8191);          // Send vibrato depth, scaled down to cents.
    sendMIDI(MIDI_CC_102_MSG, MIDI_PB_MODE_START + pitchBendMode);   // Send current pitchBend mode.
    sendMIDI(MIDI_CC_102_MSG, MIDI_BREATH_MODE_START + breathMode);  // Send current breathMode.

    sendMIDI(MIDI_CC_106_MSG, MIDI_STARTUP_CALIB + useLearnedPressure);  // Send calibration option.

    sendMIDICouplet(MIDI_SEND_LEARNED_PRESSURE_LSB, learnedPressure & 0x7F);  // Send LSB of learned pressure.
    sendMIDICouplet(MIDI_SEND_LEARNED_PRESSURE_MSB, learnedPressure >> 7);    // Send MSB of learned pressure.

    sendMIDICouplet(MIDI_SEND_BEND_RANGE, midiBendRange);  // Send MIDI bend range

    sendMIDICouplet(MIDI_SEND_MIDI_CHANNEL, mainMidiChannel);  // Send channel


    for (byte i = 0; i < 9; i++) {
        if (bitRead(vibratoHolesSelector[mode], i)) {
            sendMIDI(MIDI_CC_106_MSG, MIDI_ENA_VIBRATO_HOLES_START + i);
        } else {
            sendMIDI(MIDI_CC_106_MSG, MIDI_ENA_VIBRATO_HOLES_START + 10 + i);
        }
    }

    for (byte i = 0; i < kGESTURESnVariables; i++) {
        sendMIDICouplet(MIDI_CC_102, MIDI_GESTURE_START + i, MIDI_CC_106, MIDI_BUTTON_ACTIONS_START + buttonPrefs[mode][i][0]);  // Send  data for button commands row i (click 1, click 2, etc.).

        //This would require a "quadruplet mutex"...
        if (buttonPrefs[mode][i][0] == 1) {  // If the action is a MIDI command, send the rest of the MIDI info for that row.
            sendMIDI(MIDI_CC_102_MSG, MIDI_ACTION_MIDI_START + buttonPrefs[mode][i][1]);
            sendMIDI(MIDI_CC_106_MSG, buttonPrefs[mode][i][2]);
            sendMIDI(MIDI_CC_107_MSG, buttonPrefs[mode][i][3]);
            sendMIDI(MIDI_CC_108_MSG, buttonPrefs[mode][i][4]);
        }
    }

    for (byte i = 0; i < kSWITCHESnVariables; i++) {  // Send settings for switches in the slide/vibrato and register control panels.
        sendMIDICouplet(MIDI_CC_104, i + MIDI_SWITCHES_VARS_START, MIDI_CC_105, switches[mode][i]);
    }

    for (byte i = 0; i < MIDI_ED_VARS_NUMBER; i++) {  // Send settings for expression and drones control panels.
        sendMIDICouplet(MIDI_CC_104, i + MIDI_ED_VARS_START, MIDI_CC_105, ED[mode][i]);
    }

    for (byte i = MIDI_ED_VARS2_START; i < MIDI_ED_VARS2_END + 1; i++) {  // More settings for expression and drones control panels.
        sendMIDICouplet(MIDI_CC_104, i, MIDI_CC_105, ED[mode][i - MIDI_ED_VARS2_OFFSET]);
    }

    for (byte i = 0; i < 3; i++) {
        sendMIDICouplet(MIDI_CC_102, MIDI_GESTURE_START + i, MIDI_CC_102, MIDI_MOMENTARY_OFF + momentary[mode][i]);  // Send  data for momentary.
    }

    for (byte i = 0; i < 12; i++) {
        sendMIDICouplet(MIDI_CC_104, i + MIDI_PRESS_SELECT_VARS_START, MIDI_CC_105, pressureSelector[mode][i]);  // Send pressure variable
    }

    sendMIDI(MIDI_CC_102_MSG, MIDI_CC_102_VALUE_121);  // Tell the Config Tool that the bell sensor is present (always on this version of the WARBL).

    for (byte i = MIDI_WARBL2_SETTINGS_START; i < MIDI_WARBL2_SETTINGS_START + kWARBL2SETTINGSnVariables; i++) {  // Send the WARBL2settings array.
        sendMIDICouplet(MIDI_CC_106, i, MIDI_CC_119, WARBL2settings[i - MIDI_WARBL2_SETTINGS_START]);
    }

    manageBattery(true);  // Do this to send voltage and charging status to Config Tool.

    sendMIDICouplet(MIDI_CC_106, MIDI_BLE_INTERVAL_LSB, MIDI_CC_119, (connIntvl * 100) & 0x7F);  // Send low byte of the connection interval.
    sendMIDICouplet(MIDI_CC_106, MIDI_BLE_INTERVAL_MSB, MIDI_CC_119, (connIntvl * 100) >> 7);    // high low byte of the connection interval.

    for (byte i = 0; i < kIMUnVariables; i++) {  // IMU settings
        sendMIDICouplet(MIDI_CC_109, i, MIDI_CC_105, IMUsettings[mode][i]);
    }
}










// Load saved fingering patterns
void loadFingering() {
    for (byte i = 0; i < 3; i++) {
        modeSelector[i] = readEEPROM(EEPROM_FINGERING_PATTERN_START + i);
        noteShiftSelector[i] = (int8_t)readEEPROM(EEPROM_NOTE_SHIFT_SEL_START + i);

        if (communicationMode) {
            sendMIDICouplet(MIDI_CC_102, MIDI_FINGERING_PATTERN_MODE_START + i, MIDI_CC_102, MIDI_FINGERING_PATTERN_START + modeSelector[i]);  //Send the fingering pattern for instrument i.

            if (noteShiftSelector[i] >= 0) {
                sendMIDI(MIDI_SEND_CC, (MIDI_CC_111 + i), noteShiftSelector[i]);
            }  // Send noteShift, with a transformation for sending negative values over MIDI.
            else {
                sendMIDI(MIDI_SEND_CC, (MIDI_CC_111 + i), noteShiftSelector[i] + 127);
            }
        }
    }
}










// Save settings for current instrument as defaults for given instrument (i).
void saveSettings(byte i) {
    writeEEPROM(EEPROM_FINGERING_PATTERN_START + i, modeSelector[mode]);
    writeEEPROM(EEPROM_NOTE_SHIFT_SEL_START + i, noteShiftSelector[mode]);
    writeEEPROM(EEPROM_SENS_DISTANCE_START + i, senseDistanceSelector[mode]);

    for (byte n = 0; n < kSWITCHESnVariables; n++) {  // Saved in this format, we can add more variables to the arrays without overwriting the existing EEPROM locations.
        writeEEPROM((EEPROM_SWITCHES_START + i + (n * 3)), switches[mode][n]);
    }

    writeEEPROM(EEPROM_VIBRATO_HOLES_START + (i * 2), lowByte(vibratoHolesSelector[mode]));
    writeEEPROM(EEPROM_VIBRATO_HOLES_START + 1 + (i * 2), highByte(vibratoHolesSelector[mode]));

    writeEEPROM(EEPROM_VIBRATO_DEPTH_START + (i * 2), lowByte(vibratoDepthSelector[mode]));
    writeEEPROM(EEPROM_VIBRATO_DEPTH_START + 1 + (i * 2), highByte(vibratoDepthSelector[mode]));

    writeEEPROM(EEPROM_USE_LEARNED_PRESS_START + i, useLearnedPressureSelector[mode]);

    for (byte j = 0; j < 5; j++) {  // Save button configuration for current mode
        for (byte k = 0; k < kGESTURESnVariables; k++) {
            writeEEPROM(EEPROM_BUTTON_PREFS_START + (i * 50) + (j * 10) + k, buttonPrefs[mode][k][j]);
        }
    }

    for (byte h = 0; h < 3; h++) {
        writeEEPROM(EEPROM_MOMENTARY_MODE_START + (i * 3) + h, momentary[mode][h]);
    }

    for (byte q = 0; q < 12; q++) {
        writeEEPROM((EEPROM_PRESSURE_SETTINGS_START + q + (i * 20)), pressureSelector[mode][q]);
    }

    writeEEPROM(EEPROM_LEARNED_PRESSURE_START + (i * 2), lowByte(learnedPressureSelector[mode]));
    writeEEPROM(EEPROM_LEARNED_PRESSURE_START + 1 + (i * 2), highByte(learnedPressureSelector[mode]));

    writeEEPROM(EEPROM_PB_MODE_START + i, pitchBendModeSelector[mode]);
    writeEEPROM(EEPROM_BREATH_MODE_START + i, breathModeSelector[mode]);
    writeEEPROM(EEPROM_MIDI_BEND_RANGE_START + i, midiBendRangeSelector[mode]);
    writeEEPROM(EEPROM_MIDI_CHANNEL_START + i, midiChannelSelector[mode]);

    for (byte n = 0; n < kEXPRESSIONnVariables; n++) {
        writeEEPROM((EEPROM_ED_VARS_START + i + (n * 3)), ED[mode][n]);
    }

    for (byte n = 0; n < kIMUnVariables; n++) {
        writeEEPROM((EEPROM_IMU_SETTINGS_START + i + (n * 3)), IMUsettings[mode][n]);
    }
}









// Load settings for all three instruments from EEPROM.
void loadSettingsForAllModes() {
    // Some things that are independent of mode.
    defaultMode = readEEPROM(EEPROM_DEFAULT_MODE);  // Load default mode.

    for (byte r = 0; r < kWARBL2SETTINGSnVariables; r++) {  //Load the WARBL2settings array.
        WARBL2settings[r] = readEEPROM(EEPROM_WARBL2_SETTINGS_START + r);
    }

    fullRunTime = (word(readEEPROM(EEPROM_EST_RUNTIME_START), readEEPROM(EEPROM_EST_RUNTIME_START + 1)));  // The total run time available on a full charge (minutes)
    prevRunTime = (word(readEEPROM(EEPROM_RUNTIME_START), readEEPROM(EEPROM_RUNTIME_START + 1)));          // The total run time since the last full charge (minutes)

    if (readEEPROM(EEPROM_XGYRO_CALIB_SAVED) == 3) {  // If there has been a gyro calibration saved
        getEEPROM(EEPROM_XGYRO_CALIB_START, gyroXCalibration);
        getEEPROM(EEPROM_YGYRO_CALIB_START, gyroYCalibration);
        getEEPROM(EEPROM_ZGYRO_CALIB_START, gyroZCalibration);
    }

    // Do all this for each mode.
    for (byte i = 0; i < 3; i++) {
        senseDistanceSelector[i] = readEEPROM(EEPROM_SENS_DISTANCE_START + i);

        for (byte n = 0; n < kSWITCHESnVariables; n++) {
            switches[i][n] = readEEPROM(EEPROM_SWITCHES_START + i + (n * 3));
        }


        vibratoHolesSelector[i] = word(readEEPROM(EEPROM_VIBRATO_HOLES_START + 1 + (i * 2)), readEEPROM(EEPROM_VIBRATO_HOLES_START + (i * 2)));
        vibratoDepthSelector[i] = word(readEEPROM(EEPROM_VIBRATO_DEPTH_START + 1 + (i * 2)), readEEPROM(EEPROM_VIBRATO_DEPTH_START + (i * 2)));
        useLearnedPressureSelector[i] = readEEPROM(EEPROM_USE_LEARNED_PRESS_START + i);

        for (byte j = 0; j < 5; j++) {
            for (byte k = 0; k < kGESTURESnVariables; k++) {
                buttonPrefs[i][k][j] = readEEPROM(EEPROM_BUTTON_PREFS_START + (i * 50) + (j * 10) + k);
            }
        }

        for (byte h = 0; h < 3; h++) {
            momentary[i][h] = readEEPROM(EEPROM_MOMENTARY_MODE_START + (i * 3) + h);
        }

        for (byte m = 0; m < 12; m++) {
            pressureSelector[i][m] = readEEPROM(EEPROM_PRESSURE_SETTINGS_START + m + (i * 20));
        }

        learnedPressureSelector[i] = word(readEEPROM(EEPROM_LEARNED_PRESSURE_START + 1 + (i * 2)), readEEPROM(EEPROM_LEARNED_PRESSURE_START + (i * 2)));
        pitchBendModeSelector[i] = readEEPROM(EEPROM_PB_MODE_START + i);
        breathModeSelector[i] = readEEPROM(EEPROM_BREATH_MODE_START + i);
        midiBendRangeSelector[i] = readEEPROM(EEPROM_MIDI_BEND_RANGE_START + i);
        midiChannelSelector[i] = readEEPROM(EEPROM_MIDI_CHANNEL_START + i);


        for (byte n = 0; n < kEXPRESSIONnVariables; n++) {
            ED[i][n] = readEEPROM(EEPROM_ED_VARS_START + i + (n * 3));
        }

        for (byte p = 0; p < kIMUnVariables; p++) {
            IMUsettings[i][p] = readEEPROM(EEPROM_IMU_SETTINGS_START + i + (p * 3));
        }
    }
}







// Resets current mode's expression override defaults.
// Useful for populating older devices after first installing the version that has them and for manually restoring to a "sane" default for the new setup.
void resetExpressionOverrideDefaults() {

    ED[mode][EXPRESSION_MIN] = 0;
    ED[mode][EXPRESSION_MIN_HIGH] = 7;
    ED[mode][EXPRESSION_FIXED_CENTER_PRESSURE] = 8;
    ED[mode][EXPRESSION_MAX_LOW] = 11;
    ED[mode][EXPRESSION_MAX] = 20;
    // 2x cents signed where 64 = 0, and 64+10 = +20 cents, and 64-10 = -20 cents for example
    ED[mode][EXPRESSION_OUT_LOW_CENTS] = 64 - 25;   // -50 cents
    ED[mode][EXPRESSION_OUT_HIGH_CENTS] = 64 + 50;  // +100 cents
    ED[mode][EXPRESSION_OUT_CLAMP] = 1;             // boolean
    ED[mode][EXPRESSION_CURVE_LOW] = 64;            // linear
    ED[mode][EXPRESSION_CURVE_HIGH] = 64;           // linear
}








// Load the correct user settings for the current mode (instrument). This is used at startup and any time settings are changed.
void loadPrefs() {

    vibratoHoles = vibratoHolesSelector[mode];
    octaveShift = octaveShiftSelector[mode];
    noteShift = noteShiftSelector[mode];
    pitchBendMode = pitchBendModeSelector[mode];
    useLearnedPressure = useLearnedPressureSelector[mode];
    learnedPressure = learnedPressureSelector[mode];
    senseDistance = senseDistanceSelector[mode];
    vibratoDepth = vibratoDepthSelector[mode];
    breathMode = breathModeSelector[mode];
    midiBendRange = midiBendRangeSelector[mode];
    mainMidiChannel = midiChannelSelector[mode];
    transientFilterDelay = (pressureSelector[mode][9] + 1) / 1.25;  // This variable was formerly used for vented dropTime (unvented is now unused). Includes a correction for milliseconds

    // Set these variables depending on whether "vented" is selected
    offset = pressureSelector[mode][(switches[mode][VENTED] * 6) + 0];
    multiplier = pressureSelector[mode][(switches[mode][VENTED] * 6) + 1];
    hysteresis = pressureSelector[mode][(switches[mode][VENTED] * 6) + 2];
    jumpTime = ((pressureSelector[mode][(switches[mode][VENTED] * 6) + 4]) + 1) / 1.25;  // Includes a correction for milliseconds
    dropTime = ((pressureSelector[mode][(switches[mode][VENTED] * 6) + 5]) + 1) / 1.25;  // Includes a correction for milliseconds

    // Read a custom chart from EEPROM if we're using one.
    if (modeSelector[mode] == kWARBL2Custom1 || modeSelector[mode] == kWARBL2Custom2 || modeSelector[mode] == kWARBL2Custom3 || modeSelector[mode] == kWARBL2Custom4) {
        for (int i = 0; i < 256; i++) {
            WARBL2CustomChart[i] = readEEPROM((EEPROM_CUSTOM_FINGERING_START + (256 * (modeSelector[mode] - 67))) + i);
        }
    }

    pitchBend = 8192;
    prevPitchBend = 8192;
    expression = 0;
    shakeVibrato = 0;
    IMUpitchbend = 0;
    shakePressureCCMod = 0;
    shakePressureChanPressMod = 0.0f;
    shakePressureKeyPressMod = 0;
    prevHoleCovered = 1;  // Necessary so we know to call getNote() again if the fingering chart has been changed.


    sendMIDI(PITCH_BEND, mainMidiChannel, pitchBend & 0x7F, pitchBend >> 7);

    for (byte i = 0; i < 9; i++) {
        iPitchBend[i] = 0;  // Turn off pitchbend.
        pitchBendOn[i] = 0;
    }

    if (switches[mode][CUSTOM] && pitchBendMode != kPitchBendNone) {
        customEnabled = 1;
    } else (customEnabled = 0);  // Decide here whether custom vibrato can currently be used, so we don't have to do it every time we need to check pitchBend.

    if (switches[mode][FORCE_MAX_VELOCITY]) {
        velocity = 127;  // Set velocity
    } else {
        velocity = 64;
    }


    if (!useLearnedPressure) {
        sensorThreshold[0] = (sensorCalibration + soundTriggerOffset);  // Pressure sensor calibration at startup. We set the on/off threshhold just a bit higher than the reading at startup.
    }

    else {
        sensorThreshold[0] = (learnedPressure + soundTriggerOffset);
    }

    sensorThreshold[1] = sensorThreshold[0] + (offset << 2);  // Threshold for move to second octave

    for (byte i = 0; i < 9; i++) {
        toneholeScale[i] = (((16383.0f / midiBendRange)) / max(toneholeCovered[i] - toneholeBaseline[i] - senseDistance, 1) / 2.0f);                     // This one is for sliding. We multiply by 8 first to reduce rounding errors. We'll divide again later.
        vibratoScale[i] = ((2.0f * (((float)vibratoDepth) / midiBendRange)) / max(toneholeCovered[i] - toneholeBaseline[i] - senseDistance, 1) / 2.0f);  // This one is for vibrato.
    }

    adjvibdepth = vibratoDepth / midiBendRange;  // Precalculations for pitchbend range
    pitchBendPerSemi = 8192.0f / midiBendRange;

    inputPressureBounds[0][0] = (ED[mode][INPUT_PRESSURE_MIN] * 9);  // Precalculate input and output pressure ranges for sending pressure as CC.
    inputPressureBounds[0][1] = (ED[mode][INPUT_PRESSURE_MAX] * 9);
    inputPressureBounds[1][0] = (ED[mode][VELOCITY_INPUT_PRESSURE_MIN] * 9);  // Precalculate input and output pressure ranges for sending pressure as velocity.
    inputPressureBounds[1][1] = (ED[mode][VELOCITY_INPUT_PRESSURE_MAX] * 9);
    inputPressureBounds[2][0] = (ED[mode][AFTERTOUCH_INPUT_PRESSURE_MIN] * 9);  // Precalculate input and output pressure ranges for sending pressure as aftertouch.
    inputPressureBounds[2][1] = (ED[mode][AFTERTOUCH_INPUT_PRESSURE_MAX] * 9);
    inputPressureBounds[3][0] = (ED[mode][POLY_INPUT_PRESSURE_MIN] * 9);  // Precalculate input and output pressure ranges for sending pressure as poly.
    inputPressureBounds[3][1] = (ED[mode][POLY_INPUT_PRESSURE_MAX] * 9);

    outputBounds[0][0] = ED[mode][OUTPUT_PRESSURE_MIN];  // Move all these variables to a more logical order so they can be accessed in FOR loops.
    outputBounds[0][1] = ED[mode][OUTPUT_PRESSURE_MAX];
    outputBounds[1][0] = ED[mode][VELOCITY_OUTPUT_PRESSURE_MIN];
    outputBounds[1][1] = ED[mode][VELOCITY_OUTPUT_PRESSURE_MAX];
    outputBounds[2][0] = ED[mode][AFTERTOUCH_OUTPUT_PRESSURE_MIN];
    outputBounds[2][1] = ED[mode][AFTERTOUCH_OUTPUT_PRESSURE_MAX];
    outputBounds[3][0] = ED[mode][POLY_OUTPUT_PRESSURE_MIN];
    outputBounds[3][1] = ED[mode][POLY_OUTPUT_PRESSURE_MAX];

    curve[0] = ED[mode][CURVE];
    curve[1] = ED[mode][VELOCITY_CURVE];
    curve[2] = ED[mode][AFTERTOUCH_CURVE];
    curve[3] = ED[mode][POLY_CURVE];

    if (enableRegisterHold || IMUsettings[mode][SEND_ROLL] || IMUsettings[mode][SEND_PITCH] || IMUsettings[mode][SEND_YAW] || IMUsettings[mode][PITCH_REGISTER] || IMUsettings[mode][STICKS_MODE] || IMUsettings[mode][MAP_ROLL_TO_PITCHBEND] || IMUsettings[mode][MAP_ELEVATION_TO_PITCHBEND] || IMUsettings[mode][MAP_YAW_TO_PITCHBEND]) {
        sox.setGyroDataRate(LSM6DS_RATE_208_HZ);  // Turn on the gyro if we need it.
    }

    // Calculate upper and lower bounds for IMU pitch (elevation) register mapping.
    byte pitchPerRegister = (IMUsettings[mode][PITCH_REGISTER_INPUT_MAX] - IMUsettings[mode][PITCH_REGISTER_INPUT_MIN]) * 5 / IMUsettings[mode][PITCH_REGISTER_NUMBER];  // Number of degrees per register
    IMUsettings[mode][PITCH_REGISTER_NUMBER] = constrain(IMUsettings[mode][PITCH_REGISTER_NUMBER], 2, 5);                                                                // Sanity check if uninitialized. Higher values will result in writing outside of the pitchRegisterBounds[i] array.
    for (byte i = 0; i < IMUsettings[mode][PITCH_REGISTER_NUMBER] + 1; i++) {
        pitchRegisterBounds[i] = ((i * pitchPerRegister) + IMUsettings[mode][PITCH_REGISTER_INPUT_MIN] * 5) - 90;  // Upper/lower bounds for each register
    }
}






// Calibrate the sensors and store them in EEPROM.
// Mode 1 calibrates all sensors, mode 2 calibrates bell sensor only.
void calibrate() {

    static unsigned long calibrationTimer = 0;

    if (calibration > 0) {
        if (!LEDon[GREEN_LED]) {
            analogWrite(LEDpins[GREEN_LED], 1023);
            LEDon[GREEN_LED] = 1;
            calibrationTimer = millis();

            if (calibration == 1) {  // Calibrate all sensors if we're in calibration "mode" 1.
                for (byte i = 1; i < 9; i++) {
                    toneholeCovered[i] = 0;      // First set the calibration to 0 for all of the sensors so it can only be increassed by calibrating.
                    toneholeBaseline[i] = 1024;  // And set baseline high so it can only be reduced.
                }
            }

            toneholeCovered[0] = 0;  // Also zero the bell sensor if it's plugged in (doesn't matter which calibration mode for this one).
            toneholeBaseline[0] = 1024;

            return;  // We return once to make sure we've gotten some new sensor readings.
        }

        if ((calibration == 1 && ((millis() - calibrationTimer) <= 10000)) || (calibration == 2 && ((millis() - calibrationTimer) <= 5000))) {  // Then set the calibration to the highest reading during the next ten seconds(or five seconds if we're only calibrating the bell sensor).
            if (calibration == 1) {
                for (byte i = 1; i < 9; i++) {
                    if (toneholeCovered[i] < toneholeRead[i]) {  // Covered calibration
                        toneholeCovered[i] = toneholeRead[i];
                    }

                    if (toneholeBaseline[i] > toneholeRead[i]) {  // Baseline calibration
                        toneholeBaseline[i] = toneholeRead[i];
                    }
                }
            }

            if (toneholeCovered[0] < toneholeRead[0]) {
                toneholeCovered[0] = toneholeRead[0];  // Calibrate the bell sensor too if it's plugged in.
            }
            if (toneholeBaseline[0] > toneholeRead[0]) {
                toneholeBaseline[0] = toneholeRead[0];  // Calibrate the bell sensor too if it's plugged in.
            }
        }

        if ((calibration == 1 && ((millis() - calibrationTimer) > 10000)) || (calibration == 2 && ((millis() - calibrationTimer) > 5000))) {

            // Just in case bell wasn't exercised
            if (abs(toneholeCovered[0] - toneholeBaseline[0]) < 50) {
                toneholeCovered[0] = toneholeBaseline[0] + 50;

            } else {
                int feeloffset = (int)((toneholeCovered[0] - toneholeBaseline[0]) * 0.2f);
                toneholeCovered[0] -= feeloffset;
            }

            if (calibration == 1) {
                // Adjust for calibration feel.
                for (byte i = 1; i < 9; i++) {
                    int feeloffset = (int)((toneholeCovered[i] - toneholeBaseline[i]) * 0.2f);
                    toneholeCovered[i] -= feeloffset;
                }
            }

            saveCalibration();
            loadPrefs();  // Do this so pitchbend scaling will be recalculated.
        }
    }
}







// Save sensor calibration (EEPROM bytes up to 34 are used (plus byte 37 to indicate a saved calibration).
void saveCalibration() {
    for (byte i = 0; i < 9; i++) {
        writeEEPROM((i + 9) * 2, highByte(toneholeCovered[i]));
        writeEEPROM(((i + 9) * 2) + 1, lowByte(toneholeCovered[i]));
        writeEEPROM((1 + i), lowByte(toneholeBaseline[i]));
        if (i > 0) {
            // Baseline high byte stored for toneholes (only had this much reserved space)
            writeEEPROM(9 + i, highByte(toneholeBaseline[i]));
        }
    }
    calibration = 0;
    writeEEPROM(EEPROM_SENSOR_CALIB_SAVED, 3);  // We write a 3 to address 37 to indicate that we have stored a set of calibrations.
    analogWrite(LEDpins[GREEN_LED], 0);
    LEDon[GREEN_LED] = 0;
}








// Load the stored sensor calibrations from EEPROM
void loadCalibration() {

    for (byte i = EEPROM_SENSOR_CALIB_START; i < EEPROM_SENSOR_CALIB_START + 18; i += 2) {
        byte high = readEEPROM(i);
        byte low = readEEPROM(i + 1);
        byte index = (i - EEPROM_SENSOR_CALIB_START) / 2;
        toneholeCovered[index] = word(high, low);

        low = readEEPROM(EEPROM_BASELINE_CALIB_START + index);
        if (index > 0) {
            high = readEEPROM(EEPROM_BASELINE_CALIB_START + 8 + index);
        } else {
            high = 0;
        }
        toneholeBaseline[index] = word(high, low);
    }
}








void calculateAndSendPressure() {
    bool shakemod = IMUsettings[mode][Y_SHAKE_MOD_CHPRESS] || IMUsettings[mode][Y_SHAKE_MOD_KEYPRESS] || IMUsettings[mode][Y_SHAKE_MOD_CC];
    if (abs(prevSensorValue - smoothed_pressure) > 1 || shakemod) {  // If pressure has changed more than a little, send it.
        if (ED[mode][SEND_PRESSURE]) {
            calculatePressure(0);
        }
        if (switches[mode][SEND_VELOCITY]) {
            calculatePressure(1);
        }
        if (switches[mode][SEND_AFTERTOUCH] & 1) {
            calculatePressure(2);
        }
        if (switches[mode][SEND_AFTERTOUCH] & 2) {
            calculatePressure(3);
        }

        sendPressure(false);

        prevSensorValue = smoothed_pressure;
    }
    static int previousTenBitPressure = sensorValue;

    if (abs(previousTenBitPressure - sensorValue) > 2) {  // Only send pressure to the Config Tool if the 10-bit value has changed, because it's less noisy than 12 bit.
        sendToConfig(false, true);                        // Put the new pressure into a queue to be sent later so that it's not sent during the same connection interval as a new note (to decrease BLE payload size).
        previousTenBitPressure = sensorValue;
    }
}








// Calculate pressure data for CC, velocity, channel pressure, and key pressure if those options are selected.
void calculatePressure(byte pressureOption) {

    long scaledPressure;

    scaledPressure = smoothed_pressure - (sensorThreshold[0] << 2);  // 12-bit input pressure range is ~400-4000. Bring this down to 0-3600.

    if (scaledPressure < 0) {
        scaledPressure = 0;
    }

    scaledPressure = constrain(scaledPressure, inputPressureBounds[pressureOption][0] << 2, inputPressureBounds[pressureOption][1] << 2);     // Constrain the pressure to the input range.
    scaledPressure = map(scaledPressure, inputPressureBounds[pressureOption][0] << 2, inputPressureBounds[pressureOption][1] << 2, 0, 1024);  // Scale pressure to a range of 0-1024.

    if (curve[pressureOption] == 1) {  // For this curve, cube the input and scale back down.
        scaledPressure = ((scaledPressure * scaledPressure * scaledPressure) >> 20);
    }

    else if (curve[pressureOption] == 2 && scaledPressure != 0) {  // Log curve.
        float pressureLog2 = log(scaledPressure) / log(2);
        scaledPressure = pressureLog2 * pressureLog2 * 10.24f;
    }

    // Else curve 0 is linear, so no transformation.
    mappedPressureHiRes[pressureOption] = (scaledPressure * (outputBounds[pressureOption][1] - outputBounds[pressureOption][0]) / 1024.0f) + outputBounds[pressureOption][0];  // Map to output pressure range.

    inputPressureBounds[pressureOption][3] = (unsigned int)mappedPressureHiRes[pressureOption];

    if (pressureOption == 1) {  // Set velocity to mapped pressure if desired.
        velocity = inputPressureBounds[pressureOption][3];
        if (velocity == 0) {  // Use a minumum of 1 for velocity so a note is still turned on (changed in v. 2.1).
            velocity = 1;
        }
    }
}








// Calculate how often to send pressure data.
byte calculatePressureInterval(void) {

    byte ret = 5;

    if ((millis() - noteOnTimestamp) < 20) {
        ret = 2;
    }

    if (WARBL2settings[MIDI_DESTINATION] != 0) {  // Use a longer interval if sending BLE.
        ret = connIntvl + 2;
    }
    return ret;
}









// Send pressure data.
void sendPressure(bool force) {

    if (ED[mode][SEND_PRESSURE] == 1) {
        const int usedmod = inputPressureBounds[0][3] > 0 ? shakePressureCCMod : 0;
        int sendp = constrain((int)inputPressureBounds[0][3] + usedmod, 0, 127);
        if (sendp != prevCCPressure || force) {
            sendMIDI(CONTROL_CHANGE, ED[mode][PRESSURE_CHANNEL], ED[mode][PRESSURE_CC], sendp);  // Send MSB of pressure mapped to the output range.
            prevCCPressure = sendp;
        }
    }

    if ((switches[mode][SEND_AFTERTOUCH] & 1)) {
        // hack
        const float usedmod = mappedPressureHiRes[2] > 0.0f ? shakePressureChanPressMod : 0.0f;
        float hiresOut = constrain(mappedPressureHiRes[2] + usedmod, 0.0f, 127.0f);
        float ipart = 0.0f;
        int sendl = (int)(modf(hiresOut, &ipart) * 128);
        sendl = (!noteon && sensorValue <= 100) ? 0 : sendl;
        int sendm = (!noteon && sensorValue <= 100) ? 0 : constrain((int)ipart, 0, 127);
        if (sendm != prevChanPressure || (ED[mode][AFTERTOUCH_MPEPLUS] && sendl != prevChanPressureLSB) || force) {


            // A bit of midi bandwidth optimization to not bother sending LSB if the MSB has a large jumps (moving fast).
            if (ED[mode][AFTERTOUCH_MPEPLUS] && abs(sendm - prevChanPressure) < 16) {
                // MPE+ uses CC 87 sent just prior as LSB
                sendMIDI(CONTROL_CHANGE, mainMidiChannel, 87, sendl);  // Send LSB
                prevChanPressureLSB = sendl;
            }
            sendMIDI(CHANNEL_PRESSURE, mainMidiChannel, sendm);  // Send MSB of pressure mapped to the output range.
            prevChanPressure = sendm;
        }
    }

    // Poly aftertouch uses 2nd lowest bit of ED flag.
    if ((switches[mode][SEND_AFTERTOUCH] & 2) && noteon) {
        // Hack
        const int usedmod = inputPressureBounds[3][3] > 0 ? shakePressureKeyPressMod : 0;
        int sendm = (!noteon && sensorValue <= 100) ? 0 : constrain((int)inputPressureBounds[3][3] + usedmod, 0, 127);
        if (sendm != prevPolyPressure || force) {
            sendMIDI(KEY_PRESSURE, mainMidiChannel, notePlaying, sendm);  // Send MSB of pressure mapped to the output range.
            prevPolyPressure = sendm;
        }
    }
}









// Starting advertising BLE
void startAdv(void) {

    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);  // Set General Discoverable Mode flag
    Bluefruit.Advertising.addTxPower();                                           // Advertise TX Power
    Bluefruit.Advertising.addService(blemidi);                                    // Advertise BLE MIDI Service
    Bluefruit.ScanResponse.addName();                                             // Secondary Scan Response packet (optional)

    //For recommended advertising interval
    //https://developer.apple.com/library/content/qa/qa1931/_index.html

    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setInterval(32, 244);  // In units of 0.625 ms -- these are the settings recommended by Apple (20 ms and 152.5 ms).
    Bluefruit.Advertising.setFastTimeout(30);    // Number of seconds in fast mode
    Bluefruit.Advertising.start(0);              // 0 = Don't stop advertising after n seconds.
}









void sendMIDICouplet(uint8_t indexCC, uint8_t indexValue, uint8_t valueCC, uint8_t value) {

    xSemaphoreTake(midiSendCoupletMutex, portMAX_DELAY);

    sendMIDI(MIDI_SEND_CC, indexCC, indexValue);
    sendMIDI(MIDI_SEND_CC, valueCC, value);

    xSemaphoreGive(midiSendCoupletMutex);
}








// These convert MIDI messages from the old WARBL code to the correct format for the MIDI.h library. ToDo: These could be made shorter/more efficient.
void sendMIDI(uint8_t m, uint8_t c, uint8_t d1, uint8_t d2) {  // Send a 3-byte MIDI event over USB.

    m &= 0xF0;
    c = constrain(c, 1, 16);
    d1 &= 0x7F;
    d2 &= 0x7F;

    bool sendUsb = WARBL2settings[MIDI_DESTINATION] != MIDI_DESTINATION_BLE_ONLY || connIntvl == 0;         // If we're not only sending BLE or if we're not connected to BLE
    bool sendBle = WARBL2settings[MIDI_DESTINATION] != MIDI_DESTINATION_USB_ONLY || USBstatus != USB_HOST;  // If we're not only sending USB or if we're not connected to USB
    switch (m) {

        case 0x80:  // Note Off
            {
                if (sendUsb) {
                    MIDI.sendNoteOff(d1, d2, c);  // Send USB MIDI.
                }
                if (sendBle) {
                    BLEMIDI.sendNoteOff(d1, d2, c);  // Send BLE MIDI.
                }
                break;
            }
        case 0x90:  // Note On
            {
                if (sendUsb) {
                    MIDI.sendNoteOn(d1, d2, c);
                }
                if (sendBle) {
                    BLEMIDI.sendNoteOn(d1, d2, c);
                }
                break;
            }
        case 0xA0:  // Key Pressure
            {
                if (sendUsb) {
                    MIDI.sendAfterTouch(d1, d2, c);
                }
                if (sendBle) {
                    BLEMIDI.sendAfterTouch(d1, d2, c);
                }
                break;
            }
        case 0xB0:  // CC
            {
                bool isForConfigTool = (c == MIDI_CONFIG_TOOL_CHANNEL && d1 >= 102);  //CC is for config tool

                if (sendUsb && (!isForConfigTool ||  // Always send CC messages within the Config Tool range so user can't get locked out of Config Tool if only BLE or USB is selected (changed by AM 4/11/24).
                                (communicationModeSource == MIDI_SOURCE_NONE || communicationModeSource == MIDI_SOURCE_USB))) {

                    MIDI.sendControlChange(d1, d2, c);
                }
                if (sendBle && (!isForConfigTool || (communicationModeSource == MIDI_SOURCE_NONE || communicationModeSource == MIDI_SOURCE_BLE))) {
                    BLEMIDI.sendControlChange(d1, d2, c);
                }
                break;
            }
        case 0xC0:  // Program Change
            {
                break;
            }
        case 0xD0:  // Channel Pressure
            {
                break;
            }
        case 0xE0:  // Pitchbend
            {
                int16_t pitch = ((d2 << 7) + d1) - 8192;
                if (sendUsb) {
                    MIDI.sendPitchBend(pitch, c);
                }
                if (sendBle) {
                    BLEMIDI.sendPitchBend(pitch, c);
                }
                break;
            }
        default:
            {
                break;
            }
    }
}









// Send a 2-byte MIDI event over USB.
void sendMIDI(uint8_t m, uint8_t c, uint8_t d) {
    m &= 0xF0;
    c = constrain(c, 1, 16);
    d &= 0x7F;

    switch (m) {

        case 0xD0:  // Channel pressure
            {
                if (WARBL2settings[MIDI_DESTINATION] != MIDI_DESTINATION_BLE_ONLY || connIntvl == 0) {
                    MIDI.sendAfterTouch(d, c);
                }
                if (WARBL2settings[MIDI_DESTINATION] != MIDI_DESTINATION_USB_ONLY || USBstatus != USB_HOST) {
                    BLEMIDI.sendAfterTouch(d, c);
                }
                break;
            }


        case 0xC0:  // Program Change
            {
                if (WARBL2settings[MIDI_DESTINATION] != MIDI_DESTINATION_BLE_ONLY || connIntvl == 0) {
                    MIDI.sendProgramChange(d, c);
                }
                if (WARBL2settings[MIDI_DESTINATION] != MIDI_DESTINATION_USB_ONLY || USBstatus != USB_HOST) {
                    BLEMIDI.sendProgramChange(d, c);
                }
                break;
            }
    }
}









// Retrieve BLE connection information.
void connect_callback(uint16_t conn_handle) {

    BLEConnection* connection = Bluefruit.Connection(conn_handle);  // Get the reference to current connection

    char central_name[32] = { 0 };

    connection->getPeerName(central_name, sizeof(central_name));

    connIntvl = connection->getConnectionInterval();  // Get the current connection agreed upon connection interval in units of 0.625 ms
    connIntvl = connIntvl * 0.625;                    // Convert to ms. A 7.5 ms interval gets truncated to 7.


    if (connIntvl < 7) {  // When connecting to a CME Widi Bud Pro, getConnectionInterval() erroneously returns a connection interval of 3 ms. No idea why :).
        connIntvl = 7;
    }

    if (communicationMode) {
        sendMIDICouplet(MIDI_CC_106, MIDI_BLE_INTERVAL_LSB, MIDI_CC_119, (connIntvl * 100) & 0x7F);  // Send low byte of the connection interval.
        sendMIDICouplet(MIDI_CC_106, MIDI_BLE_INTERVAL_MSB, MIDI_CC_119, (connIntvl * 100) >> 7);    // high low byte of the connection interval.
    }

    blinkNumber[BLUE_LED] = 2;  // Indicate connection.
}









// Detect BLE disconnect.
void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
    (void)conn_handle;
    (void)reason;

    connIntvl = 0;

    if (communicationMode) {
        sendMIDICouplet(MIDI_CC_106, MIDI_BLE_INTERVAL_LSB, MIDI_CC_119, 0);  // Send low byte of the connection interval.
        sendMIDICouplet(MIDI_CC_106, MIDI_BLE_INTERVAL_MSB, MIDI_CC_119, 0);  // high low byte of the connection interval.

        if (communicationModeSource == MIDI_SOURCE_BLE) {
            communicationModeSource = MIDI_SOURCE_NONE;
        }
    }
}









// Watchdog enable, taken from Adafruit Sleepydog library.
void watchdog_enable(int maxPeriodMS) {
    if (maxPeriodMS < 0)
        return;

    if (nrf_wdt_started(NRF_WDT))  // Cannot change wdt config register once it is started
        return;

    nrf_wdt_behaviour_set(NRF_WDT, NRF_WDT_BEHAVIOUR_RUN_SLEEP);  // WDT run when CPU is asleep
    nrf_wdt_reload_value_set(NRF_WDT, (maxPeriodMS * 32768) / 1000);
    nrf_wdt_reload_request_enable(NRF_WDT, NRF_WDT_RR0);  // Use channel 0.
    nrf_wdt_task_trigger(NRF_WDT, NRF_WDT_TASK_START);    // Start WDT. After started CRV, RREN and CONFIG is blocked--there is no way to stop/disable watchdog using source code, it can only be reset by WDT timeout, Pin reset, Power reset.
}









// Watchdog reset
void watchdogReset() {
    nrf_wdt_reload_request_set(NRF_WDT, NRF_WDT_RR0);
}








// Make any necessary changes to EEPROM when new firmware is installed.
// Note: This is called before loadSettingsForAllModes() (before variables are filled from EEPROM) so we can use initialized values of variables for writing defaults to EEPROM.
void checkFirmwareVersion() {
    byte currentVersion = readEEPROM(EEPROM_FIRMWARE_VERSION);  // Previous firmware version.

    if (VERSION != currentVersion) {

        if (currentVersion < 42) {  // Manage all changes made in version 42.

            for (byte i = 0; i < 3; i++) {
                writeEEPROM((EEPROM_SWITCHES_START + i + (BUTTON_DOUBLE_CLICK * 3)), 0);                                  // Initialize button double-click preferences as false (0) for all three modes.
                writeEEPROM((EEPROM_SWITCHES_START + i + (BUTTON_DOUBLE_CLICK * 3)) + EEPROM_FACTORY_SETTINGS_START, 0);  // Initialize factory settings for same.

                for (byte n = EXPRESSION_MIN_HIGH; n < (CUSTOM_FINGERING_11 + 1); n++) {  // Reset EEPROM for these unused variables so they don't cause problems later.
                    writeEEPROM((EEPROM_ED_VARS_START + i + (n * 3)), 255);
                    writeEEPROM(((EEPROM_ED_VARS_START + i + (n * 3)) + EEPROM_FACTORY_SETTINGS_START), 255);  // Same for factory settings.
                }

                writeEEPROM(EEPROM_IMU_SETTINGS_START + i + (STICKS_MODE * 3), 0);  // Sticks mode
                writeEEPROM((EEPROM_IMU_SETTINGS_START + i + (STICKS_MODE * 3) + EEPROM_FACTORY_SETTINGS_START), 0);
            }
        }

        if (currentVersion < 43) {  // Manage all changes made in version 43.
            // Reset pitch expression to defaults with logic.
            for (byte i = 0; i < 3; i++) {
                byte exprmin = readEEPROM(EEPROM_ED_VARS_START + i + (EXPRESSION_MIN * 3));
                byte exprmax = readEEPROM(EEPROM_ED_VARS_START + i + (EXPRESSION_MAX * 3));
                byte override = readEEPROM(EEPROM_SWITCHES_START + i + (OVERRIDE * 3));

                writeEEPROM((EEPROM_ED_VARS_START + i + (EXPRESSION_MIN * 3)), 0);
                writeEEPROM(((EEPROM_ED_VARS_START + i + (EXPRESSION_MIN * 3)) + EEPROM_FACTORY_SETTINGS_START), 0);
                writeEEPROM((EEPROM_ED_VARS_START + i + (EXPRESSION_MIN_HIGH * 3)), 7);
                writeEEPROM(((EEPROM_ED_VARS_START + i + (EXPRESSION_MIN_HIGH * 3)) + EEPROM_FACTORY_SETTINGS_START), 7);
                writeEEPROM((EEPROM_ED_VARS_START + i + (EXPRESSION_FIXED_CENTER_PRESSURE * 3)), 8);
                writeEEPROM(((EEPROM_ED_VARS_START + i + (EXPRESSION_FIXED_CENTER_PRESSURE * 3)) + EEPROM_FACTORY_SETTINGS_START), 8);
                writeEEPROM((EEPROM_ED_VARS_START + i + (EXPRESSION_MAX_LOW * 3)), 11);
                writeEEPROM(((EEPROM_ED_VARS_START + i + (EXPRESSION_MAX_LOW * 3)) + EEPROM_FACTORY_SETTINGS_START), 11);
                writeEEPROM((EEPROM_ED_VARS_START + i + (EXPRESSION_MAX * 3)), 20);
                writeEEPROM(((EEPROM_ED_VARS_START + i + (EXPRESSION_MAX * 3)) + EEPROM_FACTORY_SETTINGS_START), 20);
                writeEEPROM((EEPROM_ED_VARS_START + i + (EXPRESSION_OUT_LOW_CENTS * 3)), 64 - 25);  // -50 cents
                writeEEPROM(((EEPROM_ED_VARS_START + i + (EXPRESSION_OUT_LOW_CENTS * 3)) + EEPROM_FACTORY_SETTINGS_START), 64 - 25);
                writeEEPROM((EEPROM_ED_VARS_START + i + (EXPRESSION_OUT_HIGH_CENTS * 3)), 64 + 50);  // +100 cents
                writeEEPROM(((EEPROM_ED_VARS_START + i + (EXPRESSION_OUT_HIGH_CENTS * 3)) + EEPROM_FACTORY_SETTINGS_START), 64 + 50);
                writeEEPROM((EEPROM_ED_VARS_START + i + (EXPRESSION_OUT_CLAMP * 3)), 1);  // boolean
                writeEEPROM(((EEPROM_ED_VARS_START + i + (EXPRESSION_OUT_CLAMP * 3)) + EEPROM_FACTORY_SETTINGS_START), 1);
                writeEEPROM((EEPROM_ED_VARS_START + i + (EXPRESSION_CURVE_LOW * 3)), 64);  // linear
                writeEEPROM(((EEPROM_ED_VARS_START + i + (EXPRESSION_CURVE_LOW * 3)) + EEPROM_FACTORY_SETTINGS_START), 64);
                writeEEPROM((EEPROM_ED_VARS_START + i + (EXPRESSION_CURVE_HIGH * 3)), 64);  // linear
                writeEEPROM(((EEPROM_ED_VARS_START + i + (EXPRESSION_CURVE_HIGH * 3)) + EEPROM_FACTORY_SETTINGS_START), 64);

                // Convert from original value to the new values if they were in use, otherwise keep our new defaults.
                if (override) {
                    // but sanitize exprmax because the old default was way too high.
                    if (exprmax > 50) {
                        exprmax = 50;
                    }
                    byte midpoint = (exprmax - exprmin) / 2;
                    writeEEPROM((EEPROM_ED_VARS_START + i + (EXPRESSION_MIN * 3)), exprmin);
                    writeEEPROM(((EEPROM_ED_VARS_START + i + (EXPRESSION_MIN * 3)) + EEPROM_FACTORY_SETTINGS_START), exprmin);
                    writeEEPROM((EEPROM_ED_VARS_START + i + (EXPRESSION_MAX * 3)), exprmax);
                    writeEEPROM(((EEPROM_ED_VARS_START + i + (EXPRESSION_MAX * 3)) + EEPROM_FACTORY_SETTINGS_START), exprmax);
                    writeEEPROM((EEPROM_ED_VARS_START + i + (EXPRESSION_MAX_LOW * 3)), midpoint);
                    writeEEPROM(((EEPROM_ED_VARS_START + i + (EXPRESSION_MAX_LOW * 3)) + EEPROM_FACTORY_SETTINGS_START), midpoint);
                    writeEEPROM((EEPROM_ED_VARS_START + i + (EXPRESSION_MIN_HIGH * 3)), midpoint);
                    writeEEPROM(((EEPROM_ED_VARS_START + i + (EXPRESSION_MIN_HIGH * 3)) + EEPROM_FACTORY_SETTINGS_START), midpoint);
                }

                for (int n = Y_SHAKE_MOD_CC; n <= Y_SHAKE_MOD_KEYPRESS; ++n) {
                    writeEEPROM(EEPROM_IMU_SETTINGS_START + i + (n * 3), 0);  // default shake mod to off
                    writeEEPROM((EEPROM_IMU_SETTINGS_START + i + (n * 3) + EEPROM_FACTORY_SETTINGS_START), 0);
                }
                for (int n = Y_SHAKE_MOD_CC_DEPTH; n <= Y_SHAKE_MOD_KEYPRESS_DEPTH; ++n) {
                    writeEEPROM(EEPROM_IMU_SETTINGS_START + i + (n * 3), 40);  // default shake mod depth to 40%
                    writeEEPROM((EEPROM_IMU_SETTINGS_START + i + (n * 3) + EEPROM_FACTORY_SETTINGS_START), 50);
                }
                for (int n = Y_SHAKE_MOD_CC_MODE; n <= Y_SHAKE_MOD_KEYPRESS_MODE; ++n) {
                    writeEEPROM(EEPROM_IMU_SETTINGS_START + i + (n * 3), 0);  // default shake mod mode - Up/Down
                    writeEEPROM((EEPROM_IMU_SETTINGS_START + i + (n * 3) + EEPROM_FACTORY_SETTINGS_START), 0);
                }
                // pressure MPE+ default to off
                writeEEPROM((EEPROM_ED_VARS_START + i + (AFTERTOUCH_MPEPLUS * 3)), 0);
                writeEEPROM(((EEPROM_ED_VARS_START + i + (AFTERTOUCH_MPEPLUS * 3)) + EEPROM_FACTORY_SETTINGS_START), 0);
            }
        }

        if (currentVersion < 44) {
            // Clear the high bytes for baseline.
            for (int i = 0; i < 8; ++i) {  // Fingerholes only
                writeEEPROM(EEPROM_BASELINE_CALIB_START + 9 + i, 0);
            }

            // We've moved the "feel" cal offset that was hardcoded as -50 into the actual calibration, so make that change to the cal covered data.
            for (byte i = EEPROM_SENSOR_CALIB_START; i < EEPROM_SENSOR_CALIB_START + 18; i += 2) {
                byte high = readEEPROM(i);
                byte low = readEEPROM(i + 1);
                int caldata = word(high, low);
                caldata -= 50;
                writeEEPROM(i, highByte(caldata));
                writeEEPROM(i + 1, lowByte(caldata));
            }

            // Do the same for the factory calibration.
            for (int i = 0; i < 8; ++i) {  // Fingerholes only
                writeEEPROM(EEPROM_BASELINE_CALIB_START + 9 + i + EEPROM_FACTORY_SETTINGS_START, 0);
            }

            for (int i = EEPROM_SENSOR_CALIB_START + EEPROM_FACTORY_SETTINGS_START; i < EEPROM_SENSOR_CALIB_START + 18 + EEPROM_FACTORY_SETTINGS_START; i += 2) {
                byte high = readEEPROM(i);
                byte low = readEEPROM(i + 1);
                int caldata = word(high, low);
                caldata -= 50;
                writeEEPROM(i, highByte(caldata));
                writeEEPROM(i + 1, lowByte(caldata));
            }

            // New IMU settings
            for (int i = 0; i < 3; ++i) {      // Each mode
                for (int n = 0; n < 3; ++n) {  // Roll, pitch, yaw settings, taken from IMUsettings initialized values.
                    writeEEPROM(EEPROM_IMU_SETTINGS_START + i + ((IMU_ROLL_PITCH_MIN + (n * 9)) * 3), IMUsettings[i][IMU_ROLL_PITCH_MIN + (n * 9)]);
                    writeEEPROM((EEPROM_IMU_SETTINGS_START + i + ((IMU_ROLL_PITCH_MIN + (n * 9)) * 3) + EEPROM_FACTORY_SETTINGS_START), IMUsettings[i][IMU_ROLL_PITCH_MIN + (n * 9)]);
                    writeEEPROM(EEPROM_IMU_SETTINGS_START + i + ((IMU_ROLL_PITCH_MAX + (n * 9)) * 3), IMUsettings[i][IMU_ROLL_PITCH_MAX + (n * 9)]);
                    writeEEPROM((EEPROM_IMU_SETTINGS_START + i + ((IMU_ROLL_PITCH_MAX + (n * 9)) * 3) + EEPROM_FACTORY_SETTINGS_START), IMUsettings[i][IMU_ROLL_PITCH_MAX + (n * 9)]);
                    writeEEPROM(EEPROM_IMU_SETTINGS_START + i + ((IMU_ROLL_PITCH_MIN_HIGH + (n * 9)) * 3), IMUsettings[i][IMU_ROLL_PITCH_MIN_HIGH + (n * 9)]);
                    writeEEPROM((EEPROM_IMU_SETTINGS_START + i + ((IMU_ROLL_PITCH_MIN_HIGH + (n * 9)) * 3) + EEPROM_FACTORY_SETTINGS_START), IMUsettings[i][IMU_ROLL_PITCH_MIN_HIGH + (n * 9)]);
                    writeEEPROM(EEPROM_IMU_SETTINGS_START + i + ((IMU_ROLL_PITCH_MAX_LOW + (n * 9)) * 3), IMUsettings[i][IMU_ROLL_PITCH_MAX_LOW + (n * 9)]);
                    writeEEPROM((EEPROM_IMU_SETTINGS_START + i + ((IMU_ROLL_PITCH_MAX_LOW + (n * 9)) * 3) + EEPROM_FACTORY_SETTINGS_START), IMUsettings[i][IMU_ROLL_PITCH_MAX_LOW + (n * 9)]);
                    writeEEPROM(EEPROM_IMU_SETTINGS_START + i + ((IMU_ROLL_PITCH_OUT_LOW_CENTS + (n * 9)) * 3), IMUsettings[i][IMU_ROLL_PITCH_OUT_LOW_CENTS + (n * 9)]);
                    writeEEPROM((EEPROM_IMU_SETTINGS_START + i + ((IMU_ROLL_PITCH_OUT_LOW_CENTS + (n * 9)) * 3) + EEPROM_FACTORY_SETTINGS_START), IMUsettings[i][IMU_ROLL_PITCH_OUT_LOW_CENTS + (n * 9)]);
                    writeEEPROM(EEPROM_IMU_SETTINGS_START + i + ((IMU_ROLL_PITCH_OUT_HIGH_CENTS + (n * 9)) * 3), IMUsettings[i][IMU_ROLL_PITCH_OUT_HIGH_CENTS + (n * 9)]);
                    writeEEPROM((EEPROM_IMU_SETTINGS_START + i + ((IMU_ROLL_PITCH_OUT_HIGH_CENTS + (n * 9)) * 3) + EEPROM_FACTORY_SETTINGS_START), IMUsettings[i][IMU_ROLL_PITCH_OUT_HIGH_CENTS + (n * 9)]);
                    writeEEPROM(EEPROM_IMU_SETTINGS_START + i + ((IMU_ROLL_PITCH_OUT_CLAMP + (n * 9)) * 3), IMUsettings[i][IMU_ROLL_PITCH_OUT_CLAMP + (n * 9)]);
                    writeEEPROM((EEPROM_IMU_SETTINGS_START + i + ((IMU_ROLL_PITCH_OUT_CLAMP + (n * 9)) * 3) + EEPROM_FACTORY_SETTINGS_START), IMUsettings[i][IMU_ROLL_PITCH_OUT_CLAMP + (n * 9)]);
                    writeEEPROM(EEPROM_IMU_SETTINGS_START + i + ((IMU_ROLL_PITCH_CURVE_LOW + (n * 9)) * 3), IMUsettings[i][IMU_ROLL_PITCH_CURVE_LOW + (n * 9)]);
                    writeEEPROM((EEPROM_IMU_SETTINGS_START + i + ((IMU_ROLL_PITCH_CURVE_LOW + (n * 9)) * 3) + EEPROM_FACTORY_SETTINGS_START), IMUsettings[i][IMU_ROLL_PITCH_CURVE_LOW + (n * 9)]);
                    writeEEPROM(EEPROM_IMU_SETTINGS_START + i + ((IMU_ROLL_PITCH_CURVE_HIGH + (n * 9)) * 3), IMUsettings[i][IMU_ROLL_PITCH_CURVE_HIGH + (n * 9)]);
                    writeEEPROM((EEPROM_IMU_SETTINGS_START + i + ((IMU_ROLL_PITCH_CURVE_HIGH + (n * 9)) * 3) + EEPROM_FACTORY_SETTINGS_START), IMUsettings[i][IMU_ROLL_PITCH_CURVE_HIGH + (n * 9)]);
                }
                writeEEPROM(EEPROM_IMU_SETTINGS_START + i + MAP_ROLL_TO_PITCHBEND * 3, 0);
                writeEEPROM((EEPROM_IMU_SETTINGS_START + i + MAP_ROLL_TO_PITCHBEND * 3 + EEPROM_FACTORY_SETTINGS_START), 0);
                writeEEPROM(EEPROM_IMU_SETTINGS_START + i + MAP_ELEVATION_TO_PITCHBEND * 3, 0);
                writeEEPROM((EEPROM_IMU_SETTINGS_START + i + MAP_ELEVATION_TO_PITCHBEND * 3 + EEPROM_FACTORY_SETTINGS_START), 0);
                writeEEPROM(EEPROM_IMU_SETTINGS_START + i + MAP_YAW_TO_PITCHBEND * 3, 0);
                writeEEPROM((EEPROM_IMU_SETTINGS_START + i + MAP_YAW_TO_PITCHBEND * 3 + EEPROM_FACTORY_SETTINGS_START), 0);
            }

            // New overblow semitones setting
            for (int i = 0; i < 3; ++i) {  // Each mode
                writeEEPROM(EEPROM_ED_VARS_START + i + (OVERBLOW_SEMITONES * 3), ED[mode][OVERBLOW_SEMITONES]);
                writeEEPROM(EEPROM_ED_VARS_START + i + (OVERBLOW_SEMITONES * 3) + EEPROM_FACTORY_SETTINGS_START, ED[mode][OVERBLOW_SEMITONES]);
            }
        }

        writeEEPROM(EEPROM_FIRMWARE_VERSION, VERSION);  // Update the firmware version if it has changed.
    }
}









// EEPROM functions

// Write to EEPROOM
void writeEEPROM(int address, byte val) {
    if (address < 16384) {                 // Make sure we're not trying to write beyond the highest address in the M24128 EEPROM.
        if (val != readEEPROM(address)) {  // Don't write if the value is already there.
            Wire.beginTransmission(EEPROM_I2C_ADDRESS);
            Wire.write((int)(address >> 8));    // Write the MSB.
            Wire.write((int)(address & 0xFF));  // Write the LSB.
            Wire.write(val);
            Wire.endTransmission();
            while (EEPROMbusy() == true) {  // Poll until the previous transmission has completed.
                delayMicroseconds(200);
            }
        }
    }
}



// Read from EEPROM
byte readEEPROM(int address) {
    byte rcvData = 0xFF;
    Wire.beginTransmission(EEPROM_I2C_ADDRESS);
    Wire.write((int)(address >> 8));    // Write the MSB.
    Wire.write((int)(address & 0xFF));  // Write the LSB.
    Wire.endTransmission();
    Wire.requestFrom(EEPROM_I2C_ADDRESS, 1);
    rcvData = Wire.read();
    while (EEPROMbusy() == true) {  // Poll until the previous transmission has completed.
        delayMicroseconds(200);
    }
    return rcvData;
}



// ACK polling to see if previous EEPROM transaction has finished
bool EEPROMbusy(void) {
    Wire.beginTransmission(EEPROM_I2C_ADDRESS);
    if (Wire.endTransmission() == 0) {
        return (false);
    }
    return (true);
}



// Put an int to EEPROM
void putEEPROM(int address, int val) {
    byte byte1 = val >> 8;
    byte byte2 = val & 0xFF;
    writeEEPROM(address, byte1);
    writeEEPROM(address + 1, byte2);
}



// Get an int from EEPROM
void getEEPROM(int address, int& var) {
    byte byte1 = readEEPROM(address);
    byte byte2 = readEEPROM(address + 1);
    var = (byte1 << 8) + byte2;
}



// Put a float to EEPROM
void putEEPROM(int address, float value) {
    byte* p = (byte*)(void*)&value;
    for (int i = 0; i < sizeof(value); i++)
        writeEEPROM(address++, *p++);
}




// Get a float from EEPROM
void getEEPROM(int address, float& var) {
    float value = 0.0f;
    byte* p = (byte*)(void*)&value;
    for (int i = 0; i < sizeof(value); i++)
        *p++ = readEEPROM(address++);
    var = value;
}



// For testing purposes
void eraseEEPROM(void) {
    for (int i = 0; i < 16384; i++) {
        writeEEPROM(i, 255);
    }
    analogWrite(LEDpins[GREEN_LED], 1023);  // Success
    delay(500);
    analogWrite(LEDpins[GREEN_LED], 0);
}

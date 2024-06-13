

// Debug
void printStuff(void) {

    //static float CPUtemp = readCPUTemperature(); // If needed for something like calibrating sensors. Can also use IMU temp. The CPU is in the middle of the PCB and the IMU is near the mouthpiece.

    //This can be used to print out the currently selected fingering chart (all 256 possible values) when button 1 is clicked.
    /*
    static byte printed = 0;
    if (digitalRead(4) == 0) {
        for (unsigned int i = 0; i < 256; i++) {
            Serial.print(getNote(i << 1));
            Serial.print(" ");
        }
        delay(5000);
    } 
*/


    for (byte i = 0; i < 9; i++) {
        //Serial.println(toneholeRead[i]);
    }
    //Serial.println(" ");
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

    // Note: Gyro is turned off by default to save power unless using roll/pitch/yaw or elevation register (see loadPrefs()).

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
    deltat = constrain(deltat, 0.001f, 0.01f);  // AM 9/16/23 Enabled DWT to make micros() higher resolution for this calculation. Not sure that it matters.

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

    /*
    Serial.print(-180);
    Serial.print(" , ");
    Serial.print(roll);
    Serial.print(" , ");
    Serial.print(pitch);
    Serial.print(" , ");
    Serial.print(yaw);
    Serial.print(" , ");
    Serial.println(180);
*/


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
            analogWrite(LEDpins[GREEN_LED], hitVelocity << 3);  // Fire teal LED to indicate a hit, with brightness based on note velocity.
            analogWrite(LEDpins[BLUE_LED], hitVelocity << 1);
            LEDCounter = 1;  // Start counting to turn the LED off again.
            armed = false;   // Don't allow another hit until we've passed the threshold again.
            maxGyro = 0;
            powerDownTimer = pitchBendTimer;  // Reset the powerDown timer because we're sending notes.
        }
    }
}








// Calibrate the IMU when the command is received from the Config Tool.
void calibrateIMU() {

    sox.setGyroDataRate(LSM6DS_RATE_208_HZ);  // Make sure the gyro is on.
    delay(50);                                // Give it a bit of time.
    readIMU();                                // Get a reading in case we haven't been reading it.
    delay(50);                                // Give it a bit of time.
    readIMU();                                // Another reading seems to be necessary after turning the gyro on.

    gyroXCalibration = rawGyroX;
    gyroYCalibration = rawGyroY;
    gyroZCalibration = rawGyroZ;

    putEEPROM(1975, gyroXCalibration);  // Put the current readings into EEPROM.
    putEEPROM(1979, gyroYCalibration);
    putEEPROM(1983, gyroZCalibration);
    writeEEPROM(36, 3);  // Remember that we have saved a calibration.
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
        //Serial.println(pitchOutput);

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
        //Serial.println(pitchOutput);

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
        //Serial.println(yawOutput);

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

    const float shakeBendDepth = 4.0f * IMUsettings[mode][Y_PITCHBEND_DEPTH] / 100;  // Adjust the vibrato depth range based on the Config Tool setting.
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


    if (abs(accelFilteredB) > kShakeGestureThresh) {
        shakeDetected = true;
    } else {
        shakeDetected = false;
    }

    if (IMUsettings[mode][Y_SHAKE_PITCHBEND]) {  // Do this part ony if shake pitchbend is turned on.

        shakeVibrato = 0;

        if (tapFilterActive == false && abs(accelFilteredB) > kShakeStartThresh) {
            tapFilterActive = true;
            tapFilterStartTime = nowtime;
            // Serial.println("Shake Tap Filt");
        }

        if ((nowtime - tapFilterStartTime) < ktapFilterTimeMs) {  // Return if we haven't waited long enough after crossing the shake threshold, for further filtering brief tone hole taps.
            return;
        }

        if (!preTrigger && abs(accelFilteredB) > kShakeStartThresh) {
            preTrigger = true;
            // Serial.println("Pre Shake Trigger");
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
            float normshake = constrain(accelFilteredB * 0.06666f, -1.0f, 1.0f);
            if (IMUsettings[mode][Y_PITCHBEND_MODE] == Y_PITCHBEND_MODE_UPDOWN) {
                normshake *= -1.0f;  // reverse phase
            } else if (IMUsettings[mode][Y_PITCHBEND_MODE] == Y_PITCHBEND_MODE_DOWNONLY) {
                normshake = constrain(normshake, -1.0f, 0.0f);
            } else if (IMUsettings[mode][Y_PITCHBEND_MODE] == Y_PITCHBEND_MODE_UPONLY) {
                normshake = constrain(-1.0f * normshake, 0.0f, 1.0f);
            }

            shakeVibrato = (int)(normshake * shakeBendDepth * pitchBendPerSemi);
        }

        if (pitchBendMode == kPitchBendNone) {  // If we don't have finger vibrato and/or slide turned on, we need to send the pitchbend now.
            sendPitchbend();
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
        if ((toneholeRead[i]) > (toneholeCovered[i] - 50)) {
            bitWrite(holeCovered, i, 1);  // Use the tonehole readings to decide which holes are covered
        } else if ((toneholeRead[i]) <= (toneholeCovered[i] - 54)) {
            bitWrite(holeCovered, i, 0);  // Decide which holes are uncovered -- the "hole uncovered" reading is a little less then the "hole covered" reading, to prevent oscillations.
        }
    }
}










// This should be called right after a new hole state change, before sending out any new note.
bool isMaybeInTransition() {
    // Look at all finger hole state to see if there might be other
    // fingers in motion (partial hole covered) which means this
    // might be a multi-finger note transition and we may want to wait
    // a bit to see if the pending fingers land before triggering the new note.
    int pendingHoleCovered = holeCovered;
    unsigned long now = millis();
    int otherholes = 0;

    for (byte i = 0; i < 9; i++) {
        int thresh = senseDistance;  //  ((toneholeCovered[i] - 50) - senseDistance) / 2;
        if (toneholeRead[i] > thresh) {

            if (bitRead(holeCovered, i) != 1) {
                bitWrite(pendingHoleCovered, i, 1);
                ++otherholes;
                /*
                Serial.print("offs: ");
                Serial.print(offsetSteps);
                Serial.print(" tscale: ");
                Serial.print(toneholeScale[i]);
                Serial.print(" bend: ");
                Serial.println(iPitchBend[i]);
                */
            }
        }
    }

    if (pendingHoleCovered != holeCovered) {
        // See if the pending is a new note.
        int tempNewNote = getNote(holeCovered);
        int pendingNote = getNote(pendingHoleCovered);
        if (pendingNote >= 0 && pendingNote != tempNewNote) {
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









// Key delay feature for delaying response to tone holes and filtering out transient notes, originally by Louis Barman, but reworked by Jesse Chappell
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

    if (transitionFilter > 0 && (debounceDelta >= transitionFilter || !isMaybeInTransition())) {
        // reset it if necessary
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
        tempNewNote = getNote(holeCovered);  // Get the next MIDI note from the fingering pattern if it has changed. 3us.
        sendToConfig(true, false);           // Put the new pattern into a queue to be sent later so that it's not sent during the same connection interval as a new note (to decrease BLE payload size).

        if (tempNewNote != -1 && newNote != tempNewNote) {  // If a new note has been triggered,
            if (pitchBendMode != kPitchBendNone) {
                holeLatched = holeCovered;  // remember the pattern that triggered it (it will be used later for vibrato).
                for (byte i = 0; i < 9; i++) {
                    iPitchBend[i] = 0;  // Reset pitchbend.
                    pitchBendOn[i] = 0;
                }
            }
        }

#if DEBUG_TRANSITION_FILTER
        Serial.print(now);
        Serial.print(" : Commit: ");
        Serial.println(tempNewNote);
#endif

        newNote = tempNewNote;
        tempNewNote = -1;
        getState();                       // Get state again if the note has changed.
        fingeringChangeTimer = millis();  // Start timing after the fingering pattern has changed.
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

        if (patternChanged && (nowtime - patternSendTimer) > 25) {  // If some time has past, send the new pattern to the Config Tool.
            sendMIDI(CONTROL_CHANGE, 7, 114, holeCovered >> 7);     // Because it's MIDI we have to send it in two 7-bit chunks.
            sendMIDI(CONTROL_CHANGE, 7, 115, lowByte(holeCovered));
            patternChanged = false;
        }

        if (pressureChanged && (nowtime - pressureSendTimer) > 25) {  // If some time has past, send the new pressure to the Config Tool.
            sendMIDI(CONTROL_CHANGE, 7, 116, sensorValue & 0x7F);     // Send LSB of current pressure to Configuration Tool.
            sendMIDI(CONTROL_CHANGE, 7, 118, sensorValue >> 7);       // Send MSB of current pressure.
            pressureChanged = false;
        }
    }
}









// Return a MIDI note number (0-127) based on the current fingering.
int getNote(unsigned int fingerPattern) {
    int ret = -1;  // Default for unknown fingering

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

    return ret;
}









// Add up any transposition based on key and register.
void getShift() {

    byte pitchShift;

    if (IMUsettings[mode][PITCH_REGISTER] == true) {
        pitchShift = pitchRegister();
    }


    shift = ((octaveShift * 12) + noteShift + (pitchShift * 12));  // Adjust for key and octave shift.

    // Overblow if allowed.
    if (newState == 3 && !(modeSelector[mode] == kModeEVI || (modeSelector[mode] == kModeSax && newNote < 58) || (modeSelector[mode] == kModeSaxBasic && newNote < 70) || (modeSelector[mode] == kModeRecorder && newNote < 74)) && !(newNote == 62 && (modeSelector[mode] == kModeUilleann || modeSelector[mode] == kModeUilleannStandard))) {  // If overblowing (except EVI, sax and recorder in the lower register, and low D with uilleann fingering, which can't overblow)
        shift = shift + 12;                                                                                                                                                                                                                                                                                                                      // Add a register jump to the transposition if overblowing.
        if (modeSelector[mode] == kModeKaval) {                                                                                                                                                                                                                                                                                                  // Kaval only plays a fifth higher in the second register.
            shift = shift - 5;
        }
    }
    // Use the bell sensor to control register if desired.
    if (breathMode == kPressureBell && modeSelector[mode] != kModeUilleann && modeSelector[mode] != kModeUilleannStandard) {
        if (bitRead(holeCovered, 0) == switches[mode][INVERT]) {
            shift = shift + 12;
            if (modeSelector[mode] == kModeKaval) {
                shift = shift - 5;
            }
        }
    }

    // ToDo: Are there any others that don 't use the thumb that can be added here? For custom charts the thumb needs to hard-coded instead.
    else if ((breathMode == kPressureThumb && (modeSelector[mode] == kModeWhistle || modeSelector[mode] == kModeChromatic || modeSelector[mode] == kModeNAF))) {  // If we're using the left thumb to control the regiser with a fingering patern that doesn't normally use the thumb
        if (bitRead(holeCovered, 8) == switches[mode][INVERT]) {
            shift = shift + 12;  // Add an octave jump to the transposition if necessary.
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








// Calculate pitchbend expression based on pressure
void getExpression() {

    // Calculate the center pressure value for the current note, regardless of register, unless "override" is turned on and we're not in overblow mode. In that case, use the override bounds instead.

    int lowerBound;
    int useUpperBound;

    if (switches[mode][OVERRIDE] && (breathMode != kPressureBreath)) {
        lowerBound = (ED[mode][EXPRESSION_MIN] * 9) + 100;
        useUpperBound = (ED[mode][EXPRESSION_MAX] * 9) + 100;
    } else {
        lowerBound = sensorThreshold[0];
        if (newState == 3) {
            useUpperBound = upperBoundLow;  // Get the register boundary taking hysteresis into consideration
        } else {
            useUpperBound = upperBoundHigh;
        }
    }

    unsigned int halfway = ((useUpperBound - lowerBound) >> 1) + lowerBound;  // Calculate the midpoint of the curent register, where the note should play in tune.

    if (newState == 3) {
        halfway = useUpperBound + halfway;
        lowerBound = useUpperBound;
    }

    if (sensorValue < halfway) {
        byte scale = (((halfway - sensorValue) * ED[mode][EXPRESSION_DEPTH] * 20) / (halfway - lowerBound));  // Should maybe figure out how to do this without dividing.
        expression = -((scale * scale) >> 3);
    } else {
        expression = (sensorValue - halfway) * ED[mode][EXPRESSION_DEPTH];
    }

    if (expression > ED[mode][EXPRESSION_DEPTH] * 200) {
        expression = ED[mode][EXPRESSION_DEPTH] * 200;  // Put a cap on it, because in the upper register or in single-register mode, there's no upper limit
    }

    if (pitchBendMode == kPitchBendNone) {  // If we're not using vibrato, send the pitchbend now instead of adding it in later.
        pitchBend = 0;
        sendPitchbend();
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

    if (pitchBendMode == kPitchBendSlideVibrato || pitchBendMode == kPitchBendLegatoSlideVibrato) {  // Calculate slide if necessary.
        getSlide();
    }

    // this method only cares if 2 or 3 are slideholes
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
                            iPitchBend[i] = 0 - (int)(((toneholeCovered[i] - 50.0f - toneholeRead[i]) * vibratoScale[i]));  // Bend up, yielding a negative pitchbend value.
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
    sendPitchbend();
}









// Andrew's version of vibrato
void handlePitchBend() {
    for (byte i = 0; i < 9; i++) {  // Reset
        iPitchBend[i] = 0;
    }

    if (pitchBendMode == kPitchBendSlideVibrato || pitchBendMode == kPitchBendLegatoSlideVibrato) {  // Calculate slide if necessary.
        getSlide();
    }


    for (byte i = 0; i < 9; i++) {

        if (bitRead(holeLatched, i) == 1 && toneholeRead[i] < senseDistance) {
            (bitWrite(holeLatched, i, 0));  // We "unlatch" (enable for vibrato) a hole if it was covered when the note was triggered but now the finger has been completely removed.
        }

        // If this is a vibrato hole and we're in a mode that uses vibrato, and the hole is unlatched, and not already being used for slide
        if (bitRead(vibratoHoles, i) == 1 && bitRead(holeLatched, i) == 0
            && (pitchBendMode == kPitchBendVibrato || (iPitchBend[i] == 0))) {
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


    sendPitchbend();
}









// Calculate slide pitchBend, to be added with vibrato.
void getSlide() {
    for (byte i = 0; i < 9; i++) {
        if (toneholeRead[i] > senseDistance && transitionFilter == 0) {
            const int offsetLimit = constrain(ED[mode][SLIDE_LIMIT_MAX], 0, midiBendRange);

            int offsetSteps = findStepsOffsetFor(i);

            if (pitchBendModeSelector[mode] == kPitchBendSlideVibrato && offsetSteps < -offsetLimit) {  // Added by AM 5/24 to make the slide behavior more like that on the original WARBL.
                offsetSteps = -offsetLimit;
            }

            if (offsetSteps != 0
                && bitRead(holeCovered, i) != 1
                && offsetSteps <= offsetLimit && offsetSteps >= -offsetLimit) {
                iPitchBend[i] = ((((int)((toneholeRead[i] - senseDistance) * toneholeScale[i])) * -offsetSteps));  // scale
                /*
                Serial.print("offs: ");
                Serial.print(offsetSteps);
                Serial.print(" tscale: ");
                Serial.print(toneholeScale[i]);
                Serial.print(" bend: ");
                Serial.println(iPitchBend[i]);
                */
            } else {
                iPitchBend[i] = 0;
            }
        } else {
            iPitchBend[i] = 0;
        }
    }
}









void sendPitchbend() {

    static int prevPitchBend = 8192;  // A record of the previous pitchBend value, so we don't send the same one twice

    pitchBend = 0;  // Reset the overall pitchbend in preparation for adding up the contributions from all the toneholes.
    for (byte i = 0; i < 9; i++) {
        pitchBend = pitchBend + iPitchBend[i];
    }

    int noteshift = 0;
    if (noteon && pitchBendModeSelector[mode] == kPitchBendLegatoSlideVibrato) {
        noteshift = (notePlaying - shift) - newNote;
        pitchBend += (int)(noteshift * pitchBendPerSemi);
    }

    pitchBend = 8192 - pitchBend + expression + shakeVibrato;

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









void calculateAndSendPitchbend() {
    if (ED[mode][EXPRESSION_ON] && !switches[mode][BAGLESS]) {
        getExpression();  // IF using pitchbend expression, calculate pitchbend based on pressure reading.
    }

    if (!customEnabled && pitchBendMode != kPitchBendNone) {
        handlePitchBend();
    } else if (customEnabled) {
        handleCustomPitchBend();
    }
}








// Send MIDI NoteOn/NoteOff events when necessary.
void sendNote() {
    const int velDelayMs = switches[mode][SEND_AFTERTOUCH] != 0 ? 3 : 16;  // Keep this minimal to avoid latency if also sending aftertouch, but enough to get a good reading, otherwise use longer

    if (  // Several conditions to tell if we need to turn on a new note.
      (!noteon
       || (pitchBendModeSelector[mode] != kPitchBendLegatoSlideVibrato && newNote != (notePlaying - shift))
       || (pitchBendModeSelector[mode] == kPitchBendLegatoSlideVibrato && abs(newNote - (notePlaying - shift)) > midiBendRange - 1))
      &&                                                                                    // If there wasn't any note playing or the current note is different than the previous one
      ((newState > 1 && !switches[mode][BAGLESS]) || (switches[mode][BAGLESS] && play)) &&  // And the state machine has determined that a note should be playing, or we're in bagless mode and the sound is turned on
      //!(prevNote == 62 && (newNote + shift) == 86) &&                                                   // And if we're currently on a middle D in state 3 (all finger holes covered), we wait until we get a new state reading before switching notes. This it to prevent erroneous octave jumps to a high D.
      !(switches[mode][SEND_VELOCITY] && !noteon && ((millis() - velocityDelayTimer) < velDelayMs)) &&  // And not waiting for the pressure to rise to calculate note on velocity if we're transitioning from not having any note playing.
      !(modeSelector[mode] == kModeNorthumbrian && newNote == 63) &&                                    // And if we're in Northumbrian mode don't play a note if all holes are covered. That simulates the closed pipe.
      !(breathMode != kPressureBell && bellSensor && holeCovered == 0b111111111))                       // Don't play a note if the bell sensor and all other holes are covered, and we're not in "bell register" mode. Again, simulating a closed pipe.
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

        // Send pitch bend immediately prior to note if necessary.
        if (switches[mode][IMMEDIATE_PB]) {
            calculateAndSendPitchbend();
        }

        if (newNote != 0) {                                                 // With a custom chart a MIDI note of 0 can be used as a silennt position, so don't play the note.
            sendMIDI(NOTE_ON, mainMidiChannel, newNote + shift, velocity);  // Send the new note.
        }

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
          ((newState == 1 && !switches[mode][BAGLESS]) || (switches[mode][BAGLESS] && !play)) ||  // If the state drops to 1 (off) or we're in bagless mode and the sound has been turned off.
          (modeSelector[mode] == kModeNorthumbrian && newNote == 63) ||                           // Or closed Northumbrian pipe.
          (breathMode != kPressureBell && bellSensor && holeCovered == 0b111111111)) {            // Or completely closed pipe with any fingering chart.
            sendMIDI(NOTE_OFF, mainMidiChannel, notePlaying, 64);                                 // Turn the note off if the breath pressure drops or the bell sensor is covered and all the finger holes are covered.
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










// Check for and handle incoming MIDI messages from the WARBL Configuration Tool.
void handleControlChange(byte channel, byte number, byte value) {
    //Serial.println(channel);
    //Serial.println(number);
    //Serial.println(value);
    //Serial.println("");

    if (number < 120) {  // Chrome sends CC 121 and 123 on all channels when it connects, so ignore these.

        if ((channel & 0x0f) == 7) {     // If we're on channel 7, we may be receiving messages from the configuration tool.
            powerDownTimer = millis();   // Reset the powerDown timer because we've heard from the Config Tool.
            blinkNumber[GREEN_LED] = 1;  // Blink once, indicating a received message. Some commands below will change this to three (or zero) blinks.


            /////// CC 102
            if (number == 102) {                 // Many settings are controlled by a value in CC 102 (always channel 7).
                if (value > 0 && value <= 18) {  // Handle sensor calibration commands from the configuration tool.
                    if ((value & 1) == 0) {
                        toneholeCovered[(value >> 1) - 1] -= 5;
                        if ((toneholeCovered[(value >> 1) - 1] - 54) < 5) {  //if the tonehole calibration gets set too low so that it would never register as being uncovered, send a message to the configuration tool.
                            sendMIDI(CONTROL_CHANGE, 7, 102, (20 + ((value >> 1) - 1)));
                        }
                    } else {
                        toneholeCovered[((value + 1) >> 1) - 1] += 5;
                    }
                }

                if (value == 19) {  // Save calibration if directed.
                    saveCalibration();
                    blinkNumber[GREEN_LED] = 3;
                }

                else if (value == 127) {  // Begin auto-calibration if directed.
                    blinkNumber[GREEN_LED] = 0;
                    calibration = 1;
                }

                else if (value == 126) {  // When communication is established, send all current settings to tool.
                    communicationMode = 1;
                    sendSettings();
                }

                else if (value == 104) {  // Turn off communication mode.
                    communicationMode = 0;
                }


                for (byte i = 0; i < 3; i++) {  // Update the three selected fingering patterns if prompted by the tool.
                    if (value == 30 + i) {
                        fingeringReceiveMode = i;
                    }
                }

                if ((value > 32 && value < 60) || (value > 99 && value < 104)) {
                    modeSelector[fingeringReceiveMode] = value - 33;
                    loadPrefs();
                }


                for (byte i = 0; i < 3; i++) {  // Update current mode (instrument) if directed.
                    if (value == 60 + i) {
                        mode = i;
                        play = 0;
                        loadPrefs();     // Load the correct user settings based on current instrument.
                        sendSettings();  // Send settings for new mode to tool.
                        blinkNumber[GREEN_LED] = abs(mode) + 1;
                    }
                }

                for (byte i = 0; i < 4; i++) {  // Update current pitchbend mode if directed.
                    if (value == 70 + i) {
                        pitchBendModeSelector[mode] = i;
                        loadPrefs();
                        blinkNumber[GREEN_LED] = abs(pitchBendMode) + 1;
                    }
                }

                for (byte i = 0; i < 5; i++) {  // Update current breath mode if directed.
                    if (value == 80 + i) {
                        breathModeSelector[mode] = i;
                        loadPrefs();  // Load the correct user settings based on current instrument.
                        blinkNumber[GREEN_LED] = abs(breathMode) + 1;
                    }
                }

                for (byte i = 0; i < kGESTURESnVariables; i++) {  // Update button receive mode (this indicates the row in the button settings for which the next received byte will be).
                    if (value == 90 + i) {
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
                        if (value == 117) {
                            momentary[mode][i] = 0;
                            noteOnOffToggle[i] = 0;
                        } else if (value == 118) {
                            momentary[mode][i] = 1;
                            noteOnOffToggle[i] = 0;
                        }
                    }
                }

                if (value == 85) {  // Set current Instrument as default and save default to settings.
                    defaultMode = mode;
                    writeEEPROM(48, defaultMode);
                }


                if (value == 123) {  // Save settings as the defaults for the current instrument
                    saveSettings(mode);
                    blinkNumber[GREEN_LED] = 3;
                }


                else if (value == 124) {  // Save settings as the defaults for all instruments
                    for (byte k = 0; k < 3; k++) {
                        saveSettings(k);
                    }
                    loadFingering();
                    loadSettingsForAllModes();
                    loadPrefs();
                    blinkNumber[GREEN_LED] = 3;

                }

                else if (value == 125) {  // Restore all factory settings
                    restoreFactorySettings();
                    blinkNumber[GREEN_LED] = 3;
                }
            }




            /////// CC 103
            else if (number == 103) {
                senseDistanceSelector[mode] = value;
                loadPrefs();
            }

            else if (number == 117) {
                unsigned long v = value * 8191UL / 100;
                vibratoDepthSelector[mode] = v;  // Scale vibrato depth in cents up to pitchbend range of 0-8191.
                loadPrefs();
            }


            for (byte i = 0; i < 3; i++) {  // Update noteshift.
                if (number == 111 + i) {
                    if (value == 109) {
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
            if (number == 104) {  // Update receive mode, used for advanced pressure range sliders, switches, and expression and drones panel settings (this indicates the variable for which the next received byte on CC 105 will be).
                pressureReceiveMode = value - 1;
            }



            /////// CC 105
            else if (number == 105) {

                if (pressureReceiveMode < 12) {
                    pressureSelector[mode][pressureReceiveMode] = value;  // Advanced pressure values
                    loadPrefs();
                }

                else if (pressureReceiveMode < 33) {
                    ED[mode][pressureReceiveMode - 12] = value;  // Expression and drones settings
                    loadPrefs();
                }

                else if (pressureReceiveMode == 33) {
                    LSBlearnedPressure = value;

                }

                else if (pressureReceiveMode == 34) {
                    learnedPressureSelector[mode] = (value << 7) | LSBlearnedPressure;
                    loadPrefs();
                }


                else if (pressureReceiveMode < 53) {
                    switches[mode][pressureReceiveMode - 39] = value;  // Switches in the slide/vibrato and register control panels.
                    loadPrefs();
                }

                else if (pressureReceiveMode == 60) {
                    midiBendRangeSelector[mode] = value;
                    loadPrefs();
                }

                else if (pressureReceiveMode == 61) {
                    midiChannelSelector[mode] = value;
                    loadPrefs();
                }

                else if (pressureReceiveMode < 98) {
                    ED[mode][pressureReceiveMode - 48] = value;  // More expression and drones settings.
                    loadPrefs();
                }

                else if (pressureReceiveMode >= 200 && pressureReceiveMode < (kIMUnVariables + 200)) {
                    IMUsettings[mode][pressureReceiveMode - 200] = value;  // IMU settings
                    loadPrefs();
                }

                else if (pressureReceiveMode >= 300 && pressureReceiveMode < 304) {  // WARBL2 Custom fingering charts
                    blinkNumber[GREEN_LED] = 0;
                    WARBL2CustomChart[WARBL2CustomChartReceiveByte] = value;  // Put the value in the custom chart.
                    WARBL2CustomChartReceiveByte++;                           // Increment the location.

                    if (WARBL2CustomChartReceiveByte == 256) {
                        WARBL2CustomChartReceiveByte = 0;  // Reset for next time;
                        for (int i = 0; i < 256; i++) {    // Write the chart to EEPROM.
                            writeEEPROM((4000 + (256 * (pressureReceiveMode - 300))) + i, WARBL2CustomChart[i]);
                        }
                        blinkNumber[GREEN_LED] = 3;
                        sendMIDI(CONTROL_CHANGE, 7, 109, 100);  // Indicate success.
                        loadPrefs();
                    }
                }
            }



            /////// CC 109
            if ((number == 109 && value < kIMUnVariables) || (number == 109 && value > 99 && value < 104)) {  // Indicates that value for IMUsettings variable will be sent on CC 105.
                pressureReceiveMode = value + 200;                                                            // Add to the value because lower pressureReceiveModes are used for other variables.
                blinkNumber[GREEN_LED] = 0;
            }


            /////// CC 106
            if (number == 106 && value > 15) {


                if (value > 19 && value < 29) {  // Update enabled vibrato holes for "universal" vibrato.
                    bitSet(vibratoHolesSelector[mode], value - 20);
                    loadPrefs();
                }

                else if (value > 29 && value < 39) {
                    bitClear(vibratoHolesSelector[mode], value - 30);
                    loadPrefs();
                }

                else if (value == 39) {
                    useLearnedPressureSelector[mode] = 0;
                    loadPrefs();
                }

                else if (value == 40) {
                    useLearnedPressureSelector[mode] = 1;
                    loadPrefs();
                }

                else if (value == 41) {
                    learnedPressureSelector[mode] = sensorValue;
                    sendMIDI(CONTROL_CHANGE, 7, 104, 34);                                    // Indicate that LSB of learned pressure is about to be sent.
                    sendMIDI(CONTROL_CHANGE, 7, 105, learnedPressureSelector[mode] & 0x7F);  // Send LSB of learned pressure.
                    sendMIDI(CONTROL_CHANGE, 7, 104, 35);                                    // Indicate that MSB of learned pressure is about to be sent.
                    sendMIDI(CONTROL_CHANGE, 7, 105, learnedPressureSelector[mode] >> 7);    // Send MSB of learned pressure.
                    loadPrefs();
                }

                else if (value == 42) {  // Autocalibrate bell sensor only, or turn on stick mode using "hidden" Config Tool sequence.
                    blinkNumber[GREEN_LED] = 0;
                    if ((millis() - sticksModeTimer) < 10000) {  // Hidden way to turn on sticks mode.
                        IMUsettings[mode][STICKS_MODE] = !IMUsettings[mode][STICKS_MODE];
                        if (IMUsettings[mode][STICKS_MODE] == true) {
                            blinkNumber[GREEN_LED] = 3;
                        } else {
                            blinkNumber[GREEN_LED] = 1;
                        }
                        noteShiftSelector[mode] = prevKey;  // Reset the key to the previous value because it was only changed to toggle sticksMode.
                        sendMIDI(CONTROL_CHANGE, 7, (111 + mode), noteShiftSelector[mode]);
                        loadPrefs();
                        return;
                    }
                    calibration = 2;

                }


                else if (value == 43) {
                    int tempPressure = sensorValue;
                    ED[mode][DRONES_PRESSURE_LOW_BYTE] = tempPressure & 0x7F;
                    ED[mode][DRONES_PRESSURE_HIGH_BYTE] = tempPressure >> 7;
                    sendMIDI(CONTROL_CHANGE, 7, 104, 32);                                   // Indicate that LSB of learned drones pressure is about to be sent
                    sendMIDI(CONTROL_CHANGE, 7, 105, ED[mode][DRONES_PRESSURE_LOW_BYTE]);   // Send LSB of learned drones pressure
                    sendMIDI(CONTROL_CHANGE, 7, 104, 33);                                   // Indicate that MSB of learned drones pressure is about to be sent
                    sendMIDI(CONTROL_CHANGE, 7, 105, ED[mode][DRONES_PRESSURE_HIGH_BYTE]);  // Send MSB of learned drones pressure
                }


                else if (value == 45) {  // Save current sensor calibration as factory calibration
                    for (byte i = 18; i < 38; i++) {
                        writeEEPROM(i + 2000, readEEPROM(i));
                    }
                    for (int i = 1; i < 10; i++) {  // Save baseline calibration as factory baseline
                        writeEEPROM(i + 2000, readEEPROM(i));
                    }
                    for (int i = 1975; i < 1987; i++) {  // Save gyroscope calibration as factory calibration
                        writeEEPROM(i + 2000, readEEPROM(i));
                    }
                    blinkNumber[GREEN_LED] = 2;
                }

                else if (value == 54) {
                    calibrateIMU();
                }

                else if (value > 54 && value < (55 + kWARBL2SETTINGSnVariables)) {
                    WARBL2settingsReceiveMode = value - 55;
                }

                else if (value == 60) {  // Recenter IMU heading based on current
                    centerIMU();
                }

                else if (value > 99) {
                    for (byte i = 0; i < kGESTURESnVariables; i++) {  // Update button configuration
                        if (buttonReceiveMode == i) {

                            for (byte j = 0; j < 27; j++) {  // Update column 0 (action).
                                if (value == 100 + j) {
                                    buttonPrefs[mode][i][0] = j;
                                }
                            }
                        }
                    }
                }
            }




            /////// CC 119
            if (number == 119) {
                WARBL2settings[WARBL2settingsReceiveMode] = value;
                for (byte r = 0; r < kWARBL2SETTINGSnVariables; r++) {  // Save the WARBL2settings array each time it is changed by the Config Tool because it is independent of mode.
                    writeEEPROM(600 + r, WARBL2settings[r]);
                }
            }



            for (byte i = 0; i < 8; i++) {  // Update channel, byte 2, byte 3 for MIDI message for button MIDI command for row i
                if (buttonReceiveMode == i) {
                    if (number == 106 && value < 16) {
                        buttonPrefs[mode][i][2] = value;
                    } else if (number == 107) {
                        buttonPrefs[mode][i][3] = value;
                    } else if (number == 108) {
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
            if (!specialPressUsed[i]) {                                                            // We ignore it if the button was just used for a hard-coded command involving a combination of fingerholes.
                performAction(i);
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
    //Serial.println((buttonPrefs[mode][action][0]));

    if (communicationMode) {
        sendMIDI(CONTROL_CHANGE, 7, 109, 127);
        sendMIDI(CONTROL_CHANGE, 7, 105, action);
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
            }

            if (buttonPrefs[mode][action][1] == 1) {
                sendMIDI(CONTROL_CHANGE, buttonPrefs[mode][action][2], buttonPrefs[mode][action][3], buttonPrefs[mode][action][4]);
            }

            if (buttonPrefs[mode][action][1] == 2) {
                sendMIDI(PROGRAM_CHANGE, buttonPrefs[mode][action][2], buttonPrefs[mode][action][3]);
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
                sendMIDI(CONTROL_CHANGE, i, 123, 0);
                dronesOn = 0;  // Remember that drones are off, because MIDI panic will have most likely turned them off in all apps.
            }
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
                sendMIDI(CONTROL_CHANGE, 7, 102, 80 + breathMode);  // Send current breathMode
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
            } else {
                noteShift--;  // Shift down if we're in momentary mode, because the button is being released and a previous press has shifted up.
            }
            break;


        case SEMI_SHIFT_DOWN:
            if (!momentary[mode][action]) {
                noteShift--;  // Shift up if we're not in momentary mode
            } else {
                noteShift++;  // Shift down if we're in momentary mode, because the button is being released and a previous press has shifted up.
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
        sendMIDI(CONTROL_CHANGE, 7, 102, 70 + pitchBendMode);  //send current pitchbend mode to configuration tool.
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

    writeEEPROM(48, defaultMode);  // Save default mode

    for (byte r = 0; r < kWARBL2SETTINGSnVariables; r++) {  // Save the WARBL2settings array
        writeEEPROM(600 + r, WARBL2settings[r]);
    }

    writeEEPROM(1989, highByte(fullRunTime));  // The initial estimate of the total run time available on a full charge (minutes)
    writeEEPROM(1990, lowByte(fullRunTime));

    writeEEPROM(1993, highByte(prevRunTime));  // The elapsed run time on the currrent charge (minutes)--from the "factory" we set this to an estimated number of minutes because the battery charge state is unknown.
    writeEEPROM(1994, lowByte(prevRunTime));

    writeEEPROM(44, 3);  // Indicates settings have been saved.

    for (int i = 39; i < 1975; i++) {          // Then we read every byte in EEPROM from 39 to 1974 (Doesn't include sensor calibrations because we don't want to overwrite factory calibrations).
        writeEEPROM(2000 + i, readEEPROM(i));  // And rewrite them from 2039 to 3974. Then they'll be available to restore later if necessary.
    }

    blinkNumber[GREEN_LED] = 3;
}








// Restore original settings from EEPROM
void restoreFactorySettings() {
    for (int i = 1; i < 1987; i++) {  // Read factory settings and rewrite to the normal settings locations.
        writeEEPROM(i, readEEPROM(2000 + i));
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
    sendMIDI(CONTROL_CHANGE, 7, 110, VERSION);  //Send the firmware version.

    for (byte i = 0; i < 3; i++) {
        sendMIDI(CONTROL_CHANGE, 7, 102, 30 + i);                // Indicate that we'll be sending the fingering pattern for instrument i.
        sendMIDI(CONTROL_CHANGE, 7, 102, 33 + modeSelector[i]);  // Send

        if (noteShiftSelector[i] >= 0) {
            sendMIDI(CONTROL_CHANGE, 7, 111 + i, noteShiftSelector[i]);
        }  // Send noteShift, with a transformation for sending negative values over MIDI.
        else {
            sendMIDI(CONTROL_CHANGE, 7, 111 + i, noteShiftSelector[i] + 127);
        }
    }

    sendMIDI(CONTROL_CHANGE, 7, 102, 60 + mode);         // Send current instrument.
    sendMIDI(CONTROL_CHANGE, 7, 102, 85 + defaultMode);  // Send default instrument.

    sendMIDI(CONTROL_CHANGE, 7, 103, senseDistance);  // Send sense distance

    sendMIDI(CONTROL_CHANGE, 7, 117, vibratoDepth * 100UL / 8191);  // Send vibrato depth, scaled down to cents.
    sendMIDI(CONTROL_CHANGE, 7, 102, 70 + pitchBendMode);           // Send current pitchBend mode.
    sendMIDI(CONTROL_CHANGE, 7, 102, 80 + breathMode);              // Send current breathMode.
    sendMIDI(CONTROL_CHANGE, 7, 102, 120 + bellSensor);             // Send bell sensor state.
    sendMIDI(CONTROL_CHANGE, 7, 106, 39 + useLearnedPressure);      // Send calibration option.
    sendMIDI(CONTROL_CHANGE, 7, 104, 34);                           // Indicate that LSB of learned pressure is about to be sent.
    sendMIDI(CONTROL_CHANGE, 7, 105, learnedPressure & 0x7F);       // Send LSB of learned pressure.
    sendMIDI(CONTROL_CHANGE, 7, 104, 35);                           // Indicate that MSB of learned pressure is about to be sent.
    sendMIDI(CONTROL_CHANGE, 7, 105, learnedPressure >> 7);         // Send MSB of learned pressure.

    sendMIDI(CONTROL_CHANGE, 7, 104, 61);             // Indicate MIDI bend range is about to be sent.
    sendMIDI(CONTROL_CHANGE, 7, 105, midiBendRange);  // MIDI bend range

    sendMIDI(CONTROL_CHANGE, 7, 104, 62);               // Indicate MIDI channel is about to be sent.
    sendMIDI(CONTROL_CHANGE, 7, 105, mainMidiChannel);  // Midi bend range



    for (byte i = 0; i < 9; i++) {
        sendMIDI(CONTROL_CHANGE, 7, 106, 20 + i + (10 * (bitRead(vibratoHolesSelector[mode], i))));  // Send enabled vibrato holes.
    }

    for (byte i = 0; i < kGESTURESnVariables; i++) {
        sendMIDI(CONTROL_CHANGE, 7, 102, 90 + i);                         // Indicate that we'll be sending data for button commands row i (click 1, click 2, etc.).
        sendMIDI(CONTROL_CHANGE, 7, 106, 100 + buttonPrefs[mode][i][0]);  // Send action (i.e. none, send MIDI message, etc.).
        if (buttonPrefs[mode][i][0] == 1) {                               // If the action is a MIDI command, send the rest of the MIDI info for that row.
            sendMIDI(CONTROL_CHANGE, 7, 102, 112 + buttonPrefs[mode][i][1]);
            sendMIDI(CONTROL_CHANGE, 7, 106, buttonPrefs[mode][i][2]);
            sendMIDI(CONTROL_CHANGE, 7, 107, buttonPrefs[mode][i][3]);
            sendMIDI(CONTROL_CHANGE, 7, 108, buttonPrefs[mode][i][4]);
        }
    }

    for (byte i = 0; i < kSWITCHESnVariables; i++) {  // Send settings for switches in the slide/vibrato and register control panels.
        sendMIDI(CONTROL_CHANGE, 7, 104, i + 40);
        sendMIDI(CONTROL_CHANGE, 7, 105, switches[mode][i]);
    }

    for (byte i = 0; i < 21; i++) {  // Send settings for expression and drones control panels.
        sendMIDI(CONTROL_CHANGE, 7, 104, i + 13);
        sendMIDI(CONTROL_CHANGE, 7, 105, ED[mode][i]);
    }

    for (byte i = 21; i < 49; i++) {  // More settings for expression and drones control panels.
        sendMIDI(CONTROL_CHANGE, 7, 104, i + 49);
        sendMIDI(CONTROL_CHANGE, 7, 105, ED[mode][i]);
    }

    for (byte i = 0; i < 3; i++) {
        sendMIDI(CONTROL_CHANGE, 7, 102, 90 + i);  // Indicate that we'll be sending data for momentary.
        sendMIDI(CONTROL_CHANGE, 7, 102, 117 + momentary[mode][i]);
    }

    for (byte i = 0; i < 12; i++) {
        sendMIDI(CONTROL_CHANGE, 7, 104, i + 1);                      // Indicate which pressure variable we'll be sending.
        sendMIDI(CONTROL_CHANGE, 7, 105, pressureSelector[mode][i]);  // Send the data.
    }

    sendMIDI(CONTROL_CHANGE, 7, 102, 121);  // Tell the Config Tool that the bell sensor is present (always on this version of the WARBL).

    for (byte i = 55; i < 55 + kWARBL2SETTINGSnVariables; i++) {  // Send the WARBL2settings array.
        sendMIDI(CONTROL_CHANGE, 7, 106, i);
        sendMIDI(CONTROL_CHANGE, 7, 119, WARBL2settings[i - 55]);
    }

    manageBattery(true);  // Do this to send voltage and charging status to Config Tool.

    sendMIDI(CONTROL_CHANGE, 7, 106, 72);
    sendMIDI(CONTROL_CHANGE, 7, 119, (connIntvl * 100) & 0x7F);  // Send low byte of the connection interval.
    sendMIDI(CONTROL_CHANGE, 7, 106, 73);
    sendMIDI(CONTROL_CHANGE, 7, 119, (connIntvl * 100) >> 7);  // Send high byte of the connection interval.

    for (byte i = 0; i < kIMUnVariables; i++) {  // IMU settings
        sendMIDI(CONTROL_CHANGE, 7, 109, i);
        sendMIDI(CONTROL_CHANGE, 7, 105, IMUsettings[mode][i]);
    }
}










// Load saved fingering patterns
void loadFingering() {
    for (byte i = 0; i < 3; i++) {
        modeSelector[i] = readEEPROM(40 + i);
        noteShiftSelector[i] = (int8_t)readEEPROM(53 + i);

        if (communicationMode) {
            sendMIDI(CONTROL_CHANGE, 7, 102, 30 + i);                // Indicate that we'll be sending the fingering pattern for instrument i
            sendMIDI(CONTROL_CHANGE, 7, 102, 33 + modeSelector[i]);  // Send

            if (noteShiftSelector[i] >= 0) {
                sendMIDI(CONTROL_CHANGE, 7, (111 + i), noteShiftSelector[i]);
            }  // Send noteShift, with a transformation for sending negative values over MIDI.
            else {
                sendMIDI(CONTROL_CHANGE, 7, (111 + i), noteShiftSelector[i] + 127);
            }
        }
    }
}










// Save settings for current instrument as defaults for given instrument (i).
void saveSettings(byte i) {
    writeEEPROM(40 + i, modeSelector[mode]);
    writeEEPROM(53 + i, noteShiftSelector[mode]);
    writeEEPROM(50 + i, senseDistanceSelector[mode]);

    for (byte n = 0; n < kSWITCHESnVariables; n++) {  // Saved in this format, we can add more variables to the arrays without overwriting the existing EEPROM locations.
        writeEEPROM((56 + i + (n * 3)), switches[mode][n]);
    }

    writeEEPROM(333 + (i * 2), lowByte(vibratoHolesSelector[mode]));
    writeEEPROM(334 + (i * 2), highByte(vibratoHolesSelector[mode]));
    writeEEPROM(339 + (i * 2), lowByte(vibratoDepthSelector[mode]));
    writeEEPROM(340 + (i * 2), highByte(vibratoDepthSelector[mode]));
    writeEEPROM(345 + i, useLearnedPressureSelector[mode]);

    for (byte j = 0; j < 5; j++) {  // Save button configuration for current mode
        for (byte k = 0; k < kGESTURESnVariables; k++) {
            writeEEPROM(1000 + (i * 50) + (j * 10) + k, buttonPrefs[mode][k][j]);
        }
    }

    for (byte h = 0; h < 3; h++) {
        writeEEPROM(250 + (i * 3) + h, momentary[mode][h]);
    }

    for (byte q = 0; q < 12; q++) {
        writeEEPROM((260 + q + (i * 20)), pressureSelector[mode][q]);
    }

    writeEEPROM(273 + (i * 2), lowByte(learnedPressureSelector[mode]));
    writeEEPROM(274 + (i * 2), highByte(learnedPressureSelector[mode]));

    writeEEPROM(313 + i, pitchBendModeSelector[mode]);
    writeEEPROM(316 + i, breathModeSelector[mode]);
    writeEEPROM(319 + i, midiBendRangeSelector[mode]);
    writeEEPROM(322 + i, midiChannelSelector[mode]);

    for (byte n = 0; n < kEXPRESSIONnVariables; n++) {
        writeEEPROM((351 + i + (n * 3)), ED[mode][n]);
    }

    for (byte n = 0; n < kIMUnVariables; n++) {
        writeEEPROM((625 + i + (n * 3)), IMUsettings[mode][n]);
    }
}











// Load settings for all three instruments from EEPROM.
void loadSettingsForAllModes() {
    // Some things that are independent of mode.
    defaultMode = readEEPROM(48);  // Load default mode.

    for (byte r = 0; r < kWARBL2SETTINGSnVariables; r++) {  //Load the WARBL2settings array.
        WARBL2settings[r] = readEEPROM(600 + r);
    }

    fullRunTime = (word(readEEPROM(1989), readEEPROM(1990)));  // The total run time available on a full charge (minutes)
    prevRunTime = (word(readEEPROM(1993), readEEPROM(1994)));  // The total run time since the last full charge (minutes)

    if (readEEPROM(36) == 3) {  // If there has been a gyro calibration saved
        getEEPROM(1975, gyroXCalibration);
        getEEPROM(1979, gyroYCalibration);
        getEEPROM(1983, gyroZCalibration);
    }

    // Do all this for each mode.
    for (byte i = 0; i < 3; i++) {
        senseDistanceSelector[i] = readEEPROM(50 + i);

        for (byte n = 0; n < kSWITCHESnVariables; n++) {
            switches[i][n] = readEEPROM(56 + i + (n * 3));
        }

        vibratoHolesSelector[i] = word(readEEPROM(334 + (i * 2)), readEEPROM(333 + (i * 2)));
        vibratoDepthSelector[i] = word(readEEPROM(340 + (i * 2)), readEEPROM(339 + (i * 2)));
        useLearnedPressureSelector[i] = readEEPROM(345 + i);

        for (byte j = 0; j < 5; j++) {
            for (byte k = 0; k < kGESTURESnVariables; k++) {
                buttonPrefs[i][k][j] = readEEPROM(1000 + (i * 50) + (j * 10) + k);
            }
        }

        for (byte h = 0; h < 3; h++) {
            momentary[i][h] = readEEPROM(250 + (i * 3) + h);
        }

        for (byte m = 0; m < 12; m++) {
            pressureSelector[i][m] = readEEPROM(260 + m + (i * 20));
        }

        learnedPressureSelector[i] = word(readEEPROM(274 + (i * 2)), readEEPROM(273 + (i * 2)));


        pitchBendModeSelector[i] = readEEPROM(313 + i);
        breathModeSelector[i] = readEEPROM(316 + i);

        midiBendRangeSelector[i] = readEEPROM(319 + i);
        midiBendRangeSelector[i] = midiBendRangeSelector[i] > 96 ? 2 : midiBendRangeSelector[i];  // Sanity check in case uninitialized

        midiChannelSelector[i] = readEEPROM(322 + i);
        midiChannelSelector[i] = midiChannelSelector[i] > 16 ? 1 : midiChannelSelector[i];  // Sanity check in case uninitialized


        for (byte n = 0; n < kEXPRESSIONnVariables; n++) {
            ED[i][n] = readEEPROM(351 + i + (n * 3));
        }

        for (byte p = 0; p < kIMUnVariables; p++) {
            IMUsettings[i][p] = readEEPROM(625 + i + (p * 3));
        }

        IMUsettings[i][32] = IMUsettings[i][32] > 1 ? 0 : IMUsettings[i][32];  // Sanity check for sticks mode in case uninitialized
    }
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
            WARBL2CustomChart[i] = readEEPROM((4000 + (256 * (modeSelector[mode] - 67))) + i);
        }
    }

    pitchBend = 8192;
    expression = 0;
    shakeVibrato = 0;

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
        //toneholeScale[i] = ((8 * (16383 / midiBendRange)) / (toneholeCovered[i] - 50 - senseDistance) / 2);            // Precalculate scaling factors for pitchbend. This one is for sliding. We multiply by 8 first to reduce rounding errors. We'll divide again later.
        toneholeScale[i] = (((16383.0f / midiBendRange)) / (toneholeCovered[i] - 50.0f - senseDistance) / 2.0f);                     // Precalculate scaling factors for pitchbend. This one is for sliding. We multiply by 8 first to reduce rounding errors. We'll divide again later.
        vibratoScale[i] = ((2.0f * (((float)vibratoDepth) / midiBendRange)) / (toneholeCovered[i] - 50.0f - senseDistance) / 2.0f);  // This one is for vibrato.
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

    if (IMUsettings[mode][SEND_ROLL] || IMUsettings[mode][SEND_PITCH] || IMUsettings[mode][SEND_YAW] || IMUsettings[mode][PITCH_REGISTER] || IMUsettings[mode][STICKS_MODE]) {
        sox.setGyroDataRate(LSM6DS_RATE_208_HZ);  // Turn on the gyro if we need it.
    }

    // Calculate upper and lower bounds for IMU pitch register mapping.
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
                    toneholeCovered[i] = 0;     // First set the calibration to 0 for all of the sensors so it can only be increassed by calibrating.
                    toneholeBaseline[i] = 255;  // And set baseline high so it can only be reduced.
                }
            }
            if (bellSensor) {
                toneholeCovered[0] = 0;  // Also zero the bell sensor if it's plugged in (doesn't matter which calibration mode for this one).
                toneholeBaseline[0] = 255;
            }
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

            if (bellSensor && toneholeCovered[0] < toneholeRead[0]) {
                toneholeCovered[0] = toneholeRead[0];  // Calibrate the bell sensor too if it's plugged in.
            }
            if (bellSensor && toneholeBaseline[0] > toneholeRead[0]) {
                toneholeBaseline[0] = toneholeRead[0];  // Calibrate the bell sensor too if it's plugged in.
            }
        }

        if ((calibration == 1 && ((millis() - calibrationTimer) > 10000)) || (calibration == 2 && ((millis() - calibrationTimer) > 5000))) {
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
        writeEEPROM((1 + i), lowByte(toneholeBaseline[i]));  // The baseline readings can be stored in a single byte because they should be close to zero.
    }
    calibration = 0;
    writeEEPROM(37, 3);  // We write a 3 to address 37 to indicate that we have stored a set of calibrations.
    analogWrite(LEDpins[GREEN_LED], 0);
    LEDon[GREEN_LED] = 0;
}







// Load the stored sensor calibrations from EEPROM
void loadCalibration() {
    for (byte i = 0; i < 9; i++) {
        byte high = readEEPROM((i + 9) * 2);
        byte low = readEEPROM(((i + 9) * 2) + 1);
        toneholeCovered[i] = word(high, low);
        toneholeBaseline[i] = readEEPROM(1 + i);
    }
}








void calculateAndSendPressure() {

    if (abs(prevSensorValue - smoothed_pressure) > 1) {  // If pressure has changed more than a little, send it.
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

    inputPressureBounds[pressureOption][3] = (scaledPressure * (outputBounds[pressureOption][1] - outputBounds[pressureOption][0]) >> 10) + outputBounds[pressureOption][0];  // Map to output pressure range.

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









// Send pressure data
void sendPressure(bool force) {

    // Experimental smoothing of pressure at the expense of responsiveness

    /*
    static float filteredOld;
    const float timeConstant = 0.2f;
    float filtered = timeConstant * inputPressureBounds[0][3] + (1.0f - timeConstant) * filteredOld;  // Low-pass filter.
    inputPressureBounds[0][3] = filtered;
    filteredOld = filtered;
*/

    if (ED[mode][SEND_PRESSURE] == 1 && (inputPressureBounds[0][3] != prevCCPressure || force)) {
        sendMIDI(CONTROL_CHANGE, ED[mode][PRESSURE_CHANNEL], ED[mode][PRESSURE_CC], inputPressureBounds[0][3]);  // Send MSB of pressure mapped to the output range.
        prevCCPressure = inputPressureBounds[0][3];
    }

    if ((switches[mode][SEND_AFTERTOUCH] & 1)) {
        // hack
        int sendm = (!noteon && sensorValue <= 100) ? 0 : inputPressureBounds[2][3];
        if (sendm != prevChanPressure || force) {
            sendMIDI(CHANNEL_PRESSURE, mainMidiChannel, sendm);  // Send MSB of pressure mapped to the output range.
            prevChanPressure = sendm;
        }
    }

    // Poly aftertouch uses 2nd lowest bit of ED flag.
    if ((switches[mode][SEND_AFTERTOUCH] & 2) && noteon) {
        // Hack
        int sendm = (!noteon && sensorValue <= 100) ? 0 : inputPressureBounds[3][3];
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









// These convert MIDI messages from the old WARBL code to the correct format for the MIDI.h library. ToDo: These could be made shorter/more efficient.
void sendMIDI(uint8_t m, uint8_t c, uint8_t d1, uint8_t d2)  // Send a 3-byte MIDI event over USB.
{
    m &= 0xF0;
    c &= 0xF;
    d1 &= 0x7F;
    d2 &= 0x7F;


    switch (m) {

        case 0x80:  // Note Off
            {
                if (WARBL2settings[MIDI_DESTINATION] != 1 || connIntvl == 0) {  // If we're not only sending BLE or if we're not connected to BLE
                    MIDI.sendNoteOff(d1, d2, c);                                // Send USB MIDI.
                }
                if (WARBL2settings[MIDI_DESTINATION] != 0 || USBstatus != USB_HOST) {  // If we're not only sending USB or if we're not connected to USB
                    BLEMIDI.sendNoteOff(d1, d2, c);                                    // Send BLE MIDI.
                }
                break;
            }
        case 0x90:  // Note On
            {
                if (WARBL2settings[MIDI_DESTINATION] != 1 || connIntvl == 0) {
                    MIDI.sendNoteOn(d1, d2, c);
                }
                if (WARBL2settings[MIDI_DESTINATION] != 0 || USBstatus != USB_HOST) {
                    BLEMIDI.sendNoteOn(d1, d2, c);
                }
                break;
            }
        case 0xA0:  // Key Pressure
            {
                if (WARBL2settings[MIDI_DESTINATION] != 1 || connIntvl == 0) {
                    MIDI.sendAfterTouch(d1, d2, c);
                }
                if (WARBL2settings[MIDI_DESTINATION] != 0 || USBstatus != USB_HOST) {
                    BLEMIDI.sendAfterTouch(d1, d2, c);
                }
                break;
            }
        case 0xB0:  // CC
            {
                if (WARBL2settings[MIDI_DESTINATION] != 1 || connIntvl == 0 || (c == 7 && d1 >= 102)) {  // Always send CC messages within the Config Tool range so user can't get locked out of Config Tool if only BLE or USB is selected (changed by AM 4/11/24).
                    MIDI.sendControlChange(d1, d2, c);
                }
                if (WARBL2settings[MIDI_DESTINATION] != 0 || USBstatus != USB_HOST || (c == 7 && d1 >= 102)) {
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
                if (WARBL2settings[MIDI_DESTINATION] != 1 || connIntvl == 0) {
                    MIDI.sendPitchBend(pitch, c);
                }
                if (WARBL2settings[MIDI_DESTINATION] != 0 || USBstatus != USB_HOST) {
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
    c &= 0xF;
    d &= 0x7F;

    switch (m) {

        case 0xD0:  // Channel pressure
            {
                if (WARBL2settings[MIDI_DESTINATION] != 1 || connIntvl == 0) {
                    MIDI.sendAfterTouch(d, c);
                    // MIDI.sendPolyPressure(d, c); // deprecated
                }
                if (WARBL2settings[MIDI_DESTINATION] != 0 || USBstatus != USB_HOST) {
                    BLEMIDI.sendAfterTouch(d, c);
                    // BLEMIDI.sendPolyPressure(d, c); // deprecated
                }
                break;
            }


        case 0xC0:  // Program Change
            {
                if (WARBL2settings[MIDI_DESTINATION] != 1 || connIntvl == 0) {
                    MIDI.sendProgramChange(d, c);
                }
                if (WARBL2settings[MIDI_DESTINATION] != 0 || USBstatus != USB_HOST) {
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
    connIntvl = connIntvl * 0.625;                    // Convert to ms.

    if (communicationMode) {
        sendMIDI(CONTROL_CHANGE, 7, 106, 72);
        sendMIDI(CONTROL_CHANGE, 7, 119, (connIntvl * 100) & 0x7F);  // Send LSB of the connection interval to Config Tool.
        sendMIDI(CONTROL_CHANGE, 7, 106, 73);
        sendMIDI(CONTROL_CHANGE, 7, 119, (connIntvl * 100) >> 7);  // Send MSB of the connection interval to Config Tool.
    }

    blinkNumber[BLUE_LED] = 2;  // Indicate connection.
}









// Detect BLE disconnect.
void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
    (void)conn_handle;
    (void)reason;

    connIntvl = 0;

    if (communicationMode) {
        sendMIDI(CONTROL_CHANGE, 7, 106, 72);
        sendMIDI(CONTROL_CHANGE, 7, 119, 0);  // Send 0 to Config Tool.
        sendMIDI(CONTROL_CHANGE, 7, 106, 73);
        sendMIDI(CONTROL_CHANGE, 7, 119, 0);  // Send 0 to Config Tool.
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

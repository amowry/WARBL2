/*

Charging:

-monitor STAT pin from BQ25172 (detect 1 Hz blink on STAT pin for fault)
-when it goes high (and not fault), mark charging as on and set charging start time
-possibly subtract charging time from coulometry calculation in case WARBL is unplugged before termination (will have to frequently update charging time in EEPROM if we do this, but be careful of EEPROM wear (maybe ~ 15 minutes)-- an EEPROM write alo takes ~5 mS)
-after we terminate charge or STAT pin goes low, mark charging as off and reset run time in EEPROM
-use coulometry to calculate remaining battery percentage, assuming constant current consumption
-if battery drops to 1.0 V recalculate run time on total charge for coulometry calculation.
-indicate charging by pulsing or slowly flashing red or purple LED that times out if WARBL is playing notes (so as not to be annoying). LED should turn green when fully charged

*/

/*
Notes:
The charger will only start charging if the battery voltage is below ~ 1.35V and the battery temperature is between ~ 0 and 40 degrees C.

*/

void manageBattery(bool send) {

    static byte USBstatus = nrf_power_usbregstatus_vbusdet_get(NRF_POWER) + tud_ready();  //A combination of usbregstatus and tud_ready() can be used to detect battery power (0), dumb charger (1), or connected USB host (2).
    static byte chargingStatus = 0;                                                       //0 is not charging, 1 is charging, 2 is fault
    static byte prevChargingStatus1 = 1;                                                  //Keep track in case it has changed.
    static byte prevChargingStatus2 = 2;                                                  //FIFO with length of 3 for detecting a fault (blinking STAT pin)
    static bool chargeEnabled = 0;                                                        //Whether the carger is currently powered (by enabling the buck converter). The charger will then decide whether to charge, and report status on the STAT pin.
    static float voltageQueue[21];                                                        //Holds readings for finding the slope of the voltage curve while charging.
    static float voltageSlope;                                                            //Change in smoothed voltage over the previous 10 minutes
    static long chargingTime = 0;                                                         //How many milliseconds we've been charging
    static bool chargeTerminated = 0;                                                     //Tells us that a charge cycle has been terminated because the cell is full.

    if (digitalRead(STAT) == 0) {  //charging
        digitalWrite(redLED, HIGH);
        chargingStatus = 1;
    } else {
        digitalWrite(redLED, LOW);
        chargingStatus = 0;
    }


    if (prevChargingStatus1 != chargingStatus) {
        if (communicationMode) {  //Send the status to the Config Tool if it has changed
            sendMIDI(CC, 7, 106, 71);
            sendMIDI(CC, 7, 119, chargingStatus);
        }
        if (chargingStatus == 1) {
            chargingTime = millis();  //Start a timer if we just started charging.
        }
        prevChargingStatus1 = chargingStatus;
    }

    static float battVoltage = getBattVoltage();  //Read the battery

    const float alpha = 0.2;
    static float smoothed_voltage = battVoltage;
    smoothed_voltage = (1.0 - alpha) * smoothed_voltage + alpha * battVoltage;  //Exponential moving average -- takes several seconds to level out after powerup.

    //Serial.println(battVoltage);
    //Serial.println(smoothed_voltage);
    //Serial.println("");

    static byte cycles = 24;  //Every 30 seconds, record a new voltage reading, add it to the queue, and find the difference.
    if (cycles == 24 || send) {
        cycles = 0;

        if (chargingStatus == 1) {           //If we're charging, try to detect a full cell by monitoring the slope of dV/dt.
            for (byte i = 0; i < 20; i++) {  //Shift a new smoothed voltage reading into the queue.
                voltageQueue[20 - i] = voltageQueue[(20 - i) - 1];
            }
            voltageQueue[0] = smoothed_voltage;
            voltageSlope = voltageQueue[0] - voltageQueue[20];               //Find the difference in voltage over the past ten minutes (this number will be meaningless until 10 minutes has past)
            if (millis() - chargingTime > 660000 && voltageSlope < 0.002) {  // If we've been charging longer than 11 minutes and the curve has been mostly flat for the previous 10 minutes, terminate charging.
                digitalWrite(chargeEnable, LOW);                             //Terminate charging.
                chargeEnabled = 0;
                chargeTerminated = 1;  //This tells us not to enable charging again until the power is cycled.
            }

            Serial.println(smoothed_voltage, 3);
            Serial.println(voltageQueue[0], 3);
            Serial.println(voltageQueue[20], 3);
            Serial.println(voltageSlope, 3);
            Serial.println("");
            Serial.println("");
        }


        if (communicationMode) {  //Send voltage and charging status to Config Tool.
            sendMIDI(CC, 7, 106, 70);
            sendMIDI(CC, 7, 119, (((smoothed_voltage + 0.005) * 100) - 50));  //Convert to 0-127 for sending to Config Tool as 7 bits (possible range of 0.5 - 1.77 V in this format)

            sendMIDI(CC, 7, 106, 71);
            sendMIDI(CC, 7, 119, chargingStatus);  //Send charging status
        }
    }
    cycles++;



    //static float CPUtemp = readCPUTemperature(); //if needed for something like calibrating sensors. Can also use IMU temp.


    if (millis() > 2000 && (!chargeTerminated && !chargeEnabled && (WARBL2settings[CHARGE_FROM_HOST] && !battPower) || USBstatus == DUMB_CHARGER)) {
        digitalWrite(chargeEnable, HIGH);  //Enable charging (the charger will determine if it should actually start charging, based on batt voltage and temp.)
        chargeEnabled = 1;
    } else if (chargeEnabled == 1 && ((WARBL2settings[CHARGE_FROM_HOST] == 0 && USBstatus != DUMB_CHARGER) || battPower)) {  //Disable charging if we're on battery power or connected to a host and host charging isn't allowed.
        digitalWrite(chargeEnable, LOW);                                                                                     //Disable charging.
        chargeEnabled = 0;
    }


    if (battPower && USBstatus != BATTERY_POWER) {  //Check if we've been plugged in to power.
        digitalWrite(powerEnable, LOW);             //Disable the boost converter if there is USB power. The device will then power down if USB is unplugged again.
        battPower = false;
        recordRuntime();  //Record how long we were under battery power.
    }


    if (nowtime - powerDownTimer > WARBL2settings[POWERDOWN_TIME] * 60000) {  //Check to see if we've been idle long enough to power down.
        powerDown();
    }


    if (battPower && battVoltage <= 1.0) {  //Shut down when the battery is low.
        digitalWrite(redLED, HIGH);         //Indicate power down.
        delay(5000);                        //Long red LED to indicate shutdown because of low battery
        powerDown();
    }


    //Serial.print(word(EEPROM.read(1013), EEPROM.read(1014)));  //read the run time on battery since last full charge (minutes)
}








void powerDown() {

    if (battPower) {
        recordRuntime();
        digitalWrite(redLED, HIGH);  //Indicate power down.
        delay(500);
        digitalWrite(powerEnable, LOW);  //Disable the boost converter to cut power to the entire device.
    }
}








void recordRuntime() {
    runTimer = (millis() - runTimer) / 60000;  //Calculate how many minutes we have been powered by the battery.

    byte high = EEPROM.read(1013);  //Read previous time.
    byte low = EEPROM.read(1014);

    runTimer = runTimer + word(high, low);  //Rebuild stored run time.

    EEPROM.write(1013, highByte(runTimer));  //Update the recorded time in EEPROM. The EEPROM can handle more than 4 million write cycles.
    EEPROM.write(1014, lowByte(runTimer));   //Update the recorded time in EEPROM.
}








float getBattVoltage() {

    //Read battery voltage
    analogReference(AR_INTERNAL_1_8);    //Use 1.8 V reference to maximize resolution. The battery should never read higher than ~1.7 V because that's the charger overvoltage cutoff.
    analogOversampling(256);             //Increase oversampling for precise 12-bit reading.
    digitalWrite(battReadEnable, HIGH);  //We only connect the battery to the pin when reading to make sure it's not connected when the MCU is powered down.
    float battReading = analogRead(battRead);
    digitalWrite(battReadEnable, LOW);
    analogReference(AR_VDD4);  //Switch back to VDD reference for reading pressure sensor.
    analogOversampling(8);     //Change back to 8X oversampling for reading pressure sensor.

    //Calculate voltage. Assumes 12-bit ADC and 1.8V ref
    battReading = (battReading * 0.439453125f) / 1000;  //mV

    return battReading;
}

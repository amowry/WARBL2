/*
-ToDo: indicate charging by pulsing or slowly flashing red or purple LED that times out if WARBL is playing notes (so as not to be annoying). LED should turn green when fully charged
-ToDo: detect fault (1 Hz STAT pin blink)
*/


/*
Notes:

The battery is a 400 mAH 2/3 AAA NiMH cell. Example good brands are PKCell, Kastar, Dantona, and GP. Either flat top or button top works. The real-world capacity of these is usually around 350 mAH.
The BQ25172 charger will charge safely without any of this code when enabled by driving the chargeEnable pin high. The code monitors the battery level and does a better job of charge termination (0 dV/dt algorithm) to prolong battery life.
The charger will only start charging if the battery voltage is below ~ 1.35V and the battery temperature is between ~ 0 and 40 degrees C.
Charging is set by resistors to ~120 mA (0.3C) with a 4-hr safety timer. The charger will also terminate if battery voltage is > 1.7 V or battery temperature is over 40 degrees C.
Total device consumption is < 100 mA @ 5V, so any USB host (including iOS) will be able to charge.
The charger will report a fault (missing battery, out of temperature range) by blinking the STAT pin at 1 Hz.
*/

void manageBattery(bool send) {


    static byte chargingStatus = 0;      //0 is not charging, 1 is charging, 2 is fault
    static byte prevChargingStatus = 1;  //Keep track in case it has changed.
    static bool chargeEnabled = 0;       //Whether the charger is currently powered (by enabling the buck converter). The charger will then decide whether to charge, and report status on the STAT pin.
    static float voltageQueue[21];       //Holds readings for finding the slope of the voltage curve while charging.
    static float voltageSlope;           //Change in smoothed voltage over the previous 10 minutes
    static long chargeStartTime = 0;     //When we started charging
    static bool chargeTerminated = 0;    //Tells us that a charge cycle has been terminated because the cell is full.
    static byte battLevel;               //Estimated battery percentage remaining


    byte USBstatus = nrf_power_usbregstatus_vbusdet_get(NRF_POWER) + tud_ready();  //A combination of usbregstatus and tud_ready() can be used to detect battery power (0), dumb charger (1), or connected USB host (2).


    //Monitor STAT pin to tell if we're charging
    if (digitalRead(STAT) == 0) {    //charging
        digitalWrite(redLED, HIGH);  //ToDo: improve indication
        chargingStatus = 1;
    } else {  //Not charging
        digitalWrite(redLED, LOW);
        chargingStatus = 0;
    }



    //Detect change in charging status
    if (prevChargingStatus != chargingStatus) {
        if (communicationMode) {  //Send the status to the Config Tool if it has changed
            sendMIDI(CC, 7, 106, 71);
            sendMIDI(CC, 7, 119, chargingStatus);
        }
        if (chargingStatus == 1) {
            chargeStartTime = millis();                 //Start a timer if we just started charging.
        } else if (millis() > 2000 && chargeEnabled) {  //Charging was just stopped by the charger
            chargeTerminated = 1;                       //If charging was just stopped by the charger (rather than because we disabled it), mark it as terminated so we don't start again until power is cycled.
            digitalWrite(greenLED, HIGH);               //Indicate end of charge. ToDo: improve indication
            EEPROM.write(1013, 0);                      //Reset the total run time because we are now fully charged (high byte).
            EEPROM.write(1014, 0);                      //Low byte
            prevRunTime = 0;
        }
        prevChargingStatus = chargingStatus;
    }



    //Read the battery
    float battVoltage = getBattVoltage();
    const float alpha = 0.2;  //Time constant can be tweaked.
    static float smoothed_voltage = battVoltage;
    smoothed_voltage = (1.0 - alpha) * smoothed_voltage + alpha * battVoltage;  //Exponential moving average -- takes several seconds to level out after powerup.



    //Estimate the battery percentage remaining via coulometry. This is a rough estimate and mostly meaningless before the first full charge because we don't know the initial state of the battery. It becomes still more accurate after the first full discharge.

    //If we're charging, every 10 minutes subtract the estimated added run time due to charging from the stored run time on the current charge.
    if (chargingStatus == 1 && (nowtime - chargeStartTime) > 600000) {
        prevRunTime = prevRunTime - (((nowtime - chargeStartTime) / 60000) * (fullRunTime / 0.0055));
        chargeStartTime = nowtime;
        EEPROM.write(1013, highByte(prevRunTime));  //Update the recorded run time in EEPROM in case the power is cut. The EEPROM can handle more than 4 million write cycles, but don't do this too often. An EEPROM write also takes up to 5 mS.
        EEPROM.write(1014, lowByte(prevRunTime));
    }

    //Calculate the current battery percentage based on the run time and the run time available from a full charge.
    if (battPower) {
        battLevel = ((fullRunTime - prevRunTime - ((nowtime - runTimer) / 60000)) / float(fullRunTime)) * 100;  //If we're on battery power we also have to subtract time since we powered up.
    } else {
        battLevel = ((fullRunTime - prevRunTime) / float(fullRunTime)) * 100;  //If we're not on battery power, just use the saved run time.
    }


    //Serial.println(battLevel);



    static byte cycles = 24;  //24 cycles is 30 seconds.

    //Send voltage and charging status to Config Tool.
    if (cycles == 24 || send) {
        if (communicationMode) {
            sendMIDI(CC, 7, 106, 70);
            sendMIDI(CC, 7, 119, (((smoothed_voltage + 0.005) * 100) - 50));  //Convert to 0-127 for sending to Config Tool as 7 bits (possible range of 0.5 - 1.77 V in this format).

            sendMIDI(CC, 7, 106, 71);
            sendMIDI(CC, 7, 119, chargingStatus);  //Send charging status

            sendMIDI(CC, 7, 106, 74);
            sendMIDI(CC, 7, 119, battLevel);  //Send battery level
        }
    }


    //If we're charging, try to detect a full cell earlier than the charger timeout by monitoring dV/dt.
    if (cycles == 24) {  //Every 30 seconds, record a new smoothed voltage reading, add it to the queue, and find the difference between the first and last reading in the queue.
        cycles = 0;

        if (chargingStatus == 1) {
            for (byte i = 0; i < 20; i++) {  //Shift a new smoothed voltage reading into the queue.
                voltageQueue[20 - i] = voltageQueue[(20 - i) - 1];
            }
            voltageQueue[0] = smoothed_voltage;
            voltageSlope = voltageQueue[0] - voltageQueue[20];  //Find the difference in voltage over the past ten minutes (this number will be large and meaningless meaningless until 10 minutes has past and the queue has been populated--that's okay because we don't need to terminate in the first 10 minutes anyway).
            if (voltageSlope < 0.001) {                         //If the curve has been flat for the previous 10 minutes, terminate charging.
                digitalWrite(chargeEnable, LOW);                //Disable charging.
                chargeEnabled = 0;
                chargeTerminated = 1;          //This tells us not to enable charging again until the power is cycled.
                digitalWrite(greenLED, HIGH);  //Indicate end of charge. ToDo: improve indication
                EEPROM.write(1013, 0);         //Reset the total run time because we are now fully charged (high byte).
                EEPROM.write(1014, 0);         //Low byte
                prevRunTime = 0;
            }


            //Serial.print(smoothed_voltage, 3);
            //Serial.print(",");
            //Serial.println(voltageSlope, 3);
        }
    }
    cycles++;



    //Enable or disable charging (by supplying power to the charger with the buck converter) based on settings and whether USB power is available.
    if (nowtime > 2000 && (!chargeTerminated && !chargeEnabled && (WARBL2settings[CHARGE_FROM_HOST] && !battPower) || USBstatus == DUMB_CHARGER)) {
        digitalWrite(chargeEnable, HIGH);  //Enable charging (the charger will determine if it should actually start charging, based on batt voltage and temp.)
        chargeEnabled = 1;
    } else if (chargeEnabled && ((WARBL2settings[CHARGE_FROM_HOST] == 0 && USBstatus != DUMB_CHARGER) || battPower)) {  //Disable charging if we're on battery power or connected to a host and host charging isn't allowed.
        digitalWrite(chargeEnable, LOW);                                                                                //Disable charging.
        chargeEnabled = 0;
    }



    //Check if we've been plugged in to power.
    if (battPower && USBstatus != BATTERY_POWER) {
        digitalWrite(powerEnable, LOW);  //Disable the boost converter if there is USB power. The device will then power down if USB is unplugged again.
        battPower = false;
        recordRuntime(false);  //Record how long we were under battery power.
    }



    //Check to see if we've been idle long enough to power down.
    if (nowtime - powerDownTimer > WARBL2settings[POWERDOWN_TIME] * 60000) {
        powerDown(false);
    }



    //Shut down when the battery is low.
    if (nowtime > 2000 && battPower && battVoltage <= 1.0) {  //Give some time to make sure we detect USB power if it's present.
        digitalWrite(redLED, HIGH);                           //Indicate power down.
        delay(5000);                                          //Long red LED to indicate shutdown because of low battery
        powerDown(true);                                      //power down and reset the total run time available on a full charge (because we have just measured it by using up a full charge)
    }
}








void powerDown(bool resetTotalRuntime) {

    if (battPower) {
        recordRuntime(resetTotalRuntime);
        digitalWrite(redLED, HIGH);  //Indicate power down.
        delay(500);
        digitalWrite(powerEnable, LOW);  //Disable the boost converter to cut power to the entire device.
    }
}








void recordRuntime(bool resetTotalRuntime) {

    runTimer = (nowtime - runTimer) / 60000;  //Calculate how many minutes we have been powered by the battery.

    runTimer = runTimer + prevRunTime;  //Rebuild stored run time.

    if (resetTotalRuntime) {  //Use the elapsed run time to update the total run time available on a full charge, because we have terminated because of a low battery.
        EEPROM.write(1009, highByte(runTimer));
        EEPROM.write(1010, lowByte(runTimer));
    }

    EEPROM.write(1013, highByte(runTimer));  //Update the recorded run time in EEPROM.
    EEPROM.write(1014, lowByte(runTimer));
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

/*

Charging:

-monitor STAT pin from BQ25172 (detect 1 Hz blink on STAT pin for fault)
-when it goes high (and not fault), mark charging as on and set charging start time
-possibly subtract charging time from coulometry calculation in case WARBL is unplugged before termination (will have to frequently update charging time in EEPROM if we do this, but be careful of EEPROM wear (maybe ~ 15 minutes)-- an EEPROM write alo takes ~5 mS)
-terminate if smoothed voltage is ~flat (will need to tune this value)
-don't terminate for first ~5-10 minutes, or if voltage is below ~1.4-1.45 V?
-after we terminate charge or STAT pin goes low, mark charging as off and reset run time in EEPROM
-use coulometry to calculate remaining battery percentage, assuming constant current consumption
-if battery drops to 1.0 V recalculate run time on total charge for coulometry calculation.
-indicate charging by pulsing or slowly flashing red or purple LED that times out if WARBL is playing notes (so as not to be annoying). LED should turn green when fully charged

*/

void manageBattery(bool send) {

    USBstatus = nrf_power_usbregstatus_vbusdet_get(NRF_POWER) + tud_ready();  //A combination of usbregstatus and tud_ready() can be used to detect battery power (0), dumb charger (1), or connected USB host (2).


    static byte chargingStatus = 0;       //0 is not charging, 1 is charging, 2 is fault
    static byte prevChargingStatus1 = 1;  //keep track in case it has changed
    static byte prevChargingStatus2 = 2;  //FIFO with length of 3 for detecting a fault (blinking STAT pin)

    if (digitalRead(STAT) == 0) {
        digitalWrite(redLED, HIGH);  //charging
        chargingStatus = 1;
    } else {
        digitalWrite(redLED, LOW);
        chargingStatus = 0;
    }

    //Serial.println(chargingStatus);
    //Serial.println(prevChargingStatus);
    //Serial.println("");


    if (prevChargingStatus1 != chargingStatus) {  //send the status to the Config Tool if it has changed
        if (communicationMode) {
            sendMIDI(CC, 7, 106, 71);
            sendMIDI(CC, 7, 119, chargingStatus);
        }
        prevChargingStatus1 = chargingStatus;
    }

    battVoltage = getBattVoltage();

    const float alpha = 0.2;
    static float smoothed_voltage = battVoltage;
    smoothed_voltage = (1.0 - alpha) * smoothed_voltage + alpha * battVoltage;  //exponential moving average -- takes several seconds to level out after powerup (smoothing might not even be necessary)


    static byte cycles = 24;  //send voltage and charging status to Config Tool every 30 seconds
    if (cycles == 24 || send) {
        cycles = 0;
        if (communicationMode) {
            sendMIDI(CC, 7, 106, 70);
            sendMIDI(CC, 7, 119, (((smoothed_voltage + 0.005) * 100) - 50));  //convert to 0-127 for sending to Config Tool as 7 bits (possible range of 0.5 - 1.77 V in this format)

            sendMIDI(CC, 7, 106, 71);
            sendMIDI(CC, 7, 119, chargingStatus);  //send charging status
        }
    }
    cycles++;

    //CPUtemp = readCPUTemperature();


    if (millis() > 2000 && (chargingStatus == 0 && (WARBL2settings[CHARGE_FROM_HOST] && !battPower) || USBstatus == DUMB_CHARGER)) {
        digitalWrite(chargeEnable, HIGH);  //enable charging (the charger will determine if it should actually start charging, based on batt voltage and temp.)
        //chargingStatus = 1;
    } else if (chargingStatus == 1 && (WARBL2settings[CHARGE_FROM_HOST] == 0 || battPower)) {
        digitalWrite(chargeEnable, LOW);  //disable charging
        //chargingStatus = 0;
    }


    if (battPower && USBstatus != BATTERY_POWER) {  //Check if we've been plugged in to power.
        digitalWrite(powerEnable, LOW);             //Disable the boost converter if there is USB power. The device will then power down if USB is unplugged again.
        battPower = false;
        recordRuntime();  //Record how long we were under battery power.
    }


    if (nowtime - powerDownTimer > WARBL2settings[POWERDOWN_TIME] * 60000) {  //Check to see if we've been idle long enough to power down.
        powerDown();
    }


    if (battPower && battVoltage <= 1.0) {  //shut down when the battery is low
        //digitalWrite(redLED, HIGH);         //Indicate power down.
        delay(5000);  //long red LED to indicate shutdown because of low battery
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

/*

Charging:

-indicate charging, fault (detect 1 Hz STAT pin for fault)
-using running average for V?
-don't terminate within the first several minutes or when voltage is below ~1.4-1.45 V?
-use 0 dV/dt for termination
-after termination reset run time in EEPROM

*/

void manageBattery() {


    battVoltage = getBattVoltage();

    WARBL2settings[VOLTAGE_FOR_SENDING] = (((battVoltage + 0.005) * 100) - 50);  //convert to 0-127 for sending to Config Tool as 7 bits (possible range of 0.5 - 1.77 V in this format)

    static byte cycles = 24;
    if (cycles == 24) {
        cycles = 0;
        if (communicationMode) {
            sendVoltage();  //send voltage to ConFig Tool every 30 seconds
        }
    }
    cycles++;

    //battTemp = getBattTemp();

    //CPUtemp = readCPUTemperature();

    USBstatus = nrf_power_usbregstatus_vbusdet_get(NRF_POWER) + tud_ready();  //A combination of usbregstatus and tud_ready() can be used to detect battery power (0), dumb charger (1), or connected USB host (2).


    if (USBstatus != BATTERY_POWER) {    //Check if we've been plugged in to power.
        digitalWrite(powerEnable, LOW);  //Disable the boost converter if there is USB power. The device will then power down if USB is unplugged again.
        if (battPower) {                 //if previously running on battery power
            battPower = false;
            recordRuntime();  //Record how long we were under battery power.
        }
    }



    if (nowtime - powerDownTimer > POWER_DOWN_TIME * 60000) {  //Check to see if we've been idle long enough to power down.
        powerDown();
    }



    if (battVoltage < 1.0) {  //shut down when the battery is low
        powerDown();
    }


    /*
    Serial.println(battVoltage, 3);
    //Serial.println(",");
    Serial.println(battTemp, 2);
    Serial.println("");
    */

    //Serial.print(word(EEPROM.read(1013), EEPROM.read(1014)));  //read the run time on battery since last full charge (minutes)
}








void powerDown() {

    if (USBstatus == BATTERY_POWER) {
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
    analogReference(AR_VDD4);  //Switch back to VDD reference.
    analogOversampling(8);     //Change back to 8X oversampling for reading pressure sensor.

    //Calculate voltage. Assumes 12-bit ADC and 1.8V ref
    battReading = (battReading * 0.439453125f) / 1000;  //mV

    return battReading;
}









float getBattTemp() {

    //Read battery temp
    analogOversampling(256);             //Increase oversampling for precise 12-bit reading.
    digitalWrite(battTempEnable, HIGH);  //We only direct current through the thermistor while reading to prevent self heating.
    float thermReading = analogRead(battTempRead);
    digitalWrite(battTempEnable, LOW);
    analogOversampling(8);  //Change back to 8X oversampling for reading pressure sensor

    //Use thermistor B parameter (3380) to calculate temp in degrees C. Assumes 12-bit ADC and VDD ref.
    float battTempC = (1.00 / ((1.00 / 298.15) + (1.00 / 3380.00) * (log(4096 / (float)thermReading - 1.00)))) - 273.15;

    return battTempC;
}









//send voltage to ConFig Tool
void sendVoltage() {

    sendMIDI(CC, 7, 106, 57);
    sendMIDI(CC, 7, 119, WARBL2settings[VOLTAGE_FOR_SENDING]);
}
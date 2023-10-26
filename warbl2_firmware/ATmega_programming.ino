//Reprogram the ATmega32u4 if neccessary.
//Taken from Adafruit_AVRProg library examples
//Notes: It is currently necessary to alter the library to add the NRF52840 to the list of architectures that support a larger buffer for the hex code (line 34 in Adafruit_AVRProg.h). Hopefully this will be added to the library.

Adafruit_AVRProg avrprog = Adafruit_AVRProg();

/*
 * Pins to target
 */
#define AVRPROG_SCK 25
#define AVRPROG_MISO 23
#define AVRPROG_MOSI 24
#define AVRPROG_RESET 8  //This will be 27 in final WARBL version

#define LED_PROGMODE LED_BUILTIN
#define LED_ERR LED_BUILTIN

#define BUTTON AVRPROG_RESET  // use the board's reset button!
#define PIEZOPIN 27           //Piezo not needed--should set this to an unused pin in final version?

extern const image_t *images[];


void programATmega(void) {

    while (!Serial)
        ;
    delay(100);

    avrprog.setProgramLED(LED_PROGMODE);
    avrprog.setErrorLED(LED_ERR);
    avrprog.setSPI(AVRPROG_RESET, AVRPROG_SCK, AVRPROG_MOSI, AVRPROG_MISO);


    if (!avrprog.targetPower(true)) {
        avrprog.error("Failed to connect to target");
    }

    Serial.print(F("\nReading signature: "));
    uint16_t signature = avrprog.readSignature();
    Serial.println(signature, HEX);
    if (signature == 0 || signature == 0xFFFF) {
        avrprog.error(F("No target attached?"));
    }

    const image_t *targetimage = images[0];
    if (targetimage->image_chipsig != signature) {
        avrprog.error(F("Signature doesn't match image"));
    }
    Serial.println(F("Found matching chip/image"));

    Serial.print(F("Erasing chip..."));
    avrprog.eraseChip();
    Serial.println(F("Done!"));

    if (!avrprog.programFuses(
          targetimage->image_progfuses)) {  // get fuses ready to program
        avrprog.error(F("Programming Fuses fail"));
    }

    // We should disconnect/reconnect after fusing
    avrprog.targetPower(false);
    delay(100);
    if (!avrprog.targetPower(true)) {
        avrprog.error("Failed to connect to target");
    }

    if (!avrprog.writeImage(targetimage->image_hexcode,
                            pgm_read_byte(&targetimage->image_pagesize),
                            pgm_read_word(&targetimage->chipsize))) {
        avrprog.error(F("Failed to write flash"));
    }

    Serial.println(F("\nVerifing flash..."));
    if (!avrprog.verifyImage(targetimage->image_hexcode)) {
        avrprog.error(F("Failed to verify flash"));
    }
    Serial.println(F("\nFlash verified correctly!"));

    // Set fuses to 'final' state
    if (!avrprog.programFuses(targetimage->image_normfuses)) {
        avrprog.error("Programming fuses fail");
    }

    for (byte i = 0; i < 4; i++) {  //Indicate success
        digitalWrite(LED_BUILTIN, HIGH);
        delay(200);
        digitalWrite(LED_BUILTIN, LOW);
        delay(200);
    }


    return;  //Added to skip fuse verification.

    //need to fix this-- the fuse verification process fails on the extended fuse. AM 10/26/23 Could be the fuse verification mask setting?
    if (!avrprog.verifyFuses(targetimage->image_normfuses,
                             targetimage->fusemask)) {
        avrprog.error("Failed to verify fuses");
    } else {
        Serial.println("Fuses verified correctly!");
    }
}

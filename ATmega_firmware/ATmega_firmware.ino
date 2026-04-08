
/*
    The ATmega32U4 is dedicated to reading the WARBL tone hole sensors. It reads the sensors
     then goes to sleep. When wakened by a pin interrupt it
    sends all tone hole data by SPI and then repeats.

    To compile this firmware, use the Adafruit ItsyBitsy 32u4 board package.

    To allow the NRF52840 to automatically update the firmware on the ATmega, compile this code as hex and paste it into the ATmega_programming.ino tab in the WARBL2 firmware.

    Copyright (C) 2023 Andrew Mowry warbl.xyz

    This program is free software: you can redistribute it and/or modif
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

#include <DIO2.h>             // Fast digitalWrite library used for toggling IR LEDs
#define GPIO2_PREFER_SPEED 1  // Digitalread speed, see: https://github.com/Locoduino/DIO2/blob/master/examples/standard_outputs/standard_outputs.ino
#include <avr/sleep.h>
#include <avr/power.h>
#include <SPI.h>

// GPIO constants
const volatile uint8_t holeTrans[] = { 0, 9, 8, 7, 11, 6, 4, 2, 1 };                      // The analog pins used for the tone hole phototransistors, in the following order: Bell,R4,R3,R2,R1,L3,L2,L1,Lthumb
const volatile GPIO_pin_t pins[] = { DP7, DP13, DP5, DP11, DP0, DP1, DP23, DP21, DP10 };  // The digital pins used for the tone hole leds, in the following order: Bell,R4,R3,R2,R1,L3,L2,L1,Lthumb. Uses a special declaration format for the GPIO library.

// Variables for reading tonehole sensors
int toneholeRead[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };                       // Storage for tonehole sensor readings
int tempToneholeReadA[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };                  // Temporary storage for ambient light tonehole sensor readings, written during the timer ISR.
volatile byte toneholePacked[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };  // We pack the 9 10-bit tonehole readings into 12 bytes to send via SPI to the NRF52840.
volatile bool useBellSensor = true;                                       // Whether we should read the bell sensor (not doing so helps conserve power).




void setup() {

    USBCON |= (1 << FRZCLK);  // Freeze the USB Clock.
    PLLCSR &= ~(1 << PLLE);   // Disable the USB Clock (PPL).
    USBCON &= ~(1 << USBE);   // Disable USB.

    // Disable other things we don't need, to save power. The ATmega32U4 iteslf consumes ~1.5 mA when awakened for a reading every 3 mS or ~1.8 when awakened very 2 ms.
    power_usb_disable();
    power_usart0_disable();
    power_timer0_disable();  // This disables millis()!! We don't want any timer ticks because they mess with SPI.
    power_timer1_disable();
    power_timer2_disable();
    power_timer3_disable();
    power_usart1_disable();

    DIDR0 = 0xff;  // Disable digital input circuits for analog pins.
    DIDR2 = 0xf3;

    for (byte i = 0; i < 9; i++) {  // Initialize the tonehole sensor IR LEDs.
        pinMode2f(pins[i], OUTPUT);
    }

    ADC_init();  // Initialize the ADC. The first set of readings will be all zeros because we won't start reading sensors until the first SPI request is received.

    // Turn on SPI in peripheral mode.
    SPCR |= bit(SPE);
    pinMode(MISO, OUTPUT);
    SPI.attachInterrupt();
    pinMode(3, INPUT_PULLUP);
    attachInterrupt(0, pinFall, FALLING);  // This pin is tied to CS and wakes us up from Standby mode (digital pin 3).
}




void pinFall() {
    SPDR = 0xff;  // Wake up and preload the SPI buffer with a verification byte.
}




void loop() {

    // Sleep after getting and preparing the readings.
    set_sleep_mode(SLEEP_MODE_STANDBY);
    cli();                // Turn off interrupts.
    power_adc_disable();  // Turn off the ADC to save a little more power.
    sleep_enable();
    sei();  // Turn on interrupts immediately before sleeping.
    sleep_cpu();

    // sleeping__________________________________
    // Wake and enter SPI interrupt.

    sleep_disable();  // Disable sleep after waking.
    power_adc_enable();

    while (digitalRead(SS) == LOW) {}  // Make sure the SPI transfer is done before proceeding.

    readSensors();
}




// SPI interrupt routine
ISR(SPI_STC_vect) {
    byte c = SPDR;  // Read request from NRF52840.
    if (c < 12) {
        SPDR = toneholePacked[c];  // Send requested byte to the NRF52840.
    } else if (c == 20) {
        useBellSensor = false;  // Don't use bell sensor.
    } else if (c == 21) {
        useBellSensor = true;  // Use bell sensor.
    }
}




EMPTY_INTERRUPT(ADC_vect);  // We're not using the ADC complete interrupt.




// 830 us to read all sensors and prepare the data. The total time that IR LEDs are on is 800 us, so power consumed by sensors is: 800 uS/3000 us * 13mA/LED == 3.5 mA when polled every 3 ms.
void readSensors(void) {

    if (useBellSensor) {
        digitalWrite2f(pins[0], HIGH);  // Turn on LED 0. If we can skip this step we save ~0.7 mA.
    }
    ADC_read(holeTrans[0]);                                           // Throwaway to give sensor 0 extra time to rise after turning on LED (it's a slower sensor).
    tempToneholeReadA[1] = ADC_read(holeTrans[1]);                    // Get ambient reading for sensor 1.
    toneholeRead[0] = ADC_read(holeTrans[0]) - tempToneholeReadA[0];  // Get illuminated reading for 0 and subtract previously measured ambient reading.
    if (!useBellSensor) {
        toneholeRead[0] = 0;  // Make sure it's 0 if we're not using the bell sensor.
    }
    digitalWrite2f(pins[0], LOW);   // Turn off LED 0.
    digitalWrite2f(pins[1], HIGH);  // Turn on LED 1.

    tempToneholeReadA[2] = ADC_read(holeTrans[2]);  // etc.
    toneholeRead[1] = ADC_read(holeTrans[1]) - tempToneholeReadA[1];
    digitalWrite2f(pins[1], LOW);
    digitalWrite2f(pins[2], HIGH);

    tempToneholeReadA[3] = ADC_read(holeTrans[3]);
    toneholeRead[2] = ADC_read(holeTrans[2]) - tempToneholeReadA[2];
    digitalWrite2f(pins[2], LOW);
    digitalWrite2f(pins[3], HIGH);

    tempToneholeReadA[4] = ADC_read(holeTrans[4]);
    toneholeRead[3] = ADC_read(holeTrans[3]) - tempToneholeReadA[3];
    digitalWrite2f(pins[3], LOW);
    digitalWrite2f(pins[4], HIGH);

    tempToneholeReadA[5] = ADC_read(holeTrans[5]);
    toneholeRead[4] = ADC_read(holeTrans[4]) - tempToneholeReadA[4];
    digitalWrite2f(pins[4], LOW);
    digitalWrite2f(pins[5], HIGH);

    tempToneholeReadA[6] = ADC_read(holeTrans[6]);
    toneholeRead[5] = ADC_read(holeTrans[5]) - tempToneholeReadA[5];
    digitalWrite2f(pins[5], LOW);
    digitalWrite2f(pins[6], HIGH);

    tempToneholeReadA[7] = ADC_read(holeTrans[7]);
    toneholeRead[6] = ADC_read(holeTrans[6]) - tempToneholeReadA[6];
    digitalWrite2f(pins[6], LOW);
    digitalWrite2f(pins[7], HIGH);

    tempToneholeReadA[8] = ADC_read(holeTrans[8]);
    toneholeRead[7] = ADC_read(holeTrans[7]) - tempToneholeReadA[7];
    digitalWrite2f(pins[7], LOW);
    digitalWrite2f(pins[8], HIGH);

    tempToneholeReadA[0] = ADC_read(holeTrans[0]);  // Here's where we get the ambient reading for hole 0, used in the caculation the next time around. For this hole (bell sensor), there is a delay of the polling interval (e.g. 3 ms) between the ambient and illuminated reading.
    toneholeRead[8] = ADC_read(holeTrans[8]) - tempToneholeReadA[8];
    digitalWrite2f(pins[8], LOW);


    for (byte i = 0; i < 9; i++) {
        if (toneholeRead[i] < 0) {  // Just in case
            toneholeRead[i] = 0;
        }
    }

    noInterrupts();  // There shouldn't be any interrupts now, but just in case...

    // Put 9 10-bit tone hole readings into 12 bytes.

    for (byte i = 0; i < 9; i++) {
        toneholePacked[i] = toneholeRead[i];  // Put lower 8 bits of tonehole readings into first 9 bytes of array.
    }

    toneholePacked[9] = 0;  // Clear these for the next reading.
    toneholePacked[10] = 0;
    toneholePacked[11] = 0;

    for (byte i = 0; i < 4; i++) {
        toneholePacked[9] = toneholePacked[9] | (toneholeRead[i] >> 8);  // Put upper 2 bits of first 4 toneholes into byte 9 of array, left to right.
        if (i != 3) { toneholePacked[9] = toneholePacked[9] << 2; }
    }

    for (byte i = 4; i < 8; i++) {
        toneholePacked[10] = toneholePacked[10] | (toneholeRead[i] >> 8);  // Put upper 2 bits of next 4 toneholes into byte 10 of array, left to right.
        if (i != 7) { toneholePacked[10] = toneholePacked[10] << 2; }
    }

    // byte 11:
    // bits 1..0 = high 2 bits of sensor 8
    // bits 3..2 = unused = 0
    // bits 7..4 = checksum
    toneholePacked[11] = toneholeRead[8] >> 8;                    // Put upper 2 bits of final tone hole into byte 11.
    toneholePacked[11] &= 0x0F;                                   // clear upper nibble
    uint8_t checksum = computeToneholeChecksum4(toneholePacked);  // Compute a checksum.
    toneholePacked[11] |= (checksum << 4);                        // Insert the checksum.


    interrupts();
}




// Initialize the ADC.
void ADC_init(void) {

    ADCSRA &= ~(bit(ADPS0) | bit(ADPS1) | bit(ADPS2));  // Clear ADC prescaler bits.
    ADCSRA = (1 << ADEN) | ((1 << ADPS2));              // Enable ADC Division Factor 16 (36 us).
    ADMUX = (1 << REFS0);                               // Voltage reference from Avcc (3.0 V).
}



// We use ADC noise reduction mode to sleep the CPU while performing conversions. It saves a lot of power and gives stable readings.
int ADC_read(byte pin) {

    if (pin >= 18) pin -= 18;  // Allow for channel or pin numbers.
    pin = analogPinToChannel(pin);

    ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((pin >> 3) & 0x01) << MUX5);
    ADMUX = (1 << REFS0) | (pin & 0x07);

    noInterrupts();
    set_sleep_mode(SLEEP_MODE_ADC);  // Sleep during sample.
    sleep_enable();

    ADCSRA |= bit(ADSC) | bit(ADIE);
    interrupts();
    sleep_cpu();  // Start the conversion.
    sleep_disable();

    while (bit_is_set(ADCSRA, ADSC)) {}  // Awake again, reading should be done, but better make sure.

    return ADC;
}



// Compute checksum for sensor values.
uint8_t computeToneholeChecksum4(const uint8_t *p) {
    // Fold all packet bytes except byte 11 into 4 bits.
    // Also include only the low 2 data bits of byte 11 (sensor 8 high bits),
    // not the checksum nibble itself.
    uint8_t x = 0;

    for (uint8_t i = 0; i < 11; i++) {
        x ^= p[i];
    }

    x ^= (p[11] & 0x03);  // only sensor8 top two bits

    // Fold to 4 bits
    x ^= (x >> 4);
    return x & 0x0F;
}

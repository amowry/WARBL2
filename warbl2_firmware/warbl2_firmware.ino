


/*
    Copyright (C) 2018-2023 Andrew Mowry warbl.xyz

    Many thanks to Michael Eskin, Jesse Chappell, Gerard Kilbride, Louis Barman, Randy George, and many other WARBL users for their additions and suggestions.

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


Approximate WARBL2 power budget (at 3.0 V): ~ 2.5 mA for NRF52840, 1.5 mA for ATmega32u4, 3.5 mA for tone hole sensors, 1.5 mA for other peripherals. 8.7 mA total, for ~ 12 hour battery life with 350 mAH battery and 86% efficient boost converter


***Notes about the FFC expansion port:
Connector is Molex 0512810598 (see datasheet for cable design): https://tools.molex.com/pdm_docs/sd/512810598_sd.pdf
An example of an off-the-shelf cable is GCT 05-05-A-0050-A-4-06-4-T.
Pinout from left to right, holding WARBL with mouthpiece pointing up, looking at the port: GND, D31, D5, 3V0, A0/D14. A0 can be used for reading an analog sensor, and any of the GPIO can be used at high speed for I2S, software I2C, etc. If reading a sensor, it's recommended to conserve battery life by using a digital pin to turn the sensor on only when needed. 


*/

#include "Defines.h"

// Installed with board package
#include <nrfx_power.h>  // For detecting VBUS
#include <nrf_wdt.h>     // Watchdog timer
#include <bluefruit.h>
#include <Arduino.h>
#include <Wire.h>  // I2C communication with EEPROM
#include <SPI.h>   // Communication with ATmega32U4 and IMU
#include <math.h>

// Libraries below may need to be installed.
#include <MIDI.h>
#include <Adafruit_LSM6DSOX.h>     //IMU
#include <SensorFusion.h>          // IMU fusion
#include <ResponsiveAnalogRead.h>  // Fast smoothing of 12-bit pressure sensor readings


// Create instances of library classes.
BLEDis bledis;
BLEMidi blemidi;
Adafruit_USBD_MIDI usb_midi;
Adafruit_LSM6DSOX sox;
SF sfusion;
#if defined(PROTOTYPE46)
ResponsiveAnalogRead analogPressure(A0, true);
#else
ResponsiveAnalogRead analogPressure(A1, true);
#endif

// Custom settings for MIDI library
struct MySettings : public MIDI_NAMESPACE::DefaultSettings {
    static const bool Use1ByteParsing = false;  //parse more than 1 byte per MIDI.read()
};

// Create instances of the Arduino MIDI Library classes.
MIDI_CREATE_CUSTOM_INSTANCE(BLEMidi, blemidi, BLEMIDI, MySettings);
MIDI_CREATE_CUSTOM_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI, MySettings);


// GPIO constants
const uint8_t LEDpins[] = { 8, 10, 3 };  // RGB. Blue is also LED_BUILTIN.
const uint8_t powerEnable = 19;          // Driving this high enables the boost converter, keeping the device powered from the battery after button 3 has been released.
const uint8_t chargeEnable = 7;          // Enables charging.
#if defined(PROTOTYPE46)
const uint8_t STAT = 15;  // Battery charger STAT pin for early prototype
#else
const uint8_t STAT = 27;  // Battery charger STAT pin
#endif
const uint8_t battReadEnable = 6;         // Driving this high connects the battery to NRF ADC for reading voltage.
const uint8_t battRead = 16;              // Analog pin for reading battery voltage
const uint8_t buttons[] = { 4, 17, 18 };  // Buttons 1, 2, 3


// Battery variables
unsigned long runTimer;             // The time when WARBL started running on battery power
bool battPower = false;             // Keeps track of when we're on battery power, for above timer
unsigned long fullRunTime = 720;    // The available run time on a full charge, in minutes. This is initialized with an estimate of 12 hours and then adjusted each time the battery is discharged.
unsigned long prevRunTime = 120;    // The total run time since the last full charge (minutes). Initialized at around 80% full (typical of new PKCell battery). It is zeroed after each full charge.
unsigned long powerDownTimer;       // For powering down after a period of no activity
byte USBstatus = 0;                 // Battery power (0), dumb charger (1), or connected USB host (2).
unsigned long chargeStartTime = 0;  // When we started charging.
byte battLevel;                     // Estimated battery percentage remaining


// BLE
uint16_t connIntvl = 0;  // The negotiated connection interval


// IMU data
float rawGyroX = 0.0f;
float rawGyroY = 0.0f;
float rawGyroZ = 0.0f;
float gyroX = 0.0f;
float gyroY = 0.0f;
float gyroZ = 0.0f;
float accelX = 0.0f;
float accelY = 0.0f;
float accelZ = 0.0f;
float currYaw = 0.0f;
float yawOffset = 0.0f;
float IMUtemp = 0.0f;
float gyroXCalibration = 0.0f;
float gyroYCalibration = 0.0f;
float gyroZCalibration = 0.0f;
float roll;
float pitch;
float yaw;
int shakeVibrato;                  // Shake vibrato depth, from -8192 to 8192
unsigned long autoCenterYawTimer;  // For determining when to auto-recenter the yaw after silence
bool pitchRegisterShifted;         // Whether the register has been shifted by IMU pitch
int pitchRegisterBounds[6];        // Pitch boundaries (in degrees) between registers, i.e. lower bound of register 1, upper bound of register 1, etc. for up to five registers.
bool shakeDetected = 0;


// Instrument
byte mode = 0;         // The current mode (instrument), from 0-2.
byte defaultMode = 0;  // The default mode, from 0-2.


// WARBL2 variables that are independent of instrument
byte WARBL2settings[] = { 2, 1, 5 };   // See defines above
uint8_t WARBL2CustomChart[256];        // The currently selected custom fingering chart. This is only populated if a custom chart is currently selected or if we're transferring a chart from the Config Tool to EEPROM.
int WARBL2CustomChartReceiveByte = 0;  // The byte in the custom chart currently being received from the Config Tool


// Variables that can change according to instrument.
int8_t octaveShift = 0;                       // Octave transposition
int8_t noteShift = 0;                         // Note transposition, for changing keys. All fingering patterns are initially based on the key of D, and transposed with this variable to the desired key.
byte pitchBendMode = kPitchBendSlideVibrato;  // 0 means slide and vibrato are on. 1 means only vibrato is on. 2 is all pitchbend off, 3 is legato slide/vibrato.
byte senseDistance = 10;                      // The sensor value above which the finger is sensed for bending notes. Needs to be higher than the baseline sensor readings, otherwise vibrato will be turned on erroneously.
byte breathMode = kPressureBreath;            // The desired presure sensor behavior: single register, overblow, thumb register control, bell register control.
unsigned int vibratoDepth = 1024;             // Vibrato depth from 0 (no vibrato) to 8191 (one semitone)
bool useLearnedPressure = 0;                  // Whether we use learned pressure for note on threshold, or we use calibration pressure from startup
byte midiBendRange = 2;                       // +/- semitones that the midi bend range represents
byte mainMidiChannel = 1;                     // Current MIDI channel to send notes on


// These are containers for the above variables, storing the value used by the three different instruments (modes).  First variable in array is for instrument 0, etc.
byte modeSelector[] = { kModeWhistle, kModeUilleann, kModeGHB };  // The fingering patterns chosen in the configuration tool, for the three instruments.
int8_t octaveShiftSelector[] = { 0, 0, 0 };
int8_t noteShiftSelector[] = { 0, 0, 0 };
byte pitchBendModeSelector[] = { 1, 1, 1 };
byte senseDistanceSelector[] = { 10, 10, 10 };
byte breathModeSelector[] = { 1, 1, 0 };
byte useLearnedPressureSelector[] = { 0, 1, 1 };  // Default to using learned pressure for isntruments 2 and 3
int learnedPressureSelector[] = { 0, 280, 475 };  // Some reasonable default bag pressure setting for uilleann and GHB
byte LSBlearnedPressure;                          // Used to reconstruct learned pressure from two MIDI bytes.
unsigned int vibratoHolesSelector[] = { 0b011111111, 0b011111111, 0b011111111 };
unsigned int vibratoDepthSelector[] = { 1024, 1024, 1024 };
byte midiBendRangeSelector[] = { 2, 2, 2 };
byte midiChannelSelector[] = { 1, 1, 1 };

bool momentary[3][3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };  // Whether momentary click behavior is desired for MIDI on/off message sent with a button. Dimension 0 is mode (instrument), dimension 1 is button 0,1,2.

byte switches[3][kSWITCHESnVariables] =           // Settings for the switches in various Config Tool panels (see defines)
  { { 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0 },    // Instrument 0
    { 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0 },    // Instrument 1
    { 0, 0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0 } };  // Instrument 2

byte IMUsettings[3][kIMUnVariables] =                                                                                   // Settings for mapping and sending IMU readings (see defines above)
  { { 0, 0, 0, 1, 1, 0, 36, 0, 127, 0, 36, 0, 127, 0, 36, 0, 127, 1, 1, 1, 2, 11, 10, 0, 0, 1, 0, 50, 0, 90, 2, 0 },    // Instrument 0
    { 0, 0, 0, 1, 1, 0, 36, 0, 127, 0, 36, 0, 127, 0, 36, 0, 127, 1, 1, 1, 2, 11, 10, 0, 0, 1, 0, 50, 0, 90, 2, 0 },    // Instrument 1
    { 0, 0, 0, 1, 1, 0, 36, 0, 127, 0, 36, 0, 127, 0, 36, 0, 127, 1, 1, 1, 2, 11, 10, 0, 0, 1, 0, 50, 0, 90, 2, 0 } };  // Instrument 2

byte ED[3][kEXPRESSIONnVariables] =                                                                                                                                                           // Settings for the Expression and Drones Control panels in the Configuration Tool (see defines).
  { { 0, 3, 0, 0, 1, 7, 0, 100, 0, 127, 0, 1, 51, 36, 0, 1, 51, 36, 0, 0, 0, 0, 127, 0, 127, 0, 127, 0, 127, 0, 127, 0, 127, 0, 0, 0, 0, 100, 2, 74, 73, 72, 71, 69, 67, 66, 64, 62, 61 },    // Instrument 0
    { 0, 3, 0, 0, 1, 7, 0, 100, 0, 127, 0, 1, 51, 36, 0, 1, 51, 36, 0, 0, 0, 0, 127, 0, 127, 0, 127, 0, 127, 0, 127, 0, 127, 0, 0, 0, 0, 100, 2, 74, 73, 72, 71, 69, 67, 66, 64, 62, 61 },    // Instrument 1
    { 0, 3, 0, 0, 1, 7, 0, 100, 0, 127, 0, 1, 51, 36, 0, 1, 51, 36, 0, 0, 0, 0, 127, 0, 127, 0, 127, 0, 127, 0, 127, 0, 127, 0, 0, 0, 0, 100, 2, 74, 73, 72, 71, 69, 67, 66, 64, 62, 61 } };  // Instrument 2

byte pressureSelector[3][12] =                         // Register control variables that can be changed in the Configuration Tool, Dimension 2 is variable: Bag: threshold, multiplier, hysteresis, (unused), jump time, drop time. Breath/mouthpiece: threshold, multiplier, hysteresis, transientFilter, jump time, drop time.
  { { 50, 20, 20, 15, 50, 75, 3, 7, 20, 0, 3, 10 },    // Instrument 0
    { 50, 20, 20, 15, 50, 75, 3, 7, 20, 0, 3, 10 },    // Instrument 1
    { 50, 20, 20, 15, 50, 75, 3, 7, 20, 0, 3, 10 } };  // Instrument 2

uint8_t buttonPrefs[3][kGESTURESnVariables][5] =                                                                                                                                                          // The button configuration settings. Dimension 1 is the three instruments. Dimension 2 is the button combination (see button/gesture defines). Dimension 3 is the desired result: Action, MIDI command type (noteon/off, CC, PC), MIDI channel, MIDI byte 2, MIDI byte 3.
  { { { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 13, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 } },    // Instrument 0
    { { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 13, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 } },    // Instrument 1
    { { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 13, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 } } };  // Instrument 2


// Other misc. variables
unsigned long wakeTime = 0;        // When we woke from sleep
byte blinkNumber[] = { 0, 0, 0 };  // The number of time to blink LEDs R, G, B.
bool LEDon[] = { 0, 0, 0 };        // Whether each LED is currently on
bool play = 0;                     // Turns sound off and on (with the use of a button action) when in bagless mode.
bool bellSensor = 1;               // Whether the bell sensor is plugged in
byte program = 0;                  // Current MIDI program change value. This always starts at 0 but can be increased/decreased with assigned buttons.
bool dronesState = 0;              // Keeps track of whether we're above or below the pressure threshold for turning drones on.
bool pulseLED[] = { 0, 0, 0 };     // Whether currently pulsing LEDs

// Misc. timers used in loop()
unsigned long pitchBendTimer = 0;
unsigned long timerD = 0;
unsigned long timerE = 0;
unsigned long timerF = 0;
unsigned long pressureTimer = 0;

// Variables for reading pressure sensor
int sensorValue = 0;                   // Current pressure sensor reading
int prevSensorValue = 0;               // Previous sensor reading, used to tell if the pressure has changed and should be sent.
int twelveBitPressure;                 // Raw 12-bit reading
int smoothed_pressure;                 // Smoothed 12-bit pressure for mapping to CC, aftertouch, etc.
int sensorCalibration = 0;             // The sensor reading at startup
byte offset = 15;                      // Called "threshold" in the Configuration Tool-- used along with the multiplier for calculating the transition to the second register.
byte multiplier = 15;                  // Controls how much more difficult it is to jump to second octave from higher first-octave notes than from lower first-octave notes.
int sensorThreshold[] = { 260, 0 };    // The pressure sensor thresholds for initial note on and shift from register 1 to register 2, before some transformations.
int upperBound = 255;                  // This represents the pressure transition between the first and second registers. It is calculated on the fly as: (sensorThreshold[1] + ((newNote - 60) * multiplier))
byte newState;                         // The note/octave state based on the sensor readings (1=not enough force to sound note, 2=enough force to sound first octave, 3 = enough force to sound second octave)
byte prevState = 1;                    // The previous state, used to reset the velocity timer.
unsigned long velocityDelayTimer = 0;  // A timer to wait for pressure to build for calculating velocity.
int jumpTime = 15;                     // The amount of time to wait before dropping back down from an octave jump to first octave because of insufficient pressure
int dropTime = 15;                     // The amount of time to wait (ms) before turning a note back on after dropping directly from second octave to note off
byte hysteresis = 15;                  // Register hysteresis
byte soundTriggerOffset = 3;           // The number of 10-bit sensor values above the calibration setting at which a note on will be triggered (first octave)
int learnedPressure = 0;               // The learned pressure reading, used as a base value
int currentState;                      // These several are used by the new state machine.
int rateChangeIdx = 0;
int previousPressure = 0;
bool holdoffActive = false;
int holdoffCounter = 0;
int upperBoundHigh;                  // Register boundary for moving up
int upperBoundLow;                   // Register boudary for moving down (with hysteresis)
unsigned long fingeringChangeTimer;  // Times how long it's been since the most recent fingering change. Used to hold off the register drop feature until we've "settled" in to a fingering pattern

unsigned int inputPressureBounds[4][4] =  // For mapping pressure input range to output range. Dimension 1 is CC, velocity, aftertouch, poly. Dimension 2 is minIn, maxIn, scaledMinIn, mappedPressure.
  { { 100, 800, 0, 0 },
    { 100, 800, 0, 0 },
    { 100, 800, 0, 0 },
    { 100, 800, 0, 0 } };

byte outputBounds[4][2] =  // Container for ED output pressure range variables (CC, velocity, aftertouch, poly)-- the ED variables will be copied here so they're in a more logical order. This is a fix for variables that were added later.
  { { 0, 127 },
    { 0, 127 },
    { 0, 127 },
    { 0, 127 } };

byte curve[4] = { 0, 0, 0, 0 };  // Similar to above-- more logical ordering for the pressure curve variable


// Variables for reading tonehole sensors
unsigned int toneholeCovered[] = { 100, 100, 100, 100, 100, 100, 100, 100, 100 };  // Value at which each tone hole is considered to be covered. These are set to a low value initially for testing sensors after assembly.
int toneholeBaseline[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };                            // Baseline (uncovered) hole tonehole sensor readings
int toneholeRead[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };                                // Tonehole sensor readings after being reassembled from above bytes
unsigned int holeCovered = 0;                                                      // Whether each hole is covered-- each bit corresponds to a tonehole.
bool fingersChanged = 1;                                                           // Keeps track of when the fingering pattern has changed.
unsigned int prevHoleCovered = 1;                                                  // So we can track changes.
volatile int tempNewNote = 0;
byte prevNote;
byte newNote = -1;              // The next note to be played, based on the fingering chart (does not include transposition).
byte notePlaying;               // The actual MIDI note being played, which we remember so we can turn it off again.
byte transientFilterDelay = 0;  // Small delay for filtering out transient notes
unsigned long transitionFilter = 0;


// Pitchbend variables
byte pitchBendOn[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };                           // Whether pitchbend is currently turned for for a specific hole
int pitchBend = 8192;                                                         // Total current pitchbend value
int iPitchBend[] = { 8192, 8192, 8192, 8192, 8192, 8192, 8192, 8192, 8192 };  // Current pitchbend value for each tonehole
float pitchBendPerSemi = 4096.0f;
int prevChanPressure = 0;
int prevCCPressure = 0;
int prevPolyPressure = 0;
unsigned long noteOnTimestamp = 0;                                                 // When the note was activated
byte vibratoEnable = 0;                                                            // If non-zero, send vibrato pitch bend
unsigned int holeLatched = 0b000000000;                                            // Holes that are disabled for vibrato because they were covered when the note was triggered. They become unlatched (0) when the finger is removed all the way.
unsigned int vibratoHoles = 0b111111111;                                           // Holes to be used for vibrato, left thumb on left, bell sensor far right.
float toneholeScale[] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };  // A scale for normalizing the range of each sensor, for sliding
float vibratoScale[] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };   // Same as above but for vibrato
int expression = 0;                                                                // Pitchbend up or down from current note based on pressure
bool customEnabled = 0;                                                            // Whether the custom vibrato above is currently enabled based on fingering pattern and pitchbend mode.
int adjvibdepth;                                                                   // Vibrato depth scaled to MIDI bend range.


// Variables for managing MIDI note output
bool noteon = 0;      // Whether a note is currently turned on
bool shiftState = 0;  // Whether the octave is shifted (could be combined with octaveShift)
int8_t shift = 0;     // The total amount of shift up or down from the base note 62 (D). This takes into account octave shift and note shift.
byte velocity = 127;  // MIDI note velocity


// Tonehole calibration variables
byte calibration = 0;  // Whether we're currently calibrating. 1 is for calibrating all sensors, 2 is for calibrating bell sensor only, 3 is for calibrating all sensors plus baseline calibration (normally only done once, in the "factory").


// Variables for reading buttons
bool pressed[] = { 0, 0, 0 };          // Whether a button is currently presed
bool released[] = { 0, 0, 0 };         // If a button has just been released
bool justPressed[] = { 0, 0, 0 };      // If a button has just been pressed
bool longPress[] = { 0, 0, 0 };        // Long button press
bool noteOnOffToggle[] = { 0, 0, 0 };  // If using a button to toggle a noteOn/noteOff command, keep track of state.
bool longPressUsed[] = { 0, 0, 0 };    // If we used a long button press, we set a flag so we don't use it again unless the button has been released first.
bool specialPressUsed[] = { 0, 0, 0 };
bool dronesOn = 0;  //used to monitor drones on/off.


// Variables for communication with the WARBL Configuration Tool
bool communicationMode = 0;          // Whether we are currently communicating with the tool.
byte buttonReceiveMode = 100;        // Which row in the button configuration matrix for which we're currently receiving data.
int pressureReceiveMode = 100;       // Indicates the variable for which we're currently receiving data
byte fingeringReceiveMode = 0;       // Indicates the mode (instrument) for  which a fingering pattern is going to be sent
byte WARBL2settingsReceiveMode = 0;  // Indicates the mode (instrument) for  which a WARBL2settings array variable is going to be sent








void setup() {

#if defined(RELEASE)
    Serial.end();  // Turn off CDC. Necessary for release to make a class-compliant device
#endif

    // AM 5/24 -- starting watchdog here in case something goes wrong during setup, like initializing peripherals.
    watchdog_enable(WATCHDOG_TIMEOUT_SECS * 1000);  // Enable the watchdog timer, to recover from hangs. If the watchdog triggers while on battery power, the WARBL will power down. On USB power, the NRF will reset but the peripherals will remain powered.

    // NRF stuff
    dwt_enable();                 // Enable DWT for high-resolution micros() (uses a bit more power).
    sd_clock_hfclk_request();     // Enable the high-frequency clock. This is necessary because of a hardware bug that requires the HFCLK for SPI. Instead you can alter SPI.cpp to force using SPIM2, which will use 0.15 mA less current. See issue: https://github.com/adafruit/Adafruit_nRF52_Arduino/issues/773
    NRF_POWER->DCDCEN = 1;        // ENABLE DC/DC CONVERTER, cuts power consumption.
    NRF_UART0->TASKS_STOPTX = 1;  // Disable UART-- saves ~0.1 mA average.
    NRF_UART0->TASKS_STOPRX = 1;
    NRF_UART0->ENABLE = 0;

    // Configure pins
    digitalWrite(battReadEnable, LOW);  // The default with this board is for output pins to be high, so drive them all low before setting them as outputs.
    digitalWrite(chargeEnable, LOW);
    digitalWrite(powerEnable, LOW);
    digitalWrite(LEDpins[RED_LED], LOW);
    digitalWrite(LEDpins[BLUE_LED], LOW);
    digitalWrite(LEDpins[GREEN_LED], LOW);

    pinMode(battReadEnable, OUTPUT);  // Set various pins as outputs.
    pinMode(chargeEnable, OUTPUT);
    pinMode(powerEnable, OUTPUT);
    pinMode(LEDpins[RED_LED], OUTPUT);
    pinMode(LEDpins[GREEN_LED], OUTPUT);
    pinMode(LEDpins[BLUE_LED], OUTPUT);
    pinMode(buttons[0], INPUT_PULLUP);  // Set buttons as inputs and enable internal pullup.
    pinMode(buttons[1], INPUT_PULLUP);
    pinMode(buttons[2], INPUT_PULLUP);
    pinMode(STAT, INPUT_PULLUP);  // STAT from charger

    analogWriteResolution(10);  // Increase resolution for pulsing LEDs.

    // Set up ADC
    const int adcBits = 12;
    analogOversampling(8);  // Takes 55 uS regardless of resolution.
    //analogOversampling(16); // 88 uS
    analogReference(AR_VDD4);       // Use VDD for analog reference.
    analogReadResolution(adcBits);  // 12 bit

    // Set up responsive analog read for adaptive filtering of pressure.
    const float snapmult = 0.01f;
    const float actthresh = 2.0f;
    analogPressure.setSnapMultiplier(snapmult);
    analogPressure.setActivityThreshold(actthresh);
    analogPressure.setAnalogResolution(1 << adcBits);

    // USB MIDI stuff
    usb_midi.setStringDescriptor("WARBL USB MIDI");
    MIDI.begin(MIDI_CHANNEL_OMNI);  // Initialize MIDI, and listen to all MIDI channels. This will also call usb_midi's begin().
    MIDI.turnThruOff();
    MIDI.setHandleControlChange(handleControlChange);  // Handle received MIDI CC messages.

    //delay(2000); // Makes it so button 3 must be held down for two seconds to power on.

    digitalWrite(powerEnable, HIGH);  // Enable the boost converter at startup at least until we have time to check for USB power.
    runTimer = millis();
    battPower = true;  // We'll change this if we detect USB.

    digitalWrite(LEDpins[GREEN_LED], HIGH);  // Indicate powerup.
    delay(600);
    digitalWrite(LEDpins[GREEN_LED], LOW);

    // Reset ATmega32u4 in case only the NRF52840 has been restarted (which can happen when powered by USB). I suspect things can occasionally hang if we try to start SPI when the ATmega hasn't been reset.
    #ifndef PROTOTYPE46
    pinMode(26, OUTPUT);  // ATmega reset pin
    digitalWrite(26, LOW);
    delay(5);
    pinMode(26, INPUT);
    delay(75);
    #endif

    // BLE MIDI stuff:
    Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
    Bluefruit.begin();
    Bluefruit.Periph.setConnIntervalMS(7.5, 15);           // Request the lowest possible connection interval.
    Bluefruit.setTxPower(8);                               // Max power.
    Bluefruit.autoConnLed(false);                          // Don't indicate connection (we'll do this in the connect callback instead).
    bledis.setManufacturer("Mowry Stringed Instruments");  // Configure and Start Device Information Service.
    bledis.setModel("WARBL BLE MIDI");
    bledis.begin();
    BLEMIDI.begin(MIDI_CHANNEL_OMNI);  // Initialize MIDI, and listen to all MIDI channels. This will also call blemidi service's begin().
    BLEMIDI.turnThruOff();
    BLEMIDI.setHandleControlChange(handleControlChange);          // Handle received MIDI CC messages.
    Bluefruit.Periph.setConnectCallback(connect_callback);        // Get connection information and handle indication.
    Bluefruit.Periph.setDisconnectCallback(disconnect_callback);  // Detect disconnect.
    startAdv();                                                   // Set up and start advertising. Comment this out for testing without BLE on.

    // I2C
    Wire.begin();           // Join i2c bus for EEPROM.
    Wire.setClock(400000);  // High speed

    // SPI
    pinMode(2, OUTPUT);     // CS for Atmega
    digitalWrite(2, HIGH);  // Ensure CS stays high for now.
    SPI.begin();

    // IMU
    sox.begin_SPI(12, &SPI, 0, 10000000);       // Start IMU (CS pin is D12) at 10 Mhz.
    sox.setGyroDataRate(LSM6DS_RATE_SHUTDOWN);  // Shut down the gyro for now to save power, and we'll turn it on in loadPrefs() if necessary. IMU uses 0.55 mA if both gyro and accel are on, or 170 uA for just accel.
    sox.setAccelDataRate(LSM6DS_RATE_208_HZ);   // Turn on the accel.

    //writeEEPROM(44, 255);  // This line can be uncommented to make a version of the software that will resave factory settings every time it is run.

    if (readEEPROM(44) != 3) {
        saveFactorySettings();  // If we're running the software for the first time, or if a factory reset has been requested, copy all settings to EEPROM.
    }

    if (readEEPROM(37) == 3) {
        loadCalibration();  // If there has been a calibration saved, reload it at startup.
    }

    loadFingering();
    loadSettingsForAllModes();
    mode = defaultMode;       // Set the startup instrument.
    analogPressure.update();  // Read the pressure sensor for calibration to ambient pressure.
    twelveBitPressure = analogPressure.getRawValue();
    sensorCalibration = twelveBitPressure >> 2;  // Reduce the reading to 10 bits and use it to calibrate.
    loadPrefs();                                 // Load the correct user settings based on current instrument.
    powerDownTimer = millis();                   // Reset the powerDown timer.

    //eraseEEPROM(); // Testing

    writeEEPROM(1991, VERSION);  // Update the firmware version if it has changed.

    // Reprogram the ATmega32U4 if necessary (doesn't work with 4.6 prototypes because they don't have a reset trace from the NRF to the ATmega reset pin.)
#ifndef PROTOTYPE46
    //while(!Serial); // Can uncomment this if not using release version, to show verbose programming output.
    if (ATMEGA_FIRMWARE_VERSION != readEEPROM(1995)) {
        if (programATmega()) {
            Serial.println("Success");
            writeEEPROM(1995, ATMEGA_FIRMWARE_VERSION);  // Update the stored ATmega version after burning.
        }
    }
#endif
}









void loop() {


    /////////// Things here happen ~ every 3 ms if connected to BLE and 2 ms otherwise.

    byte delayTime = calculateDelayTime();  // Figure out how long to sleep based on how much time has been consumed previously (delayTime ranges from 0 to 3 ms).
    delay(delayTime);                       // Put the NRF52840 in tickless sleep, saving power. ~ 2.5 mA NRF consumption with delay of 3 ms. Total device consumption is ~8.7 mA with 3 ms delay, or 10.9 mA with 2 ms delay.
    wakeTime = millis();                    // When we woke from sleep
    getSensors();                           // 200 us, 55 of which is reading the pressure sensor.
    getFingers();                           // Find which holes are covered. 4 us.
    getState();                             // Get the breath state. 3 us.
    debounceFingerHoles();                  // Get the new MIDI note if the fingering has changed.
    getShift();                             // Shift the next note up or down based on register and key.
    sendNote();                             // Send the note as soon as we know the note, state, and shift.
    readMIDI();                             // Read incoming MIDI messages .



    /////////// Things here happen ~ every 2 to 17 ms.

    byte pressureInterval = calculatePressureInterval();  // Determine how frequently to send MIDI messages based on pressure.
    if ((millis() - pressureTimer) >= pressureInterval) {
        pressureTimer = millis();
        calculateAndSendPressure();
    }



    /////////// Things here happen ~ every 5 ms. These are things that should happen at a regular rate regardless of connection but don't need to happen as fast as possible.

    if ((millis() - timerE) > 5) {
        timerE = millis();
        readIMU();    // Takes about 145 us using SensorFusion's Mahony.
        blink();      // Blink any LED if necessary.
        pulse();      // Pulse any LED if necessary.
        calibrate();  // Calibrate/continue calibrating if the command has been received.
        checkButtons();
        detectSip();
        detectShake();
        sendToConfig(false, false);  // Check the queue and send to the Configuration Tool if it is time.
    }



    /////////// Things here happen ~ every 9 ms if not connected to BLE or connected at a fast interval, and longer if connected at a slow interval. This ensures that we aren't sending pitchbend too much faster than the connection interval.

    if ((millis() - pitchBendTimer) >= ((connIntvl > 8 && WARBL2settings[MIDI_DESTINATION] != 0) ? (12) : 9)) {
        pitchBendTimer = millis();    // This timer is also reset when we send a note, so none if these things will happen until the next connection interval if using BLE.
        calculateAndSendPitchbend();  // 11-200 us depending on whether holes are partially covered.
        printStuff();                 // Debug
        sendIMU();                    // ~ 130 us
        shakeForVibrato();            // ~ 200 uS
    }



    /////////// Things here happen ~ every 0.75 s.

    if ((millis() - timerF) > 750) {  // This period was chosen for detection of a 1 Hz fault signal from the battery charger STAT pin.
        timerF = millis();
        manageBattery(false);  // Check the battery and manage charging. Takes about 300 us because of reading the battery voltage.
        watchdogReset();       // Feed the watchdog.
    }


    // timerD = micros(); // For benchmarking--can paste these lines around any of the function calls above.
    // Serial.println(micros() - timerD);
}
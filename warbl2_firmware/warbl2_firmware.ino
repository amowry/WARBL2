
/*
    Copyright (C) 2018-2023 Andrew Mowry warbl.xyz

    Many thanks to Michael Eskin, Jesse Chappell, and Louis Barman for their additions.

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


Approximate WARBL2 power budget: ~ 2.5 mA for NRF52840, 1.5 mA for ATmega32u4, 3.5 mA for tone hole sensors, 1.5 mA for other peripherals. 8.7 mA total, for ~ 12 hour battery life with 350 mAH battery and 86% efficient boost converter


*/

//Installed with board package
#include "nrfx_power.h"  //For detecting VBUS
#include <nrf_nvic.h>
#include "nrf_wdt.h"  //Watchdog timer
#include <bluefruit.h>
#include <Arduino.h>
#include <Wire.h>  //I2C communication with EEPROM
#include <SPI.h>   //communication with ATmega32U4 and IMU

//Libraries below may need to be installed.
#include <MIDI.h>
#include <SparkFun_External_EEPROM.h>
#include <Adafruit_LSM6DSOX.h>     //IMU
#include <SensorFusion.h>          // IMU fusion
#include "ResponsiveAnalogRead.h"  //Fast smoothing of 12 bt pressure sensor readings

BLEDis bledis;
BLEMidi blemidi;
Adafruit_USBD_MIDI usb_midi;

Adafruit_LSM6DSOX sox;  //IMU instance
SF sfusion;

ResponsiveAnalogRead analogPressure(A0, true);

ExternalEEPROM EEPROM;


//#define RELEASE //Uncomment for release version (turns off CDC)

#define VERSION 40            //software version number (without decimal point)
#define HARDWARE_REVISION 41  //hardware

#define HARDWARE_WATCHDOG_TIMEOUT_SECS 15  //To recover from hangups. The timeout needs to be set longer than any task. Currently, receiving custom fingering charts from the Config Tool takes about 6 seconds.

#define DEBOUNCE_TIME 0.02                          //button debounce time, in seconds---Integrating debouncing algorithm is taken from debounce.c, written by Kenneth A. Kuhn:http://www.kennethkuhn.com/electronics/debounce.c
#define SAMPLE_FREQUENCY 200                        //button sample frequency, in Hz
#define MAXIMUM (DEBOUNCE_TIME * SAMPLE_FREQUENCY)  //the integrator value required to register a button press

#define BATTERY_POWER 0  //no USB connected (running on battery power)
#define DUMB_CHARGER 1   //powered by a dumb USB charger
#define USB_HOST 2       //connected to a USB host

//MIDI commands
#define NOTE_OFF 0x80          //127
#define NOTE_ON 0x90           // 144
#define KEY_PRESSURE 0xA0      // 160
#define CC 0xB0                // 176
#define PROGRAM_CHANGE 0xC0    // 192
#define CHANNEL_PRESSURE 0xD0  // 208
#define PITCH_BEND 0xE0        // 224

//Fingering Patterns
#define kModeWhistle 0
#define kModeUilleann 1
#define kModeGHB 2
#define kModeNorthumbrian 3
#define kModeChromatic 4
#define kModeGaita 5
#define kModeNAF 6
#define kModeKaval 7
#define kModeRecorder 8
#define kModeBansuriWARBL 9       //only used for a custom regulators implementation, not the "official" software, or BansuriWARBL
#define kModeUilleannStandard 10  //contains no accidentals
#define kModeXiao 11
#define kModeSax 12
#define kModeGaitaExtended 13
#define kModeSaxBasic 14
#define kModeEVI 15
#define kModeShakuhachi 16
#define kModeSackpipaMajor 17
#define kModeSackpipaMinor 18
#define kModeCustom 19
#define kModeBombarde 20
#define kModeBaroqueFlute 21
#define kModeMedievalPipes 22
#define kModeBansuri 23
#define kWARBL2Custom1 67
#define kWARBL2Custom2 68
#define kWARBL2Custom3 69
#define kWARBL2Custom4 70
#define kModeNModes 28

//Pitch bend modes
#define kPitchBendSlideVibrato 0
#define kPitchBendVibrato 1
#define kPitchBendNone 2
#define kPitchBendLegatoSlideVibrato 3
#define kPitchBendNModes 4

//Register control modes
#define kPressureSingle 0
#define kPressureBreath 1
#define kPressureThumb 2
#define kPressureBell 3
#define kPressureNModes 4

//Drones control mode
#define kNoDroneControl 0
#define kSecretDroneControl 1
#define kBaglessDroneControl 2
#define kPressureDroneControl 3
#define kDroneNModes 4

//Used in register state machine
#define SILENCE 1
#define BOTTOM_REGISTER 2
#define TOP_REGISTER 3
#define SILENCE_HYSTERESIS 1
#define JUMP 0
#define DROP 1

//Variables in the switches array (settings for the swiches in the slide/vibrato and register control panels)
#define VENTED 0
#define BAGLESS 1
#define SECRET 2
#define INVERT 3
#define CUSTOM 4
#define SEND_VELOCITY 5
#define SEND_AFTERTOUCH 6  //second bit of this one is used for poly
#define FORCE_MAX_VELOCITY 7
#define IMMEDIATE_PB 8
#define LEGATO 9
#define OVERRIDE 10
#define THUMB_AND_OVERBLOW 11
#define R4_FLATTEN 12
#define kSWITCHESnVariables 13

//Variables in the ED array (settings for expression and drones panels, and misc. other Config Tool settings)
#define EXPRESSION_ON 0
#define EXPRESSION_DEPTH 1
#define SEND_PRESSURE 2
#define CURVE 3  // (0 is linear, 1 and 2 are power curves)
#define PRESSURE_CHANNEL 4
#define PRESSURE_CC 5
#define INPUT_PRESSURE_MIN 6
#define INPUT_PRESSURE_MAX 7
#define OUTPUT_PRESSURE_MIN 8
#define OUTPUT_PRESSURE_MAX 9
#define DRONES_ON_COMMAND 10
#define DRONES_ON_CHANNEL 11
#define DRONES_ON_BYTE2 12
#define DRONES_ON_BYTE3 13
#define DRONES_OFF_COMMAND 14
#define DRONES_OFF_CHANNEL 15
#define DRONES_OFF_BYTE2 16
#define DRONES_OFF_BYTE3 17
#define DRONES_CONTROL_MODE 18
#define DRONES_PRESSURE_LOW_BYTE 19
#define DRONES_PRESSURE_HIGH_BYTE 20
#define VELOCITY_INPUT_PRESSURE_MIN 21
#define VELOCITY_INPUT_PRESSURE_MAX 22
#define VELOCITY_OUTPUT_PRESSURE_MIN 23
#define VELOCITY_OUTPUT_PRESSURE_MAX 24
#define AFTERTOUCH_INPUT_PRESSURE_MIN 25
#define AFTERTOUCH_INPUT_PRESSURE_MAX 26
#define AFTERTOUCH_OUTPUT_PRESSURE_MIN 27
#define AFTERTOUCH_OUTPUT_PRESSURE_MAX 28
#define POLY_INPUT_PRESSURE_MIN 29
#define POLY_INPUT_PRESSURE_MAX 30
#define POLY_OUTPUT_PRESSURE_MIN 31
#define POLY_OUTPUT_PRESSURE_MAX 32
#define VELOCITY_CURVE 33
#define AFTERTOUCH_CURVE 34
#define POLY_CURVE 35
#define EXPRESSION_MIN 36
#define EXPRESSION_MAX 37
#define CUSTOM_FINGERING_1 38  //None of these "custom" variables these are used by WARBL2.
#define CUSTOM_FINGERING_2 39
#define CUSTOM_FINGERING_3 40
#define CUSTOM_FINGERING_4 41
#define CUSTOM_FINGERING_5 42
#define CUSTOM_FINGERING_6 43
#define CUSTOM_FINGERING_7 44
#define CUSTOM_FINGERING_8 45
#define CUSTOM_FINGERING_9 46
#define CUSTOM_FINGERING_10 47
#define CUSTOM_FINGERING_11 48
#define kEXPRESSIONnVariables 49

//Variables in the WARBL2settings array (independent of mode)
#define MIDI_DESTINATION 0  //0 means send MIDI to USB only, 1 means send to BLE only, 2 means send to both
#define CHARGE_FROM_HOST 1  //Charge from USB host in addition to "dumb" charging brick.
#define POWERDOWN_TIME 2
#define kWARBL2SETTINGSnVariables 3

//Variables in the IMUsettings array
#define SEND_ROLL 0  //On/off
#define SEND_PITCH 1
#define SEND_YAW 2
#define CENTER_ROLL 3  //On/off
#define CENTER_YAW 4
#define ROLL_INPUT_MIN 5  //0-36
#define ROLL_INPUT_MAX 6
#define ROLL_OUTPUT_MIN 7  //0-127
#define ROLL_OUTPUT_MAX 8
#define PITCH_INPUT_MIN 9
#define PITCH_INPUT_MAX 10
#define PITCH_OUTPUT_MIN 11
#define PITCH_OUTPUT_MAX 12
#define YAW_INPUT_MIN 13
#define YAW_INPUT_MAX 14
#define YAW_OUTPUT_MIN 15
#define YAW_OUTPUT_MAX 16
#define ROLL_CC_CHANNEL 17
#define PITCH_CC_CHANNEL 18
#define YAW_CC_CHANNEL 19
#define ROLL_CC_NUMBER 20
#define PITCH_CC_NUMBER 21
#define YAW_CC_NUMBER 22
#define AUTOCENTER_YAW 23           //On/Off
#define Y_SHAKE_PITCHBEND 24        //On/OffF (Only this axis is currently used for shake vibrato.)
#define AUTOCENTER_YAW_INTERVAL 25  //0-20 (represents 0-5s pause interval for yaw recentering)
#define X_PITCHBEND_DEPTH 26        //unused
#define Y_PITCHBEND_DEPTH 27        //0-100
#define Z_PITCHBEND_DEPTH 28        //unused
#define kIMUnVariables 29

//Custom settings for MIDI library
struct MySettings : public MIDI_NAMESPACE::DefaultSettings {
    static const bool Use1ByteParsing = false;  //parse more than 1 byte per MIDI.read()
};


//Create instances of the Arduino MIDI Library.
MIDI_CREATE_CUSTOM_INSTANCE(BLEMidi, blemidi, BLEMIDI, MySettings);
MIDI_CREATE_CUSTOM_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI, MySettings);

//Create instances of the Arduino MIDI Library.
//MIDI_CREATE_INSTANCE(BLEMidi, blemidi, BLEMIDI);
//MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI);

//GPIO constants
const uint8_t redLED = 8;
const uint8_t greenLED = 10;  //also LED_BUILTIN
const uint8_t blueLED = 3;

const uint8_t powerEnable = 19;    //Driving this high enables the boost converter, keeping the device powered from the battery after button 3 has been released.
const uint8_t chargeEnable = 7;    //Enables charging @ 120 mA current  (~0.33 C, should take around 3 hours to charge)
const uint8_t STAT = 15;           //STAT pin  -- input with pullup
const uint8_t battReadEnable = 6;  //Driving this high connects the battery to NRF for reading voltage
const uint8_t battRead = 16;       //Analog pin for reading battery voltage

const uint8_t buttons[] = { 4, 17, 18 };  //buttons 1, 2, 3


byte USBstatus = 0;  //Battery power (0), dumb charger (1), or connected USB host (2).


//Battery variables
unsigned long runTimer;           //The time when WARBL started running on battery power
bool battPower = false;           //Keeps track of when we're on battery power, for above timer
unsigned long fullRunTime = 720;  //The available run time on a full charge, in minutes. This is initialized with an estimate of 12 hours and then adjusted each time the battery is discharged.
unsigned long prevRunTime = 600;  //The total run time since the last full charge (minutes). Initialized at around 80% full (typical of new PKCell battery). It is zeroed after each full charge.
unsigned long powerDownTimer;     //For powering down after a period of no activity


//BLE
uint16_t connIntvl = 0;  // The negotiated connection interval


//Misc.
unsigned long timerC = 0;  //For timing various intervals
unsigned long timerD = 0;
unsigned long timerE = 0;
unsigned long timerF = 0;
unsigned long nowtime;


//IMU data
float rawGyroX = 0.0f;
float rawGyroY = 0.0f;
float rawGyroZ = 0.0f;
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
int shakeVibrato;                  //Shake vibrato depth, from -8192 to 8192
unsigned long autoCenterYawTimer;  //For determining when to auto-recenter the yaw after silence


//Instrument
byte mode = 0;         // The current mode (instrument), from 0-2.
byte defaultMode = 0;  // The default mode, from 0-2.


//WARBL2 variables that are independent of instrument
byte WARBL2settings[] = { 2, 1, 5 };   //see defines above
uint8_t WARBL2CustomChart[256];        //The currently selected custom fingering chart. This is only populated if a custom chart is currently selected or if we're transferring a chart from the Config Tool to EEPROM.
int WARBL2CustomChartReceiveByte = 0;  //The byte in the custom chart currently being received from the Config Tool


//Variables that can change according to instrument.
int8_t octaveShift = 0;                       //octave transposition
int8_t noteShift = 0;                         //note transposition, for changing keys. All fingering patterns are initially based on the key of D, and transposed with this variable to the desired key.
byte pitchBendMode = kPitchBendSlideVibrato;  //0 means slide and vibrato are on. 1 means only vibrato is on. 2 is all pitchbend off, 3 is legato slide/vibrato.
byte senseDistance = 10;                      //the sensor value above which the finger is sensed for bending notes. Needs to be higher than the baseline sensor readings, otherwise vibrato will be turned on erroneously.
byte breathMode = kPressureBreath;            //the desired presure sensor behavior: single register, overblow, thumb register control, bell register control.
unsigned int vibratoDepth = 1024;             //vibrato depth from 0 (no vibrato) to 8191 (one semitone)
bool useLearnedPressure = 0;                  //whether we use learned pressure for note on threshold, or we use calibration pressure from startup
byte midiBendRange = 2;                       // +/- semitones that the midi bend range represents
byte mainMidiChannel = 1;                     // current MIDI channel to send notes on


//These are containers for the above variables, storing the value used by the three different instruments.  First variable in array is for instrument 0, etc.
byte modeSelector[] = { kModeWhistle, kModeUilleann, kModeGHB };  //the fingering patterns chosen in the configuration tool, for the three instruments.
int8_t octaveShiftSelector[] = { 0, 0, 0 };
int8_t noteShiftSelector[] = { 0, 0, 8 };
byte pitchBendModeSelector[] = { 1, 1, 1 };
byte senseDistanceSelector[] = { 10, 10, 10 };
byte breathModeSelector[] = { 1, 1, 0 };
byte useLearnedPressureSelector[] = { 0, 1, 1 };  //default to using learned pressure for isntruments 2 and 3
int learnedPressureSelector[] = { 0, 280, 475 };  //some reasonable default bag pressure setting for uilleann and GHB
byte LSBlearnedPressure;                          //used to reconstruct learned pressure from two MIDI bytes.
unsigned int vibratoHolesSelector[] = { 0b011111111, 0b011111111, 0b011111111 };
unsigned int vibratoDepthSelector[] = { 1024, 1024, 1024 };
byte midiBendRangeSelector[] = { 2, 2, 2 };
byte midiChannelSelector[] = { 1, 1, 1 };

bool momentary[3][3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };  //whether momentary click behavior is desired for MIDI on/off message sent with a button. Dimension 0 is mode (instrument), dimension 1 is button 0,1,2.

byte switches[3][13] =  //Settings for the switches in various Config Tool panels (see defines above)

  {
      { 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0 },  //instrument 0
      { 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0 },  //same for instrument 1
      { 0, 0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0 }   //same for instrument 2
  };

byte IMUsettings[3][29] =  //Settings for mapping and sending IMU readings (see defines above)

  {
      { 0, 0, 0, 1, 1, 0, 36, 0, 127, 0, 36, 0, 127, 0, 36, 0, 127, 1, 1, 1, 2, 11, 10, 0, 0, 1, 64, 50, 64 },  //Instrument 0
      { 0, 0, 0, 1, 1, 0, 36, 0, 127, 0, 36, 0, 127, 0, 36, 0, 127, 1, 1, 1, 2, 11, 10, 0, 0, 1, 64, 50, 64 },  //Same for instrument 1
      { 0, 0, 0, 1, 1, 0, 36, 0, 127, 0, 36, 0, 127, 0, 36, 0, 127, 1, 1, 1, 2, 11, 10, 0, 0, 1, 64, 50, 64 },  //Same for instrument 2
  };

byte ED[3][49] =  //Settings for the Expression and Drones Control panels in the Configuration Tool (see defines above).

  {
      { 0, 3, 0, 0, 1, 7, 0, 100, 0, 127, 0, 1, 51, 36, 0, 1, 51, 36, 0, 0, 0, 0, 127, 0, 127, 0, 127, 0, 127, 0, 127, 0, 127, 0, 0, 0, 0, 100, 0, 74, 73, 72, 71, 69, 67, 66, 64, 62, 61 },  //instrument 0
      { 0, 3, 0, 0, 1, 7, 0, 100, 0, 127, 0, 1, 51, 36, 0, 1, 51, 36, 0, 0, 0, 0, 127, 0, 127, 0, 127, 0, 127, 0, 127, 0, 127, 0, 0, 0, 0, 100, 0, 74, 73, 72, 71, 69, 67, 66, 64, 62, 61 },  //same for instrument 1
      { 0, 3, 0, 0, 1, 7, 0, 100, 0, 127, 0, 1, 51, 36, 0, 1, 51, 36, 0, 0, 0, 0, 127, 0, 127, 0, 127, 0, 127, 0, 127, 0, 127, 0, 0, 0, 0, 100, 0, 74, 73, 72, 71, 69, 67, 66, 64, 62, 61 }   //same for instrument 2
  };

byte pressureSelector[3][12] =  //A selector array for all the register control variables that can be changed in the Configuration Tool

  {                              //instrument 0
      { 50, 20, 20, 15, 50, 75,  //bag: offset, multiplier, hysteresis, drop (now unused), jump time, drop time
        3, 7, 20, 0, 3, 10 },    //breath/mouthpiece: offset, multiplier, transientFilter, jump time, drop time
      //instrument 1
      {
        50, 20, 20, 15, 50, 75,
        3, 7, 20, 0, 3, 10 },
      //instrument 2
      {
        50, 20, 20, 15, 50, 75,
        3, 7, 20, 0, 3, 10 }
  };

uint8_t buttonPrefs[3][8][5] =  //The button configuration settings (no default actions as of firmware 2.1 to avoid confusion). Dimension 1 is the three instruments. Dimension 2 is the button combination: click 1, click 2, click3, hold 2 click 1, hold 2 click 3, longpress 1, longpress2, longpress3
                                //Dimension 3 is the desired action: Action, MIDI command type (noteon/off, CC, PC), MIDI channel, MIDI byte 2, MIDI byte 3.
                                //instrument 0---The actions are: 0 none, 1 send MIDI message, 2 change pitchbend mode, 3 instrument, 4 play/stop (bagless mode), 5 octave shift up, 6 octave shift down, 7 MIDI panic, 8 change register control mode, 9 drones on/off, 10 semitone shift up, 11 semitone shift down, 12 begin autocalibration, 13 power down, 14 recenter yaw.
  { { { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 } },
    { { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 } },    //same for instrument 1
    { { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 } } };  //same for instrument 2


//other misc. variables
unsigned long ledTimer = 0;  //for blinking LED
byte blinkNumber = 0;        //the number of remaining blinks when blinking LED to indicate control changes
bool LEDon = 0;              //whether the LED is currently on
bool play = 0;               //turns sound off and on (with the use of a button action) when in bagless mode
bool bellSensor = 1;         //whether the bell sensor is plugged in
byte program = 0;            //current MIDI program change value. This always starts at 0 but can be increased/decreased with assigned buttons.
bool dronesState = 0;        //keeps track of whether we're above or below the pressure threshold for turning drones on.


//variables for reading pressure sensor
volatile unsigned int tempSensorValue = 0;  //for holding the pressure sensor value inside the ISR
int sensorValue = 0;                        // first value read from the pressure sensor
int sensorValue2 = 0;                       // second value read from the pressure sensor, for measuring rate of change in pressure
int prevSensorValue = 0;                    // previous sensor reading, used to tell if the pressure has changed and should be sent.
int twelveBitPressure;                      //Raw 12-bit reading
int smoothed_pressure;                      // smoothed 12-bit pressure for mapping to CC, aftertouch, etc.
int sensorCalibration = 0;                  //the sensor reading at startup, used as a base value
byte offset = 15;                           //called "threshold" in the Configuration Tool-- used along with the multiplier for calculating the transition to the second register
byte multiplier = 15;                       //controls how much more difficult it is to jump to second octave from higher first-octave notes than from lower first-octave notes. Increasing this makes playing with a bag more forgiving but requires more force to reach highest second-octave notes. Can be set according to fingering mode and breath mode (i.e. a higher jump factor may be used for playing with a bag). Array indices 1-3 are for breath mode jump factor, indices 4-6 are for bag mode jump factor.
byte customScalePosition;                   //used to indicate the position of the current note on the custom chart scale (needed for state calculation)
int sensorThreshold[] = { 260, 0 };         //the pressure sensor thresholds for initial note on and shift from register 1 to register 2, before some transformations.
int upperBound = 255;                       //this represents the pressure transition between the first and second registers. It is calculated on the fly as: (sensorThreshold[1] + ((newNote - 60) * multiplier))
byte newState;                              //the note/octave state based on the sensor readings (1=not enough force to sound note, 2=enough force to sound first octave, 3 = enough force to sound second octave)
byte prevState = 1;                         //the previous state, used to monitor change necessary for adding a small delay when a note is turned on from silence and we're sending not on velocity based on pressure.
unsigned long velocityDelayTimer = 0;       //a timer to wait for calculating velocity.
int jumpTime = 15;                          //the amount of time to wait before dropping back down from an octave jump to first octave because of insufficient pressure
int dropTime = 15;                          //the amount of time to wait (ms) before turning a note back on after dropping directly from second octave to note off
byte hysteresis = 15;                       //register hysteresis
byte soundTriggerOffset = 3;                //the number of sensor values above the calibration setting at which a note on will be triggered (first octave)
int learnedPressure = 0;                    //the learned pressure reading, used as a base value
int currentState;                           //these several are used by the new state machine
int rateChangeIdx = 0;
int previousPressure = 0;
bool holdoffActive = false;
int holdoffCounter = 0;
int upperBoundHigh;                  //register boundary for moving up
int upperBoundLow;                   //register boudary for moving down (with hysteresis)
unsigned long fingeringChangeTimer;  //times how long it's been since the most recent fingering change. Used to hold off the register drop feature until we've "settled" in to a fingering pattern

unsigned int inputPressureBounds[4][4] = {
    //for mapping pressure input range to output range. Dimension 1 is CC, velocity, aftertouch, poly. Dimension 2 is minIn, maxIn, scaledMinIn, mappedPressure
    { 100, 800, 0, 0 },
    { 100, 800, 0, 0 },
    { 100, 800, 0, 0 },
    { 100, 800, 0, 0 },
};

byte outputBounds[4][2] = {  // container for ED output pressure range variables (CC, velocity, aftertouch, poly)-- the ED variables will be copied here so they're in a more logical order. This is a fix for variables that were added later.
    { 0, 127 },
    { 0, 127 },
    { 0, 127 },
    { 0, 127 }
};

byte curve[4] = { 0, 0, 0, 0 };  //similar to above-- more logical odering for the pressure curve variable


//variables for reading tonehole sensors
volatile byte lastRead = 0;                                      //the transistor that was last read, so we know which to read the next time around the loop.
unsigned int toneholeCovered[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };  //covered hole tonehole sensor readings for calibration
int toneholeBaseline[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };          //baseline (uncovered) hole tonehole sensor readings
byte toneholePacked[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };  // Used for receiving tone hole readings from the ATmega32U4
int toneholeRead[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };              //tonehole sensor readings after being reassembled from above bytes
unsigned int holeCovered = 0;                                    //whether each hole is covered-- each bit corresponds to a tonehole.
uint8_t tempCovered = 0;                                         //used when masking holeCovered to ignore certain holes depending on the fingering pattern.
bool fingersChanged = 1;                                         //keeps track of when the fingering pattern has changed.
unsigned int prevHoleCovered = 1;                                //so we can track changes.
volatile int tempNewNote = 0;
byte prevNote;
byte newNote = -1;         //the next note to be played, based on the fingering chart (does not include transposition).
byte notePlaying;          //the actual MIDI note being played, which we remember so we can turn it off again.
byte transientFilter = 0;  // small delay for filtering out transient notes


//pitchbend variables
unsigned long pitchBendTimer = 0;                                             //to keep track of the last time we sent a pitchbend message
byte pitchBendOn[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };                           //whether pitchbend is currently turned for for a specific hole
int pitchBend = 8192;                                                         //total current pitchbend value
int prevPitchBend = 8192;                                                     //a record of the previous pitchBend value, so we don't send the same one twice
int iPitchBend[] = { 8192, 8192, 8192, 8192, 8192, 8192, 8192, 8192, 8192 };  //current pitchbend value for each tonehole
int pitchBendPerSemi = 4096;
int prevChanPressure = 0;
int prevCCPressure = 0;
int prevPolyPressure = 0;
unsigned long pressureTimer = 0;                               //to keep track of the last time we sent a pressure message
unsigned long noteOnTimestamp = 0;                             // ms timestamp the note was activated
byte slideHole;                                                //the hole above the current highest uncovered hole. Used for detecting slides between notes.
byte stepsDown = 1;                                            //the number of half steps down from the slideHole to the next lowest note on the scale, used for calculating pitchbend values.
byte vibratoEnable = 0;                                        // if non-zero, send vibrato pitch bend
unsigned int holeLatched = 0b000000000;                        //holes that are disabled for vibrato because they were covered when the note was triggered. They become unlatched (0) when the finger is removed all the way.
unsigned int vibratoHoles = 0b111111111;                       //holes to be used for vibrato, left thumb on left, bell sensor far right.
unsigned int toneholeScale[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };  //a scale for normalizing the range of each sensor, for sliding
unsigned int vibratoScale[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };   //same as above but for vibrato
int expression = 0;                                            //pitchbend up or down from current note based on pressure
bool customEnabled = 0;                                        //Whether the custom vibrato above is currently enabled based on fingering pattern and pitchbend mode.
int adjvibdepth;                                               //vibrato depth scaled to MIDI bend range.


//variables for managing MIDI note output
bool noteon = 0;      //whether a note is currently turned on
bool shiftState = 0;  //whether the octave is shifted (could be combined with octaveShift)
int8_t shift = 0;     //the total amount of shift up or down from the base note 62 (D). This takes into account octave shift and note shift.
byte velocity = 127;  //default MIDI note velocity


//tonehole calibration variables
byte calibration = 0;  //whether we're currently calibrating. 1 is for calibrating all sensors, 2 is for calibrating bell sensor only, 3 is for calibrating all sensors plus baseline calibration (normally only done once, in the "factory").
unsigned long calibrationTimer = 0;


//variables for reading buttons
unsigned long buttonReadTimer = 0;              //for telling when it's time to read the buttons
byte integrator[] = { 0, 0, 0 };                //stores integration of button readings. When this reaches MAXIMUM, a button press is registered. When it reaches 0, a release is registered.
bool pressed[] = { 0, 0, 0 };                   //whether a button is currently presed (this it the output from the integrator)
bool released[] = { 0, 0, 0 };                  //if a button has just been released
bool justPressed[] = { 0, 0, 0 };               //if a button has just been pressed
bool prevOutput[] = { 0, 0, 0 };                //previous state of button, to track state through time
bool longPress[] = { 0, 0, 0 };                 //long button press
unsigned int longPressCounter[] = { 0, 0, 0 };  //for counting how many readings each button has been held, to indicate a long button press
bool noteOnOffToggle[] = { 0, 0, 0 };           //if using a button to toggle a noteOn/noteOff command, keep track of state.
bool longPressUsed[] = { 0, 0, 0 };             //if we used a long button press, we set a flag so we don't use it again unless the button has been released first.
bool buttonUsed = 0;                            //flags any button activity, so we know to handle it.
bool specialPressUsed[] = { 0, 0, 0 };
bool dronesOn = 0;  //used to monitor drones on/off.


//variables for communication with the WARBL Configuration Tool
bool communicationMode = 0;          //whether we are currently communicating with the tool.
byte buttonReceiveMode = 100;        //which row in the button configuration matrix for which we're currently receiving data.
int pressureReceiveMode = 100;       //indicates the variable for which we're currently receiving data
byte fingeringReceiveMode = 0;       // indicates the mode (instrument) for  which a fingering pattern is going to be sent
byte WARBL2settingsReceiveMode = 0;  // indicates the mode (instrument) for  which a WARBL2settings array variable is going to be sent








void setup() {

#if defined(RELEASE)
    Serial.end();  //Turn off CDC. Necessary for release to make a class-compliant device
#endif
    dwt_enable();  //Enable DWT for high-resolution micros() for testing purposes only. Will consume more power(?)

    sd_clock_hfclk_request();  //Enable the high=frequency clock. This is necessary because of a hardware bug that requires the HFCLK for SPI. Instead you can alter SPI.cpp to force using SPIM2, which will use 0.15 mA less current. See issue: https://github.com/adafruit/Adafruit_nRF52_Arduino/issues/773

    NRF_POWER->DCDCEN = 1;  //ENABLE DC/DC CONVERTER, cuts power consumption.

    //disable UART-- saves ~0.1 mA average
    NRF_UART0->TASKS_STOPTX = 1;
    NRF_UART0->TASKS_STOPRX = 1;
    NRF_UART0->ENABLE = 0;

    digitalWrite(battReadEnable, LOW);  //The default with this board is for output pins to be high, so drive them all low before setting them as outputs.
    digitalWrite(chargeEnable, LOW);
    digitalWrite(powerEnable, LOW);
    digitalWrite(redLED, LOW);
    digitalWrite(blueLED, LOW);
    digitalWrite(greenLED, LOW);

    pinMode(battReadEnable, OUTPUT);  //Set various pins as outputs.
    pinMode(chargeEnable, OUTPUT);
    pinMode(powerEnable, OUTPUT);
    pinMode(redLED, OUTPUT);
    pinMode(greenLED, OUTPUT);
    pinMode(blueLED, OUTPUT);

    pinMode(buttons[0], INPUT_PULLUP);  //Set buttons as inputs and enable internal pullup.
    pinMode(buttons[1], INPUT_PULLUP);
    pinMode(buttons[2], INPUT_PULLUP);

    pinMode(STAT, INPUT_PULLUP);  //STAT from charger

    //set up ADC
    const int adcBits = 12;
    analogOversampling(8);  //Takes 55 uS regardless of resolution.
    //analogOversampling(16); //88 uS
    analogReference(AR_VDD4);       //Use VDD for analog reference.
    analogReadResolution(adcBits);  //12 bit

    // setup responsive analog read for adaptive filtering of pressure
    const float snapmult = 0.01f;
    const float actthresh = 2.0f;
    analogPressure.setSnapMultiplier(snapmult);
    analogPressure.setActivityThreshold(actthresh);
    analogPressure.setAnalogResolution(1 << adcBits);

    //USB MIDI stuff
    usb_midi.setStringDescriptor("WARBL USB MIDI");
    MIDI.begin(MIDI_CHANNEL_OMNI);  //Initialize MIDI, and listen to all MIDI channels. This will also call usb_midi's begin().
    MIDI.turnThruOff();
    MIDI.setHandleControlChange(handleControlChange);  //Handle received MIDI CC messages.


    digitalWrite(powerEnable, HIGH);  //Enable the boost converter at startup at least until we have time to check for USB power.
    runTimer = millis();
    battPower = true;


    digitalWrite(greenLED, HIGH);  //Indicate powerup.
    delay(500);
    digitalWrite(greenLED, LOW);


    //BLE MIDI stuff:
    Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
    Bluefruit.begin();
    Bluefruit.Periph.setConnIntervalMS(7.5, 15);           //Request the lowest possible connection interval.
    Bluefruit.setTxPower(8);                               //Supported values: -40dBm, -20dBm, -16dBm, -12dBm, -8dBm, -4dBm, 0dBm, +2dBm, +3dBm, +4dBm, +5dBm, +6dBm, +7dBm and +8dBm.
    Bluefruit.autoConnLed(false);                          //Don't indicate connection (the blue LED may still flash briefly when connecting).
    bledis.setManufacturer("Mowry Stringed Instruments");  //Configure and Start Device Information Service.
    bledis.setModel("WARBL BLE MIDI");
    bledis.begin();
    BLEMIDI.begin(MIDI_CHANNEL_OMNI);  //Initialize MIDI, and listen to all MIDI channels. This will also call blemidi service's begin().
    BLEMIDI.turnThruOff();
    BLEMIDI.setHandleControlChange(handleControlChange);          //Handle received MIDI CC messages.
    Bluefruit.Periph.setConnectCallback(connect_callback);        //Allows us to get connection information
    Bluefruit.Periph.setDisconnectCallback(disconnect_callback);  //Detect disconnect.
    startAdv();                                                   //Set up and start advertising. Comment this out for testing without BLE on.


    //I2C
    Wire.begin();                          //Join i2c bus for EEPROM.
    Wire.setClock(400000);                 //high speed
    EEPROM.setMemorySize(128 * 1024 / 8);  //In bytes. 128kbit = 16kbyte
    EEPROM.setPageSize(64);                //In bytes
    EEPROM.enablePollForWriteComplete();   //Supports I2C polling of write completion. This shortens the amount of time waiting between writes but hammers the I2C bus by polling every 100 uS. disablePollForWriteComplete() will add 5 mS between each write.
    EEPROM.setPageWriteTime(5);            //5 ms max write time
    EEPROM.begin();                        //Start M24128 EEPROM communication using the default address of 0b1010000



    //SPI
    pinMode(2, OUTPUT);     //SS for Atmega
    digitalWrite(2, HIGH);  //Ensure SS stays high for now.
    SPI.begin();

    //IMU
    sox.begin_SPI(12, &SPI, 0, 10000000);      //Start IMU (CS pin is D12) at 10 Mhz.
                                               // sox.setAccelDataRate(LSM6DS_RATE_SHUTDOWN);  //Shut down for now to save power, and we'll turn accel and/or gyro on in loadPrefs() if necessary. IMU uses 0.55 mA if both gyro and accel are on, or 170 uA for just accel.
                                               // sox.setGyroDataRate(LSM6DS_RATE_SHUTDOWN);
    sox.setAccelDataRate(LSM6DS_RATE_208_HZ);  //Turn on the accel.
    sox.setGyroDataRate(LSM6DS_RATE_208_HZ);   //Turn on the gyro.


    //EEPROM.write(44, 255);  //This line can be uncommented to make a version of the software that will resave factory settings every time it is run.

    if (EEPROM.read(44) != 3) {
        saveFactorySettings();  //If we're running the software for the first time, if a factory reset has been requested, copy all settings to EEPROM.
    }

    if (EEPROM.read(37) == 3) {
        loadCalibration();  //If there has been a calibration saved, reload it at startup.
    }


    loadFingering();
    loadSettingsForAllModes();
    mode = defaultMode;  //Set the startup instrument.

    analogRead(A0);  //The first analog readings are sometimes nonsense, so we read a few times and throw them away.
    analogRead(A0);
    sensorCalibration = analogRead(A0) >> 2;
    sensorValue = sensorCalibration;  //An initial reading

    loadPrefs();  //Load the correct user settings based on current instrument.

    powerDownTimer = millis();  //Reset the powerDown timer.

    //watchdog_enable(HARDWARE_WATCHDOG_TIMEOUT_SECS * 1000);  //Enable the watchdog timer, to recover from hangs. If the watchdog triggers while on battery power, the WARBL will power down. On USB power, the NRF will reset.
}








void loop() {


    /////////// Things here happen ~ every 3 ms if on battery power and 2 ms if plugged in.


    // delay() puts the NRF52840 in tickless sleep, saving power. ~ 2.5 mA NRF consumption with delay of 3 ms delay. Total device consumption is 8.7 mA with 3 ms delay, or 10.9 mA with 2 ms delay.


    byte delayCorrection = (millis() - nowtime);  // Include a correction factor to reduce jitter if something else in the loop or the radio interrupt has eaten some time.
                                                  // The main thing that adds jitter to the loop is sending lots of data over BLE, i.e. pitchbend, CC, etc. (though it's typically not noticeable). Being connected to the Config Tool is also a significant source of jitter.
                                                  //With USB MIDI only there is virtually no jitter with a loop period of 2 ms.
    //if (delayCorrection > 3) { Serial.println(delayCorrection); }  //Print the amount of time that other things have consumed.

    byte delayTime = 3;

    if (!battPower) {  //Use a 2 ms sleep instead if we have USB power
        delayTime = 2;
    }

    if (delayCorrection < delayTime) {  //If we haven't used up too much time since the last time through the loop, we can sleep for a bit.
        delayTime = delayTime - delayCorrection;
        delay(delayTime);
    }

    getSensors();  //180 us, 55 of which is reading the pressure sensor.

    nowtime = millis();  //Get the current time for the timers used below and in various functions.

    get_state();  //Get the breath state. 3 us.

    MIDI.read();  // Read any new USBMIDI messages.

    if (Bluefruit.connected()) {        // Don't read if we aren't connected to BLE.
        if (blemidi.notifyEnabled()) {  // ...and ready to receive messages.
            BLEMIDI.read();             //Read new BLEMIDI messages.
        }
    }


    if (blinkNumber > 0) {
        blink();  //Blink the LED if necessary.
    }

    if (calibration > 0) {
        calibrate();  //Calibrate/continue calibrating if the command has been received.
    }


    get_fingers();  //Find which holes are covered. 4 us.


    if (debounceFingerHoles()) {
        fingersChanged = 1;
        tempNewNote = get_note(holeCovered);  //Get the next MIDI note from the fingering pattern if it has changed. 3us.
        sendToConfig(true, false);            //Put the new pattern into a queue to be sent later so that it's not sent during the same connection interval as a new note (to decrease BLE payload size).
        if (pitchBendMode == kPitchBendSlideVibrato || pitchBendMode == kPitchBendLegatoSlideVibrato) {
            findStepsDown();
        }


        if (tempNewNote != -1 && newNote != tempNewNote) {  //If a new note has been triggered,
            if (pitchBendMode != kPitchBendNone) {
                holeLatched = holeCovered;  //remember the pattern that triggered it (it will be used later for vibrato).
                for (byte i = 0; i < 9; i++) {
                    iPitchBend[i] = 0;  // Reset pitchbend.
                    pitchBendOn[i] = 0;
                }
            }
        }


        newNote = tempNewNote;
        tempNewNote = -1;
        fingeringChangeTimer = nowtime;  //Start timing after the fingering pattern has changed.

        get_state();  //Recalculate state if the fingering has changed because state depends on both pressure and fingering.
    }



    if (switches[mode][SEND_VELOCITY]) {  //If we're sending NoteOn velocity based on pressure,
        if (prevState == SILENCE && newState != SILENCE) {
            velocityDelayTimer = nowtime;  //reset the delay timer used for calculating velocity when a note is turned on after silence.
        }
        prevState = newState;
    }

    get_shift();  //Shift the next note up or down based on register, key, and characteristics of the current fingering pattern.



    byte pressureInterval;  //Determine how frequently to send MIDI messages based on pressure.
    if ((nowtime - noteOnTimestamp) < 20) {
        pressureInterval = 2;
    } else pressureInterval = 5;
    if ((pressureInterval < connIntvl + 2) && WARBL2settings[MIDI_DESTINATION] != 0) {  //Use a longer interval if sending BLE.
        pressureInterval = connIntvl + 2;
    }


    if ((nowtime - pressureTimer) >= pressureInterval) {

        pressureTimer = nowtime;
        if (abs(prevSensorValue - smoothed_pressure) > 1) {  //If pressure has changed more than a little, send it.
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

            // if (noteon) { //Only send if there's a note playing? The issue with this is that the output won't necessarily drop all the way to zero when a note turns off.
            sendPressure(false);
            // }
            prevSensorValue = smoothed_pressure;
        }
        static int previousTenBitPressure = sensorValue;

        if (abs(previousTenBitPressure - sensorValue) > 1) {  // Only send pressure to the Config Tool if the 10-bit value has changed, because it's less noisy than 12 bit.
            sendToConfig(false, true);                        // Put the new pressure into a queue to be sent later so that it's not sent during the same connection interval as a new note (to decrease BLE payload size).
            previousTenBitPressure = sensorValue;
        }
    }




    /////////// Things here happen ~ every 5 ms --these things should happen at a regular rate regardless of connection but don't need to happen as fast as possible.

    if ((nowtime - timerE) > 5) {
        timerE = nowtime;
        // timerD = micros(); // testing--micros requres turning on DWT in setup()

        //Read the IMU all the time for now (for development)
        //if (IMUsettings[mode][SEND_ROLL] || IMUsettings[mode][SEND_PITCH] || IMUsettings[mode][SEND_YAW] || IMUsettings[mode][Y_SHAKE_PITCHBEND]) {
        readIMU();  // Takes about 145 us using SensorFusion's Mahony
        // Serial.println(micros() - timerD);
        //}
        checkButtons();

        if (buttonUsed) {
            handleButtons();  //If a button had been used, process the command. We only do this when we need to, so we're not wasting time.
        }

        sendToConfig(false, false);  //Occasionally send the fingering pattern and pressure to the Configuration Tool if they have changed.
    }





    /////////// Things here happen ~ every 9 ms if not connected to BLE and longer if connected at a slow interval. This ensures that we aren't sending pitchbend too much faster than the connection interval.

    if ((nowtime - timerC) >= ((connIntvl > 8 && WARBL2settings[MIDI_DESTINATION] != 0) ? (12) : 9)) {
        calculateAndSendPitchbend();  // 11-200 us depending on whether holes are partially covered.
        printStuff();
        sendIMU();          // ~ 130 us
        shakeForVibrato();  // ~ 200 uS

        timerC = nowtime;
    }





    /////////// Things here happen ~ every 0.75 S

    if ((nowtime - timerF) > 750) {  //This period was chosen for detection of a 1 Hz fault signal from the battery charger STAT pin.
        timerF = nowtime;

        manageBattery(false);  //Check the battery and manage charging. Takes about 300 us because of reading the battery voltage. Could read the voltage a little less frequently.

        //static float CPUtemp = readCPUTemperature(); //If needed for something like calibrating sensors. Can also use IMU temp. The CPU is in the middle of the PCB and the IMU is near the mouthpiece.
    }




    ////////////
    sendNote();  //Send a new MIDI note if there is one. Takes up to 325 us if there is a new note.

    //watchdog_reset();  //Feed the watchdog.
}

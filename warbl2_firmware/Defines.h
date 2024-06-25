

#define RELEASE  // Uncomment for release version (turns off CDC to make the device USB class compliant). Comment out to be able to print to the serial monitor.

#define VERSION 41  // Firmware version (without decimal point)
// #define PROTOTYPE46                 // Hardware -- version 46 uses older pinout without the expansion port or the ability to reprogram the ATmega. Comment this out for all later versions.
#define HARDWARE_REVISION 48        // Not currently used. Can be written to EEPROM 1992 to store revision number.
#define ATMEGA_FIRMWARE_VERSION 10  // Remember which ATmega firmware version we have installed so we kow when to update it.

#define WATCHDOG_TIMEOUT_SECS 5  // The timeout needs to be set longer than any task that might interrupt the loop().

#define DEBUG_TRANSITION_FILTER 0

#define EEPROM_I2C_ADDRESS 0x50

#define DEBOUNCE_TIME 0.02                          // Button debounce time, in seconds
#define SAMPLE_FREQUENCY 200                        // Button sample frequency, in Hz
#define MAXIMUM (DEBOUNCE_TIME * SAMPLE_FREQUENCY)  // The integrator value required to register a button press

#define BATTERY_POWER 0  // No USB connected (running on battery power)
#define DUMB_CHARGER 1   // Powered by a USB charging brick
#define USB_HOST 2       // Connected to a USB host

// MIDI commands
#define NOTE_OFF 0x80          // 127
#define NOTE_ON 0x90           // 144
#define KEY_PRESSURE 0xA0      // 160
#define CONTROL_CHANGE 0xB0    // 176
#define PROGRAM_CHANGE 0xC0    // 192
#define CHANNEL_PRESSURE 0xD0  // 208
#define PITCH_BEND 0xE0        // 224

// Fingering Patterns
#define kModeWhistle 0
#define kModeUilleann 1
#define kModeGHB 2
#define kModeNorthumbrian 3
#define kModeChromatic 4
#define kModeGaita 5
#define kModeNAF 6
#define kModeKaval 7
#define kModeRecorder 8
#define kModeBansuri 9
#define kModeUilleannStandard 10
#define kModeXiao 11
#define kModeSax 12
#define kModeGaitaExtended 13
#define kModeSaxBasic 14
#define kModeEVI 15
#define kModeShakuhachi 16
#define kModeSackpipaMajor 17
#define kModeSackpipaMinor 18
#define kModeCustom 19  // From original WARBL -- Currently unused
#define kModeBombarde 20
#define kModeBaroqueFlute 21
#define kModeMedievalPipes 22
#define kModeBansuriWARBL 23  // Currently unused
#define kWARBL2Custom1 67
#define kWARBL2Custom2 68
#define kWARBL2Custom3 69
#define kWARBL2Custom4 70
#define kModeNModes 28

// Pitch bend modes
#define kPitchBendSlideVibrato 0
#define kPitchBendVibrato 1
#define kPitchBendNone 2
#define kPitchBendLegatoSlideVibrato 3
#define kPitchBendNModes 4

// Register control modes
#define kPressureSingle 0
#define kPressureBreath 1
#define kPressureThumb 2
#define kPressureBell 3
#define kPressureNModes 4

// Drones control mode
#define kNoDroneControl 0
#define kSecretDroneControl 1
#define kBaglessDroneControl 2
#define kPressureDroneControl 3
#define kDroneNModes 4

// Used in register state machine
#define SILENCE 1
#define BOTTOM_REGISTER 2
#define TOP_REGISTER 3
#define SILENCE_HYSTERESIS 1
#define JUMP 0
#define DROP 1

// Variables in the switches array (settings for the swiches in the slide/vibrato and register control panels)
#define VENTED 0
#define BAGLESS 1
#define SECRET 2
#define INVERT 3
#define CUSTOM 4
#define SEND_VELOCITY 5
#define SEND_AFTERTOUCH 6  // Second bit of this one is used for poly.
#define FORCE_MAX_VELOCITY 7
#define IMMEDIATE_PB 8
#define LEGATO 9
#define OVERRIDE 10
#define THUMB_AND_OVERBLOW 11
#define R4_FLATTEN 12
#define kSWITCHESnVariables 13

// Variables in the ED array (settings for expression and drones panels, and misc. other Config Tool settings)
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
#define SLIDE_LIMIT_MAX 38
#define CUSTOM_FINGERING_2 39  // None of these "custom" variables these are used by WARBL2. Can be repurposed.
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

// Button combinations/gestures
#define CLICK_1 0
#define CLICK_2 1
#define CLICK_3 2
#define HOLD_2_CLICK_1 3
#define HOLD_2_CLICK_3 4
#define LONG_PRESS_1 5
#define LONG_PRESS_2 6
#define LONG_PRESS_3 7
#define SIP 8
#define SHAKE 9
#define kGESTURESnVariables 10

// Button/gesture actions
#define NO_ACTION 0
#define SEND_MIDI_MESSAGE 1
#define CHANGE_PITCHBEND_MODE 2
#define CHANGE_INSTRUMENT 3
#define PLAY_STOP 4
#define OCTAVE_SHIFT_UP 5
#define OCTAVE_SHIFT_DOWN 6
#define MIDI_PANIC 7
#define CHANGE_REGISTER_CONTROL_MODE 8
#define DRONES_ON_OFF 9
#define SEMI_SHIFT_UP 10
#define SEMI_SHIFT_DOWN 11
#define AUTOCALIBRATE 12
#define POWER_DOWN 13
#define RECENTER_YAW 14
#define SHOW_BATTERY_LEVEL 15
#define kACTIONSnVariables 16

// Variables in the WARBL2settings array (independent of mode)
#define MIDI_DESTINATION 0  // 0 means send MIDI to USB only, 1 means send to BLE only, 2 means send to both
#define CHARGE_FROM_HOST 1  // Charge from USB host in addition to "dumb" charging brick.
#define POWERDOWN_TIME 2
#define kWARBL2SETTINGSnVariables 3

// Variables in the IMUsettings array
#define SEND_ROLL 0       // On/off
#define SEND_PITCH 1      // On/Off
#define SEND_YAW 2        // On/Off
#define CENTER_ROLL 3     // On/off
#define CENTER_YAW 4      // On/Off
#define ROLL_INPUT_MIN 5  // 0-36
#define ROLL_INPUT_MAX 6
#define ROLL_OUTPUT_MIN 7  // 0-127
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
#define AUTOCENTER_YAW 23           // On/Off
#define Y_SHAKE_PITCHBEND 24        // On/Off
#define AUTOCENTER_YAW_INTERVAL 25  // 0-20 (represents 0-5s pause interval for yaw recentering)
#define PITCH_REGISTER 26           // On/Off
#define Y_PITCHBEND_DEPTH 27        // 0-100
#define PITCH_REGISTER_INPUT_MIN 28
#define PITCH_REGISTER_INPUT_MAX 29
#define PITCH_REGISTER_NUMBER 30
#define Y_PITCHBEND_MODE 31  // 0 is Up/Down, 1 is Down/Up, 2 is up only, 3 is down only
#define STICKS_MODE 32 // On/off
#define kIMUnVariables 33

#define Y_PITCHBEND_MODE_UPDOWN 0
#define Y_PITCHBEND_MODE_DOWNUP 1
#define Y_PITCHBEND_MODE_UPONLY 2
#define Y_PITCHBEND_MODE_DOWNONLY 3

#define RED_LED 0
#define GREEN_LED 1
#define BLUE_LED 2


/* EEPROM Addresses */
#define EEPROM_BASELINE_CALIB_START        1 // values 0-255 baseline sensor calibrations - 9 bytes 1-9
/* 10-17 unused */
#define EEPROM_SENSOR_CALIB_START         18 // sensor calibrations - 18 bytes 18-35, two byte per sensor high-low
#define EEPROM_XGYRO_CALIB_SAVED          36 // value 3 indicates a saved gyro calibration
#define EEPROM_SENSOR_CALIB_SAVED         37 // value 3 indicates a saved sensor calibration
/* 38-39 unused */
#define EEPROM_FINGERING_PATTERN_START    40 // values 0-max fingering pattern - 3 bytes 40-42
/* 43 unused */
#define EEPROM_SETTINGS_SAVED             44 // value 3 indicates settings have been saved - this will be reset to indicate that a factory reset needs to be done.
/* 45-47 unused */
#define EEPROM_DEFAULT_MODE               48 // default mode (instrument)
/* 49 unused */
#define EEPROM_SENS_DISTANCE_START        50 // values 0-255	finger-sensing distance - 3 bytes 50-52
#define EEPROM_NOTE_SHIFT_SEL_START       53 // values 0-1 note shift selector - 3 bytes 53-55
#define EEPROM_SWITCHES_START             56 // values 0-1 switches variables - 43 bytes 56-98
/* 99 unused		
 * 100-247	unused (previously button prefs)
 * 248-249	unused
*/
#define EEPROM_MOMENTARY_MODE_START      250 // values 0-1 momentary mode 0  - 3 bytes 250-252
                                             // values 0-1 momentary mode 1  - 3 bytes 253-255
                                             // values 0-1 momentary mode 2  - 3 bytes 256-258
/* 259 unused */
#define EEPROM_PRESSURE_SETTINGS_START   260 // 261-272		pressure settings for mode 0
                                             // 281-292		pressure settings for mode 1
                                             // 301-312		pressure settings for mode 2
#define EEPROM_LEARNED_PRESSURE_START    273 // values 0-255 low byte of learned pressure for mode 0 (274 high byte)
                                              // 275 low byte of learned pressure for mode 1 (276 high byte)
                                              // 277 low byte of learned pressure for mode 2 (278 high byte)
/* 279-280 unused 
 * 293-300 unused
*/
#define EEPROM_PB_MODE_START             313 // pitch bend mode - 3 bytes 313-315
#define EEPROM_BREATH_MODE_START         316 // breath mode - 3 bytes 316-318
#define EEPROM_MIDI_BEND_RANGE_START     319 // MIDI bend range - 3 bytes 319-321
#define EEPROM_MIDI_CHANNEL_START        322 // MIDI channel - 3 bytes 322-324
/* 325-332 unused */
#define EEPROM_VIBRATO_HOLES_START       333 // values 0-1 low byte of enabled vibrato holes for mode 0 (334 high byte)
                                             // 335 low byte of enabled vibrato holes for mode 1 (336 high byte)
                                             // 337 low byte of enabled vibrato holes for mode 2 (338 high byte)
#define EEPROM_VIBRATO_DEPTH_START       339 // values 0-255 low byte of evibrato depth  for mode 0 (340 high byte)
                                             // 341 low byte of vibrato depth  for mode 1 (342 high byte)
                                             // 343 low byte of vibrato depth  for mode 2 (344 high byte)
#define EEPROM_USE_LEARNED_PRESS_START   345 //values 0-1	use learned calibration - 3 bytes 345-347 
/* 348-350 unused */
#define EEPROM_ED_VARS_START             351 // 351-497	expression and drones (ED) variables
/* 498-599 unused, room for extending above array or other variables */
#define EEPROM_WARBL2_SETTINGS_START     600 // 600-602 WARBL2settings array
/* 603-625 unused, room for extending above array or other variables */
#define EEPROM_IMU_SETTINGS_START        625 // 625-723 WARBL2 IMUsettings array
/*724-999 unused, room for extending above array or other variables */
#define EEPROM_BUTTON_PREFS_START       1000 // 1000-1247 button prefs
/* 1248-1974 unused */
/* Note: 1975-1986 currently doesn't get saved to factory settings but does get restored from factory settings (calibrations are saved to factory settings manually) */
#define EEPROM_XGYRO_CALIB_START        1975 // XGyro calibration	(float) 4 bytes
#define EEPROM_YGYRO_CALIB_START        1979 // YGyro calibration	(float) 4 bytes
#define EEPROM_ZGYRO_CALIB_START        1983 // ZGyro calibration	(float) 4 bytes
/* Note: 1987-2000 doesn't currently get saved to or restored from factory settings. */
#define EEPROM_RESERVED_TESTING         1987 // Reserved for testing purposes
#define EEPROM_LOW_CHARGE               1988 // indication that there has been a charge termination (value of 3) or shutdown because of low battery (value of 1)
#define EEPROM_EST_RUNTIME_START        1989 // operating time on a full charge (minutes, high byte) - (1990 low byte)
#define EEPROM_FIRMWARE_VERSION         1991 // firmware version
#define EEPROM_HARDWARE_VERSION         1992 // hardware version
#define EEPROM_RUNTIME_START            1993 // minutes operating time since last full charge (high byte) - (1994 low byte)
#define EEPROM_ATMEAGA_FIRMWARE_VERSION 1995 // ATmega firmware version
/* 1996-1999 unused, room for more settings */
#define EEPROM_FACTORY_SETTINGS_START   2000 // 2001-3999 locations of factory settings (duplicates of 1-1999, for restoring settings)	
#define EEPROM_CUSTOM_FINGERING_START   4000 // 4000-4255		Custom fingering chart 1
                                             // 4256-4511		Custom fingering chart 2
                                             // 4512-4767		Custom fingering chart 3
                                             // 4768-5023		Custom fingering chart 4
/* 5024-16383 other ~11 KB unused */

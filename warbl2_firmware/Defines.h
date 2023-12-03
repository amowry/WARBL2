

// #define RELEASE //Uncomment for release version (turns off CDC)

#define VERSION 41                  // Firmware version (without decimal point)
#define HARDWARE_REVISION 46        // Hardware
#define ATMEGA_FIRMWARE_VERSION 10  // Remember which ATMEAGA firmware version we have installed so we kow when to update it.

#define WATCHDOG_TIMEOUT_SECS 10  // The timeout needs to be set longer than any task that might interrupt the loop().

#define EEPROM_I2C_ADDRESS 0x50

#define DEBOUNCE_TIME 0.02                          // Button debounce time, in seconds
#define SAMPLE_FREQUENCY 200                        // Button sample frequency, in Hz
#define MAXIMUM (DEBOUNCE_TIME * SAMPLE_FREQUENCY)  // The integrator value required to register a button press

#define BATTERY_POWER 0  // No USB connected (running on battery power)
#define DUMB_CHARGER 1   // Powered by a dumb USB charger
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
#define kModeBansuriWARBL 9  // Only used for a custom regulators implementation, or BansuriWARBL
#define kModeUilleannStandard 10
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
#define kModeBarbaro 23  // Hasn't been added yet.
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
#define SEND_AFTERTOUCH 6  //second bit of this one is used for poly
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
//#define CUSTOM_FINGERING_1 38  //None of these "custom" variables these are used by WARBL2.
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
#define kACTIONSnVariables 15

// Variables in the WARBL2settings array (independent of mode)
#define MIDI_DESTINATION 0  //0 means send MIDI to USB only, 1 means send to BLE only, 2 means send to both
#define CHARGE_FROM_HOST 1  //Charge from USB host in addition to "dumb" charging brick.
#define POWERDOWN_TIME 2
#define kWARBL2SETTINGSnVariables 3

// Variables in the IMUsettings array
#define SEND_ROLL 0       //On/off
#define SEND_PITCH 1      //On/Off
#define SEND_YAW 2        //On/Off
#define CENTER_ROLL 3     //On/off
#define CENTER_YAW 4      //On/Off
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
#define Y_SHAKE_PITCHBEND 24        //On/Off
#define AUTOCENTER_YAW_INTERVAL 25  //0-20 (represents 0-5s pause interval for yaw recentering)
#define PITCH_REGISTER 26           //On/Off
#define Y_PITCHBEND_DEPTH 27        //0-100
#define PITCH_REGISTER_INPUT_MIN 28
#define PITCH_REGISTER_INPUT_MAX 29
#define PITCH_REGISTER_NUMBER 30
#define Y_PITCHBEND_MODE 31  // 0 is Up/Down, 1 is Down/Up, 2 is up only, 3 is down only
#define kIMUnVariables 32

#define Y_PITCHBEND_MODE_UPDOWN 0
#define Y_PITCHBEND_MODE_DOWNUP 1
#define Y_PITCHBEND_MODE_UPONLY 2
#define Y_PITCHBEND_MODE_DOWNONLY 3

#define RED_LED 0
#define GREEN_LED 1
#define BLUE_LED 2

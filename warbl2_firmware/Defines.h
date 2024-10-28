

#define RELEASE  // Uncomment for release version (turns off CDC to make the device USB class compliant). Comment out to be able to print to the serial monitor.

#define VERSION 44  // Firmware version (without decimal point)
//#define PROTOTYPE46                 // Hardware -- version 46 uses older pinout without the expansion port or the ability to reprogram the ATmega. Comment this out for all later versions.
#define HARDWARE_REVISION 49        // Not currently used. Can be written to EEPROM 1992 to store revision number.
#define ATMEGA_FIRMWARE_VERSION 10  // Remember which ATmega firmware version we have installed so we kow when to update it.

#define WATCHDOG_TIMEOUT_SECS 10  // The timeout needs to be set longer than any task that might interrupt the loop().

#define DEBUG_TRANSITION_FILTER 0
#define DEBUG_CONFIG_TOOL 0

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
#define BUTTON_DOUBLE_CLICK 13
#define kSWITCHESnVariables 14

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
#define EXPRESSION_MIN_HIGH 39
#define EXPRESSION_MAX_LOW 40
#define EXPRESSION_OUT_LOW_CENTS 41     // stored signed as 0 cents = 64
#define EXPRESSION_OUT_HIGH_CENTS 42    // stored signed as 0 cents = 64
#define EXPRESSION_FIXED_CENTER_PRESSURE 43
#define EXPRESSION_OUT_CLAMP 44   // boolean if outbend should be clamped
#define EXPRESSION_CURVE_LOW 45      // 0 -> 127, where 64 is linear, 0 is most log, 127 is most exponential
#define EXPRESSION_CURVE_HIGH 46      // 0 -> 127, where 64 is linear, 0 is most log, 127 is most exponential
#define AFTERTOUCH_MPEPLUS  47 
#define CUSTOM_FINGERING_11 48     // None of these "custom" variables these are used by WARBL2. Can be repurposed.
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
#define DOUBLE_CLICK_WAIT_INTERVAL 200  //Check button every 5 ticks, so this hould be about 1 second
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
#define MIDI_DESTINATION 0  // 0 means send MIDI to USB only, 1 means send to BLE only, 2 means send to both, see defines below
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
#define Y_PITCHBEND_MODE 31  // 0 is Up/Down, 1 is Down/Up, 2 is up only, 3 is down only, 4 is inverted up only, 5, is inverted down only
#define STICKS_MODE 32       // On/off
#define Y_SHAKE_MOD_CC 33              // On/Off
#define Y_SHAKE_MOD_CHPRESS 34         // On/Off
#define Y_SHAKE_MOD_KEYPRESS 35        // On/Off
#define Y_SHAKE_MOD_CC_DEPTH 36        // 0-100
#define Y_SHAKE_MOD_CHPRESS_DEPTH 37   // 0-100
#define Y_SHAKE_MOD_KEYPRESS_DEPTH 38  // 0-100
#define Y_SHAKE_MOD_CC_MODE 39        // 0 is Up/Down, 1 is Down/Up, 2 is up only, 3 is down only, 4 is inverted up only, 5, is inverted down only 
#define Y_SHAKE_MOD_CHPRESS_MODE 40   // 0 is Up/Down, 1 is Down/Up, 2 is up only, 3 is down only, 4 is inverted up only, 5, is inverted down only
#define Y_SHAKE_MOD_KEYPRESS_MODE 41  // 0 is Up/Down, 1 is Down/Up, 2 is up only, 3 is down only, 4 is inverted up only, 5, is inverted down only
#define kIMUnVariables 42

#define Y_PITCHBEND_MODE_UPDOWN 0
#define Y_PITCHBEND_MODE_DOWNUP 1
#define Y_PITCHBEND_MODE_UPONLY 2
#define Y_PITCHBEND_MODE_DOWNONLY 3
#define Y_PITCHBEND_MODE_UPONLY_INV 4
#define Y_PITCHBEND_MODE_DOWNONLY_INV 5

#define RED_LED 0
#define GREEN_LED 1
#define BLUE_LED 2


/* MIDI Config Tool Constants */
//General constants
#define MIDI_DEFAULT_MAIN_CHANNEL 1  // Default MIDI channel to send notes on
#define MIDI_CONFIG_TOOL_CHANNEL 7   // Config Tool MIDI channel
#define MIDI_DEFAULT_VELOCITY 127    //

//WARBL2 MIDI_DESTINATION
#define MIDI_DESTINATION_USB_ONLY 0
#define MIDI_DESTINATION_BLE_ONLY 1
#define MIDI_DESTINATION_BOTH 2

//Source of an incoming CC Message
#define MIDI_SOURCE_NONE 0  // No source set
#define MIDI_SOURCE_USB 1   // From USB
#define MIDI_SOURCE_BLE 2   // From BLE

/* To be kept in sync with constants.js in Config Tool */

//MIDI Human readable constants: see below

/* Numerical constants
 * Some cc values are reserved from Config Tool or from WARBL
 * Bidirectional Communication: same commands both ways with same cc/value 
*/
#define MIDI_CC_102 102        // from WARBL & from Config Tool.  Various values as follows:
#define MIDI_CC_102_VALUE_0 0  // unused
// sensor calibration messages
#define MIDI_CC_102_VALUE_1 1    // from Config Tool. bell sensor down
#define MIDI_CC_102_VALUE_2 2    // from Config Tool. bell sensor up
#define MIDI_CC_102_VALUE_3 3    // from Config Tool. R4 down
#define MIDI_CC_102_VALUE_4 4    // from Config Tool. R4 up
#define MIDI_CC_102_VALUE_5 5    // from Config Tool. R3 down
#define MIDI_CC_102_VALUE_6 6    // from Config Tool. R3 up
#define MIDI_CC_102_VALUE_7 7    // from Config Tool. R2 down
#define MIDI_CC_102_VALUE_8 8    // from Config Tool. R2 up
#define MIDI_CC_102_VALUE_9 9    // from Config Tool. R1 down
#define MIDI_CC_102_VALUE_10 10  // from Config Tool. R1 up
#define MIDI_CC_102_VALUE_11 11  // from Config Tool. L3 down
#define MIDI_CC_102_VALUE_12 12  // from Config Tool. L3 up
#define MIDI_CC_102_VALUE_13 13  // from Config Tool. L2 down
#define MIDI_CC_102_VALUE_14 14  // from Config Tool. L2 up
#define MIDI_CC_102_VALUE_15 15  // from Config Tool. L1 down
#define MIDI_CC_102_VALUE_16 16  // from Config Tool. L1 up
#define MIDI_CC_102_VALUE_17 17  // from Config Tool. Lthumb down
#define MIDI_CC_102_VALUE_18 18  // from Config Tool. Lthumb up
#define MIDI_CC_102_VALUE_19 19  // from Config Tool. Save optical sensor calibration
#define MIDI_CC_102_VALUE_20 20  // from WARBL. bell sensor max value reached
#define MIDI_CC_102_VALUE_21 21  // from WARBL. R4 max value reached

#define MIDI_CC_102_VALUE_22 22  // from WARBL. R3 max value reached
#define MIDI_CC_102_VALUE_23 23  // from WARBL. R2 max value reached
#define MIDI_CC_102_VALUE_24 24  // from WARBL. R1 max value reached
#define MIDI_CC_102_VALUE_25 25  // from WARBL. L3 max value reached
#define MIDI_CC_102_VALUE_26 26  // from WARBL. L2 max value reached
#define MIDI_CC_102_VALUE_27 27  // from WARBL. L1 max value reached
#define MIDI_CC_102_VALUE_28 28  // from WARBL. Lthumb max value reached
//
/* 29 unused*/

//Send fingering pattern selections:
#define MIDI_CC_102_VALUE_30 30  // Bidirectional. Indicates that the next command will be the fingering pattern for instrument 1
#define MIDI_CC_102_VALUE_31 31  // Bidirectional. Indicates that the next command will be the fingering pattern for instrument 2
#define MIDI_CC_102_VALUE_32 32  // Bidirectional. Indicates that the next command will be the fingering pattern for instrument 3
#define MIDI_CC_102_VALUE_33 33  // Bidirectional. First fingering pattern is tin whistle
#define MIDI_CC_102_VALUE_34 34  // Bidirectional. "" uilleann
#define MIDI_CC_102_VALUE_35 35  // Bidirectional. “” GHB
#define MIDI_CC_102_VALUE_36 36  // Bidirectional. “” Northumbrian
#define MIDI_CC_102_VALUE_37 37  // Bidirectional. ""tin whistle/flute chromatic
#define MIDI_CC_102_VALUE_38 38  // Bidirectional. ""Gaita
#define MIDI_CC_102_VALUE_39 39  // Bidirectional. NAF
#define MIDI_CC_102_VALUE_40 40  // Bidirectional. Kaval
#define MIDI_CC_102_VALUE_41 41  // Bidirectional. recorder
#define MIDI_CC_102_VALUE_42 42  // Bidirectional. Bansuri
#define MIDI_CC_102_VALUE_43 43  // Bidirectional. Uilleann standard
#define MIDI_CC_102_VALUE_44 44  // Bidirectional. Xiao
#define MIDI_CC_102_VALUE_45 45  // Bidirectional. Sax extended
#define MIDI_CC_102_VALUE_46 46  // Bidirectional. Gaita extended
#define MIDI_CC_102_VALUE_47 47  // Bidirectional. Saxbasic
#define MIDI_CC_102_VALUE_48 48  // Bidirectional. EVI
#define MIDI_CC_102_VALUE_49 49  // Bidirectional. Shakuhachi
#define MIDI_CC_102_VALUE_50 50  // Bidirectional. Sackpipa major
#define MIDI_CC_102_VALUE_51 51  // Bidirectional. Sackpipa minor
#define MIDI_CC_102_VALUE_52 52  // Bidirectional. Custom (original WARBL only)
#define MIDI_CC_102_VALUE_53 53  // Bidirectional. Bombarde
#define MIDI_CC_102_VALUE_54 54  // Bidirectional. Baroque flute
#define MIDI_CC_102_VALUE_55 55  // Bidirectional. Medieval bagpipes
                                 /* 56-59 unused */

#define MIDI_CC_102_VALUE_60 60  // Bidirectional. Current instrument (mode variable) is 0
#define MIDI_CC_102_VALUE_61 61  // Bidirectional. Current instrument is 1
#define MIDI_CC_102_VALUE_62 62  // Bidirectional. Current instrument is 2
/* 63-69 unused */
#define MIDI_CC_102_VALUE_70 70  // Bidirectional. Settings for current instrument: Pitchbend mode 0
#define MIDI_CC_102_VALUE_71 71  // Bidirectional. Settings for current instrument: Pitchbend mode 1
#define MIDI_CC_102_VALUE_72 72  // Bidirectional. Settings for current instrument: Pitchbend mode 2
#define MIDI_CC_102_VALUE_73 73  // Bidirectional. Settings for current instrument: Pitchbend mode 3
/* 74-79 unused */
#define MIDI_CC_102_VALUE_80 80  // Bidirectional. Settings for current instrument: Breath mode 0
#define MIDI_CC_102_VALUE_81 81  // Bidirectional. Settings for current instrument: Breath mode 1
#define MIDI_CC_102_VALUE_82 82  // Bidirectional. Settings for current instrument: Breath mode 2
#define MIDI_CC_102_VALUE_83 83  // Bidirectional. Settings for current instrument: Breath mode 3
#define MIDI_CC_102_VALUE_84 84  // Bidirectional. Settings for current instrument: Breath mode4
#define MIDI_CC_102_VALUE_85 85  // Bidirectional. Default instrument is 0 - (if Config Tool sends 85 to WARBL, WARBL sets current instrument as default)
#define MIDI_CC_102_VALUE_86 86  // Bidirectional. Default instrument is 1
#define MIDI_CC_102_VALUE_87 87  // Bidirectional. Default instrument is 2
/* 88-89 unused */

/* Populate button configuration for current instrument:	
	 * First indicate button combination to be populated:
    */
#define MIDI_CC_102_VALUE_90 90  // Bidirectional. Sending data for click 1 (dropdown row 0)
#define MIDI_CC_102_VALUE_91 91  // Bidirectional. click 2
#define MIDI_CC_102_VALUE_92 92  // Bidirectional. click 3
#define MIDI_CC_102_VALUE_93 93  // Bidirectional. hold 2, click 1
#define MIDI_CC_102_VALUE_94 94  // Bidirectional. hold 2, click 3
#define MIDI_CC_102_VALUE_95 95  // Bidirectional. longpress 1
#define MIDI_CC_102_VALUE_96 96  // Bidirectional. longpress 2
#define MIDI_CC_102_VALUE_97 97  // Bidirectional. longpress 3
#define MIDI_CC_102_VALUE_98 98  // unused
#define MIDI_CC_102_VALUE_99 99  // unused (previously disconnect command)
/* Follow with data to populate indicated button combination:
     * CC 106	values 100-127 (see below) 
     * (was previously CC 102 100-111 but ran out of room here)
     */

#define MIDI_CC_102_VALUE_100 100  // Bidirectional. WARBL2 custom fingering chart 1
#define MIDI_CC_102_VALUE_101 101  // Bidirectional. WARBL2 custom fingering chart 2
#define MIDI_CC_102_VALUE_102 102  // Bidirectional. WARBL2 custom fingering chart 3
#define MIDI_CC_102_VALUE_103 103  // Bidirectional. WARBL2 custom fingering chart 4
#define MIDI_CC_102_VALUE_104 104  //from Config Tool. exit communication mode (previously 102 99)
/* 105-111	unused -- can be used for WARBL2 */
#define MIDI_CC_102_VALUE_112 112  // Bidirectional. send midi note on/note off
#define MIDI_CC_102_VALUE_113 113  // Bidirectional. Send midi CC
#define MIDI_CC_102_VALUE_114 114  // Bidirectional. Send MIDI PC
#define MIDI_CC_102_VALUE_115 115  // Bidirectional. Increase PC
#define MIDI_CC_102_VALUE_116 116  // Bidirectional. Decrease PC
#define MIDI_CC_102_VALUE_117 117  // Bidirectional. momentary off
#define MIDI_CC_102_VALUE_118 118  // Bidirectional. momentary on
/* 119 unused */
#define MIDI_CC_102_VALUE_120 120  // from WARBL. Bell sensor disconnected (no longer used by WARBL2)
#define MIDI_CC_102_VALUE_121 121  // from WARBL. Bell sensor connected (no longer used by WARBL2)
/* 122 unused */
#define MIDI_CC_102_VALUE_123 123  // from Config Tool. save as defaults for current mode
#define MIDI_CC_102_VALUE_124 124  // from Config Tool. save as defaults for all instruments
#define MIDI_CC_102_VALUE_125 125  // from Config Tool. restore factory settings
#define MIDI_CC_102_VALUE_126 126  // from Config Tool. enter communication mode \
                                   // WARBL enters communication mode (until it is shut off or user clicks "Disconnect") and responds by sending settings for currently selected instrument.
#define MIDI_CC_102_VALUE_127 127  // from Config Tool. begin autocalibration


#define MIDI_CC_103 103  // from WARBL & from Config Tool.  Values 0-127	- Settings for current instrument: finger-sensing distance

#define MIDI_CC_104 104  // from WARBL & from Config Tool.  Various values as follows:
/* 0 unused */

/* Bag/Breath advanced settings (pressureSelector array) */
#define MIDI_CC_104_VALUE_1 1    // Bidirectional. Settings for current instrument: indicates "bag threshold" is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_2 2    // Bidirectional. Settings for current instrument: indicates "bag multiplier" is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_3 3    // Bidirectional. Settings for current instrument: indicates "bag hysteresis" is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_4 4    // Bidirectional. unused.
#define MIDI_CC_104_VALUE_5 5    // Bidirectional. Settings for current instrument: indicates "bag jump time" is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_6 6    // Bidirectional. Settings for current instrument: indicates "bag drop time" is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_7 7    // Bidirectional. Settings for current instrument: indicates "breath threshold" is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_8 8    // Bidirectional. Settings for current instrument: indicates "breath multiplier" is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_9 9    // Bidirectional. Settings for current instrument: indicates "breath hysteresis" is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_10 10  // Bidirectional. Settings for current instrument: indicates "breath transient filter" is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_11 11  // Bidirectional. Settings for current instrument: indicates "breath jump time" is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_12 12  // Bidirectional. Settings for current instrument: indicates "breath  drop time" is about to be sent with CC 105.
//
/* Expression or drones variable (ED array) - see defines above */
#define MIDI_CC_104_VALUE_13 13  // Bidirectional. Settings for current instrument: indicates ED[0] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_14 14  // Bidirectional. Settings for current instrument: indicates ED[1] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_15 15  // Bidirectional. Settings for current instrument: indicates ED[2] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_16 16  // Bidirectional. Settings for current instrument: indicates ED[3] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_17 17  // Bidirectional. Settings for current instrument: indicates ED[4] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_18 18  // Bidirectional. Settings for current instrument: indicates ED[5] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_19 19  // Bidirectional. Settings for current instrument: indicates ED[6] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_20 20  // Bidirectional. Settings for current instrument: indicates ED[7] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_21 21  // Bidirectional. Settings for current instrument: indicates ED[8] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_22 22  // Bidirectional. Settings for current instrument: indicates ED[9] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_23 23  // Bidirectional. Settings for current instrument: indicates ED[10] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_24 24  // Bidirectional. Settings for current instrument: indicates ED[11] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_25 25  // Bidirectional. Settings for current instrument: indicates ED[12]] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_26 26  // Bidirectional. Settings for current instrument: indicates ED[13] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_27 27  // Bidirectional. Settings for current instrument: indicates ED[14] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_28 28  // Bidirectional. Settings for current instrument: indicates ED[15] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_29 29  // Bidirectional. Settings for current instrument: indicates ED[16] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_30 30  // Bidirectional. Settings for current instrument: indicates ED[17] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_31 31  // Bidirectional. Settings for current instrument: indicates ED[18] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_32 32  // Bidirectional. Settings for current instrument: indicates ED[19] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_33 33  // Bidirectional. Settings for current instrument: indicates ED[20] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_34 34  // Bidirectional. Settings for current instrument: indicates that lsb of learned note trigger pressure is about to be sent on CC 105
#define MIDI_CC_104_VALUE_35 35  // Bidirectional. Settings for current instrument: indicates that msb of learned note trigger pressure is about to be sent on CC 105
/* 36-39 unused */

/* Switches array - see defines above */
#define MIDI_CC_104_VALUE_40 40  //  Bidirectional. Settings for current instrument: indicates that switches[0] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_41 41  //  Bidirectional. Settings for current instrument: indicates that switches[1] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_42 42  //  Bidirectional. Settings for current instrument: indicates that switches[2] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_43 43  //  Bidirectional. Settings for current instrument: indicates that switches[3] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_44 44  //  Bidirectional. Settings for current instrument: indicates that switches[4] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_45 45  //  Bidirectional. Settings for current instrument: indicates that switches[5] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_46 46  //  Bidirectional. Settings for current instrument: indicates that switches[6] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_47 47  //  Bidirectional. Settings for current instrument: indicates that switches[7] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_48 48  //  Bidirectional. Settings for current instrument: indicates that switches[8] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_49 49  //  Bidirectional. Settings for current instrument: indicates that switches[9] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_50 50  //  Bidirectional. Settings for current instrument: indicates that switches[10] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_51 51  //  Bidirectional. Settings for current instrument: indicates that switches[11] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_52 52  //  Bidirectional. Settings for current instrument: indicates that switches[12] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_53 53  //  Bidirectional. Settings for current instrument: indicates that switches[13] is about to be sent with CC 105.
//
/* 54-60 unused */
#define MIDI_CC_104_VALUE_61 61  //  Bidirectional. Settings for current instrument: MIDI bend range is about to be sent on CC 105
#define MIDI_CC_104_VALUE_62 62  //  Bidirectional. Settings for current instrument: MIDI channel is about to be sent on CC 105
/* 62-69 unused */

/* more expression or drones variables (ED array) 
     * can be extended to 127  for WARBL2, e.g. IMU functionality. If more variables are needed CC109 can also be used to indicate additional "pressureReceiveMode" options
     */
#define MIDI_CC_104_VALUE_70 70  // Bidirectional. Settings for current instrument: indicates ED[21] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_71 71  // Bidirectional. Settings for current instrument: indicates ED[22] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_72 72  // Bidirectional. Settings for current instrument: indicates ED[23] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_73 73  // Bidirectional. Settings for current instrument: indicates ED[24] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_74 74  // Bidirectional. Settings for current instrument: indicates ED[25] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_75 75  // Bidirectional. Settings for current instrument: indicates ED[26] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_76 76  // Bidirectional. Settings for current instrument: indicates ED[27] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_77 77  // Bidirectional. Settings for current instrument: indicates ED[28] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_78 78  // Bidirectional. Settings for current instrument: indicates ED[29] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_79 79  // Bidirectional. Settings for current instrument: indicates ED[30] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_80 80  // Bidirectional. Settings for current instrument: indicates ED[31] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_81 81  // Bidirectional. Settings for current instrument: indicates ED[32] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_82 82  // Bidirectional. Settings for current instrument: indicates ED[33] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_83 83  // Bidirectional. Settings for current instrument: indicates ED[34] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_84 84  // Bidirectional. Settings for current instrument: indicates ED[35] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_85 85  // Bidirectional. Settings for current instrument: indicates ED[36] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_86 86  // Bidirectional. Settings for current instrument: indicates ED[37] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_87 87  // Bidirectional. Settings for current instrument: indicates ED[38] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_88 88  // Bidirectional. Settings for current instrument: indicates ED[39] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_89 89  // Bidirectional. Settings for current instrument: indicates ED[40] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_90 90  // Bidirectional. Settings for current instrument: indicates ED[41] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_91 91  // Bidirectional. Settings for current instrument: indicates ED[42] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_92 92  // Bidirectional. Settings for current instrument: indicates ED[43] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_93 93  // Bidirectional. Settings for current instrument: indicates ED[44] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_94 94  // Bidirectional. Settings for current instrument: indicates ED[45] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_95 95  // Bidirectional. Settings for current instrument: indicates ED[46] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_96 96  // Bidirectional. Settings for current instrument: indicates ED[47] is about to be sent with CC 105.
#define MIDI_CC_104_VALUE_97 97  // Bidirectional. Settings for current instrument: indicates ED[48] is about to be sent with CC 105.
//
/* 98-127 unused */

#define MIDI_CC_105 105  // Bidirectional - From Warbl. Values 0-127. Settings for current instrument: value of above variable indicated by CC 104 or variable indicated by CC 109 (see below)

#define MIDI_CC_106 106  // from WARBL & from Config Tool.  Various values as follows:
//MIDI Channels
#define MIDI_CC_106_VALUE_0 0    // Bidirectional. MIDI channel 1
#define MIDI_CC_106_VALUE_1 1    // Bidirectional. MIDI channel 2
#define MIDI_CC_106_VALUE_2 2    // Bidirectional. MIDI channel 3
#define MIDI_CC_106_VALUE_3 3    // Bidirectional. MIDI channel 4
#define MIDI_CC_106_VALUE_4 4    // Bidirectional. MIDI channel 5
#define MIDI_CC_106_VALUE_5 5    // Bidirectional. MIDI channel 6
#define MIDI_CC_106_VALUE_6 6    // Bidirectional. MIDI channel 7
#define MIDI_CC_106_VALUE_7 7    // Bidirectional. MIDI channel 8
#define MIDI_CC_106_VALUE_8 8    // Bidirectional. MIDI channel 9
#define MIDI_CC_106_VALUE_9 9    // Bidirectional. MIDI channel 10
#define MIDI_CC_106_VALUE_10 10  // Bidirectional. MIDI channel 11
#define MIDI_CC_106_VALUE_11 11  // Bidirectional. MIDI channel 12
#define MIDI_CC_106_VALUE_12 12  // Bidirectional. MIDI channel 13
#define MIDI_CC_106_VALUE_13 13  // Bidirectional. MIDI channel 14
#define MIDI_CC_106_VALUE_14 14  // Bidirectional. MIDI channel 15
#define MIDI_CC_106_VALUE_15 15  // Bidirectional. MIDI channel 16
//
#define MIDI_CC_106_VALUE_16 16  // Bidirectional. custom vibrato off
#define MIDI_CC_106_VALUE_17 17  // Bidirectional. custom vibrato on
/* 18-19 unused */

//Vibrato holes
#define MIDI_CC_106_VALUE_20 20  // Bidirectional. enable vibrato hole, 0
#define MIDI_CC_106_VALUE_21 21  // Bidirectional. enable vibrato hole, 1
#define MIDI_CC_106_VALUE_22 22  // Bidirectional. enable vibrato hole, 2
#define MIDI_CC_106_VALUE_23 23  // Bidirectional. enable vibrato hole, 3
#define MIDI_CC_106_VALUE_24 24  // Bidirectional. enable vibrato hole, 4
#define MIDI_CC_106_VALUE_25 25  // Bidirectional. enable vibrato hole, 5
#define MIDI_CC_106_VALUE_26 26  // Bidirectional. enable vibrato hole, 6
#define MIDI_CC_106_VALUE_27 27  // Bidirectional. enable vibrato hole, 7
#define MIDI_CC_106_VALUE_28 28  // Bidirectional. enable vibrato hole, 8
/* 29 unused */
#define MIDI_CC_106_VALUE_30 30  // Bidirectional. disable vibrato hole, 0
#define MIDI_CC_106_VALUE_31 31  // Bidirectional. disable vibrato hole, 1
#define MIDI_CC_106_VALUE_32 32  // Bidirectional. disable vibrato hole, 2
#define MIDI_CC_106_VALUE_33 33  // Bidirectional. disable vibrato hole, 3
#define MIDI_CC_106_VALUE_34 34  // Bidirectional. disable vibrato hole, 4
#define MIDI_CC_106_VALUE_35 35  // Bidirectional. disable vibrato hole, 5
#define MIDI_CC_106_VALUE_36 36  // Bidirectional. disable vibrato hole, 6
#define MIDI_CC_106_VALUE_37 37  // Bidirectional. disable vibrato hole, 7
#define MIDI_CC_106_VALUE_38 38  // Bidirectional. disable vibrato hole, 8
//
#define MIDI_CC_106_VALUE_39 39  // Bidirectional. calibrate at startup
#define MIDI_CC_106_VALUE_40 40  // Bidirectional. use learned calibration
#define MIDI_CC_106_VALUE_41 41  // from Config Tool. learn initial note on pressure
#define MIDI_CC_106_VALUE_42 42  // from Config Tool. autocalibrate bell sensor only
#define MIDI_CC_106_VALUE_43 43  // from Config Tool. learn drones on pressure
/* 44 unused */
#define MIDI_CC_106_VALUE_45 45  // from Config Tool. save current sensor calibration as factory calibration (this is on a special webpage for me to use when I first program WARBL)
#define MIDI_CC_106_VALUE_46 46  // Bidirectional.invert off
#define MIDI_CC_106_VALUE_47 47  // Bidirectional.invert on
#define MIDI_CC_106_VALUE_48 48  // from WARBL. next message will be byte 1 of debug message
#define MIDI_CC_106_VALUE_49 49  // from WARBL. next message will be byte 2 of debug message
#define MIDI_CC_106_VALUE_50 50  // from WARBL. next message will be byte 3 of debug message (midi requires three bytes to send/receive an int because MIDI bytes are actually only 7 bits)
#define MIDI_CC_106_VALUE_51 51  // from WARBL. Indicates end of two-byte message
#define MIDI_CC_106_VALUE_52 52  // from WARBL. Indicates end of three-byte message
#define MIDI_CC_106_VALUE_53 53  // Bidirectional. Used to tell the Config Tool to include the uilleann regulators fingering pattern (used in a custom version of the code)
#define MIDI_CC_106_VALUE_54 54  // from Config Tool. WARBL2 calibrate IMU
#define MIDI_CC_106_VALUE_55 55  // Bidirectional. WARBL2 settings array (for settings that are independent of mode)
#define MIDI_CC_106_VALUE_56 56  // Bidirectional. WARBL2 settings array (for settings that are independent of mode)
#define MIDI_CC_106_VALUE_57 57  // Bidirectional. WARBL2 settings array (for settings that are independent of mode)
/* 58-59 unused */
#define MIDI_CC_106_VALUE_60 60  // from Config Tool. WARBL2 recenter yaw
#define MIDI_CC_106_VALUE_61 61  // from Config Tool. WARBL2 reset pitch expression override to default
/* 62-69 unused */
#define MIDI_CC_106_VALUE_70 70  // from WARBL. WARBL2 battery voltage
#define MIDI_CC_106_VALUE_71 71  // from WARBL. WARBL2 charging status

#define MIDI_CC_106_VALUE_72 72  // from WARBL. WARBL2 BLE connection interval low byte
#define MIDI_CC_106_VALUE_73 73  // from WARBL. WARBL2 BLE connection interval high byte
#define MIDI_CC_106_VALUE_74 74  // from WARBL. WARBL2 battery percentage
                                 /* 75-99	unused -- can be used for WARBL2 */

//Button Actions, see above 102  90/99
#define MIDI_CC_106_VALUE_100 100  // Bidirectional. button action 0
#define MIDI_CC_106_VALUE_101 101  // Bidirectional. button action 1
#define MIDI_CC_106_VALUE_102 102  // Bidirectional. button action 2
#define MIDI_CC_106_VALUE_103 103  // Bidirectional. button action 3
#define MIDI_CC_106_VALUE_104 104  // Bidirectional. button action 4
#define MIDI_CC_106_VALUE_105 105  // Bidirectional. button action 5
#define MIDI_CC_106_VALUE_106 106  // Bidirectional. button action 6
#define MIDI_CC_106_VALUE_107 107  // Bidirectional. button action 7
#define MIDI_CC_106_VALUE_108 108  // Bidirectional. button action 8
#define MIDI_CC_106_VALUE_109 109  // Bidirectional. button action 9
#define MIDI_CC_106_VALUE_110 110  // Bidirectional. button action 10
#define MIDI_CC_106_VALUE_111 111  // Bidirectional. button action 11
#define MIDI_CC_106_VALUE_112 112  // Bidirectional. button action 12
#define MIDI_CC_106_VALUE_113 113  // Bidirectional. button action 13
#define MIDI_CC_106_VALUE_114 114  // Bidirectional. button action 14
#define MIDI_CC_106_VALUE_115 115  // Bidirectional. button action 15
#define MIDI_CC_106_VALUE_116 116  // Bidirectional. button action 16
#define MIDI_CC_106_VALUE_117 117  // Bidirectional. button action 17
#define MIDI_CC_106_VALUE_118 118  // Bidirectional. button action 18
#define MIDI_CC_106_VALUE_119 119  // Bidirectional. button action 19
#define MIDI_CC_106_VALUE_120 120  // Bidirectional. button action 20
#define MIDI_CC_106_VALUE_121 121  // Bidirectional. button action 21
#define MIDI_CC_106_VALUE_122 122  // Bidirectional. button action 22
#define MIDI_CC_106_VALUE_123 123  // Bidirectional. button action 23
#define MIDI_CC_106_VALUE_124 124  // Bidirectional. button action 24
#define MIDI_CC_106_VALUE_125 125  // Bidirectional. button action 25
#define MIDI_CC_106_VALUE_126 126  // Bidirectional. button action 26
#define MIDI_CC_106_VALUE_127 127  // Bidirectional. button action 27
//

#define MIDI_CC_107 107  // From WARBL. Values 0-127	- MIDI byte 2
#define MIDI_CC_108 108  // From WARBL. Values 0-127	- MIDI byte 3

#define MIDI_CC_109 109         // From WARBL. Values as follows:
#define MIDI_CC_109_OFFSET 200  // Value added to received CC 109, to distinguish them from those from CC_104
//IMU Settings Array - see defines above
#define MIDI_CC_109_VALUE_0 0    // Bidirectional. Settings for current instrument: indicates IMUsettings[0] is about to be sent with CC 105.
#define MIDI_CC_109_VALUE_1 1    // Bidirectional. Settings for current instrument: indicates IMUsettings[1] is about to be sent with CC 105.
#define MIDI_CC_109_VALUE_2 2    // Bidirectional. Settings for current instrument: indicates IMUsettings[2] is about to be sent with CC 105.
#define MIDI_CC_109_VALUE_3 3    // Bidirectional. Settings for current instrument: indicates IMUsettings[3] is about to be sent with CC 105.
#define MIDI_CC_109_VALUE_4 4    // Bidirectional. Settings for current instrument: indicates IMUsettings[4] is about to be sent with CC 105.
#define MIDI_CC_109_VALUE_5 5    // Bidirectional. Settings for current instrument: indicates IMUsettings[5] is about to be sent with CC 105.
#define MIDI_CC_109_VALUE_6 6    // Bidirectional. Settings for current instrument: indicates IMUsettings[6] is about to be sent with CC 105.
#define MIDI_CC_109_VALUE_7 7    // Bidirectional. Settings for current instrument: indicates IMUsettings[7] is about to be sent with CC 105.
#define MIDI_CC_109_VALUE_8 8    // Bidirectional. Settings for current instrument: indicates IMUsettings[8] is about to be sent with CC 105.
#define MIDI_CC_109_VALUE_9 9    // Bidirectional. Settings for current instrument: indicates IMUsettings[9] is about to be sent with CC 105.
#define MIDI_CC_109_VALUE_10 10  // Bidirectional. Settings for current instrument: indicates IMUsettings[10] is about to be sent with CC 105.
#define MIDI_CC_109_VALUE_11 11  // Bidirectional. Settings for current instrument: indicates IMUsettings[11] is about to be sent with CC 105.
#define MIDI_CC_109_VALUE_12 12  // Bidirectional. Settings for current instrument: indicates IMUsettings[12] is about to be sent with CC 105.
#define MIDI_CC_109_VALUE_13 13  // Bidirectional. Settings for current instrument: indicates IMUsettings[13] is about to be sent with CC 105.
#define MIDI_CC_109_VALUE_14 14  // Bidirectional. Settings for current instrument: indicates IMUsettings[14] is about to be sent with CC 105.
#define MIDI_CC_109_VALUE_15 15  // Bidirectional. Settings for current instrument: indicates IMUsettings[15] is about to be sent with CC 105.
#define MIDI_CC_109_VALUE_16 16  // Bidirectional. Settings for current instrument: indicates IMUsettings[16] is about to be sent with CC 105.
#define MIDI_CC_109_VALUE_17 17  // Bidirectional. Settings for current instrument: indicates IMUsettings[17] is about to be sent with CC 105.
#define MIDI_CC_109_VALUE_18 18  // Bidirectional. Settings for current instrument: indicates IMUsettings[18] is about to be sent with CC 105.
#define MIDI_CC_109_VALUE_19 19  // Bidirectional. Settings for current instrument: indicates IMUsettings[19] is about to be sent with CC 105.
#define MIDI_CC_109_VALUE_20 20  // Bidirectional. Settings for current instrument: indicates IMUsettings[20] is about to be sent with CC 105.
#define MIDI_CC_109_VALUE_21 21  // Bidirectional. Settings for current instrument: indicates IMUsettings[21] is about to be sent with CC 105.
#define MIDI_CC_109_VALUE_22 22  // Bidirectional. Settings for current instrument: indicates IMUsettings[22] is about to be sent with CC 105.
#define MIDI_CC_109_VALUE_23 23  // Bidirectional. Settings for current instrument: indicates IMUsettings[23] is about to be sent with CC 105.
#define MIDI_CC_109_VALUE_24 24  // Bidirectional. Settings for current instrument: indicates IMUsettings[24] is about to be sent with CC 105.
#define MIDI_CC_109_VALUE_25 25  // Bidirectional. Settings for current instrument: indicates IMUsettings[25] is about to be sent with CC 105.
#define MIDI_CC_109_VALUE_26 26  // Bidirectional. Settings for current instrument: indicates IMUsettings[26] is about to be sent with CC 105.
#define MIDI_CC_109_VALUE_27 27  // Bidirectional. Settings for current instrument: indicates IMUsettings[27] is about to be sent with CC 105.
#define MIDI_CC_109_VALUE_28 28  // Bidirectional. Settings for current instrument: indicates IMUsettings[28] is about to be sent with CC 105.
#define MIDI_CC_109_VALUE_29 29  // Bidirectional. Settings for current instrument: indicates IMUsettings[29] is about to be sent with CC 105.
#define MIDI_CC_109_VALUE_30 30  // Bidirectional. Settings for current instrument: indicates IMUsettings[30] is about to be sent with CC 105.
#define MIDI_CC_109_VALUE_31 31  // Bidirectional. Settings for current instrument: indicates IMUsettings[31] is about to be sent with CC 105.
#define MIDI_CC_109_VALUE_32 32  // Bidirectional. Settings for current instrument: indicates IMUsettings[32] is about to be sent with CC 105.
/* 33-99	unused -- can be used to extend above array or for other variables */

#define MIDI_CC_109_VALUE_100 100  // Bidirectional. Indicates that  WARBL2 custom fingering chart 1 is about to be sent on CC 105. Same command from WARBL indicates that all 256 messages were received.
#define MIDI_CC_109_VALUE_101 101  // Bidirectional. Indicates that  WARBL2 custom fingering chart 2 is about to be sent on CC 105.
#define MIDI_CC_109_VALUE_102 102  // Bidirectional. Indicates that  WARBL2 custom fingering chart 3 is about to be sent on CC 105.
#define MIDI_CC_109_VALUE_103 103  // Bidirectional. Indicates that  WARBL2 custom fingering chart 4 is about to be sent on CC 105.
                                   /* 104-126	unused */
#define MIDI_CC_109_VALUE_127 127  // From WARBL. Indicates button/gesture action will be sent on CC 105


#define MIDI_CC_110 110            // From WARBL. Values 0-127	- firmware version
#define MIDI_CC_111 111            // Bidirectional. Values 0-127	- note shift for mode 0
#define MIDI_CC_111_VALUE_109 109  // Bidirectional. Hidden sticks mode - Same for 112 and 113
#define MIDI_CC_112 112            // Bidirectional. Values 0-127	- note shift for mode 1
#define MIDI_CC_113 113            // Bidirectional. Values 0-127	- note shift for mode 2

#define MIDI_CC_114 114  // From WARBL. Values 0-127	- highest 2 bytes of holeCovered (int indicating which holes are currently covered)
#define MIDI_CC_115 115  // From WARBL. Values 0-127	- lowest 7 bytes of holeCovered
#define MIDI_CC_116 116  // From WARBL. Values 0-127	- LSB of pressure
#define MIDI_CC_117 117  // Bidirectional. Values 0-100	- vibrato depth (cents)
#define MIDI_CC_118 118  // From WARBL. Values 0-127	- MSB of pressure
#define MIDI_CC_119 119  // From WARBL. Values 0-127	- value of above variable indicated by CC 106

#define MIDI_CC_123 123  // MIDI PANIC
//END of MIDI Numerical constants

//Human readable constants
/* Commands - VALUES ONLY */
#define MIDI_SAVE_CALIB MIDI_CC_102_VALUE_19                 // from Config Tool. Save optical sensor calibration
#define MIDI_EXIT_COMM_MODE MIDI_CC_102_VALUE_104            // from Config Tool. exit communication mode (previously 102 99)
#define MIDI_SAVE_AS_DEFAULTS_CURRENT MIDI_CC_102_VALUE_123  // from Config Tool. save as defaults for current mode
#define MIDI_SAVE_AS_DEFAULTS_ALL MIDI_CC_102_VALUE_124      // from Config Tool. save as defaults for  all instruments
#define MIDI_RESTORE_FACTORY MIDI_CC_102_VALUE_125           // from Config Tool. restore factory settings
#define MIDI_ENTER_COMM_MODE MIDI_CC_102_VALUE_126           // from Config Tool. enter communication mode
#define MIDI_START_CALIB MIDI_CC_102_VALUE_127               // from Config Tool. begin autocalibration

#define MIDI_STARTUP_CALIB MIDI_CC_106_VALUE_39             // Bidirectional. calibrate at startup
#define MIDI_USE_LEARNED_CALIB MIDI_CC_106_VALUE_40         // Bidirectional. use learned calibration
#define MIDI_LEARN_INITIAL_NOTE_PRESS MIDI_CC_106_VALUE_41  // from Config Tool. learn initial note on pressure
#define MIDI_CALIB_BELL_SENSOR MIDI_CC_106_VALUE_42         // from Config Tool. autocalibrate bell sensor only
#define MIDI_LEARN_DRONES_PRESSURE MIDI_CC_106_VALUE_43     // from Config Tool. learn drones on pressure
#define MIDI_SAVE_CALIB_AS_FACTORY MIDI_CC_106_VALUE_45     // from Config Tool. save current sensor calibration as factory calibration (this is on a special webpage for me to use when I first program WARBL)
#define MIDI_CALIB_IMU MIDI_CC_106_VALUE_54                 // from Config Tool. WARBL2 calibrate IMU
#define MIDI_CENTER_YAW MIDI_CC_106_VALUE_60                // from Config Tool. WARBL2 recenter yaw
#define MIDI_RESET_PITCH_EXPRESSION MIDI_CC_106_VALUE_61    // from Config Tool. WARBL2 reset pitch expression override to default

#define MIDI_STICKS_MODE MIDI_CC_111_VALUE_109  // Bidirectional. Hidden sticks mode - Same for 112 and 113

/* START - END Values */
#define MIDI_CALIB_MSGS_START MIDI_CC_102_VALUE_1                // Start of Calibration correction messages
#define MIDI_CALIB_MSGS_END MIDI_CC_102_VALUE_18                 // End of Calibration correction messages
#define MIDI_MAX_CALIB_MSGS_START MIDI_CC_102_VALUE_20           // Start of Calibration max values reached messages
#define MIDI_MAX_CALIB_MSGS_END MIDI_CC_102_VALUE_28             // End of Calibration max values reached messages
#define MIDI_FINGERING_PATTERN_MODE_START MIDI_CC_102_VALUE_30   // Bidirectional. indicates that the next command will be the fingering pattern for instrument 1
#define MIDI_FINGERING_PATTERN_START MIDI_CC_102_VALUE_33        // Bidirectional. first fingering pattern is tin whistle
#define MIDI_FINGERING_PATTERN_END MIDI_CC_102_VALUE_55          // Bidirectional. Medieval bagpipes
#define MIDI_CURRENT_MODE_START MIDI_CC_102_VALUE_60             // Bidirectional. current instrument (mode variable) is  0
#define MIDI_PB_MODE_START MIDI_CC_102_VALUE_70                  // Bidirectional. Settings for current instrument: Pitchbend mode 0
#define MIDI_BREATH_MODE_START MIDI_CC_102_VALUE_80              // Bidirectional. Settings for current instrument: Breath mode 0
#define MIDI_DEFAULT_MODE_START MIDI_CC_102_VALUE_85             // Bidirectional. default instrument is 0 - (if Config Tool sends 85 to WARBL, WARBL sets current instrument as default)
#define MIDI_GESTURE_START MIDI_CC_102_VALUE_90                  // Bidirectional. Sending data for click 1 (dropdown row 0)
#define MIDI_CUST_FINGERING_PATTERN_START MIDI_CC_102_VALUE_100  // Bidirectional. WARBL2 custom fingering chart 1
#define MIDI_CUST_FINGERING_PATTERN_END MIDI_CC_102_VALUE_103    // Bidirectional. WARBL2 custom fingering chart 4
#define MIDI_ACTION_MIDI_START MIDI_CC_102_VALUE_112             // Bidirectional. send midi note on/note off

#define MIDI_PRESS_SELECT_VARS_START MIDI_CC_104_VALUE_1                // Bidirectional. Settings for current instrument: indicates ""bag threshold"" is about to be sent with CC 105.
#define MIDI_PRESS_SELECT_VARS_END MIDI_CC_104_VALUE_12                 // Bidirectional. Settings for current instrument: indicates "breath  drop time" is about to be sent with CC 105.
#define MIDI_ED_VARS_START MIDI_CC_104_VALUE_13                         // Bidirectional. Settings for current instrument: indicates ED[0] is about to be sent with CC 105.
#define MIDI_ED_VARS_END MIDI_CC_104_VALUE_33                           // Bidirectional. Settings for current instrument: indicates ED[20] is about to be sent with CC 105.
#define MIDI_SWITCHES_VARS_START MIDI_CC_104_VALUE_40                   // Bidirectional. Settings for current instrument: indicates that switches[0] is about to be sent with CC 105.
#define MIDI_SWITCHES_VARS_END MIDI_CC_104_VALUE_53                     // Bidirectional. Settings for current instrument: indicates that switches[13] is about to be sent with CC 105. UNUSED?
#define MIDI_ED_VARS2_START MIDI_CC_104_VALUE_70                        // Bidirectional. Settings for current instrument: indicates ED[21] is about to be sent with CC 105.
#define MIDI_ED_VARS2_END MIDI_CC_104_VALUE_97                          // Bidirectional. Settings for current instrument: indicates ED[48] is about to be sent with CC 105.
#define MIDI_ED_VARS_NUMBER (MIDI_ED_VARS_END - MIDI_ED_VARS_START + 1)   // ED array number of vars for the first slot
#define MIDI_ED_VARS2_OFFSET (MIDI_ED_VARS2_START - MIDI_ED_VARS_NUMBER)  // ED array index for 2nd slot of MIDI Msgs

#define MIDI_ACTION_MIDI_CHANNEL_END MIDI_CC_106_VALUE_15  // Bidirectional. MIDI channel 16

#define MIDI_ENA_VIBRATO_HOLES_START MIDI_CC_106_VALUE_20  // Bidirectional. Enable vibrato hole, 0
#define MIDI_ENA_VIBRATO_HOLES_END MIDI_CC_106_VALUE_28    // Bidirectional. Enable vibrato hole, 8
#define MIDI_DIS_VIBRATO_HOLES_START MIDI_CC_106_VALUE_30  // Bidirectional. Disable vibrato hole, 0
#define MIDI_DIS_VIBRATO_HOLES_END MIDI_CC_106_VALUE_38    // Bidirectional. Disable vibrato hole, 8
#define MIDI_WARBL2_SETTINGS_START MIDI_CC_106_VALUE_55    // Bidirectional. WARBL2 settings array (for settings that are independent of mode)
#define MIDI_WARBL2_SETTINGS_END MIDI_CC_106_VALUE_74      // Bidirectional. WARBL2 settings array (for settings that are independent of mode)
#define MIDI_BUTTON_ACTIONS_START MIDI_CC_106_VALUE_100    // Bidirectional. Button action 0

#define MIDI_CUSTOM_CHARTS_START MIDI_CC_109_VALUE_100                                 // Beginning of WARBL2 CustomCharts
#define MIDI_CUSTOM_CHARTS_END MIDI_CC_109_VALUE_103                                   // End of WARBL2 CustomCharts
#define MIDI_CUSTOM_CHARTS_OFFSET_START (MIDI_CC_109_OFFSET + MIDI_CUSTOM_CHARTS_START)  // Beginning of WARBL2 CustomCharts
#define MIDI_CUSTOM_CHARTS_OFFSET_END (MIDI_CC_109_OFFSET + MIDI_CUSTOM_CHARTS_END)      // End of WARBL2 CustomCharts

/* Various single Values */
#define MIDI_MOMENTARY_OFF MIDI_CC_102_VALUE_117  // Bidirectional. momentary off
#define MIDI_MOMENTARY_ON MIDI_CC_102_VALUE_118   // Bidirectional. momentary on

#define MIDI_LEARNED_PRESS_LSB MIDI_CC_104_VALUE_34  // Bidirectional. Settings for current instrument: Indicates that lsb of learned note trigger pressure is about to be sent on CC 105
#define MIDI_LEARNED_PRESS_MSB MIDI_CC_104_VALUE_35  // Bidirectional. Settings for current instrument: Indicates that msb of learned note trigger pressure is about to be sent on CC 105
#define MIDI_BEND_RANGE MIDI_CC_104_VALUE_61         // Bidirectional. Settings for current instrument: MIDI bend range is about to be sent on CC 105
#define MIDI_MIDI_CHANNEL MIDI_CC_104_VALUE_62       // Bidirectional. Settings for current instrument: MIDI channel is about to be sent on CC 105

#define MIDI_BLE_INTERVAL_LSB MIDI_CC_106_VALUE_72  // from WARBL. WARBL2 BLE connection interval low byte
#define MIDI_BLE_INTERVAL_MSB MIDI_CC_106_VALUE_73  // from WARBL. WARBL2 BLE connection interval high byte

#define MIDI_CUSTOM_CHARTS_START MIDI_CC_109_VALUE_100  // Beginning of WARBL2 CustomCharts

/* General sendMidi Args - NO VALUE - <CONTROL_CHANGE, MIDI channel, CC Number> */
#define MIDI_SEND_CC CONTROL_CHANGE, MIDI_CONFIG_TOOL_CHANNEL

#define MIDI_CC_102_MSG MIDI_SEND_CC, MIDI_CC_102
#define MIDI_CC_103_MSG MIDI_SEND_CC, MIDI_CC_103
#define MIDI_CC_104_MSG MIDI_SEND_CC, MIDI_CC_104
#define MIDI_CC_105_MSG MIDI_SEND_CC, MIDI_CC_105
#define MIDI_CC_106_MSG MIDI_SEND_CC, MIDI_CC_106
#define MIDI_CC_107_MSG MIDI_SEND_CC, MIDI_CC_107
#define MIDI_CC_108_MSG MIDI_SEND_CC, MIDI_CC_108
#define MIDI_CC_109_MSG MIDI_SEND_CC, MIDI_CC_109
#define MIDI_CC_110_MSG MIDI_SEND_CC, MIDI_CC_110

#define MIDI_CC_114_MSG MIDI_SEND_CC, MIDI_CC_114
#define MIDI_CC_115_MSG MIDI_SEND_CC, MIDI_CC_115
#define MIDI_CC_116_MSG MIDI_SEND_CC, MIDI_CC_116
#define MIDI_CC_117_MSG MIDI_SEND_CC, MIDI_CC_117
#define MIDI_CC_118_MSG MIDI_SEND_CC, MIDI_CC_118
#define MIDI_CC_119_MSG MIDI_SEND_CC, MIDI_CC_119

/* Full sendMidi Args - WITH VALUES - <CONTROL_CHANGE, MIDI channel, CC Number, CC Value> */
#define MIDI_CUSTOM_CHARTS_RCVD MIDI_CC_109_MSG, MIDI_CC_109_VALUE_100  //from WARBL. WARBL2 Custom fingering charts - indicate success

//sendMIDICouplet *PARTIAL* Arguments
#define MIDI_SEND_DRONES_PRESSURE_LSB MIDI_CC_104, MIDI_CC_104_VALUE_32, MIDI_CC_105     // Bidirectional. Settings for current instrument: Indicates that lsb of drones pressure is about to be sent on CC 105
#define MIDI_SEND_DRONES_PRESSURE_MSB MIDI_CC_104, MIDI_CC_104_VALUE_33, MIDI_CC_105     // Bidirectional. Settings for current instrument: Indicates that msb of drones pressure is about to be sent on CC 105
#define MIDI_SEND_LEARNED_PRESSURE_LSB MIDI_CC_104, MIDI_LEARNED_PRESS_LSB, MIDI_CC_105  // Bidirectional. Settings for current instrument: Indicates that lsb of learned note trigger pressure is about to be sent on CC 105
#define MIDI_SEND_LEARNED_PRESSURE_MSB MIDI_CC_104, MIDI_LEARNED_PRESS_MSB, MIDI_CC_105  // Bidirectional. Settings for current instrument: Indicates that msb of learned note trigger pressure is about to be sent on CC 105
#define MIDI_SEND_BEND_RANGE MIDI_CC_104, MIDI_CC_104_VALUE_61, MIDI_CC_105              // Bidirectional. Settings for current instrument: MIDI bend range is about to be sent on CC 105
#define MIDI_SEND_MIDI_CHANNEL MIDI_CC_104, MIDI_CC_104_VALUE_62, MIDI_CC_105            // Bidirectional. Settings for current instrument: MIDI channel is about to be sent on CC 105

#define MIDI_SEND_BATTERY_VOLTAGE MIDI_CC_106, MIDI_CC_106_VALUE_70, MIDI_CC_119        // from WARBL. WARBL2 battery voltage
#define MIDI_SEND_BATTERY_CHARGE_STATUS MIDI_CC_106, MIDI_CC_106_VALUE_71, MIDI_CC_119  // from WARBL. WARBL2 charging status
#define MIDI_SEND_BATTERY_CHARGE_PERC MIDI_CC_106, MIDI_CC_106_VALUE_74, MIDI_CC_119    // from WARBL. WARBL2 battery percentage

#define MIDI_SEND_BUTTON_ACTION MIDI_CC_109, MIDI_CC_109_VALUE_127, MIDI_CC_105  // From WARBL. Indicates button/gesture action will be sent on CC 105



//
//END of Human readable constants
/* END of MIDI Config Tool Constants */


/* EEPROM Addresses */
#define EEPROM_BASELINE_CALIB_START 1  // values 0-255 baseline sensor calibrations - 9 bytes 1-9
/* 10-17 used for high bytes of toneholes (not bell) */
#define EEPROM_SENSOR_CALIB_START 18  // sensor calibrations - 18 bytes 18-35, two byte per sensor high-low
#define EEPROM_XGYRO_CALIB_SAVED 36   // value 3 indicates a saved gyro calibration
#define EEPROM_SENSOR_CALIB_SAVED 37  // value 3 indicates a saved sensor calibration
/* 38-39 unused */
#define EEPROM_FINGERING_PATTERN_START 40  // values 0-max fingering pattern - 3 bytes 40-42
/* 43 unused */
#define EEPROM_SETTINGS_SAVED 44  // value 3 indicates settings have been saved - this will be reset to indicate that a factory reset needs to be done.
/* 45-47 unused */
#define EEPROM_DEFAULT_MODE 48  // default mode (instrument)
/* 49 unused */
#define EEPROM_SENS_DISTANCE_START 50   // values 0-255	finger-sensing distance - 3 bytes 50-52
#define EEPROM_NOTE_SHIFT_SEL_START 53  // values 0-1 note shift selector - 3 bytes 53-55
#define EEPROM_SWITCHES_START 56        // values 0-1 switches variables - 43 bytes 56-98
/* 99 unused		
 * 100-247	unused (previously button prefs)
 * 248-249	unused
*/
#define EEPROM_MOMENTARY_MODE_START 250  // values 0-1 momentary mode 0  - 3 bytes 250-252 \
                                         // values 0-1 momentary mode 1  - 3 bytes 253-255 \
                                         // values 0-1 momentary mode 2  - 3 bytes 256-258
/* 259 unused */
#define EEPROM_PRESSURE_SETTINGS_START 260  // 261-272		pressure settings for mode 0 \
                                            // 281-292		pressure settings for mode 1 \
                                            // 301-312		pressure settings for mode 2
#define EEPROM_LEARNED_PRESSURE_START 273   // values 0-255 low byte of learned pressure for mode 0 (274 high byte) \
                                            // 275 low byte of learned pressure for mode 1 (276 high byte) \
                                            // 277 low byte of learned pressure for mode 2 (278 high byte)
/* 279-280 unused 
 * 293-300 unused
*/
#define EEPROM_PB_MODE_START 313          // pitch bend mode - 3 bytes 313-315
#define EEPROM_BREATH_MODE_START 316      // breath mode - 3 bytes 316-318
#define EEPROM_MIDI_BEND_RANGE_START 319  // MIDI bend range - 3 bytes 319-321
#define EEPROM_MIDI_CHANNEL_START 322     // MIDI channel - 3 bytes 322-324
/* 325-332 unused */
#define EEPROM_VIBRATO_HOLES_START 333      // values 0-1 low byte of enabled vibrato holes for mode 0 (334 high byte) \
                                            // 335 low byte of enabled vibrato holes for mode 1 (336 high byte) \
                                            // 337 low byte of enabled vibrato holes for mode 2 (338 high byte)
#define EEPROM_VIBRATO_DEPTH_START 339      // values 0-255 low byte of evibrato depth  for mode 0 (340 high byte) \
                                            // 341 low byte of vibrato depth  for mode 1 (342 high byte) \
                                            // 343 low byte of vibrato depth  for mode 2 (344 high byte)
#define EEPROM_USE_LEARNED_PRESS_START 345  //values 0-1	use learned calibration - 3 bytes 345-347
/* 348-350 unused */
#define EEPROM_ED_VARS_START 351  // 351-497	expression and drones (ED) variables
/* 498-599 unused, room for extending above array or other variables */
#define EEPROM_WARBL2_SETTINGS_START 600  // 600-602 WARBL2settings array
/* 603-625 unused, room for extending above array or other variables */
#define EEPROM_IMU_SETTINGS_START 625  // 625-732 WARBL2 IMUsettings array
/*733-999 unused, room for extending above array or other variables */
#define EEPROM_BUTTON_PREFS_START 1000  // 1000-1247 button prefs
/* 1248-1974 unused */
/* Note: 1975-1986 currently doesn't get saved to factory settings but does get restored from factory settings (calibrations are saved to factory settings manually) */
#define EEPROM_XGYRO_CALIB_START 1975  // XGyro calibration	(float) 4 bytes
#define EEPROM_YGYRO_CALIB_START 1979  // YGyro calibration	(float) 4 bytes
#define EEPROM_ZGYRO_CALIB_START 1983  // ZGyro calibration	(float) 4 bytes
/* Note: 1987-2000 doesn't currently get saved to or restored from factory settings. */
#define EEPROM_RESERVED_TESTING 1987         // Reserved for testing purposes
#define EEPROM_LOW_CHARGE 1988               // indication that there has been a charge termination (value of 3) or shutdown because of low battery (value of 1)
#define EEPROM_EST_RUNTIME_START 1989        // operating time on a full charge (minutes, high byte) - (1990 low byte)
#define EEPROM_FIRMWARE_VERSION 1991         // firmware version
#define EEPROM_HARDWARE_VERSION 1992         // hardware version
#define EEPROM_RUNTIME_START 1993            // minutes operating time since last full charge (high byte) - (1994 low byte)
#define EEPROM_ATMEGA_FIRMWARE_VERSION 1995  // ATmega firmware version
/* 1996-1999 unused, room for more settings */
#define EEPROM_FACTORY_SETTINGS_START 2000  // 2001-3999 locations of factory settings (duplicates of 1-1999, for restoring settings)
#define EEPROM_CUSTOM_FINGERING_START 4000  // 4000-4255		Custom fingering chart 1 \
                                            // 4256-4511		Custom fingering chart 2 \
                                            // 4512-4767		Custom fingering chart 3 \
                                            // 4768-5023		Custom fingering chart 4
/* 5024-16383 other ~11 KB unused */

/* END of EEPROM Addresses */

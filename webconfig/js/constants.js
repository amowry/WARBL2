const MIDI_DEBUG = false;

/*
* CSS constants
*/
const SMALLCONTROLBOX_1_TOP = "440px";
const SMALLCONTROLBOX_2_TOP = "930px"; //"900px";
const BUTTONBOX_TOP =  "1420px"; // "1390px";
const TOPCONTROLBOX_TOP = "1890px"; //"1855px";
/* MIDI Config Tool Constants */
//General constants
const MIDI_CONFIG_TOOL_CHANNEL = 7; // Config Tool MIDI channel

// To Be Sync'd with Defines.h in firmware

//Various constants
var HALF_HOLE_LOW_MIN_PERC = 40; //Min percentage of toneHolecCovered dedicated to lower window (hole open)
var HALF_HOLE_LOW_MAX_PERC = 60; //Max percentage of toneHolecCovered dedicated to lower window (hole open)
var HALF_HOLE_HIGH_MIN_PERC = 10; //Min percentage of toneHolecCovered dedicated to upper window (hole closed)
var HALF_HOLE_HIGH_MAX_PERC = 35; //Max percentage of toneHolecCovered dedicated to upper window (hole closed)

//MIDI Human readable constants: see below

/* Numerical constants
    * Some cc values are reserved from Config Tool or from WARBL
    * Bidirectional Communication: same commands both ways with same cc/value 
*/
const MIDI_CC_102 = 102; // from WARBL & from Config Tool. Various values as follows:
const MIDI_CC_102_VALUE_0 = 0; // unused
// sensor calibration messages
    const MIDI_CC_102_VALUE_1 = 1; //from Config Tool. bell sensor down
    const MIDI_CC_102_VALUE_2 = 2; //from Config Tool. bell sensor up
    const MIDI_CC_102_VALUE_3 = 3; //from Config Tool. R4 down
    const MIDI_CC_102_VALUE_4 = 4; //from Config Tool. R4 up
    const MIDI_CC_102_VALUE_5 = 5; //from Config Tool. R3 down
    const MIDI_CC_102_VALUE_6 = 6; //from Config Tool. R3 up
    const MIDI_CC_102_VALUE_7 = 7; //from Config Tool. R2 down
    const MIDI_CC_102_VALUE_8 = 8; //from Config Tool. R2 up
    const MIDI_CC_102_VALUE_9 = 9; //from Config Tool. R1 down
    const MIDI_CC_102_VALUE_10 = 10; //from Config Tool. R1 up
    const MIDI_CC_102_VALUE_11 = 11; //from Config Tool. L3 down
    const MIDI_CC_102_VALUE_12 = 12; //from Config Tool. L3 up
    const MIDI_CC_102_VALUE_13 = 13; //from Config Tool. L2 down
    const MIDI_CC_102_VALUE_14 = 14; //from Config Tool. L2 up
    const MIDI_CC_102_VALUE_15 = 15; //from Config Tool. L1 down
    const MIDI_CC_102_VALUE_16 = 16; //from Config Tool. L1 up
    const MIDI_CC_102_VALUE_17 = 17; //from Config Tool. Lthumb down
    const MIDI_CC_102_VALUE_18 = 18; //from Config Tool. Lthumb up		
    const MIDI_CC_102_VALUE_19 = 19; //from Config Tool. Save optical sensor calibration
    const MIDI_CC_102_VALUE_20 = 20; //from WARBL. bell sensor max value reached
    const MIDI_CC_102_VALUE_21 = 21; //from WARBL. R4 max value reached
    const MIDI_CC_102_VALUE_22 = 22; //from WARBL. R3 max value reached	
    const MIDI_CC_102_VALUE_23 = 23; //from WARBL. R2 max value reached	
    const MIDI_CC_102_VALUE_24 = 24; //from WARBL. R1 max value reached
    const MIDI_CC_102_VALUE_25 = 25; //from WARBL. L3 max value reached
    const MIDI_CC_102_VALUE_26 = 26; //from WARBL. L2 max value reached
    const MIDI_CC_102_VALUE_27 = 27; //from WARBL. L1 max value reached
    const MIDI_CC_102_VALUE_28 = 28; //from WARBL. Lthumb max value reached
; //
    /* 29 unused */

; //Send fingering pattern selections:
	const MIDI_CC_102_VALUE_30 = 30;  // Bidirectional. indicates that the next command will be the fingering pattern for instrument 1
	const MIDI_CC_102_VALUE_31 = 31;  // Bidirectional. indicates that the next command will be the fingering pattern for instrument 2
	const MIDI_CC_102_VALUE_32 = 32;  // Bidirectional. indicates that the next command will be the fingering pattern for instrument 3
	const MIDI_CC_102_VALUE_33 = 33;  // Bidirectional. first fingering pattern is tin whistle
	const MIDI_CC_102_VALUE_34 = 34;  // Bidirectional. "" uilleann
	const MIDI_CC_102_VALUE_35 = 35;  // Bidirectional. "" GHB
	const MIDI_CC_102_VALUE_36 = 36;  // Bidirectional. "" Northumbrian
	const MIDI_CC_102_VALUE_37 = 37;  // Bidirectional. ""tin whistle/flute chromatic
	const MIDI_CC_102_VALUE_38 = 38;  // Bidirectional. ""Gaita
	const MIDI_CC_102_VALUE_39 = 39;  // Bidirectional. NAF
	const MIDI_CC_102_VALUE_40 = 40;  // Bidirectional. Kaval
	const MIDI_CC_102_VALUE_41 = 41;  // Bidirectional. recorder
	const MIDI_CC_102_VALUE_42 = 42;  // Bidirectional. Bansuri
	const MIDI_CC_102_VALUE_43 = 43;  // Bidirectional. Uilleann standard
	const MIDI_CC_102_VALUE_44 = 44;  // Bidirectional. Xiao
	const MIDI_CC_102_VALUE_45 = 45;  // Bidirectional. Sax extended
	const MIDI_CC_102_VALUE_46 = 46;  // Bidirectional. Gaita extended
	const MIDI_CC_102_VALUE_47 = 47;  // Bidirectional. Saxbasic
	const MIDI_CC_102_VALUE_48 = 48;  // Bidirectional. EVI
	const MIDI_CC_102_VALUE_49 = 49;  // Bidirectional. Shakuhachi
	const MIDI_CC_102_VALUE_50 = 50;  // Bidirectional. Sackpipa major
	const MIDI_CC_102_VALUE_51 = 51;  // Bidirectional. Sackpipa minor
	const MIDI_CC_102_VALUE_52 = 52;  // Bidirectional. Custom (original WARBL only)
	const MIDI_CC_102_VALUE_53 = 53;  // Bidirectional. Bombarde
	const MIDI_CC_102_VALUE_54 = 54;  // Bidirectional. Baroque flute
	const MIDI_CC_102_VALUE_55 = 55;  // Bidirectional. Medieval bagpipes
	const MIDI_CC_102_VALUE_56 = 56;  // Bidirectional. Mep's EWI
	const MIDI_CC_102_VALUE_57 = 57;  // Bidirectional. Mep's Recorder
; //
	/* 56-59 unused */
    const MIDI_CC_102_VALUE_60 = 60; // Bidirectional. current instrument (mode variable) is 0
    const MIDI_CC_102_VALUE_61 = 61; // Bidirectional. current instrument is 1
    const MIDI_CC_102_VALUE_62 = 62; // Bidirectional. current instrument is 2
    /* 63-69 unused */
    const MIDI_CC_102_VALUE_70 = 70; // Bidirectional. Settings for current instrument: Pitchbend mode 0
    const MIDI_CC_102_VALUE_71 = 71; // Bidirectional. Settings for current instrument: Pitchbend mode 1
    const MIDI_CC_102_VALUE_72 = 72; // Bidirectional. Settings for current instrument: Pitchbend mode 2
    const MIDI_CC_102_VALUE_73 = 73; // Bidirectional. Settings for current instrument: Pitchbend mode 3
    /* 74-79 unused */
    const MIDI_CC_102_VALUE_80 = 80; // Bidirectional. Settings for current instrument: Breath mode 0
    const MIDI_CC_102_VALUE_81 = 81; // Bidirectional. Settings for current instrument: Breath mode 1
    const MIDI_CC_102_VALUE_82 = 82; // Bidirectional. Settings for current instrument: Breath mode 2
    const MIDI_CC_102_VALUE_83 = 83; // Bidirectional. Settings for current instrument: Breath mode 3
    const MIDI_CC_102_VALUE_84 = 84; // Bidirectional. Settings for current instrument: Breath mode4
    const MIDI_CC_102_VALUE_85 = 85; // Bidirectional. default instrument is 0 - (if Config Tool sends 85 to WARBL, WARBL sets current instrument as default)
    const MIDI_CC_102_VALUE_86 = 86; // Bidirectional. default instrument is 1
    const MIDI_CC_102_VALUE_87 = 87; // Bidirectional. default instrument is 2
    /* 88-89 unused */

    /* Populate button configuration for current instrument:	
    * First indicate button combination to be populated:
    */
    const MIDI_CC_102_VALUE_90 = 90; // Bidirectional. Sending data for click 1 (dropdown row 0)
    const MIDI_CC_102_VALUE_91 = 91; // Bidirectional. click 2
    const MIDI_CC_102_VALUE_92 = 92; // Bidirectional. click 3
    const MIDI_CC_102_VALUE_93 = 93; // Bidirectional. hold 2, click 1
    const MIDI_CC_102_VALUE_94 = 94; // Bidirectional. hold 2, click 3
    const MIDI_CC_102_VALUE_95 = 95; // Bidirectional. longpress 1
    const MIDI_CC_102_VALUE_96 = 96; // Bidirectional. longpress 2
    const MIDI_CC_102_VALUE_97 = 97; // Bidirectional. longpress 3
    const MIDI_CC_102_VALUE_98 = 98; // unused
    const MIDI_CC_102_VALUE_99 = 99; // unused (previously disconnect command)
    /* Follow with data to populate indicated button combination:
    * CC 106	values 100-127 (see below) 
    * (was previously CC 102 100-111 but ran out of room here)
    */


    const MIDI_CC_102_VALUE_100 = 100; // Bidirectional. WARBL2 custom fingering chart 1
    const MIDI_CC_102_VALUE_101 = 101; // Bidirectional. WARBL2 custom fingering chart 2
    const MIDI_CC_102_VALUE_102 = 102; // Bidirectional. WARBL2 custom fingering chart 3
    const MIDI_CC_102_VALUE_103 = 103; // Bidirectional. WARBL2 custom fingering chart 4
    const MIDI_CC_102_VALUE_104 = 104; //from Config Tool. exit communication mode (previously 102 99)
    /* 105-111	unused -- can be used for WARBL2 */
    const MIDI_CC_102_VALUE_112 = 112; // Bidirectional. send midi note on/note off
    const MIDI_CC_102_VALUE_113 = 113; // Bidirectional. Send midi CC
    const MIDI_CC_102_VALUE_114 = 114; // Bidirectional. Send MIDI PC
    const MIDI_CC_102_VALUE_115 = 115; // Bidirectional. Increase PC
    const MIDI_CC_102_VALUE_116 = 116; // Bidirectional. Decrease PC
    const MIDI_CC_102_VALUE_117 = 117; // Bidirectional. momentary off
    const MIDI_CC_102_VALUE_118 = 118; // Bidirectional. momentary on
    /* 119 unused */
    const MIDI_CC_102_VALUE_120 = 120; //from WARBL. bell sensor disconnected	
    const MIDI_CC_102_VALUE_121 = 121; //from WARBL. bell sensor connected	
    /* 122 unused */
    const MIDI_CC_102_VALUE_123 = 123; //from Config Tool. save as defaults for current mode
    const MIDI_CC_102_VALUE_124 = 124; //from Config Tool. save as defaults for all instruments
    const MIDI_CC_102_VALUE_125 = 125; //from Config Tool. restore factory settings
    const MIDI_CC_102_VALUE_126 = 126; //from Config Tool. enter communication mode
    // WARBL enters communication mode (until it is shut off or user clicks "Disconnect") and responds by sending settings for currently selected instrument.
    const MIDI_CC_102_VALUE_127 = 127; //from Config Tool. begin autocalibration


const MIDI_CC_103 = 103; // from WARBL & from Config Tool. Values 0-127	- Settings for current instrument: finger-sensing distance

const MIDI_CC_104 = 104; // from WARBL & from Config Tool. Various values as follows:
    /* 0 unused */

    /* Bag/Breath advanced settings (pressureSelector[] variables*/
    const MIDI_CC_104_VALUE_1 = 1; // Bidirectional. Settings for current instrument: indicates "bag threshold" is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_2 = 2; // Bidirectional. Settings for current instrument: indicates "bag multiplier" is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_3 = 3; // Bidirectional. Settings for current instrument: indicates "bag histeresis" is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_4 = 4; // Bidirectional. unused. 
    const MIDI_CC_104_VALUE_5 = 5; // Bidirectional. Settings for current instrument: indicates "bag jump time" is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_6 = 6; // Bidirectional. Settings for current instrument: indicates "bag drop time" is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_7 = 7; // Bidirectional. Settings for current instrument: indicates "breath threshold" is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_8 = 8; // Bidirectional. Settings for current instrument: indicates "breath multiplier" is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_9 = 9; // Bidirectional. Settings for current instrument: indicates "breath histeresis" is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_10 = 10; // Bidirectional. Settings for current instrument: indicates "breath transient filter" is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_11 = 11; // Bidirectional. Settings for current instrument: indicates "breath jump time" is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_12 = 12; // Bidirectional. Settings for current instrument: indicates "breath drop time" is about to be sent with CC 105.
    /* Expression or drones variable (ED array) - see defines above */
    const MIDI_CC_104_VALUE_13 = 13; // Bidirectional. Settings for current instrument: indicates ED[0] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_14 = 14; // Bidirectional. Settings for current instrument: indicates ED[1] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_15 = 15; // Bidirectional. Settings for current instrument: indicates ED[2] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_16 = 16; // Bidirectional. Settings for current instrument: indicates ED[3] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_17 = 17; // Bidirectional. Settings for current instrument: indicates ED[4] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_18 = 18; // Bidirectional. Settings for current instrument: indicates ED[5] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_19 = 19; // Bidirectional. Settings for current instrument: indicates ED[6] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_20 = 20; // Bidirectional. Settings for current instrument: indicates ED[7] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_21 = 21; // Bidirectional. Settings for current instrument: indicates ED[8] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_22 = 22; // Bidirectional. Settings for current instrument: indicates ED[9] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_23 = 23; // Bidirectional. Settings for current instrument: indicates ED[10] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_24 = 24; // Bidirectional. Settings for current instrument: indicates ED[11] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_25 = 25; // Bidirectional. Settings for current instrument: indicates ED[12]] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_26 = 26; // Bidirectional. Settings for current instrument: indicates ED[13] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_27 = 27; // Bidirectional. Settings for current instrument: indicates ED[14] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_28 = 28; // Bidirectional. Settings for current instrument: indicates ED[15] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_29 = 29; // Bidirectional. Settings for current instrument: indicates ED[16] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_30 = 30; // Bidirectional. Settings for current instrument: indicates ED[17] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_31 = 31; // Bidirectional. Settings for current instrument: indicates ED[18] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_32 = 32; // Bidirectional. Settings for current instrument: indicates ED[19] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_33 = 33; // Bidirectional. Settings for current instrument: indicates ED[20] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_34 = 34; // Bidirectional. Settings for current instrument: indicates that lsb of learned note trigger pressure is about to be sent on CC 105
    const MIDI_CC_104_VALUE_35 = 35; // Bidirectional. Settings for current instrument: indicates that msb of learned note trigger pressure is about to be sent on CC 105
    /* 36-39 unused */

    /* Switches array - see defines above */
    const MIDI_CC_104_VALUE_40 = 40; // Bidirectional. Settings for current instrument: indicates that switches[0] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_41 = 41; // Bidirectional. Settings for current instrument: indicates that switches[1] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_42 = 42; // Bidirectional. Settings for current instrument: indicates that switches[2] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_43 = 43; // Bidirectional. Settings for current instrument: indicates that switches[3] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_44 = 44; // Bidirectional. Settings for current instrument: indicates that switches[4] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_45 = 45; // Bidirectional. Settings for current instrument: indicates that switches[5] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_46 = 46; // Bidirectional. Settings for current instrument: indicates that switches[6] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_47 = 47; // Bidirectional. Settings for current instrument: indicates that switches[7] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_48 = 48; // Bidirectional. Settings for current instrument: indicates that switches[8] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_49 = 49; // Bidirectional. Settings for current instrument: indicates that switches[9] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_50 = 50; // Bidirectional. Settings for current instrument: indicates that switches[10] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_51 = 51; // Bidirectional. Settings for current instrument: indicates that switches[11] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_52 = 52; // Bidirectional. Settings for current instrument: indicates that switches[12] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_53 = 53; // Bidirectional. Settings for current instrument: indicates that switches[13] is about to be sent with CC 105.
    const MIDI_CC_104_VALUE_54 = 54 //  Bidirectional. Settings for current instrument: indicates that switches[14] is about to be sent with CC 105.
    const MIDI_CC_104_VALUE_55 = 55 //  Bidirectional. Settings for current instrument: indicates that switches[14] is about to be sent with CC 105.
    //
    /* 55-60 unused */
; //
    /* 54-60 unused */
    const MIDI_CC_104_VALUE_61 = 61; // Bidirectional. Settings for current instrument: MIDI bend range is about to be sent on CC 105
    const MIDI_CC_104_VALUE_62 = 62; // Bidirectional. Settings for current instrument: MIDI channel is about to be sent on CC 105
    /* 62-69 unused */

	/* more expression or drones variables (ED array) 
    * can be extended to 127 for WARBL2, e.g. IMU functionality. If more variables are needed CC109 can also be used to indicate additional "pressureReceiveMode" options
    */
    const MIDI_CC_104_VALUE_70 = 70; // Bidirectional. Settings for current instrument: indicates ED[21] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_71 = 71; // Bidirectional. Settings for current instrument: indicates ED[22] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_72 = 72; // Bidirectional. Settings for current instrument: indicates ED[23] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_73 = 73; // Bidirectional. Settings for current instrument: indicates ED[24] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_74 = 74; // Bidirectional. Settings for current instrument: indicates ED[25] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_75 = 75; // Bidirectional. Settings for current instrument: indicates ED[26] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_76 = 76; // Bidirectional. Settings for current instrument: indicates ED[27] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_77 = 77; // Bidirectional. Settings for current instrument: indicates ED[28] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_78 = 78; // Bidirectional. Settings for current instrument: indicates ED[29] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_79 = 79; // Bidirectional. Settings for current instrument: indicates ED[30] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_80 = 80; // Bidirectional. Settings for current instrument: indicates ED[31] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_81 = 81; // Bidirectional. Settings for current instrument: indicates ED[32] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_82 = 82; // Bidirectional. Settings for current instrument: indicates ED[33] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_83 = 83; // Bidirectional. Settings for current instrument: indicates ED[34] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_84 = 84; // Bidirectional. Settings for current instrument: indicates ED[35] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_85 = 85; // Bidirectional. Settings for current instrument: indicates ED[36] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_86 = 86; // Bidirectional. Settings for current instrument: indicates ED[37] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_87 = 87; // Bidirectional. Settings for current instrument: indicates ED[38] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_88 = 88; // Bidirectional. Settings for current instrument: indicates ED[39] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_89 = 89; // Bidirectional. Settings for current instrument: indicates ED[40] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_90 = 90; // Bidirectional. Settings for current instrument: indicates ED[41] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_91 = 91; // Bidirectional. Settings for current instrument: indicates ED[42] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_92 = 92; // Bidirectional. Settings for current instrument: indicates ED[43] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_93 = 93; // Bidirectional. Settings for current instrument: indicates ED[44] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_94 = 94; // Bidirectional. Settings for current instrument: indicates ED[45] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_95 = 95; // Bidirectional. Settings for current instrument: indicates ED[46] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_96 = 96; // Bidirectional. Settings for current instrument: indicates ED[47] is about to be sent with CC 105. 
    const MIDI_CC_104_VALUE_97 = 97; // Bidirectional. Settings for current instrument: indicates ED[48] is about to be sent with CC 105. 
; //
    /* 98-127 unused */

const MIDI_CC_105 = 105; // Bidirectional - From Warbl. Values 0-127. Settings for current instrument: value of above variable indicated by CC 104 or variable indicated by CC 109 (see below)

const MIDI_CC_106 = 106; // from WARBL & from Config Tool. Various values as follows:
; //MIDI Channels
    const MIDI_CC_106_VALUE_0 = 0; //Bidirectional. MIDI channel 1
    const MIDI_CC_106_VALUE_1 = 1; //Bidirectional. MIDI channel 2
    const MIDI_CC_106_VALUE_2 = 2; //Bidirectional. MIDI channel 3
    const MIDI_CC_106_VALUE_3 = 3; //Bidirectional. MIDI channel 4
    const MIDI_CC_106_VALUE_4 = 4; //Bidirectional. MIDI channel 5
    const MIDI_CC_106_VALUE_5 = 5; //Bidirectional. MIDI channel 6
    const MIDI_CC_106_VALUE_6 = 6; //Bidirectional. MIDI channel 7
    const MIDI_CC_106_VALUE_7 = 7; //Bidirectional. MIDI channel 8
    const MIDI_CC_106_VALUE_8 = 8; //Bidirectional. MIDI channel 9
    const MIDI_CC_106_VALUE_9 = 9; //Bidirectional. MIDI channel 10
    const MIDI_CC_106_VALUE_10 = 10; //Bidirectional. MIDI channel 11
    const MIDI_CC_106_VALUE_11 = 11; //Bidirectional. MIDI channel 12
    const MIDI_CC_106_VALUE_12 = 12; //Bidirectional. MIDI channel 13
    const MIDI_CC_106_VALUE_13 = 13; //Bidirectional. MIDI channel 14
    const MIDI_CC_106_VALUE_14 = 14; //Bidirectional. MIDI channel 15
    const MIDI_CC_106_VALUE_15 = 15; //Bidirectional. MIDI channel 16
; //
    const MIDI_CC_106_VALUE_16 = 16; //Bidirectional. custom vibrato off
    const MIDI_CC_106_VALUE_17 = 17; //Bidirectional. custom vibrato on
    /* 18-19 unused */

; //Vibrato holes
    const MIDI_CC_106_VALUE_20 = 20; //Bidirectional. enable vibrato hole, 0	
    const MIDI_CC_106_VALUE_21 = 21; //Bidirectional. enable vibrato hole, 1	
    const MIDI_CC_106_VALUE_22 = 22; //Bidirectional. enable vibrato hole, 2	
    const MIDI_CC_106_VALUE_23 = 23; //Bidirectional. enable vibrato hole, 3	
    const MIDI_CC_106_VALUE_24 = 24; //Bidirectional. enable vibrato hole, 4	
    const MIDI_CC_106_VALUE_25 = 25; //Bidirectional. enable vibrato hole, 5	
    const MIDI_CC_106_VALUE_26 = 26; //Bidirectional. enable vibrato hole, 6	
    const MIDI_CC_106_VALUE_27 = 27; //Bidirectional. enable vibrato hole, 7	
    const MIDI_CC_106_VALUE_28 = 28; //Bidirectional. enable vibrato hole, 8	
    /* 29 unused */
    const MIDI_CC_106_VALUE_30 = 30; //Bidirectional. disable vibrato hole, 0
    const MIDI_CC_106_VALUE_31 = 31; //Bidirectional. disable vibrato hole, 1
    const MIDI_CC_106_VALUE_32 = 32; //Bidirectional. disable vibrato hole, 2
    const MIDI_CC_106_VALUE_33 = 33; //Bidirectional. disable vibrato hole, 3
    const MIDI_CC_106_VALUE_34 = 34; //Bidirectional. disable vibrato hole, 4
    const MIDI_CC_106_VALUE_35 = 35; //Bidirectional. disable vibrato hole, 5
    const MIDI_CC_106_VALUE_36 = 36; //Bidirectional. disable vibrato hole, 6
    const MIDI_CC_106_VALUE_37 = 37; //Bidirectional. disable vibrato hole, 7
    const MIDI_CC_106_VALUE_38 = 38; //Bidirectional. disable vibrato hole, 8
; //
    const MIDI_CC_106_VALUE_39 = 39; //Bidirectional. calibrate at startup
    const MIDI_CC_106_VALUE_40 = 40; //Bidirectional. use learned calibration
    const MIDI_CC_106_VALUE_41 = 41; // from Config Tool. learn initial note on pressure
    const MIDI_CC_106_VALUE_42 = 42; // from Config Tool. autocalibrate bell sensor only	
    const MIDI_CC_106_VALUE_43 = 43; // from Config Tool. learn drones on pressure
    /* 44 unused */
    const MIDI_CC_106_VALUE_45 = 45; // from Config Tool. save current sensor calibration as factory calibration (this is on a special webpage for me to use when I first program WARBL)
    const MIDI_CC_106_VALUE_46 = 46; //Bidirectional.invert off
    const MIDI_CC_106_VALUE_47 = 47; //Bidirectional.invert on
	const MIDI_CC_106_VALUE_48 = 48; //from WARBL. next message will be byte 1 of debug message
	const MIDI_CC_106_VALUE_49 = 49; //from WARBL. next message will be byte 2 of debug message
	const MIDI_CC_106_VALUE_50 = 50; //from WARBL. next message will be byte 3 of debug message (midi requires three bytes to send/receive an int because MIDI bytes are actually only 7 bits)
	const MIDI_CC_106_VALUE_51 = 51; //from WARBL. Indicates end of two-byte message 
	const MIDI_CC_106_VALUE_52 = 52; //from WARBL. Indicates end of three-byte message 
	const MIDI_CC_106_VALUE_53 = 53; //Bidirectional. Used to tell the Config Tool to include the uilleann regulators fingering pattern (used in a custom version of the code)	
    const MIDI_CC_106_VALUE_54 = 54; //from Config Tool. WARBL2 calibrate IMU
    const MIDI_CC_106_VALUE_55 = 55; //Bidirectional. WARBL2 settings array (for settings that are independent of mode)
    const MIDI_CC_106_VALUE_56 = 56; //Bidirectional. WARBL2 settings array (for settings that are independent of mode)
    const MIDI_CC_106_VALUE_57 = 57; //Bidirectional. WARBL2 settings array (for settings that are independent of mode)
    /* 58-59 unused */
    const MIDI_CC_106_VALUE_60 = 60; // from Config Tool. WARBL2 recenter yaw
    /* 61-69 unused */
    const MIDI_CC_106_VALUE_70 = 70; //from WARBL. WARBL2 battery voltage
	const MIDI_CC_106_VALUE_71 = 71; //from WARBL. WARBL2 charging status
	const MIDI_CC_106_VALUE_72 = 72; //from WARBL. WARBL2 BLE connection interval low byte
	const MIDI_CC_106_VALUE_73 = 73; //from WARBL. WARBL2 BLE connection interval high byte
	const MIDI_CC_106_VALUE_74 = 74; //from WARBL. WARBL2 battery percentage

    /* 75-99	unused -- can be used for WARBL2 */

; //Button Actions, see above 102 90/00
	const MIDI_CC_106_VALUE_100 = 100; //Bidirectional. button action 0 
	const MIDI_CC_106_VALUE_101 = 101; //Bidirectional. button action 1
	const MIDI_CC_106_VALUE_102 = 102; //Bidirectional. button action 2 
	const MIDI_CC_106_VALUE_103 = 103; //Bidirectional. button action 3 
	const MIDI_CC_106_VALUE_104 = 104; //Bidirectional. button action 4 
	const MIDI_CC_106_VALUE_105 = 105; //Bidirectional. button action 5 
	const MIDI_CC_106_VALUE_106 = 106; //Bidirectional. button action 6 
	const MIDI_CC_106_VALUE_107 = 107; //Bidirectional. button action 7 
	const MIDI_CC_106_VALUE_108 = 108; //Bidirectional. button action 8 
	const MIDI_CC_106_VALUE_109 = 109; //Bidirectional. button action 9 
	const MIDI_CC_106_VALUE_110 = 110; //Bidirectional. button action 10 
	const MIDI_CC_106_VALUE_111 = 111; //Bidirectional. button action 11
	const MIDI_CC_106_VALUE_112 = 112; //Bidirectional. button action 12
	const MIDI_CC_106_VALUE_113 = 113; //Bidirectional. button action 13
	const MIDI_CC_106_VALUE_114 = 114; //Bidirectional. button action 14
	const MIDI_CC_106_VALUE_115 = 115; //Bidirectional. button action 15
	const MIDI_CC_106_VALUE_116 = 116; //Bidirectional. button action 16
	const MIDI_CC_106_VALUE_117 = 117; //Bidirectional. button action 17
	const MIDI_CC_106_VALUE_118 = 118; //Bidirectional. button action 18
	const MIDI_CC_106_VALUE_119 = 119; //Bidirectional. button action 19
	const MIDI_CC_106_VALUE_120 = 120; //Bidirectional. button action 20
	const MIDI_CC_106_VALUE_121 = 121; //Bidirectional. button action 21
	const MIDI_CC_106_VALUE_122 = 122; //Bidirectional. button action 22
	const MIDI_CC_106_VALUE_123 = 123; //Bidirectional. button action 23
	const MIDI_CC_106_VALUE_124 = 124; //Bidirectional. button action 24
	const MIDI_CC_106_VALUE_125 = 125; //Bidirectional. button action 25
	const MIDI_CC_106_VALUE_126 = 126; //Bidirectional. button action 26
	const MIDI_CC_106_VALUE_127 = 127; //Bidirectional. button action 27
; //

const MIDI_CC_107 = 107; // From WARBL. Values 0-127	- MIDI byte 2
const MIDI_CC_108 = 108; // From WARBL. Values 0-127	- MIDI byte 3

const MIDI_CC_109 = 109; // From WARBL. Values as follows:
    const MIDI_CC_109_OFFSET = 200; //Value added to received CC 109, to distinguish them from those from CC_104
; //IMU Settings Array - see defines above
    const MIDI_CC_109_VALUE_0 = 0; // Bidirectional. Settings for current instrument: indicates IMUsettings[0] is about to be sent with CC 105. 
    const MIDI_CC_109_VALUE_1 = 1; // Bidirectional. Settings for current instrument: indicates IMUsettings[1] is about to be sent with CC 105. 
    const MIDI_CC_109_VALUE_2 = 2; // Bidirectional. Settings for current instrument: indicates IMUsettings[2] is about to be sent with CC 105. 
    const MIDI_CC_109_VALUE_3 = 3; // Bidirectional. Settings for current instrument: indicates IMUsettings[3] is about to be sent with CC 105. 
    const MIDI_CC_109_VALUE_4 = 4; // Bidirectional. Settings for current instrument: indicates IMUsettings[4] is about to be sent with CC 105. 
    const MIDI_CC_109_VALUE_5 = 5; // Bidirectional. Settings for current instrument: indicates IMUsettings[5] is about to be sent with CC 105. 
    const MIDI_CC_109_VALUE_6 = 6; // Bidirectional. Settings for current instrument: indicates IMUsettings[6] is about to be sent with CC 105. 
    const MIDI_CC_109_VALUE_7 = 7; // Bidirectional. Settings for current instrument: indicates IMUsettings[7] is about to be sent with CC 105. 
    const MIDI_CC_109_VALUE_8 = 8; // Bidirectional. Settings for current instrument: indicates IMUsettings[8] is about to be sent with CC 105. 
    const MIDI_CC_109_VALUE_9 = 9; // Bidirectional. Settings for current instrument: indicates IMUsettings[9] is about to be sent with CC 105. 
    const MIDI_CC_109_VALUE_10 = 10; // Bidirectional. Settings for current instrument: indicates IMUsettings[10] is about to be sent with CC 105. 
    const MIDI_CC_109_VALUE_11 = 11; // Bidirectional. Settings for current instrument: indicates IMUsettings[11] is about to be sent with CC 105. 
    const MIDI_CC_109_VALUE_12 = 12; // Bidirectional. Settings for current instrument: indicates IMUsettings[12] is about to be sent with CC 105. 
    const MIDI_CC_109_VALUE_13 = 13; // Bidirectional. Settings for current instrument: indicates IMUsettings[13] is about to be sent with CC 105. 
    const MIDI_CC_109_VALUE_14 = 14; // Bidirectional. Settings for current instrument: indicates IMUsettings[14] is about to be sent with CC 105. 
    const MIDI_CC_109_VALUE_15 = 15; // Bidirectional. Settings for current instrument: indicates IMUsettings[15] is about to be sent with CC 105. 
    const MIDI_CC_109_VALUE_16 = 16; // Bidirectional. Settings for current instrument: indicates IMUsettings[16] is about to be sent with CC 105. 
    const MIDI_CC_109_VALUE_17 = 17; // Bidirectional. Settings for current instrument: indicates IMUsettings[17] is about to be sent with CC 105. 
    const MIDI_CC_109_VALUE_18 = 18; // Bidirectional. Settings for current instrument: indicates IMUsettings[18] is about to be sent with CC 105. 
    const MIDI_CC_109_VALUE_19 = 19; // Bidirectional. Settings for current instrument: indicates IMUsettings[19] is about to be sent with CC 105. 
    const MIDI_CC_109_VALUE_20 = 20; // Bidirectional. Settings for current instrument: indicates IMUsettings[20] is about to be sent with CC 105. 
    const MIDI_CC_109_VALUE_21 = 21; // Bidirectional. Settings for current instrument: indicates IMUsettings[21] is about to be sent with CC 105. 
    const MIDI_CC_109_VALUE_22 = 22; // Bidirectional. Settings for current instrument: indicates IMUsettings[22] is about to be sent with CC 105. 
    const MIDI_CC_109_VALUE_23 = 23; // Bidirectional. Settings for current instrument: indicates IMUsettings[23] is about to be sent with CC 105. 
    const MIDI_CC_109_VALUE_24 = 24; // Bidirectional. Settings for current instrument: indicates IMUsettings[24] is about to be sent with CC 105. 
    const MIDI_CC_109_VALUE_25 = 25; // Bidirectional. Settings for current instrument: indicates IMUsettings[25] is about to be sent with CC 105. 
    const MIDI_CC_109_VALUE_26 = 26; // Bidirectional. Settings for current instrument: indicates IMUsettings[26] is about to be sent with CC 105. 
    const MIDI_CC_109_VALUE_27 = 27; // Bidirectional. Settings for current instrument: indicates IMUsettings[27] is about to be sent with CC 105. 
    const MIDI_CC_109_VALUE_28 = 28; // Bidirectional. Settings for current instrument: indicates IMUsettings[28] is about to be sent with CC 105. 
    const MIDI_CC_109_VALUE_29 = 29; // Bidirectional. Settings for current instrument: indicates IMUsettings[29] is about to be sent with CC 105. 
    const MIDI_CC_109_VALUE_30 = 30; // Bidirectional. Settings for current instrument: indicates IMUsettings[30] is about to be sent with CC 105. 
    const MIDI_CC_109_VALUE_31 = 31; // Bidirectional. Settings for current instrument: indicates IMUsettings[31] is about to be sent with CC 105. 
    const MIDI_CC_109_VALUE_32 = 32; // Bidirectional. Settings for current instrument: indicates IMUsettings[32] is about to be sent with CC 105. 
    /* 33-49	unused */

    //Half-holing 
    const MIDI_CC_109_VALUE_50 = 50  // Bidirectional. enable half-holing, 1
    const MIDI_CC_109_VALUE_51 = 51  // Bidirectional. enable half-holing, 1
    const MIDI_CC_109_VALUE_52 = 52  // Bidirectional. enable half-holing, 2
    const MIDI_CC_109_VALUE_53 = 53  // Bidirectional. enable half-holing, 3
    const MIDI_CC_109_VALUE_54 = 54  // Bidirectional. enable half-holing, 4
    const MIDI_CC_109_VALUE_55 = 55  // Bidirectional. enable half-holing, 5
    const MIDI_CC_109_VALUE_56 = 56  // Bidirectional. enable half-holing, 6
    const MIDI_CC_109_VALUE_57 = 57  // Bidirectional. enable half-holing, 7
    const MIDI_CC_109_VALUE_58 = 58  // Bidirectional. enable half-holing, 8
    /* 59	unused */
    const MIDI_CC_109_VALUE_60 = 60  // Bidirectional. disable half-holing, 1
    const MIDI_CC_109_VALUE_61 = 61  // Bidirectional. disable half-holing, 1
    const MIDI_CC_109_VALUE_62 = 62  // Bidirectional. disable half-holing, 2
    const MIDI_CC_109_VALUE_63 = 63  // Bidirectional. disable half-holing, 3
    const MIDI_CC_109_VALUE_64 = 64  // Bidirectional. disable half-holing, 4
    const MIDI_CC_109_VALUE_65 = 65  // Bidirectional. disable half-holing, 5
    const MIDI_CC_109_VALUE_66 = 66  // Bidirectional. disable half-holing, 6
    const MIDI_CC_109_VALUE_67 = 67  // Bidirectional. disable half-holing, 7
    const MIDI_CC_109_VALUE_68 = 68  // Bidirectional. disable half-holing, 8


    const MIDI_CC_109_VALUE_69 = 69  // From WARBL. send holes on MIDI_CC_114 / 115
    const MIDI_CC_109_VALUE_70 = 70  // From WARBL. send half-holes on MIDI_CC_114 / 115
/* 71-99	unused -- can be used to extend above array or for other variables */

    const MIDI_CC_109_VALUE_100 = 100; // Bidirectional. Indicates that WARBL2 custom fingering chart 1 is about to be sent on CC 105. Same command from WARBL indicates that all 256 messages were received.
    const MIDI_CC_109_VALUE_101 = 101; // Bidirectional. Indicates that WARBL2 custom fingering chart 2 is about to be sent on CC 105. 
    const MIDI_CC_109_VALUE_102 = 102; // Bidirectional. Indicates that WARBL2 custom fingering chart 3 is about to be sent on CC 105. 
    const MIDI_CC_109_VALUE_103 = 103; // Bidirectional. Indicates that WARBL2 custom fingering chart 4 is about to be sent on CC 105. 
	/* 104-126	unused */
    const MIDI_CC_109_VALUE_127 = 127; // From WARBL. Indicates button/gesture action will be sent on CC 105


const MIDI_CC_110 = 110; // From WARBL. Values 0-127	- firmware version
const MIDI_CC_111 = 111; // Bidirectional. Values 0-127	- note shift for mode 0
    const MIDI_CC_111_VALUE_109 = 109; // Bidirectional. Hidden sticks mode - Same for 112 and 113
const MIDI_CC_112 = 112; // Bidirectional. Values 0-127	- note shift for mode 1
const MIDI_CC_113 = 113; // Bidirectional. Values 0-127	- note shift for mode 2

const MIDI_CC_114 = 114; // From WARBL. Values 0-127	- highest 2 bytes of holeCovered (int indicating which holes are currently covered)
const MIDI_CC_115 = 115; // From WARBL. Values 0-127	- lowest 7 bytes of holeCovered
const MIDI_CC_116 = 116; // From WARBL. Values 0-127	- LSB of pressure
const MIDI_CC_117 = 117; // Bidirectional. Values 0-100	- vibrato depth (cents)
const MIDI_CC_118 = 118; // From WARBL. Values 0-127	- MSB of pressure
const MIDI_CC_119 = 119; // From WARBL. Values 0-127	- value of above variable indicated by CC 106

const MIDI_CC_123 = 123; // MIDI PANIC
//END of MIDI Numerical constants

//Human readable constants
/* Commands - VALUES ONLY */
const MIDI_SAVE_CALIB = MIDI_CC_102_VALUE_19; //from Config Tool. Save optical sensor calibration
const MIDI_EXIT_COMM_MODE = MIDI_CC_102_VALUE_104; //from Config Tool. exit communication mode (previously 102 99)
const MIDI_SAVE_AS_DEFAULTS_CURRENT = MIDI_CC_102_VALUE_123; //from Config Tool. save as defaults for current mode
const MIDI_SAVE_AS_DEFAULTS_ALL = MIDI_CC_102_VALUE_124; //from Config Tool. save as defaults for all instruments
const MIDI_RESTORE_FACTORY = MIDI_CC_102_VALUE_125; //from Config Tool. restore factory settings
const MIDI_ENTER_COMM_MODE = MIDI_CC_102_VALUE_126; //from Config Tool. enter communication mode
const MIDI_START_CALIB = MIDI_CC_102_VALUE_127; //from Config Tool. begin autocalibration

const MIDI_LEARN_INITIAL_NOTE_PRESS = MIDI_CC_106_VALUE_41; // from Config Tool. learn initial note on pressure
const MIDI_CALIB_BELL_SENSOR = MIDI_CC_106_VALUE_42; // from Config Tool. autocalibrate bell sensor only
const MIDI_LEARN_DRONES_PRESSURE = MIDI_CC_106_VALUE_43; // from Config Tool. learn drones on pressure
const MIDI_SAVE_CALIB_AS_FACTORY = MIDI_CC_106_VALUE_45; // from Config Tool. save current sensor calibration as factory calibration (this is on a special webpage for me to use when I first program WARBL)
const MIDI_CALIB_IMU = MIDI_CC_106_VALUE_54; //from Config Tool. WARBL2 calibrate IMU
const MIDI_CENTER_YAW = MIDI_CC_106_VALUE_60; // from Config Tool. WARBL2 recenter yaw

const MIDI_STICKS_MODE = MIDI_CC_111_VALUE_109; // Bidirectional. Hidden sticks mode - Same for 112 and 113

/* START - END Values */
const MIDI_CALIB_MSGS_START = MIDI_CC_102_VALUE_1; // Start of Calibration correction messages
const MIDI_CALIB_MSGS_END = MIDI_CC_102_VALUE_18; // End of Calibration correction messages
const MIDI_MAX_CALIB_MSGS_START = MIDI_CC_102_VALUE_20; // Start of Calibration max values reached messages
const MIDI_MAX_CALIB_MSGS_END = MIDI_CC_102_VALUE_28; // End of Calibration max values reached messages
const MIDI_FINGERING_PATTERN_MODE_START = MIDI_CC_102_VALUE_30; // Bidirectional. indicates that the next command will be the fingering pattern for instrument 1
const MIDI_FINGERING_PATTERN_START = MIDI_CC_102_VALUE_33; // Bidirectional. first fingering pattern is tin whistle
const MIDI_FINGERING_PATTERN_END = MIDI_CC_102_VALUE_57; // Bidirectional. Mep's Recorder
const MIDI_CURRENT_MODE_START = MIDI_CC_102_VALUE_60; // Bidirectional. current instrument (mode variable) is 0
const MIDI_PB_MODE_START = MIDI_CC_102_VALUE_70; // Bidirectional. Settings for current instrument: Pitchbend mode 0
const MIDI_BREATH_MODE_START = MIDI_CC_102_VALUE_80; // Bidirectional. Settings for current instrument: Breath mode 0
const MIDI_DEFAULT_MODE_START = MIDI_CC_102_VALUE_85; // Bidirectional. default instrument is 0 - (if Config Tool sends 85 to WARBL, WARBL sets current instrument as default)
const MIDI_GESTURE_START = MIDI_CC_102_VALUE_90; // Bidirectional. Sending data for click 1 (dropdown row 0)
const MIDI_CUST_FINGERING_PATTERN_START = MIDI_CC_102_VALUE_100; // Bidirectional. WARBL2 custom fingering chart 1
const MIDI_CUST_FINGERING_PATTERN_END = MIDI_CC_102_VALUE_103; // Bidirectional. WARBL2 custom fingering chart 4
const MIDI_ACTION_MIDI_START = MIDI_CC_102_VALUE_112; // Bidirectional. send midi note on/note off

const MIDI_PRESS_SELECT_VARS_START = MIDI_CC_104_VALUE_1; // Bidirectional. Settings for current instrument: indicates ""bag threshold"" is about to be sent with CC 105.
const MIDI_PRESS_SELECT_VARS_END = MIDI_CC_104_VALUE_12; // Bidirectional. Settings for current instrument: indicates "breath drop time" is about to be sent with CC 105.
const MIDI_ED_VARS_START = MIDI_CC_104_VALUE_13; // Bidirectional. Settings for current instrument: indicates ED[0] is about to be sent with CC 105. 
const MIDI_ED_VARS_END = MIDI_CC_104_VALUE_33; // Bidirectional. Settings for current instrument: indicates ED[20] is about to be sent with CC 105. 
const MIDI_SWITCHES_VARS_START = MIDI_CC_104_VALUE_40; // Bidirectional. Settings for current instrument: indicates that switches[0] is about to be sent with CC 105. 
const MIDI_SWITCHES_VARS_END = MIDI_CC_104_VALUE_55; // Bidirectional. Settings for current instrument: indicates that switches[13] is about to be sent with CC 105. UNUSED?
const MIDI_ED_VARS2_START = MIDI_CC_104_VALUE_70; // Bidirectional. Settings for current instrument: indicates ED[21] is about to be sent with CC 105. 
const MIDI_ED_VARS2_END = MIDI_CC_104_VALUE_97; // Bidirectional. Settings for current instrument: indicates ED[48] is about to be sent with CC 105. 
const MIDI_ED_VARS_NUMBER = MIDI_ED_VARS_END - MIDI_ED_VARS_START + 1; //ED array number of vars for the first slot
const MIDI_ED_VARS2_OFFSET = MIDI_ED_VARS2_START - MIDI_ED_VARS_NUMBER; //ED array index for 2nd slot of MIDI Msgs

const MIDI_ACTION_MIDI_CHANNEL_END = MIDI_CC_106_VALUE_15; //Bidirectional. MIDI channel 16

const MIDI_ENA_VIBRATO_HOLES_START = MIDI_CC_106_VALUE_20; //Bidirectional. enable vibrato hole, 0	
const MIDI_ENA_VIBRATO_HOLES_END = MIDI_CC_106_VALUE_28; //Bidirectional. enable vibrato hole, 8	
const MIDI_DIS_VIBRATO_HOLES_START = MIDI_CC_106_VALUE_30; //Bidirectional. disable vibrato hole, 0	
const MIDI_DIS_VIBRATO_HOLES_END = MIDI_CC_106_VALUE_38; //Bidirectional. disable vibrato hole, 8	
const MIDI_WARBL2_SETTINGS_START = MIDI_CC_106_VALUE_55; //Bidirectional. WARBL2 settings array (for settings that are independent of mode)
const MIDI_WARBL2_SETTINGS_END = MIDI_CC_106_VALUE_74; //Bidirectional. WARBL2 settings array (for settings that are independent of mode)
const MIDI_BUTTON_ACTIONS_START = MIDI_CC_106_VALUE_100; //Bidirectional. button action 0 

const MIDI_HALF_HOLE_ENABLED_START = MIDI_CC_109_VALUE_50                                 // Beginning of half-hole enabled
const MIDI_HALF_HOLE_ENABLED_END = MIDI_CC_109_VALUE_58                                   // End of half-hole enabled
const MIDI_HALF_HOLE_ENABLED_OFFSET_START =  (MIDI_CC_109_OFFSET + MIDI_HALF_HOLE_ENABLED_START)  // Beginning of half-hole enabled
const MIDI_HALF_HOLE_ENABLED_OFFSET_END =  (MIDI_CC_109_OFFSET + MIDI_HALF_HOLE_ENABLED_END)      // End of half-hole enabled

const MIDI_HALF_HOLE_DISABLED_START = MIDI_CC_109_VALUE_60                                 // Beginning of half-hole enabled
const MIDI_HALF_HOLE_DISABLED_END = MIDI_CC_109_VALUE_68                                   // End of half-hole enabled
const MIDI_HALF_HOLE_DISABLED_OFFSET_START =  (MIDI_CC_109_OFFSET + MIDI_HALF_HOLE_DISABLED_START)  // Beginning of half-hole enabled
const MIDI_HALF_HOLE_DISABLED_OFFSET_END =  (MIDI_CC_109_OFFSET + MIDI_HALF_HOLE_DISABLED_END)      // End of half-hole enabled


const MIDI_CUSTOM_CHARTS_START = MIDI_CC_109_VALUE_100; //Beginning of WARBL2 CustomCharts
const MIDI_CUSTOM_CHARTS_END = MIDI_CC_109_VALUE_103; //End of WARBL2 CustomCharts
const MIDI_CUSTOM_CHARTS_OFFSET_START = MIDI_CC_109_OFFSET + MIDI_CUSTOM_CHARTS_START; //Beginning of WARBL2 CustomCharts
const MIDI_CUSTOM_CHARTS_OFFSET_END = MIDI_CC_109_OFFSET + MIDI_CUSTOM_CHARTS_END; //End of WARBL2 CustomCharts

/* Various single Values */
const MIDI_MOMENTARY_OFF = MIDI_CC_102_VALUE_117; // Bidirectional. momentary off
const MIDI_MOMENTARY_ON = MIDI_CC_102_VALUE_118; // Bidirectional. momentary on

const MIDI_LEARNED_PRESS_LSB = MIDI_CC_104_VALUE_34; // Bidirectional. Settings for current instrument: indicates that lsb of learned note trigger pressure is about to be sent on CC 105
const MIDI_LEARNED_PRESS_MSB = MIDI_CC_104_VALUE_35; // Bidirectional. Settings for current instrument: indicates that msb of learned note trigger pressure is about to be sent on CC 105
const MIDI_BEND_RANGE = MIDI_CC_104_VALUE_61; // Bidirectional. Settings for current instrument: MIDI bend range is about to be sent on CC 105
const MIDI_MIDI_CHANNEL = MIDI_CC_104_VALUE_62; // Bidirectional. Settings for current instrument: MIDI channel is about to be sent on CC 105

const MIDI_STARTUP_CALIB = MIDI_CC_106_VALUE_39; //Bidirectional. calibrate at startup
const MIDI_USE_LEARNED_CALIB = MIDI_CC_106_VALUE_40; //Bidirectional. use learned calibration
const MIDI_BLE_INTERVAL_LSB = MIDI_CC_106_VALUE_72; //from WARBL. WARBL2 BLE connection interval low byte
const MIDI_BLE_INTERVAL_MSB = MIDI_CC_106_VALUE_73; //from WARBL. WARBL2 BLE connection interval high byte

const MIDI_SEND_HOLES = MIDI_CC_109_VALUE_69                // from Config Tool. WARBL2 send holes on 114 / 115
const MIDI_SEND_HALF_HOLES = MIDI_CC_109_VALUE_70           // from Config Tool. WARBL2 send half-holes on 114 / 115
//END of Human readable constants
//END of constants To Be Sync'd with Defines.h in firmware


/* Config tool only Constants */
const MIDI_IMU_SETTINGS_START =  MIDI_CC_109_VALUE_0; // Bidirectional. Settings for current instrument: indicates IMUsettings[0] is about to be sent with CC 105. 
const MIDI_IMU_SETTINGS_END =  MIDI_CC_109_VALUE_32; // Bidirectional. Settings for current instrument: indicates IMUsettings[32] is about to be sent with CC 105. 

const MIDI_CUSTOM_CHARTS_RCVD = MIDI_CC_109_VALUE_100 ////from WARBL. WARBL2 Custom fingering charts - indicate success
const MIDI_EXPRESSION_DEPTH = MIDI_CC_104_VALUE_14; // Bidirectional. Settings for current instrument: indicates ED[1] is about to be sent with CC 105. 
const MIDI_SEND_PRESSURE = MIDI_CC_104_VALUE_15; // Bidirectional. Settings for current instrument: indicates ED[2] is about to be sent with CC 105. 
const MIDI_CURVE = MIDI_CC_104_VALUE_16; // Bidirectional. Settings for current instrument: indicates ED[3] is about to be sent with CC 105. 
const MIDI_PRESSURE_CHANNEL = MIDI_CC_104_VALUE_17; // Bidirectional. Settings for current instrument: indicates ED[4] is about to be sent with CC 105. 
const MIDI_PRESSURE_CC = MIDI_CC_104_VALUE_18; // Bidirectional. Settings for current instrument: indicates ED[5] is about to be sent with CC 105. 
const MIDI_INPUT_PRESSURE_MIN = MIDI_CC_104_VALUE_19; // Bidirectional. Settings for current instrument: indicates ED[6] is about to be sent with CC 105. 
const MIDI_INPUT_PRESSURE_MAX = MIDI_CC_104_VALUE_20; // Bidirectional. Settings for current instrument: indicates ED[7] is about to be sent with CC 105. 
const MIDI_OUTPUT_PRESSURE_MIN = MIDI_CC_104_VALUE_21; // Bidirectional. Settings for current instrument: indicates ED[8] is about to be sent with CC 105. 
const MIDI_OUTPUT_PRESSURE_MAX = MIDI_CC_104_VALUE_22; // Bidirectional. Settings for current instrument: indicates ED[9] is about to be sent with CC 105. 
const MIDI_DRONES_ON_COMMAND = MIDI_CC_104_VALUE_23; // Bidirectional. Settings for current instrument: indicates ED[10] is about to be sent with CC 105. 
const MIDI_DRONES_ON_CHANNEL = MIDI_CC_104_VALUE_24; // Bidirectional. Settings for current instrument: indicates ED[11] is about to be sent with CC 105. 
const MIDI_DRONES_ON_BYTE2 = MIDI_CC_104_VALUE_25; // Bidirectional. Settings for current instrument: indicates ED[12] is about to be sent with CC 105. 
const MIDI_DRONES_ON_BYTE3 = MIDI_CC_104_VALUE_26; // Bidirectional. Settings for current instrument: indicates ED[13] is about to be sent with CC 105. 
const MIDI_DRONES_OFF_COMMAND = MIDI_CC_104_VALUE_27; // Bidirectional. Settings for current instrument: indicates ED[14] is about to be sent with CC 105. 
const MIDI_DRONES_OFF_CHANNEL = MIDI_CC_104_VALUE_28; // Bidirectional. Settings for current instrument: indicates ED[15] is about to be sent with CC 105. 
const MIDI_DRONES_OFF_BYTE2 = MIDI_CC_104_VALUE_29; // Bidirectional. Settings for current instrument: indicates ED[16] is about to be sent with CC 105. 
const MIDI_DRONES_OFF_BYTE3 = MIDI_CC_104_VALUE_30; // Bidirectional. Settings for current instrument: indicates ED[17] is about to be sent with CC 105. 
const MIDI_DRONES_CONTROL_MODE = MIDI_CC_104_VALUE_31; // Bidirectional. Settings for current instrument: indicates ED[18] is about to be sent with CC 105. 
const MIDI_DRONES_PRESSURE_LOW_BYTE = MIDI_CC_104_VALUE_32; // Bidirectional. Settings for current instrument: indicates ED[19] is about to be sent with CC 105. 
const MIDI_DRONES_PRESSURE_HIGH_BYTE = MIDI_CC_104_VALUE_33; // Bidirectional. Settings for current instrument: indicates ED[20] is about to be sent with CC 105. 
const MIDI_VELOCITY_CURVE = MIDI_CC_104_VALUE_82; // Bidirectional. Settings for current instrument: indicates ED[33] is about to be sent with CC 105. 
const MIDI_EXPRESSION_MIN = MIDI_CC_104_VALUE_85; // Bidirectional. Settings for current instrument: indicates ED[36] is about to be sent with CC 105. 
const MIDI_EXPRESSION_MAX = MIDI_CC_104_VALUE_86; // Bidirectional. Settings for current instrument: indicates ED[37] is about to be sent with CC 105. 

const MIDI_SLIDE_LIMIT_MAX = MIDI_CC_104_VALUE_87; // Bidirectional. Settings for current instrument: indicates ED[38] is about to be sent with CC 105. 

const MIDI_SEND_ROLL = MIDI_CC_109_VALUE_0; // Bidirectional. Settings for current instrument: indicates IMUsettings[0] is about to be sent with CC 105. 
const MIDI_SEND_PITCH = MIDI_CC_109_VALUE_1; // Bidirectional. Settings for current instrument: indicates IMUsettings[1] is about to be sent with CC 105. 
const MIDI_SEND_YAW = MIDI_CC_109_VALUE_2; // Bidirectional. Settings for current instrument: indicates IMUsettings[2] is about to be sent with CC 105. 
const MIDI_SEND_CENTER_ROLL = MIDI_CC_109_VALUE_3; // Bidirectional. Settings for current instrument: indicates IMUsettings[3] is about to be sent with CC 105. 
const MIDI_SEND_CENTER_YAW = MIDI_CC_109_VALUE_4; // Bidirectional. Settings for current instrument: indicates IMUsettings[4] is about to be sent with CC 105. 

const MIDI_IMU_CHANNEL_START = MIDI_CC_109_VALUE_17; // Bidirectional. Settings for current instrument: indicates IMUsettings[17] is about to be sent with CC 105. 
const MIDI_IMU_CC_START = MIDI_CC_109_VALUE_19; // Bidirectional. Settings for current instrument: indicates IMUsettings[19] is about to be sent with CC 105. 

const MIDI_AUTOCENTER_YAW = MIDI_CC_109_VALUE_23; // Bidirectional. Settings for current instrument: indicates IMUsettings[23] is about to be sent with CC 105. 
const MIDI_Y_SHAKE_PITCHBEND = MIDI_CC_109_VALUE_24; // Bidirectional. Settings for current instrument: indicates IMUsettings[24] is about to be sent with CC 105. 
const MIDI_AUTOCENTER_YAW_INTERVAL = MIDI_CC_109_VALUE_25; // Bidirectional. Settings for current instrument: indicates IMUsettings[25] is about to be sent with CC 105. 
const MIDI_PITCH_REGISTER = MIDI_CC_109_VALUE_26; // Bidirectional. Settings for current instrument: indicates IMUsettings[26] is about to be sent with CC 105. 
const MIDI_Y_PITCHBEND_DEPTH = MIDI_CC_109_VALUE_27; // Bidirectional. Settings for current instrument: indicates IMUsettings[27] is about to be sent with CC 105. 
const MIDI_PITCH_REGISTER_INPUT_MIN = MIDI_CC_109_VALUE_28; // Bidirectional. Settings for current instrument: indicates IMUsettings[28] is about to be sent with CC 105. 
const MIDI_PITCH_REGISTER_INPUT_MAX = MIDI_CC_109_VALUE_29; // Bidirectional. Settings for current instrument: indicates IMUsettings[29] is about to be sent with CC 105. 

const MIDI_PITCH_REGISTER_NUMBER = MIDI_CC_109_VALUE_30; // Bidirectional. Settings for current instrument: indicates IMUsettings[30] is about to be sent with CC 105. 
const MIDI_Y_PITCHBEND_MODE = MIDI_CC_109_VALUE_31; // Bidirectional. Settings for current instrument: indicates IMUsettings[31] is about to be sent with CC 105. 


//END Config tool only Constants


/* END of MIDI Config Tool Constants */


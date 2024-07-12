
/*
* Temp half holing definitions and globals
*
*/

#define TONEHOLE_SENSOR_NUMBER                    9 //Number of thone hole sensors

//For new debounce and transition filter
#define DEBOUNCE_INTERVAL_MULTIPLIER 0.1f // Portion of transient note delay to be added for each semitone interval
#define DEBOUNCE_POPCOUNT_MULTIPLIER 0.25f // Portion of transient note delay to be added for each changed finger
#define DEBOUNCE_HALFHOLE_ENTER_MULTIPLIER 2.0f // Portion of transient note delay to be added when a tonehole is going to half-hole position
#define DEBOUNCE_HALFHOLE_EXIT_MULTIPLIER 1.5f // Portion of transient note delay to be added when a tonehole is leaving half-hole position
#define DEBOUNCE_DELAY_REDUCE 0.25f // Factor that determines how much delay we subtract from current debounce timer, if no change is seen


//Hole status for holeStatus()
#define HOLE_STATUS_OPEN 0
#define HOLE_STATUS_CLOSED 1
#define HOLE_STATUS_HALF 2

#define THUMB_HOLE                8
#define R4_HOLE                   1
#define R3_HOLE                   2

#define HALF_HOLE_BIT_OFFSET      TONEHOLE_SENSOR_NUMBER //The initial bit for setting the bit of half hole in holeCovered. See getFingers() and getHalfHoleShift();

#define HOLE_COVERED_OFFSET      50 //For determining hole closed // from original firmware
#define HOLE_OPEN_OFFSET         54  //For determining hole open // from original firmware

#define HALF_HOLE_HYSTERESIS     20  //We modify the half-hole window by this quantity in debouncing half-hole-enabled holes

#define HALF_HOLE_LOW_WINDOW_PERC  55 //Default percentage of toneHolecCovered dedicated to lower window (hole open) - the higher this value, the bigger the window
#define HALF_HOLE_HIGH_WINDOW_PERC 15 //Default percentage of toneHolecCovered dedicated to upper window (hole closed) - the lower this value, the bigger the window

//Outside these values, the half hole detection doesn't work well
#define HALF_HOLE_LOW_MIN_PERC 35 //Min percentage of toneHolecCovered dedicated to lower window (hole open)
#define HALF_HOLE_LOW_MAX_PERC 60 //Max percentage of toneHolecCovered dedicated to lower window (hole open)
#define HALF_HOLE_HIGH_MIN_PERC 10 //Min percentage of toneHolecCovered dedicated to upper window (hole closed)
#define HALF_HOLE_HIGH_MAX_PERC 35//Max percentage of toneHolecCovered dedicated to upper window (hole closed)


//Baseline auto-calibration was switched off because it seems redundant on WARBL2: light conditions are successfully filtered out by ATMega fw
#define BASELINE_AUTO_CALIBRATION   false
#if BASELINE_AUTO_CALIBRATION
#define HALF_HOLE_CALIB_OFFSET  3.0 //Correction factor based on baseline moving average difference with calibration baseline average
#define BASELINE_AVRG_INTERVAL  750 //ms/ticks for calculating baseline moving average / Window size
#define BASELINE_AVRG_SPEED     0.2 //0-1 the lower the slower the moving average is
#define BASELINE_MACRO_FACTOR  10.0 //Number of decimals to be sent to the Config tool - Debug
#endif

#if BASELINE_AUTO_CALIBRATION
//Baseline Auto calibration
struct auto_calibration_t {
    unsigned long timer = 0;   //to keep track of the last time we sent a baseline message
    int toneholeBaselineCurrent[TONEHOLE_SENSOR_NUMBER] = { 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024 };            //baseline (uncovered) hole tonehole sensor readings
    float baselineAverage = 0;
    float baselineCurrentAverage = 0;
    float baselinePreviousAverage = 0;
    int maxBaseline = 30;
};
#endif

//Struct to contains all the variables related to debounceFingers
struct transition_filter_t {

    byte tempNewNote = 127; //Holds the pending note during a fingering transition
    byte prevPendingNote = 127; //Holds the pending note during a fingering transition
    
    unsigned int prevHoleCovered = 1; // So we can track changes.
    unsigned int newNoteHoleCovered = 1; //Keeps track of holes for currently debounced note - newNote

    unsigned long timer = 0; //Starting timestamp of transition timer
    unsigned long delta = 0; //Time elapsed since timer started
    byte iterations = 0; //Number of debouncing runs for the current transition
    bool timing = false; //Is the timer active?

    byte settingsDelay = 0;  // Small delay for filtering out transient notes ** was transientFilterDelay**
    unsigned long currentDelay = 0; //Current timer duration, ms ** was transientFilter**
    unsigned long additionalDelay = 0; //Amount added to timer after start

    byte prevHoleStatus[TONEHOLE_SENSOR_NUMBER] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 }; //This stores previous hole status for Transition Filter
    byte prevPopcount = 0; //Nunber of fingers changed in previous debouncing iteration
    byte prevInterval = 0; //Interval between current note and tempNewNote in previous iteration

};

//HALF HOLE DETECTION
struct half_hole_detection_t {

    unsigned int prevHoleCovered = 1; // So we can track changes.
    byte prevHoleStatus[TONEHOLE_SENSOR_NUMBER] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };  //This stores previous hole status for Half-hole detection

    float lowWindowPerc = (float) HALF_HOLE_LOW_WINDOW_PERC / 100.0f; //Percentage of toneHoleCovered dedicated to open hole
    float highWindowPerc = (float) HALF_HOLE_HIGH_WINDOW_PERC / 100.0f; //Percentage of toneHoleCovered dedicated to closed hole

#if BASELINE_AUTO_CALIBRATION
    float correction = 0.8;
#endif

};

//Prototypes

/* Returns status for the hole. 
 * If finger pattern is provided, it bases its return value on it
 * otherwise, it is based on current sensor readings
 */
byte holeStatus(byte hole, unsigned int fingerPattern = 0xFFFFFFFF);



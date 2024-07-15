#include <stdint.h>

/*
* Temp half holing definitions and globals
*
*/

#define TONEHOLE_SENSOR_NUMBER                    9 //Number of thone hole sensors

//For new debounce and transition filter
#define DEBOUNCE_INTERVAL_MULTIPLIER 0.1f // Portion of transient note delay to be added for each semitone interval
#define DEBOUNCE_POPCOUNT_MULTIPLIER 0.25f // Portion of transient note delay to be added for each changed finger
#define DEBOUNCE_HALFHOLE_ENTER_MULTIPLIER 2.0f // Portion of transient note delay to be added when a tonehole is going to half-hole position
#define DEBOUNCE_HALFHOLE_EXIT_MULTIPLIER 2.0f // Portion of transient note delay to be added when a tonehole is leaving half-hole position
#define DEBOUNCE_DELAY_REDUCE 0.20f // Factor that determines how much delay we subtract from current debounce timer, if no change is seen


//Hole status for holeStatus()
#define HOLE_STATUS_OPEN 0
#define HOLE_STATUS_CLOSED 1
#define HOLE_STATUS_HALF 2
#define HOLE_STATUS_ND 3

#define THUMB_HOLE                8
#define L1_HOLE                   7
#define L2_HOLE                   6
#define L3_HOLE                   5
#define R1_HOLE                   4
#define R2_HOLE                   3
#define R3_HOLE                   2
#define R4_HOLE                   1
#define BELL_HOLE                 0


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




//Tonehole autocalibration
#define AUTO_CALIB_AVRG_SPEED     0.15f //0-1 the lower the slower the moving average is
#define AUTO_CALIB_INTERVAL  500 //ms/ticks for calculating baseline moving average / Window size
#define AUTO_CALIB_MIN_SAMPLES  400 //number of samples to calculate the new average value

//Baseline Auto calibration
struct auto_calibration_t {
    bool enabled = false; //If it's active
    unsigned long timer = 0;   //to keep track of the last time we sent a baseline message
    float currentAverage[TONEHOLE_SENSOR_NUMBER] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };;
    float previousAverage[TONEHOLE_SENSOR_NUMBER] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };;
    unsigned long toneholeCoveredCurrentSum[TONEHOLE_SENSOR_NUMBER] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 }; //mean value when hole is considered closed
    unsigned long toneholeCoveredSampleCounter[TONEHOLE_SENSOR_NUMBER] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 }; //count # of samles taken in the interval
    SemaphoreHandle_t mutex = xSemaphoreCreateMutex();  // Semephore for sending MIDI couplets, in case there are multiple threads sending.

};

struct fingering_pattern_t {
    uint16_t holes = 0; //Use this for toneHoles open/closed
    uint16_t halfHoles = 0; //Use this for thoneHoles half-covered
};

union fingering_pattern_union_t {
    uint32_t holeCovered; //Use this for generic fingering change trigger
    fingering_pattern_t fp; //Contains fingering Pattern
};

//Struct to contains all the variables related to debounceFingers
struct transition_filter_t {

    byte tempNewNote = 127; //Holds the pending note during a fingering transition
    byte prevPendingNote = 127; //Holds the pending note during a fingering transition
    
    fingering_pattern_union_t prevHoleCovered = {0}; // So we can track changes.
    fingering_pattern_union_t newNoteHoleCovered = {0}; //Keeps track of holes for currently debounced note - newNote

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

    unsigned int halfHoleSelector = 0; //toneHole with enabled half-holing
    
    fingering_pattern_union_t prevHoleCovered = {0}; // So we can track changes.
    byte prevHoleStatus[TONEHOLE_SENSOR_NUMBER] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };  //This stores previous hole status for Half-hole detection

    float lowWindowPerc = (float) HALF_HOLE_LOW_WINDOW_PERC / 100.0f; //Percentage of toneHoleCovered dedicated to open hole
    float highWindowPerc = (float) HALF_HOLE_HIGH_WINDOW_PERC / 100.0f; //Percentage of toneHoleCovered dedicated to closed hole

};

//Prototypes

/* Returns status for the hole. 
 * If finger pattern is provided, it bases its return value on it
 * otherwise, it is based on current sensor readings
 */

byte holeStatus(byte hole, fingering_pattern_union_t fingerPattern = {0xFFFFFFFF});



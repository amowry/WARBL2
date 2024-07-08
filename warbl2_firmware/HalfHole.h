
//EEPROM TEMPORARY ADDRESSES
#define EEPROM_HALF_HOLE_BUFFER_SIZE            8000 //Current transient max value

#define DEBUG_HH    0
// #define DEBUG_AUTO_CALIB
//#define PRESSURE_DEBUG
//#define DEBUG_FINGERING

#define TONEHOLE_SENSOR_NUMBER                    9 //Number of thone hole sensors

#define THUMB_HOLE                8
#define R4_HOLE                   1
#define R3_HOLE                   2

#define HALF_HOLE_BIT_OFFSET      TONEHOLE_SENSOR_NUMBER //The initial bit for setting the bit of half hole in holeCovered. See getFingers() and getHalfHoleShift();

//Thumb hole modes - When active, in getNote it raises the equivalent position with thumb hole open or closed
#define HALF_THUMBHOLE_RAISE_OPEN   0 //Half thumbhole raises one octave the open thumb hole position
#define HALF_THUMBHOLE_RAISE_CLOSED 1 //Half thumbhole raises one octave the closed thumb hole position

#define HOLE_COVERED_OFFSET      50 //For determining hole closed // from original firmware
#define HOLE_OPEN_OFFSET         54  //For determining hole open // from original firmware

#define HALF_HOLE_BUFFER_SIZE    30 //Default additional transitional delay for special gestures
#define HALF_HOLE_UPPER_OFFSET   HOLE_COVERED_OFFSET - 4 //Negative numbers raise the upper limit closer to toneholeCovered values
#define HALF_HOLE_WINDOW_SIZE    HOLE_COVERED_OFFSET*3.4  //Size of the window for Half hole detection

#define HALF_HOLE_HIGH_WINDOW_PERC  0.12f //Percentage of toneHolecCovered dedicated to upper window (hole closed)
#define HALF_HOLE_LOW_WINDOW_PERC  0.50f //Percentage of toneHolecCovered dedicated to lower window (hole open)

#define HALF_HOLE_CALIB_OFFSET  3.0 //Correction factor based on baseline moving average difference with calibration baseline average
#define BASELINE_AVRG_INTERVAL  750 //ms/ticks for calculating baseline moving average / Window size
#define BASELINE_AVRG_SPEED     0.2 //0-1 the lower the slower the moving average is
#define BASELINE_MACRO_FACTOR  10.0 //Number of decimals to be sent to the Config tool - Debug

//Baseline Auto calibration
unsigned long autoCalibrationTimer = 0;   //to keep track of the last time we sent a baseline message
int toneholeBaselineCurrent[TONEHOLE_SENSOR_NUMBER] = { 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024 };            //baseline (uncovered) hole tonehole sensor readings
int toneholeBaselinePrevious[TONEHOLE_SENSOR_NUMBER] = { 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024 };            //baseline (uncovered) hole tonehole sensor readings
float baselineAverage = 0;
float baselineCurrentAverage = 0;
float baselinePreviousAverage = 0;
int maxBaseline = 30;

//HALF HOLE DETECTION
struct half_hole_detection_t {

    float correction = 0.8;
};

//Globals
half_hole_detection_t halfHole;      //Half-Hole Detecion
bool toneholeHalfCovered[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };         //storage for half-covered tonehole


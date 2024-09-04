# WARBL2 Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).
## [Unreleased]


## [Released]

##[4.2] - 2024-9-4


### Changed

- Changed the "basic" slide behavior so that there be will some slide even if the predicted slide range is greater than the MIDI pitchbend range.

- It is now possible to enter either zeros or blanks in a custom fingering chart to produce silent notes. Also MIDI note 127 will produce a "blank" position that has no affect, in other words the previous note will continue to play.

- Watchdog timer is now started at the beginning of setup() rather then the end, to catch hangs during initialization of peripherals.
  
- Now resetting the ATmega during setup in case only the NRF52840 has been reset.

- Fixed Config Tool connection behavior when WARBL2 is connected by both BLE and USB.

- Changed Gaita and Gaita extended note for 1 101 0001 from 71 to 68.

- Fixed bug in transient delay for closed pipe (all holes and bell sensor covered).

- Fixed bugs with pitch expression (the pitch range was not centered on the pressure range for each note).

  

### Added

- Added double-click option for button behavior.

- Added a hidden "sticks" mode where WARBL acts as a MIDI drumstick. Turned on by setting transpose to -18 in the Config Tool and then clicking "Auto-calibrate bell sensor only" within 10 seconds. Uses yaw mapping settings to map yaw to MIDI notes.

- Momentary button behavior is now enabled for sending CC messages with a button click. If the momentary switch is turned on, the button will send a CC with a value of 0 when released, for controlling CC on/off "switches", e.g. CC 64-69. When pressed, the button will send whatever CC value is currently set for the behavior.




##[4.1] - 2024-5-1

### Changed

First release of the WARBL2 firmware. Additions from the firmware for the orginal WARBL include functionality for BLE, the IMU, battery management separate processor (and associated firmware) for reading the tonehole sensors, and a new custom fingering chart feature.

### Added

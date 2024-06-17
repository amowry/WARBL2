# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).
## [Unreleased]


##[4.2] - 2024-5-17


### Changed

- Changed the "basic" slide behavior so that there will some slide even if the predicted slide range is greter than the MIDI pitchbend range.

- It is now possible to enter either zeros or blanks in a custom fingering chart to produce silent notes.

- Watchdog timer is now started at the beginning of setup() rather then the end, to catch hangs during initialization of peripherals.
  
- Now resetting the ATmega during setup in case only the NRF52840 has been reset.

- Added a hidden "sticks" mode where WARBL acts as a MIDI drumstick. Turned on by setting transpose to -18 in the Config Tool and then clicking "Auto-calibrate bell sensor only" within 10 seconds. Uses yaw mapping settings to map yaw to MIDI notes.

  

### Added

## [Released]

##[4.1] - 2024-5-1

### Changed

First release of the WARBL2 firmware. Additions from the firmware for the orginal WARBL include functionality for BLE, the IMU, separate processor for reading the tonehole sensors, and a new custom fingering chart feature.

### Added

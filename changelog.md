# WARBL2 Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).
## [Unreleased]

## [4.5] - 2025-3-3

### Changed

- Fixed a bug where pitchbend wasn't being reset to zero after IMU pitchbend was turned off.

- Fixed Bansuri fingering chart.

- For constistency, all buttons actions now cause the LED to flash green (except when the "Momentary" switch is turned on).

- Reorganization of the Configuration Tool: Import/Export buttons are now at the top so it is more clear that they apply only to the current instrument. Pressure and IMU mapping panels are now separate, to make access easier.


### Added

- Halfhole pitchbend feature where covering a hole halfway will lower the pitch one semitone (assuming covering the hole completely would play a note a whole step lower). The pitch can be changed either by using pitchbend or by changing the MIDI note. If the thumb is being used to control the register, it can now be covered halfwy to play three registers (see documentation for controlling the order of the registers).

- The "Thumb register" setting now works with custom charts by only looking at the first half of the chart (the half with the thumb hole open). You still need to enter placeholder values for the bottom half of the custom chart but it doesn't matter what the values are.
  
- New button/gesture action: "Register hold" feature that temporarily disables overblowing. This allows for finer control of dynamics in the current register without having overblowing interfere.

- Moving the "Auto power off time" slider to the max now represents an "infinity" setting, preventing the WARBL from shutting down automatically.

## [Released]

## [4.4] - 2025-1-28

### Changed

- Medieval pipes fingering X XOX OOOO will now play an F natural (77) rather than an F# (78).

- Fixes to tonehole sensor calibration routine.

- Fixed mistake in uilleann pipes "custom" vibrato.

- Fix that prevents the shake pressure IMU feature from modifying if the input pressure is zero.

- Fixed a bug where the vibrato holes selection wasn't being sent properly between the WARBL and the Configuration Tool.

### Added

- Added mapping of IMU roll/elevation/yaw to pitchbend.
  
- Added an "register semitones" setting to allow interval shifts other than an octave by overblowing, using the thumb, or the bell sensor (useful for emulating wind instruments that overblow odd intervals, or for overblowing etc. to play accidentals).

## [4.3] - 2024-10-4

### Changed

- The "pitch expression" functionality for mapping pressure to pitchbend is now much more sophisticated. If not using overblowing, you can now separately map a low pressure range and high pressure range to pitchbend while optionally maintaining a "stable" middle range where no pitchbend mapping occurs.

### Added

- It is now possible to use IMU shake to modify breath pressure mapping (this is located in the pressure mapping panels for CC, Channel Pressure, and Key Pressure).

## [Released]

## [4.2] - 2024-9-4


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




## [4.1] - 2024-5-1

### Changed

First release of the WARBL2 firmware. Additions from the firmware for the orginal WARBL include functionality for BLE, the IMU, battery management, separate processor (and associated firmware) for reading the tonehole sensors, and a new custom fingering chart feature.

### Added

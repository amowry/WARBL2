# WARBL2 Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).
## [Unreleased]

## [4.6] - 2026-4-2

### Changed

- The Configuration Tool now has a built-in SoundFont player with improved basic audio for testing purposes.
  
- The three "Instruments" are now called "Presets" for clarity, both in the Config Tool and the firmware.
  
- Preset import and export now works on iOS (WARBL app) as well as other platforms. The user is given the option of where to save export files (typically the Files app).
  
- "Yaw" is now much more useful because it is not affected by roll. The body-axis vector is now projected onto a horizontal frame to compute compass heading of the long body axis. This is now used as "yaw" so that it is entirely independent of "roll", which as we use it is defined as rotation around the long body axis.
  
- Fixed a bug where neither "Sip Mouthpiece" nor "Shake" were correctly able to send MIDI messages (channel, byte 2, and byte 3 weren't being sent from the Configuration Tool to the WARBL2).
  
- Reset the long-press counter when any button is released so that a long press of button 2 isn't registered if it was only being held in combination with a click of 1 or 3.
  
- Fixed longstanding bug where certain button actions didn't work properly because of an incorrect check for momentary mode. This particularly affected instruments (presets) 2 and 3.
  
- "Control register with elevation" now uses the "Register semitones" setting in the "Note trigger and register control" panel to determine the number of semitones in each register. This allows using elevation for a more versatile transpose method.
  
- Fixed bug where the WARBL would send CC messages back when told by CC to switch presets, even when not connected to the Config Tool. When not connected, the WARBL still responds to CC messages on channel 7 but won't send any replies.
  
- Fixed calculation of BLE connection interval and added request for renegotiation if the initial interval is greater than 15 ms.
  
- Fixed bug where the selection of holes for half hole pitchbend was ignored if "Use MIDI note" switch was turned on.
  
- Fixed bug where thumb half hole was ignored if another tone hole was also half covered.
  
- Fixed bug in Config Tool where the tone holes selected for half-holing were not synced properly with the WARBL.
  
- Added separate controls for thumb half-hole height and finger speed.
  
- Fixed bug where finger-sensing distance wasn't being considered when exiting the half-hole region.
  
- The WARBL will now always send CC messages in the Config Tool range regardless of the USB/BLE destination setting, so the user can't get locked out of the Config Tool by changing this setting.
  
- Added checksum for SPI transfer of tone hole sensor values from the ATMega.

- Now using SPIM2 instead of SPIM3 because it seems more stable. This also saves 0.15 ma power, for a total of 0.95 (+ 11% battery life) with this update if the bell sensor isn't used.

### Added

- Baroque recorder fingering that leverages half holing for the thumb and fingers to be as close as possible to the acoustic instrument.
  
- It is now possible to send Program Changes messages (1, 2, or 3) on any channel to tell the WARBL to change to preset (instrument) 1, 2, or 3 respectively. The WARBL will not send any messages back unless it is currently connected to the Config Tool (in that case it will be sending CC messages on channel 7 to keep the Config Tool in sync).
  
- Button action for toggling between vibrato/slide modes 1 and 4 (slide and legato slide).
  
- Option to not use the bell sensor. This saves 0.7 mA or about 8% of battery life. If an uilleann pipes chart is chosen the bell sensor is used automatically. Turning it on with any other chart means that sounds will stop if all sensors including the bell sensor are covered, as an option to mute the instrument (useful in "bagless" mode especially).

- There is now a version number in the Config Tool, under the title. I plan to keep it in sync with the firmware but may add patches, e.g. 4.6.2 (major, minor, patch).
  

## [Released]

## [4.5] - 2025-8-13

### Changed

- Fixed a bug where pitchbend wasn't being reset to zero after IMU pitchbend was turned off.

- Fixed Bansuri fingering chart.

- For consistency, all buttons actions now cause the LED to flash green (except when the "Momentary" switch is turned on).

- Reorganization of the Configuration Tool: Import/Export buttons are now at the top so it is more clear that they apply only to the current instrument. Pressure and IMU mapping panels are now separate, to make access easier.


### Added

- Halfhole pitchbend feature where covering a hole halfway will lower the pitch one semitone (assuming covering the hole completely would play a note a whole step lower). The pitch can be changed either by using pitchbend or by changing the MIDI note. If the thumb is being used to control the register, it can now be covered halfway to play three registers (see documentation for controlling the order of the registers).

- The "Thumb register" setting now works with custom charts by only looking at the first half of the chart (the half with the thumb hole open). You still need to enter placeholder values for the bottom half of the custom chart but it doesn't matter what the values are.
  
- New button/gesture action: "Register hold" feature that temporarily disables overblowing. This allows for finer control of dynamics in the current register without having overblowing interfere.

- Moving the "Auto power off time" slider to the max now represents an "infinity" setting, preventing the WARBL from shutting down automatically.

- There is now a "Use thumb for slide" switch (off by default) that allows to to enable/disable using the thumb for sliding.

- Two new EVI fingering charts, more similar to standardized EVI charts on other devices.

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

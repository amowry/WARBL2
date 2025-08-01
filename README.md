## WARBL2 is an open-source USB MIDI and BLE MIDI wind controller.

Please see the web site for more info:

https://warbl.xyz

To power the WARBL on, plug into USB or click button 3 (lowest button).

To manually power off the WARBL while under battery power, you can click the reset button (or use a button action assigned in the Configuration Tool, which by default is a long-press of button 3).

For updating the firmware, typical users would just hold down button 1 while plugging the WARBL in to USB and then drag/drop the supplied .uf2 file to the drive named WARBLBOOT. If the WARBL is already plugged in, you can also double-click the reset button to enter the bootloader mode. The most recent flash.uf2 file is located in the build/adafruit.nrf52.warbl2 directory.
 
Advanced users can use the Arduino IDE to modify and upload the code. Modify the code at your own risk and note that while modifying the battery management code won't have dangerous results it could result in reducing the lifespan of the battery.

### How to use WARBL2 with the Arduino IDE:

#### Install Arduino IDE

* Install / run the latest version of the Arduino IDE. [You can download the latest version of the Arduino IDE here](https://www.arduino.cc/en/Main/Software).

* In Linux, the Arduino IDE requires that you be part of the `dialout` group to successfully detect and write to hardware (see [here](https://support.arduino.cc/hc/en-us/articles/4401874331410#ser_open-permission-denied)). If running `id` in a terminal does not list `dialout` as one of the groups your user belongs to, [run `sudo usermod -a -G dialout $USER`](https://support.arduino.cc/hc/en-us/articles/360016495679-Fix-port-access-on-Linux), then log out and back in. Running `id` should now include the `dialout` group.


#### Editing and uploading the WARBL firmware


*  I used the Adafruit ItsyBitsy NRF52840 Express for prototyping WARBL, so the easiest way to set up the IDE is to install the Adafruit boards package, [following the instructions here](https://learn.adafruit.com/adafruit-itsybitsy-nrf52840-express/arduino-support-setup).
  
*  After installing this boards package, it is currently necessary to downgrade the package to v. 1.6.0 (in the Boards Manager in the IDE) because of a bug in TinyUSB. Alternatively, you can edit TinyUSB as described here: https://github.com/amowry/WARBL2/issues/1#issuecomment-2212604645

* Download the WARBL2 code from this repository (I recommend using the most recent *release* code, *NOT* the current main branch), and move the boards.local.txt file from the "firmware" directory into the directory on your machine that contains the boards.txt file for the Adafruit NR52 boards, typically found here:
   
   * Windows: `C:\Users\(username)\AppData\Local\Arduino15\packages\adafruit\hardware\nrf52\1.6.0` 
   
   * Linux: `/home/(username)/.arduino15/packages/adafruit/hardware/nrf52/1.6.0`
     
   * Mac: ` ~/LibraryArduino15/packages/adafruit/hardware/nrf52/1.5.0/`


*	Now open the warbl2_firmware.ino sketch that you saved in your sketchbook folder.
  
*	In the Tools menu select: Board: > Adafruit nRF52 > WARBL2

*	Next, you’ll need to install libraries included in the sketch that aren’t installed by default. To install them, go to Sketch > Include Library > Manage Libraries, then search for the name of each, one at a time. Then it will give you an option to install each one.


*	Then turn on “show verbose ouput" for both "compile" and "upload" under File > Preferences. Now, click the upload button and check that it compiles without errors. 


*	Then, when it tries to upload, you should see this message at the bottom of the screen:
  

`Waiting for upload port...`


*	IF the WARBL currently has a "Release" version of the firmware (see below), you'll then have to use a toothpick to double-click the reset button to enter programming mode. The LED pulse in blue and the code should upload, which will take several seconds. If the LED doesn’t light or blinks rapidly, try again. If the IDE stops trying to upload, click “Upload” again, and try double-clicking again. It can take a few tries to get the timing right. If you've loaded a non-release version then the serial CDC port will be active and the firmware upload should proceed without having to first double-click the reset button.



### A few additional notes:
The serial CDC class on WARBL needs to be turned off to make it a USB MIDI class-compliant device. This also means that you won't be able to use the serial monitor in Arduino IDE, and you'll have to double-click the programming button to install firmware (as described above). To turn serial back on, you can comment out the following line in the sketch:

`#define RELEASE //Uncomment for release version (turns off CDC)`

Turning on CDC makes it easier to debug and makes it so uploads should proceed automatically.


Have fun!

## WARBL2 is an open-source USB MIDI and BLE MIDI wind controller.

Please see the web site for more info:

https://warbl.xyz

For updating the firmware, typical users would just double-click the WARBL reset button (while connected by USB) and then drag/drop the supplied .uf2 file to the drive named ITSY840BOOT.
 
Advanced users can use the Arduino IDE to modify and upload the code. Modify the code at your own risk and note that while modifying the battery management code won't have dangerous results it could result in reducing the lifespan of the battery.

### How to use WARBL2 with the Arduino IDE:

#### Install Arduino IDE

* Install / run the latest version of the Arduino IDE. [You can download the latest version of the Arduino IDE here](https://www.arduino.cc/en/Main/Software).

* In Linux, the Arduino IDE requires that you be part of the `dialout` group to successfully detect and write to hardware (see [here](https://support.arduino.cc/hc/en-us/articles/4401874331410#ser_open-permission-denied)). If running `id` in a terminal does not list `dialout` as one of the groups your user belongs to, [run `sudo usermod -a -G dialout $USER`](https://support.arduino.cc/hc/en-us/articles/360016495679-Fix-port-access-on-Linux), then log out and back in. Running `id` should now include the `dialout` group.


#### Editing and uploading the WARBL firmware


*  I used the Adafruit ItsyBitsy NRF52840 Express for prototyping WARBL, so the easiest way to set up the IDE is to install the Adafruit boards package, [following the instructions here](https://learn.adafruit.com/adafruit-itsybitsy-nrf52840-express/arduino-support-setup).

* To optionally change the board name, manufacturer, and VID and PID numbers, copy the boards.local.txt file from the repository into the directory on your machine that contains the boards.txt file for the Adafruit NR52 boards, typically found here:
   
   * Windows: `C:\Users\(username)\AppData\Local\Arduino15\packages\adafruit\hardware\nrf52\1.3.0` 
   
   * Linux: `/home/(username)/.arduino15/packages/adafruit/hardware/nrf52/1.3.0`
     
   * Mac: ~/LibraryArduino15/packages/adafruit/hardware/nrf52/1.5.0/


*	Now open the WARBL sketch that you saved in your sketchbook folder. Four tabs should open. 

*	Next, you’ll need to install libraries included in the sketch that aren’t installed by default. To install them, go to Sketch > Include Library > Manage Libraries, then search for the name of each, one at a time. Then it will give you an option to install each one.
 
 
*	Next, tell it which board you have by going Tools > Board and select `Adafruit ItsyBitsy NRF32840 Express` The entry may also say 'WARBL' at this point.


*	Then turn on “show verbose ouput" for both "compile" and "upload" under File > Preferences. Now, click the upload button and check that it compiles without errors. 


*	Then, when it tries to upload, you should see this message at the bottom of the screen:
  

Waiting for upload port...


*	Now, use a toothpick to double-click the WARBL reset button. The LED should illuminate blue and the code should upload. If the LED doesn’t light or blinks rapidly, try again. If the IDE stops trying to upload, click “Upload” again, and try double-clicking again. It can take a few tries to get the timing right.



### A few additional notes:
The serial CDC class on WARBL needs to be turned off to make it a USB MIDI class-compliant device. This also means that you won't be able to use the serial monitor in Arduino IDE, and you'll have to double-click the programming button to install firmware. To turn serial back on, you can comment out the following line in the sketch:

#define RELEASE //Uncomment for release version (turns off CDC)

Turning on CDC makes it easier to debug and makes it so uploads should proceed automatically.


Have fun!

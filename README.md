**USB_to_RC**

This project allows the use of USB gamepads, joysticks, wheels, etc with standard RC gear and drones.  HID devices and some XINPUT devices are support for input.  PPM, SBUS, and mavlink are supported outputs.  
This allows things like connecting a USB gamepad to the "buddy box" port of your RC controller so that the gamepad can operate some or all of the vehicle.  It also output MAVLINK RC_CHANNEL_OVERRIDE messages via uart.  Some applications might include:
- Using a gamepad to control a gimbal on a drone while the rest of the RC controller operates the aircraft
- Using a racing wheel to drive an RC car
- Using the MAVLINK output to directly operate a drone over some other radio link

This project uses the RP2040 processor.  It can be run on a raspberry pi pico with some external wiring or on a Adafruit feather RP2040 with USB Type A Host with Adafruit AdaLogger FeatherWing

https://learn.adafruit.com/adafruit-feather-rp2040-with-usb-type-a-host
https://learn.adafruit.com/adafruit-adalogger-featherwing

See IO.h for pinning information
<img src="https://github.com/Doctor-Proton/USB_to_RC/blob/master/images/Adafruit%20feather%20USB_to_RC.jpg">

**cloning and build**

This section assumes you have a working RP2040 toolchain installed.  You will also need to edit the top level CMakeLists.txt to point to the location of your PICO SDK.

git clone https://github.com/Doctor-Proton/USB_to_RC.git

cd USB_to_RC

git submodule update --init --recursive

mkdir build

cd build

cmake -DCMAKE_BUILD_TYPE=Debug ..

make


**Configuration**

Connect the USB-C port (Adafruit) or USB micro-B port (raspberry pi pico) to a computer.  Use a terminal program such a teraterm (windows) or screen (linux) to connect the the serial port that is created with the RP 2040 is plugged in.  Plug in the USB controller (gamepad, joystick, etc) to the USB A port.  At this point the device should be enumerated and "variables" will be added for this device.

Some variables are automatically named, button_XX, X,Y,Z,Rx,hat_switch, etc.  Others are just called in_XX.  16 out_XX variables are automatically created.  These correspond to the 16 RC channels (only the first 9 are on PPM) that will be output.  Move the gamepad controls to see which variables are affected.

<img src="https://github.com/Doctor-Proton/USB_to_RC/blob/master/images/VT100%20main%20-%20edited.png">

Configuration files define how to "mix" the input variables to the output variables.  This is in the form of simple expressions.  These config files must live the "config" directory on the root of the SD card.  A master configuration file (lookup.cfg) looks up a configuration file for a given VID:PID pair.  See the example configuration files.

Output options (baud rates, inversion, etc) can be set by setting certain variables in the configuration files.  See the examples.

**Attributions**


The project makes use of following projects.  In many cases their submodules as well

expr
https://github.com/zserge/expr

Pico-PIO-USB
https://github.com/sekigon-gonnoc/Pico-PIO-USB

FreeRTOS-FAT-CLI-for-RPi-Pico
https://github.com/carlk3/FreeRTOS-FAT-CLI-for-RPi-Pico

HID parser code is from Lufa
https://github.com/abcminiuser/lufa








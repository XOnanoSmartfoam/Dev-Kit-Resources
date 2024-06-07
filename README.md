# XO-NANO Dev-Kit-Resources
This repository contains all of the support resources for the XO-NANO Development Kit. These resources will be updated on a constant basis so please check back if you are having issues and we may have resolved them in current releases.

# XO-NANO Impact
The primary way to measure XO-NANO impact foam is by using the XO-NANO microcontroller sysyem. The current release of this system is XO-NANO Baldy Rev 2. This board has some limitations due to a manufacturing defect which only allows the board to broadcast the measurements and doesn't allow for on board storage. When XO-NANO Baldy Rev 3 is released we will have the onboard flash working.

A user's guide for the XO-NANO microcontroller and its accompanying iOS app can be found in the XO-NANO Smartfoam Development Kit User Guide

# XO-NANO Pressure
There are two demonstrations available that you can use to measure pressure with the XO-NANO Pressure sensors. See the Single Sensor Demonstration folder for "Pressure Sensor Instructions.docx" which walks through the set up of a single XO-NANO pressure sensor. Below are tools (you only need to purchase one of the tools) that could be used to interface with the XO-NANO pressure sensor in the development kit. For the ESP32 Feather powered pressure map demo, no purchase other than the demonstration itself is necessary. See the instruction manual in the Pressure Map Demo folder for more details.

Arduino Uno

https://store-usa.arduino.cc/collections/boards/products/arduino-uno-rev3-smd

Analog Discovery

https://digilent.com/shop/analog-discovery-2-100ms-s-usb-oscilloscope-logic-analyzer-and-variable-power-supply/

Signal Generator and O-Scope

https://www.digikey.com/en/products/detail/b-k-precision/4010A/301869

https://www.tek.com/en/products/oscilloscopes/tbs1000


# XO-NANO DevKit iOS App
This app is in Beta release and you can join the beta program by following this link on your iOS device. https://testflight.apple.com/join/mG4E2b9c This app is completely local to your iOS device and will not upload any data to a cloud service.

# Signal Processing Resources
The iOS App used to record the Smartfoam measurements saves the data to a .csv file. This file can be evaluated with your choice of software. The DevKit Visualization Script.py file found in this repository can be used to filter and plot the data from the Smartfoam and IMU.

# Fishlake
Fishlake mini version 1 is used to collect dynamic and static data. Using the iOS application found here https://testflight.apple.com/join/MpLxyleo the user can start logging data to flash, stream data, save data to iOS device, and end tests. If logging to the Fishlake mini hardware, the FishlakePullandPlot.exe file can be used to pull the data from saved flash memory, and plot adc, imu, mag, and temp data.

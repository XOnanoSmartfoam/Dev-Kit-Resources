# XO-NANO Dev-Kit-Resources
This repository contains all of the support resources for the XO-NANO Development Kit. These resources will be updated on a constant basis so please check back if you are having issues and we may have resolved them in current releases.

# XO-NANO Impact
The primary way to measure XO-NANO impact foam is by using the XO-NANO microcontroller sysyem. The current release of this system is XO-NANO Baldy Rev 2. This board has some limitations due to a manufacturing defect which only allows the board to broadcast the measurements and doesn't allow for on board storage. When XO-NANO Baldy Rev 3 is released we will have the onboard flash working.

A user's guide for the XO-NANO microcontroller and its accompanying iOS app can be found in the XO-NANO Smartfoam Development Kit User Guide

# XO-Nano Pressure
There are several ways you can measure pressure with the XO-NANO Pressure sensors. See "Pressure Sensor Instructions.docx" for instructions for setting up an XO-NANO pressure sensor. Below are tools (you only need to purchase one of the tools) that could be used to interface with the XO-NANO pressure sensor in the development kit.

Arduino

https://store-usa.arduino.cc/collections/boards/products/arduino-uno-rev3-smd

Analog Discovery

https://digilent.com/shop/analog-discovery-2-100ms-s-usb-oscilloscope-logic-analyzer-and-variable-power-supply/

Signal Generator and O-Scope

https://www.digikey.com/en/products/detail/b-k-precision/4010A/301869
https://www.tek.com/en/products/oscilloscopes/tbs1000

# XO-NANO DevKit iOS App
This app is currently under development and is limited to invited testers. Please email jake.sundet@xonano.com for access to the TestFlight version of the app. As soon as we work through the bugs we will release it to the App Store.

# Signal Processing Resources
The iOS App used to record the Smartfoam measurements saves the data to a .csv file. This file can be evaluated with your choice of software. The DevKit Visualization Script.py file found in this repository can be used to filter and plot the data from the Smartfoam and IMU.

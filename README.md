# ESP32-Tilt-Repeater
Tilt hydrometer repeater for ESP32 based devices


## About this fork

This fork incorporates a number of changes to the original script:
* Converts from Arduino to ESP-IDF
* Adds MAC spoofing - replicating the MAC of the original Tilt rather than overwriting with the address of the TiltBridge's Bluetooth interface

All this fork does is modify the project to compile via platformio, and allow me to generate a release for
inclusion in [BrewFlasher](https://www.brewflasher.com/)


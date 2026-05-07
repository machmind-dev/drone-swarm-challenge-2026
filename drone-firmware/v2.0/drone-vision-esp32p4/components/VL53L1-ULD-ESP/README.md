# VL53L1-ULD-ESP

ESP-IDF library to support the VL53L1 Time-of-Flight ranging sensor on Espressif ESP32 chips.

## ST Drivers

This library incorporates the ST VL53L1 Ultralight Driver (ULD) library:
https://www.st.com/en/imaging-and-photonics-solutions/vl53l1x.html#tools-software

## Examples

* /examples/main-hires-timer.c 

Using ESP-IDF high resolution time, polling VL53L1 sensor

* /examples/main-gpio-interrupt

Using continuous measurement mode with GPIO hardware interrupt

* /examples/main-multiple-sensors.c

Demonstrates multiple sensors with polling measurements

## Installation and Use

This library is organized as an ESP-IDF component. 
1. Copy the entire library to your /components folder, i.e. you should have `./components/VL53L1-ULD-ESP/` in your project root. 
2. Modify the root file CMakeLists.txt. Add "VL53L1-ESP-IDF" as: `list(APPEND EXTRA_COMPONENT_DIRS VL53L1-ULD-ESP)`
3. Hook up the VL53L1 to your ESP32 through the 2-wire I2C port, power supply and ground. The interrupt example requires connecting the VL53L1 gpio1 pin, but the timer example does not.
4. Copy one of the /example/main*.c files to the project root, build, and download to your ESP32. 

## Documentation

The VL53L1 datasheet is available at
https://www.st.com/en/imaging-and-photonics-solutions/vl53l1x.html#documentation


# AirQualitySensors

Embbedded part of Air Quality Sensors module for my GUI application CustomKeyboard. This project also contains a simple gateway which transmits key-presses from a keyboard attached to nucleo over UART. This was needed because I wasn't able to bypass keypresses from second keyboard connected to computer (both with Windows and Linux) and just execute the required macros for given keys.

https://github.com/kurta999/CustomKeyboard

Source is made for STM32 NUCLEO L496ZG. The source is not in compilable form yet, because the ESP32 WiFi driver isn't included.

### Drivers included for Air Quality sensors:

BME680 - Pressure, Temperature, Relative Humidity  
SCD30 - CO2 (Pressure and Humidity part is not used, based on my experience, it's deviates too much even when the correct offset is applicede)  
VEML6070 - UV Index  
TCS34725 - RGB Color values in light, CCT and Kelvin  
Honeywell HPMA115S0 - Particle Sensor (pm2.5, pm10)  

The source code for sensors can be found in freertos.c and in sensors.c. The code is bloat, because this is just a protoype. I didn't have time yet to clean up everything, it just works.

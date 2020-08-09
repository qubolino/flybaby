# flybaby
A sensor using esp32, mpu6050 in DMP mode and ms5607 (despite the class names) pressure sensor and GPS, publishing NMEA sentences on serial port.

Using platformio with Arduino framework.

This is originially a port of Iain Frew's https://github.com/mwesterm/esp32-vario which itself is a port of Hari Nair's https://github.com/har-in-air/ESP8266_MPU9250_MS5611_VARIO to esp32 and using the DMP functionaility of the 6050 IMU on the GY-86 breakout board.

The majority of the code was some great work by Jeff Rowberg for the mpu6050 library and the i2cdevlib interface, and Hari Nair for the Vario design. The calibration came from  Luis Ródenas. 
The plan is to build on it.

After customization (pin-wise), subtle refactoring/adaptation, and NMEA interfacing, calibration I am able to make it work nicely with kobo mini!


![finished job](pics/fullsizeoutput_49e1.jpeg)


# flybaby
A smart variometer/flight sensor using esp32, mpu6050 using DMP for quaternions, ms5607 pressure sensor and GPS, publishing NMEA sentences on serial port and specifically designed to work with [XCSoar](https://xcsoar.org) on kobo 
I imagine it would work 
* with ESP8266 instead of ESP32 with little changes, 
* with mpu9150 or 9250 instead of mpu6050 with no change at all, 
* and with ms5611 or ms5637 instead of ms5607 with very little changes
* any serial gps module outputing standard NMEA sentences
* any other XCSoar-compatible kobo

Developed, compiled and uploaded using [PlatformIO](https://platformio.org) with [Arduino framework](https://docs.platformio.org/en/latest/platforms/espressif32.html) in [Visual Studio Code](https://code.visualstudio.com).

After customization (pin-wise), subtle refactoring/adaptation, NMEA interfacing, and calibration I am able to make it work nicely with KoBo mini!

![finished job](pics/IMG_4977.jpg)

I haven't taken it to fly yet but i can see from driving with it that the vario, accel, horizon, gps speed, altitude all make sense.

## _a_ recipe 
1. get the sensor modules and connect them to esp32
   1. MPU6050 ic2 pins SDA 4, SCL 5, INTerrupt 15
   1. MS5607  ic2 pins SDA 4, SCL 5
   1. serial GPS serial (mine is GN-801) pin RX 13 (only this one is needed as we don't send instructions to GPS module)
   1. pull-down or capacitive button on pin 12
   1. audio on pin 17 (still to be tested -- my device does not have audio for the time being)
1. Get the right kobo and [install XCsoar on it](http://max.kellermann.name/projects/xcsoar/kobo.html) (essentially copy the `KoboRoot.tgz` into the `.kobo` folder of your kobo).
1. Interface your sensor with serial either: 
   1. with a micro-usb <-> micro-usb in which case you will need to provide power to esp and sensors (possibly using the serial vcc pin of the kobo)
   1. directly [soldering on the kobo serial port](http://gethighstayhigh.co.uk/kobo-self-build/)
1. optionally remove the battery from kobo and put another on in place -- i soldered 3 18650 Li-Ion outside the kobo for exxxxtended battery life

## credits
This is originially a port of Iain Frew's https://github.com/mwesterm/esp32-vario which itself is a port of Hari Nair's https://github.com/har-in-air/ESP8266_MPU9250_MS5611_VARIO to esp32 and using the DMP functionaility of the 6050 IMU on the GY-86 breakout board.
The majority of the code was some great work by Jeff Rowberg for the mpu6050 library and the i2cdevlib interface, and Hari Nair for the Vario design. The calibration came from Luis Ródenas. Iain Frew put some glue on various components. I added GPS forwarding, NMEA output, MS5607/5611 proper temperature compensation.

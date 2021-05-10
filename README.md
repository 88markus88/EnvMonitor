# EnvMonitor

This project aims to create a flexible software for ESP32 to use multiple environmental sensors and make the data visible using <B>Blynk</B> (https://blynk.io/)

The project consists of both code and the hardware description as KiCad and Fritzing files.

It is presently work in progress, but already functional. Development is being done using Platformio with the Arduino platform. 

![PCB](https://github.com/88markus88/EnvMonitor/blob/main/EnovMonitor680-Git/Pictures/EnvMonitor%203D%20V0.6.jpg)

## Visualization:
- via Blynk, a commercial service that can also be used with a local server, e.g. on a Raspberry Pi

## Sensors:
So far the following sensors and devices on the following list can be included. Selection is done by modification of the #defines in *GlobalDefines.h*:
- BME280 (Temperature / Humidity / Pressure) via I2C
- BME680 (Temperature / Humidity / Pressure / Air Quality) via I2C
- DS18B20 (Temperature, up to 10 sensors) via OneWire
- MH-Z14A (CO2) via Serial2
- SenseAir S8 (CO2) via Serial2
- External temperature and humidity data from an Infactory NV5849 connected to an Arduino via Serial2
- Infactory NV5849 (433 MHz Temperature / Humidity sender) via GPIO 33 

## Devices:
- Relays / Transistor switches (2)
- OLED Display 0.94" (SS1306) via I2C
- LCD Display 4 x 20 via I2C
- Button to be able to switch content of displays, or mute display light
- SD card reader via SPI

## Hardware
It is also possible to receive data from a 433 MHz transmitter via serial (e.g. received from an Arduino). Not included here.

The Fritzing files does not contain the option to connect a 433 MHz transmitter. It also does not contain the external power supply that has been included in KiCad, and the option to power the DS18B20 sensors via digital output 32 (Jumper on J9) as an alternative to 3.3V. This may be necessary if the sensors are too unstable for continued operation, they can then optionally be reset by switching their power supply. May be necessary since most available DS18B20s are fake and prone to unstability (no data for longer periods). First option in this case is the reduction of the pulllup resistor R2 to 2.5K.

The board can be powered via USB. However, the USB power is relatively unstable. Most USB power supplies are not stabilized and should not be used. Raspberry Pi power supplies are better. Best is to use the option to add an internal DCDC converter <I>Traco TSR 1-2450E</I> that can be driven with 7-36V and provides excellent power conditioning. If this is not used, just leave the comoponents (DCDC converter, capacitor, diode, terminal block) out.

![Fritzing](https://github.com/88markus88/EnvMonitor/blob/main/EnovMonitor680-Git/Pictures/EnvMonitor%20Fritzing%20V0.3.jpg)
![Schema](https://github.com/88markus88/EnvMonitor/blob/main/EnovMonitor680-Git/Pictures/EnvMonitor%20Schematic%20V0.4.jpg)
![PCB](https://github.com/88markus88/EnvMonitor/blob/main/EnovMonitor680-Git/Pictures/EnvMonitor%20PCB%20V0.6.jpg)

## Arduino
PCB and Code for the Arduino Nano that can provide the data from an external 433MHz sensor is also included.
Why using and Arduino? It turns out that the Wifi used by the ESP32 interferes with the 433 MHz reception when an ESP32 is used directly. The signals are disturbed by the 100 ms heartbeat of the Wifi that cannot be changed easily. 
So a small arduino PCB and sketch have been developed to still be able to use external sensors. This arduino needs to be connected via cable to the ESP32. Pins used for this are +5V, GND, RxD and TxD (the latter switched between ESP32 and arduino)
No special libraries are needed for the Arduino Sketch. 

![PCB](https://github.com/88markus88/EnvMonitor/blob/main/EnovMonitor680-Git/Arduino%20Hardware/Pictures/Arduino433Receiver%203D%20V0.2.jpg)
![Schema](https://github.com/88markus88/EnvMonitor/blob/main/EnovMonitor680-Git/Arduino%20Hardware/Pictures/Arduino433Receiver%20Schema%20V0.2.jpg)
![PCB](https://github.com/88markus88/EnvMonitor/blob/main/EnovMonitor680-Git/Arduino%20Hardware/Pictures/Arduino433Receiver%20PCB%20V0.2.jpg)

## Parts
- PCB for ESP32 (e.g. from JLCPCB)
- ESP32 DevKit C
- Resistors. One each 4,7K, 10K, 1K, 2K, 3.3K
- Transistor BC 547
- Terminal Block 3x, 5,08 mm spacing
- 2 Terminal Block 3x, 2,54 mm spacing
- 5-9 Terminal Block 2x, 2,54 mm spacing 
- 2 Pinheader female 4x 2,54 mm spacing
- Pinheader male 3x
- OLED display 0.96' or LCD display (I2C) 4x20 
- Button
- Housing

**For power supply:**
- Terminal Block 2x, 5,08 mm spacing 
- DCDC Converter Traco TSR 1-2450E
- Diode 1N4001
- Electrolyte Capacitor 470 uF

**For Arduino:**
-  PCB for Arduino (e.g. from JLCPCB)
- Arduino Nano
- 433 MHz Receiver RXB6
- 433 MHz Antenna
- 2 x Capacitor 100 nF
- Terminal Block 4x, 2,54 mm spacing
- Two Pinheader: 4x, 2,54 mm spacing  (used for debugging only)
- Housing

## Blynk Virtual Pins
**Inputs**
VPin  |Function
------|---------
V18 | Relay 1
V19 | Relay 2
V20 | Start CO2 Sensor Calibration
V60 | Manual restart of 1Wire bus (DS18B20 Sensors)

**Outputs**
VPin  |Function
------|---------
V5  | BME280 / BME680 Temperature
V6  | BME280 / BME680 Air Pressure
V7  | BME280 / BME680 Humidity
V8  | BME680 Air Quality Score
V12 | BME680 Air Quality String
V13 | DS18B20[0]  Temperature
V10 | DS18B20[1]  Temperature
V11 | DS18B20[2]  Temperature
V15 | Infactory Temperature Ch1
V16 | Infactory Humidity Ch1
V17 | Infactory Temperatue Ch2
V18 | Infactory Humidity Ch2
V19 | Infactory Temperature Ch3
V20 | Infactory Humidity Ch3
V35 | Infactory Ch1 Counter 
V36 | Infactory Ch2 Counter 
V37 | Infactory Ch3 Counter 
V38 | Infactory Serial Fail Counter 

## Credits
- Blynk Library from here: https://blynk.io/. Using <i>Wifi, WifiClient</i> and <i>BlynkSimpleEsp32</i> libraries
- BME 280: <i>Wire</i>, <i>Adafruit_Sensor</i> and <i>Adafruit_BME280</i> libraries
- BME 680: using <i>Zanshin_BME680</i> and <i>SPI</i> libraries by Arnd SV Zanshin, https://github.com/Zanduino/BME680  and Information from this Article https://wiki.dfrobot.com/Gravity__I2C_BME680_Environmental_Sensor__VOC,_Temperature,_Humidity,_Barometer__SKU__SEN0248 
- NTP time using info from: https://randomnerdtutorials.com/esp32-date-time-ntp-client-server-arduino/ 
- OLED display: using <i>Adafruit_GFX</i> and <i>Adafruit_SSD1306</i> libraries
- DS18B20: using <i>OneWire</i> and <i>DallasTemperature</i> libraries
- SenseAir S8: using code from SFeli https://github.com/SFeli/ESP32_S8  
- MH-Z14A_ using code from the contributin by Mitch K https://www.hackster.io/kritch83/getting-started-with-the-mh-z14a-co2-detector-e96234 
- Infactory NV5849: based on work and code by Ray Wang https://rayshobby.net/reverse-engineer-wireless-temperature-humidity-rain-sensors-part-1/ 
- Macros for bit field manipulation by Shun Yan Cheung: http://www.mathcs.emory.edu/~cheung/Courses/255/Syllabus/1-C-intro/bit-array.html
- LCD library: <i>LiquidCrystal_I2C</i> : https://github.com/fdebrabander/Arduino-LiquidCrystal-I2C-library/blob/master/LiquidCrystal_I2C.h

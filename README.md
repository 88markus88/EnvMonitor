# EnvMonitor - ESP32
This project aims to create an environmental monitoring system, using ESP32 and multiple environmental sensors and make the data visible using <B>Blynk</B> (https://blynk.io/)

The project consists of both software code and the hardware description as KiCad and (partly) Fritzing files.

It is presently work in progress, but already functional. Development is being done using Platformio with the Arduino platform. 
![PCB](https://github.com/88markus88/EnvMonitor/blob/main/EnovMonitor680-Git/Pictures/EnvMonitor%203D%20V0.6.jpg)

## Visualization:
Visualization is done via Blynk, a commercial service that can also be used with a local server, e.g. on a Raspberry Pi
All data that the EnvMonitor provides can be shown via the Blynk App - very simple and powerful. Since the service is cloud based, the data can be viewed at every location.
Blynk also supports a local server (e.g. on a Raspberry) - this eliminates the costs for the Blynk service.

Now also beginning to add support vor Thingspeak and for Virtuino

## Sensors:
So far the following sensors and devices on the following list can be included. Selection is done by modification of the #defines in *GlobalDefines.h*:
- BME280 (Temperature / Humidity / Pressure) via I2C
- BME680 (Temperature / Humidity / Pressure / Air Quality) via I2C
- DS18B20 (Temperature, up to 3 sensors) via OneWire
- MH-Z14A (CO2) via Serial2
- SenseAir S8 (CO2) via Serial2
- External temperature and humidity data from an Infactory NV5849 433MHz transmitter connected to an Arduino via Serial2
- Infactory NV5849 (433 MHz Temperature / Humidity sender) directly received via 433MHz receiver at GPIO 33. This option is supported by the software, ut not implemented on the board. 

## Devices:
- Relays / Transistor switches (2)
- OLED Display 0.94" (SS1306) via I2C
- LCD Display 4 x 20 via I2C
- Button to be able to switch content of displays, or mute display light
- SD card reader via SPI

## ESP32 Hardware
The main component is an ESP32 with various peripherals attached. Communication to the peripherals is done via 
- 1Wire (DS180B20 temperature sensors)
- I2C (OLE or LCD displays, BME280 or BME 680 environment sensors
- SPI (SD card drive)
- Serial (CO2 sensors, external sensors via Arduino)
- GPIOs (button, transistor or relais)

All components are optional. They can be used in a large varity of combinations, providing that there are no confliting requirements
- Multiple devices can be combined on the I2C interface. E.g. BME280 and OLED display can be used together. Or LCD display and BME680. Or BME280 and no display.
- Up to 3 DS18B20 temperature sensors are supported. 3 connectors are provided, where they are connected in parallel. 
- Only one serial port (Serial2) is supported. Therefore only one CO2 sensor, or alternatively one Arduino, can be connected.
- Running a fan etc. via BC547 transistor, or other devices via relay, is supported by the hardware. Could also be used for a buzzer.
- An internal power source can be used, which requires 7V-36V input from an external power supply.
Components that are not used do not have to be placed on the board. They can be configured out in the software via #defines

The board can be powered via USB. However, the USB power is relatively unstable. Most USB power supplies are not stabilized and should not be used, at least with multiple components connected - this can lead to brownouts and system failure. Raspberry Pi power supplies are better suited. 
Best is to use the option to add an internal DCDC converter <I>Traco TSR 1-2450E</I> that can be driven with 7-36V and provides excellent power conditioning. If this is not chosen, just leave the comoponents (DCDC converter, capacitor, diode, terminal block) out.

It is possible to receive data from a 433 MHz transmitter via serial (e.g. received from an Arduino). (The serial port is not included in the Fritzing drawing.) 
The Arduino board, as well as it's software are described in a separate section. 

The board allows power selection for the 1Wire bus: either directly from 3.3V, or via GPIO32 of the ESP32. (In default configuration each GPIO can drive 20 mA).
This is useful to be able to completely reset the 1Wire bus - often necessary if cheap DS18B20 ("fake sesnors") are being used. These have a tendency to hang after extended use, the only viable option is then to completely cut the power.

KiCad files are provided for the electrical schemas and the PCB boards. 

There is also a Fritzing file for illustration, however it is not complete: it does not contain the option to connect a 433 MHz transmitter. It also does not contain the external power supply that has been included in KiCad, and the option to power the DS18B20 sensors via digital output 32 (Jumper on J9) as an alternative to 3.3V. This may be necessary if the sensors are too unstable for continued operation, they can then optionally be reset by switching their power supply. May be necessary since most available DS18B20s are fake and prone to unstability (no data for longer periods). First option in this case is the reduction of the pulllup resistor R2 to 2.5K.

![Fritzing](https://github.com/88markus88/EnvMonitor/blob/main/EnovMonitor680-Git/Pictures/EnvMonitor%20Fritzing%20V0.3.jpg)
![Schema](https://github.com/88markus88/EnvMonitor/blob/main/EnovMonitor680-Git/Pictures/EnvMonitor%20Schematic%20V0.4.jpg)
![PCB](https://github.com/88markus88/EnvMonitor/blob/main/EnovMonitor680-Git/Pictures/EnvMonitor%20PCB%20V0.6.jpg)

## ESP32 Code
The code is written in C++. It is intended to be flexible, to allow almost any combination of sensors and displays on the board. 
All data are sent to a Blynk server, provided that *#define isBlynk* is set.

Development platform is Platformio. The code should also compile - with minimum adaptations - on the Arduino IDE. However, Platformio is much more flexible, compile times much faster due to more efficient built. It is well worth to switch to Platformio!

In order to compile the code, the following adaptations have to be made:
1. Get the required libraries, adjust the library path in *platformio.ini* to point to your library directory
2. create your own *Credentials.h* file. A template file is provided. You need to put in the name and password for your Wifi network, this is a must to allow the ESP32 to access the network. Then insert the auth code, temperature correction data and device info strings for each device you want to use. This file can contain any number of these device definitions, each within a #ifdef /#endif construct.
3. Adapt "GlobalDefines.h" to your hardware.  
  - enter the IP address of your local Blynk server
  - set one #define for the name of the hardware configuration you want to use (one of those in the "credentials.h" file)
  - set the actual hardware definitions: use of Blynk, use of sensors, use of display etc. These hardware definitions must fit the ones set in *credentials.h*
  - determine if you want to log to a SD card and / or to the serial port
  - determine if OTA (over the air) updates shall be used
 4. modify *platformio.ini* as needed: set _default_envs_ to determine which environment is to be used. This is largely required to determine if USB  upload or OTA upload to a given IP are to be used.

## Connectivity 
In order to transmit data to a web service the ESP32 needs to log into a local network. At present, three services can be supported
- <b>Blynk</b>: Commercial service to which the data are sent, and shown on mobile device in an app that can be configured very fleibly. The data can either be sent to the Blynk web service or to a local server. fully implemented for all data. Temperature, Humidity, Pressure etc. for all sensors are transmitted via "Virtual Pins" (see below for details). Blynk Auth Codes, which are specific for any devices and generated within the Blynk App on a mobile device, need to be put into the file <i>Credentials.h</i>
<b>Notice</b> Blynk is presently transitioning to V2.0 with much more expensive licensing models, and no local server. Makes it less desirably for hobbyist users, consider Thingspeak as an alternative.
- <b>Virtuino</b>: (Experimental) Similar solution to Blynk. However, here the device itself provides a web server into which the mobile has to log in. Only partly implemented for demonstration purposes.
- <b>Thingspeak</b>: (Experimental) Web service that stores sensor data and makes them available to a variety of generic mobile apps. At present, data for BME680 and DS18B20 can be transmitted. Thingspeak is addressed via a https call to an address that encodes the Write API Key defined for a specific device, there can be up to 8 data channels per device. Presently hard coded within <i>ThingspeakServerName</i>. Needs to be more flexible for full use
Blynk and Thingspeak can be used in parallel on a device. 
Settings which connecitivity type is used are done in <i>GlobalDefines.h</i> via the #defines isBLYNK, isThingspeak, isVirtuino. 

### Setting the WiFi Credentials
To access the local network the ESP32 requires the network credentials, particularly the SSID and password. They could be encoded in source code, and indeed default data are included in <i>Credentials.h</i>. However, this method is very clumsy - it requires re-compile of the code every time the credentials are changed, or a new network is used. Therefore, two methods are provided to update the credentials at runtime, and store them in the Preferences (ESP32 EEPROM):
- <b>Captive Portal</b> Activated by the #define <i>isCaptivePortal</i>. If this is set, the device goes through a sequence at bootup: 
1. If Wifi login with valid credentials are stored in the preferences, these are used
2. If not, the captive portal is opened, the user can select a network and provide the password. If login with these data is possible, these credentials are stored in the preferences for future use. The default SSID for the captive portal is <i>ESP_Config</i>  , the default password is <i>12345678</i>
3. If neither works, the defaults are used (but not stores, since the network may be out temporarily)
- <b>Bluetooth</b> Activated by the #define <i>isBluetoothCredentials</i>. When activated, the ESP32 waits for 60 sec after Boot for a connection from a mobile device. In a Bluetooth terminal app on the mobile, the user can then connect to the device, and enter the credentials: <i>ssid:[id_of_your_network]<i/> <i>pass:[your_password]<i/>. These credentials are then stored in the Preferences and used in future logins

### Blynk Virtual Pins
Blynk uses the concept of __Virtual Pins__ for communication between the hardware devices and App (via the server). The list of virtual pins in actual use depends on the sensors that are implemented on a specific devices. They need to be known in order to connect the Widgets within the App to specific sensors.

See [List of Virtual Pins](https://github.com/88markus88/EnvMonitor/blob/main/EnovMonitor680-Git/Doc/BlynkVirtualPins.md)

# Arduino
PCB and Code for the Arduino Nano that can provide the data from an external 433MHz sensor is also included.
Why using and Arduino? It turns out that the Wifi used by the ESP32 interferes with the 433 MHz reception when an ESP32 is used directly. The signals are disturbed by the 100 ms heartbeat of the Wifi that cannot be changed easily. 
So a small arduino PCB and sketch have been developed to still be able to use external 433MHz sensors. This Arduino needs to be connected via cable to the ESP32. Pins used for this are +5V, GND, RxD and TxD (the latter switched between ESP32 and Arduino)
No special libraries are needed for the Arduino Sketch. 

![PCB](https://github.com/88markus88/EnvMonitor/blob/main/EnovMonitor680-Git/Arduino%20Hardware/Pictures/Arduino433Receiver%203D%20V0.2.jpg)
![Schema](https://github.com/88markus88/EnvMonitor/blob/main/EnovMonitor680-Git/Arduino%20Hardware/Pictures/Arduino433Receiver%20Schema%20V0.2.jpg)
![PCB](https://github.com/88markus88/EnvMonitor/blob/main/EnovMonitor680-Git/Arduino%20Hardware/Pictures/Arduino433Receiver%20PCB%20V0.2.jpg)

## Parts
See [Parts List](https://github.com/88markus88/EnvMonitor/blob/main/EnovMonitor680-Git/Doc/Partslist.md)

## GPIO Pins
A number of ESP32 I/O Pins are used. 
See  [List of GPIO Pins](https://github.com/88markus88/EnvMonitor/blob/main/EnovMonitor680-Git/Doc/GPIOs.md)

## Credits
- Blynk Library from here: https://blynk.io/. Using <i>Wifi, WifiClient</i> and <i>BlynkSimpleEsp32</i> libraries
- BME 280: <i>Wire</i>, <i>Adafruit_Sensor</i> and <i>Adafruit_BME280</i> libraries
- BME 680: using <i>Zanshin_BME680</i> and <i>SPI</i> libraries by Arnd SV Zanshin, https://github.com/Zanduino/BME680  and Information from this Article https://wiki.dfrobot.com/Gravity__I2C_BME680_Environmental_Sensor__VOC,_Temperature,_Humidity,_Barometer__SKU__SEN0248 <br>
  Alternatively: BME 680 using the manufacturer supplied __BSEC__ library by Bosch Sensortec. This library is only available as pre-compiled object code. https://platformio.org/lib/show/6979/BSEC%20Software%20Library 
- NTP time using info from: https://randomnerdtutorials.com/esp32-date-time-ntp-client-server-arduino/ 
- OLED display: using <i>Adafruit_GFX</i> and <i>Adafruit_SSD1306</i> libraries
- DS18B20: using <i>OneWire</i> and <i>DallasTemperature</i> libraries
- SenseAir S8: using code from SFeli https://github.com/SFeli/ESP32_S8  
- MH-Z14A_ using code from the contributin by Mitch K https://www.hackster.io/kritch83/getting-started-with-the-mh-z14a-co2-detector-e96234 
- Infactory NV5849: based on work and code by Ray Wang https://rayshobby.net/reverse-engineer-wireless-temperature-humidity-rain-sensors-part-1/ 
- Macros for bit field manipulation by Shun Yan Cheung: http://www.mathcs.emory.edu/~cheung/Courses/255/Syllabus/1-C-intro/bit-array.html
- LCD library from Marco Schwartz: <i>LiquidCrystal_I2C</i> : https://github.com/marcoschwartz/LiquidCrystal_I2C/archive/master.zip

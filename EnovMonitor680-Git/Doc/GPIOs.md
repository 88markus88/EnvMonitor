#GPIO Pins used in EnvMonitor

These GPIO Pins are used by EnvMonitor:
**Inputs and Outputs**
GPIO |Function
------|---------
5 | SPI CS | SD Card Reader
13 | OneWireDS18B20 | 1 Wire data for DS18B20 sensors
14 | LED output | to show that system is alive
15 | Pushbutton | push button input, 
16 | Serial RX | MH-RX 14A CO2, Senseair S8 CO2 or Arduino serial data (only one of these at any time)
17 | Serial TX | MH-RX 14A CO2, Senseair S8 CO2 or Arduino serial data (only one of these at any time)
18 | SPI SCK | SD Card Reader
19 | SPI MISO | SD Card Reader
21 | I2C SDA | OLED display, LCD display , BME 280 sensor, BME 680 sensor 
22 | I2C SCL | OLED display, LCD display , BME 280 sensor, BME 680 sensor 
23 | SPI MOSI | SD Card Reader
26 | BC 547 | Transistor to switch fan etc. 
27 | Relais 2 | relais output. presently not used
32 | OneWire Power | 3.3 V power via GPIO allows to switch 1Wire off for reset
33 | 433MHz Data In | in case that a 433 MHz receiver is directly connected. presently not used
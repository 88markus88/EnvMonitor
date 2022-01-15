# EnvMonitor 

## Libraries
The libraries for ESP32 include:
- Syslog 2.0.0 - for writing of log data to a syslog server 
- DNSServer 1.1.0 - used for captive portal
- EEPROM 1.0.3 - outdated. Writing of permanent data to ESP32 EEPROM 
- Preferences 1.0 - Writing of permanent data to ESP32 EEPROM 
- WiFi 1.0 - Base library for all WiFi functions
- WifiClientSecure 1.0 - a base library for HTTPS access
- ESPmDNS 1.0 - a base library for HTTPS access
- ESP32Ping 1.7 - a base library for HTTPS access
- HTTPClient 1.2 - enables to set up a captive portal, which is used to set the device's WiFi access data
- LiquidCrystal_I2C - to write information on a LCD display
- SimpleTimer - timer functions, used if no Blynk timer is available
- BluetoothSerial 1.0 - to enable setting of device's WiFi access data via bluetooth
- FS 1.0 - file system base functions, used for SD cards
- SD(esp32) 1.0.5 - handing of SD cards
- BME680 1.0.10 - library for BME 680 (if no precompiled libraries by BSEC are used)
- Precompiled BSEC Library for BME680
- Adafruit BME280 Library 2.1.2 - Adafruit library for BME280 sensor
- Adafruit GFX Library 1.7.5 - basic graphic library
- Adafruit Unified Sensor 1.1.4 - generic sensor library
- Adafruit SSD1306 2.1.0 - to write information on a OLED display
- ArduinoOTA 1.0 - enables update of the ESP32 firmware via over-the-air transmission (OTA)
- Update 1.0  - base library for OTA update
- Blynk 0.6.1 - enables connection to Blynk server for display of all data on a mobile device
- DallasTemperature 3.9.0 - DS18B20 temperature sensor
- OneWire 2.3.5 - base library for access to DS18B20 
- SPI 1.0 - base library for SPI devices (BME280, BME680, OLED, LCD)
- Wire 1.0.1 - base libary for SPI devices 

Dependency Tree in PlatformIO:
![Libraries](https://github.com/88markus88/EnvMonitor/blob/main/EnovMonitor680-Git/Pictures/librariesForEnvMonitor.jpg)

# Arduino

## Libraries

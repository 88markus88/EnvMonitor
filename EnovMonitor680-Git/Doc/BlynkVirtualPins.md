#Blynk Virtual Pins

The Blynk App receives data from Virtual Pins, and sends data to Virtual Pins.
The ESP32 program has to handle the same Virtual Pins via 
- BLYNK_WRITE - from App to ESP32
- Blynk.virtualwrite - from ESP32 to App
These data are provided via a Blynk server. This can either be the Blynk Cloud service, at a cost per widget, or on a local server.

More Information is available on the [Blynk Website](https://blynk.io/)

These virtual Pins are used by EnvMonitor:

**Inputs**
VPin  |Function
------|---------
V40 | Relay 1 (was V18)
V41 | Relay 2 (was V19)
V42 | tempSwitchOffset (Fan on if temp BS680 this much too high)
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

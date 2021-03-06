/******************************************************
* Global defines Header file for EnvMonitor680.cpp
*******************************************************/
 
#define PROGNAME  "EnvMonitorBME680.cpp"
#define PROGVERSION "V0.106"
#define PROGDATE "2022-04-18"


// !!! use only one option that sends or receives data from serial!
#define isBLYNK         // BLYNK Connection enabled
    #undef blynkRestartHouly // if defined, Blynk is restarted on an hourly base, for unsteady connections
    #define blynkRegularCheck // if defined, checkBlynk called() regularly by timer to reconnect
    #undef blynkCloud       // define this to use blynk cloud, undef to use local server
    #define blynkLocalIP    192,168,178,64  // IP address for local Blynk server
    #define blynkTerminal   // terminal output to virtual pin V50 

#define isLocationFFM       //ssid and passwort determined by location, defined in "credentials.h"
#undef isLocationUslar         

 // defines to determine the correct HW configuration, incl. auth string and calibration values. ONE ONLY!
 // #define blynkWebHinkelhurz
 // #define blynkBME680Kueche
 // #define blynkSchlafzimmer 
 // #define blynkEnvLocal2Bad
 // #define blynkInfactoryExternalS  // KombiSensorExt-LCD. LCD in Black Box 
 // #define blynkSenseAirRedBox
  #define blynkKombinsensor1    // KombiSensorExt-OLED. OLED in Black Box, Arduino, BME280, 2 DS18B20 
 // #define blynkExPapaKleineBox
 // #define blynkBME680BreadBoard    // BME680 auf Breadboard
 // #define blynkRedBoxYellowButton
 // #define virtuinoTestbed          // testbed for virtuino and MQTT, started 27.10.21

//***********************************************
// hardware configurations defined here
//***********************************************
#ifdef blynkInfactoryExternalS  // KombiSensorExt-LCD. LCD in Black Eur Box 
    #undef isThingspeak      // Thingspeak connection enabled. Alternative to Blynk
    #undef isVirtuino      // Virtuno connection enabled. Alternative to Blynk
    #define isOTA           // allow OTA over te air updates    
    #undef isMHZ14A        // CO2 Sensor present. communication via serial2
    #undef isSENSEAIR_S8    // alternate CO2 sensor present, communication via serial2
    #define isBME280         // BME 280 Sensor (P, T, %) present
    #undef isBME680        // BME 680 sensor (P, T, %, Gas) present auf I2C, Zanshin_BME680.h Lib
    #undef isBME680_BSECLib // BME 680 Sensor present, use with BSEC Lib
    #define isOneDS18B20    // one or more DS18B20 OneWire temperature sensor present
        #define noDS18B20Sensors 2  // number of DS18B20 expected
    #undef isDisplay       // Adafruit SSD 1306 display is present
    #define isLCD            // LCD display present
    #undef isInfactory433   // Infactory 433 MHz Sender (Type NV-5849, black). Internal connection to ESP32
    #undef isRelay         // relais connected to GPIO 26 (R1) and 27 (R2)
    #undef isFan           // Fan ist connected to RELAYPIN1
    
    #undef isWindowOpenDetector        // run routine to check window open alert
        #undef isSendBlynkWindowOpenAlert  // if alert, send it to another device via Blynk
        #undef isBeeperWindowOpenAlert     // if alert, activate beeper
        #undef isReceiveBlynkWindowOpenAlert    // receive alerts from other Blynk connected units
        #undef isStartupBeepTest  // if enabled, 500 ms Beep during setup
        #define tempSwitchSensorSelector 0  // select DS18B20 for temperature sensing

    #undef sendSERIAL       // enable if data from external sensors (temp, humdity) to be received via serial2
    #define receiveSERIAL    // enable if data to be sent via serial2 (temp, humidity) from Arduino
    #undef serialMonitor    // debugging routine - program does nothing but listen to serial and log it
    #define getNTPTIME       // get time from NTP server
    #undef isSD             // SD Card reader attached

    #undef logSD            // logging to SD card enabled. needs "isSD"
    #define logSerial       // logging to serial enabled
    #undef isLEDHeartbeat      // LED heartbeat on, PIN 14
    #undef isBluetoothCredentials  // get credentials via bluetooth
    #undef isCaptivePortal  // Code for captive portal
       #undef debugCaptivePortal // if defined, captive portal is always used.
    #undef isSyslog        // syslog logging is enabled   
        #define SYSLOG_SERVER "192.168.178.42" //"syslog-server"
        #define SYSLOG_PORT 514
        #define DEVICE_HOSTNAME infoStringShort
        #define APP_NAME PROGNAME
    #define isMeasureRSSI   // measure and transmit the Wifi signal strength (RSSI)   
    #undef isThingspeakRSSI // transmit RSSI via Thingspeak. Not here: ext. humidity on field 4
    static char infoStringLong[] = " KombiSensorExtLCD: Black Eurobox with LCD. Ext433 via serial, BME280, 2 DS18B20";
    static char infoStringShort[] = "KombiSensorExtLCD";
#endif

#ifdef blynkKombinsensor1  // KombiSensorExt-OLED. OLED in Black Box, Arduino, BME280, 2 DS18B20 
    #define isThingspeak      // Thingspeak connection enabled. Alternative to Blynk
    #undef isVirtuino      // Virtuno connection enabled. Alternative to Blynk
    #define isOTA           // allow OTA over te air updates    
    #undef isMHZ14A        // CO2 Sensor present. communication via serial2
    #undef isSENSEAIR_S8    // alternate CO2 sensor present, communication via serial2
    #define isBME280         // BME 280 Sensor (P, T, %) present
    #undef isBME680        // BME 680 sensor (P, T, %, Gas) present auf I2C, Zanshin_BME680.h Lib
    #undef isBME680_BSECLib // BME 680 Sensor present, use with BSEC Lib
    #define isOneDS18B20    // one or more DS18B20 OneWire temperature sensor present
        #define noDS18B20Sensors 2  // number of DS18B20 expected    
    #define isDisplay       // Adafruit SSD 1306 display is present
    #undef isLCD            // LCD display present
    #undef isInfactory433   // Infactory 433 MHz Sender (Type NV-5849, black). Internal connection to ESP32
    #undef isRelay         // relais connected to GPIO 26 (R1) and 27 (R2)
    #undef isFan           // Fan ist connected to RELAYPIN1
    
    #undef isWindowOpenDetector        // run routine to check window open alert
        #undef isSendBlynkWindowOpenAlert  // if alert, send it to another device via Blynk
        #undef isBeeperWindowOpenAlert     // if alert, activate beeper
        #undef isReceiveBlynkWindowOpenAlert    // receive alerts from other Blynk connected units
        #undef isStartupBeepTest  // if enabled, 500 ms Beep during setup
        #define tempSwitchSensorSelector 0  // select DS18B20 for temperature sensing

    #undef sendSERIAL       // enable if data from external sensors (temp, humdity) to be received via serial2
    #define receiveSERIAL    // enable if data to be sent via serial2 (temp, humidity) from Arduino
    #undef serialMonitor    // debugging routine - program does nothing but listen to serial and log it
    #define getNTPTIME       // get time from NTP server
    #undef isSD             // SD Card reader attached

    #undef logSD            // logging to SD card enabled. needs "isSD"
    #define logSerial       // logging to serial enabled
    #undef isLEDHeartbeat      // LED heartbeat on, PIN 14
    #undef isBluetoothCredentials  // get credentials via bluetooth
    #define isCaptivePortal  // Code for captive portal
       #undef debugCaptivePortal // if defined, captive portal is always used.
    #define isSyslog        // syslog logging is enabled   
        #define SYSLOG_SERVER "192.168.178.42" //"syslog-server"
        #define SYSLOG_PORT 514
        #define DEVICE_HOSTNAME infoStringShort
        #define APP_NAME PROGNAME
    #define isMeasureRSSI   // measure and transmit the Wifi signal strength (RSSI)
    #undef isThingspeakRSSI // transmit RSSI via Thingspeak. Not here: ext. humidity on field 4
    static char infoStringLong[] = " KombiSensorExt: Black Velleman Box with OLED. Ext433 via serial, BME280, 2 DS18B20";
    static char infoStringShort[] = "KombiSensorExt";
#endif

#ifdef blynkBME680Kueche  // K??che in black Euro Box 
    #define isThingspeak      // Thingspeak connection enabled. Alternative to Blynk
    #undef isVirtuino      // Virtuno connection enabled. Alternative to Blynk
    #define isOTA           // allow OTA over te air updates    
    #define isMHZ14A        // CO2 Sensor present. communication via serial2
    #undef isSENSEAIR_S8    // alternate CO2 sensor present, communication via serial2
    #undef isBME280         // BME 280 Sensor (P, T, %) present
    #undef isBME680        // BME 680 sensor (P, T, %, Gas) present auf I2C, Zanshin_BME680.h Lib
    #define isBME680_BSECLib // BME 680 Sensor present, use with BSEC Lib
    #define BME_Secondary_Address   // if defined, use secondary address for BME680
    #define isOneDS18B20    // one or more DS18B20 OneWire temperature sensor present
        #define noDS18B20Sensors 3  // number of DS18B20 expected
    #define isDisplay       // Adafruit SSD 1306 display is present
    #undef isLCD            // LCD display present
    #undef isInfactory433   // Infactory 433 MHz Sender (Type NV-5849, black). Internal connection to ESP32
    #define isRelay         // relais connected to GPIO 26 (R1) and 27 (R2)
    #define isFan           // Fan ist connected to RELAYPIN1

    #undef isWindowOpenDetector        // run routine to check window open alert
        #undef isSendBlynkWindowOpenAlert  // if alert, send it to another device via Blynk
        #undef isBeeperWindowOpenAlert     // if alert, activate beeper
        #undef isReceiveBlynkWindowOpenAlert    // receive alerts from other Blynk connected units
        #undef isStartupBeepTest  // if enabled, 500 ms Beep during setup
        #define tempSwitchSensorSelector 0  // select DS18B20 for temperature sensing

    #undef sendSERIAL       // enable if data from external sensors (temp, humdity) to be received via serial2
    #undef receiveSERIAL    // enable if data to be sent via serial2 (temp, humidity) from Arduino
    #undef serialMonitor    // debugging routine - program does nothing but listen to serial and log it
    #define getNTPTIME       // get time from NTP server
    #undef isSD             // SD Card reader attached

    #undef logSD            // logging to SD card enabled. needs "isSD"
    #define logSerial       // logging to serial enabled
    #undef isLEDHeartbeat      // LED heartbeat on, PIN 14
    #undef isBluetoothCredentials  // get credentials via bluetooth
    #define isCaptivePortal  // Code for captive portal
       #undef debugCaptivePortal // if defined, captive portal is always used.
    #undef isSyslog        // syslog logging is enabled   
        #define SYSLOG_SERVER "192.168.178.42" //"syslog-server"
        #define SYSLOG_PORT 514
        #define DEVICE_HOSTNAME infoStringShort
        #define APP_NAME PROGNAME
    #define isMeasureRSSI   // measure and transmit the Wifi signal strength (RSSI)   
    #undef isThingspeakRSSI // transmit RSSI via Thingspeak. Not here: air quality on field 4
    static char infoStringLong[] = " BME680 K??che: Eurobox, OLED, BME680, 3 DS18B20, MH-Z14a, Fan";
    static char infoStringShort[] = "BME680 Kueche";
#endif

#ifdef blynkSenseAirRedBox  // Red Euro Box. OLED, Senseair CO2 sensor, 3 DS18B20
    #define isThingspeak      // Thingspeak connection enabled. Alternative to Blynk
    #undef isVirtuino      // Virtuno connection enabled. Alternative to Blynk
    #define isOTA           // allow OTA over te air updates    
    #undef isMHZ14A        // CO2 Sensor present. communication via serial2
    #define isSENSEAIR_S8    // alternate CO2 sensor present, communication via serial2
    #define isBME280         // BME 280 Sensor (P, T, %) present
    #undef isBME680        // BME 680 sensor (P, T, %, Gas) present auf I2C, Zanshin_BME680.h Lib
    #undef isBME680_BSECLib // BME 680 Sensor present, use with BSEC Lib   
    #undef BME_Secondary_Address   // if defined, use secondary address for BME680
    #define isOneDS18B20    // one or more DS18B20 OneWire temperature sensor present
        #define noDS18B20Sensors 3  // number of DS18B20 expected
    #define isDisplay       // Adafruit SSD 1306 display is present
    #undef isLCD            // LCD display present
    #undef isInfactory433   // Infactory 433 MHz Sender (Type NV-5849, black). Internal connection to ESP32
    #undef isRelay         // relais connected to GPIO 26 (R1) and 27 (R2)
    #undef isFan           // Fan ist connected to RELAYPIN1

    #define isWindowOpenDetector        // run routine to check window open alert
        #define isSendBlynkWindowOpenAlert  // if alert, send it to another device via Blynk (auth: authAlertReceiver[])
        #define isBeeperWindowOpenAlert     // if alert, activate beeper
        #define isReceiveBlynkWindowOpenAlert    // receive alerts from other Blynk connected units
        #undef isStartupBeepTest  // if enabled, 500 ms Beep during setup
        #define tempSwitchSensorSelector 0  // select DS18B20 for temperature sensing

    #undef sendSERIAL       // enable if data from external sensors (temp, humdity) to be received via serial2
    #undef receiveSERIAL    // enable if data to be sent via serial2 (temp, humidity) from Arduino
    #undef serialMonitor    // debugging routine - program does nothing but listen to serial and log it
    #define getNTPTIME       // get time from NTP server
    #undef isSD             // SD Card reader attached

    #undef logSD            // logging to SD card enabled. needs "isSD"
    #define logSerial       // logging to serial enabled
    #undef isLEDHeartbeat      // LED heartbeat on, Pin 14
    #undef isBluetoothCredentials  // get credentials via bluetooth
    #undef isCaptivePortal  // Code for captive portal
       #undef debugCaptivePortal // if defined, captive portal is always used.
    #undef isSyslog        // syslog logging is enabled   
        #define SYSLOG_SERVER "192.168.178.42" //"syslog-server"
        #define SYSLOG_PORT 514
        #define DEVICE_HOSTNAME infoStringShort
        #define APP_NAME PROGNAME
    #define isMeasureRSSI   // measure and transmit the Wifi signal strength (RSSI)   
    #define isThingspeakRSSI // transmit RSSI via Thingspeak 
    static char infoStringLong[] = " SenseAirRedBox: Eurobox, OLED, BME280, 3 DS18B20, SenseAir S8, Beeper";
    static char infoStringShort[] = " SenseAirRedBox";
#endif

#ifdef  blynkEnvLocal2Bad // Bad: small box, no display
    #define isThingspeak      // Thingspeak connection enabled. Alternative to Blynk
    #undef isVirtuino      // Virtuno connection enabled. Alternative to Blynk
    #define isOTA           // allow OTA over te air updates    
    #undef isMHZ14A        // CO2 Sensor present. communication via serial2
    #undef isSENSEAIR_S8    // alternate CO2 sensor present, communication via serial2
    #define isBME280         // BME 280 Sensor (P, T, %) present
    #undef isBME680        // BME 680 sensor (P, T, %, Gas) present auf I2C, Zanshin_BME680.h Lib
    #undef isBME680_BSECLib // BME 680 Sensor present, use with BSEC Lib   
    #define isOneDS18B20    // one or more DS18B20 OneWire temperature sensor present
                #define noDS18B20Sensors 3  // number of DS18B20 expected
    #undef isDisplay       // Adafruit SSD 1306 display is present
    #undef isLCD            // LCD display present
    #undef isInfactory433   // Infactory 433 MHz Sender (Type NV-5849, black). Internal connection to ESP32
    #undef isRelay         // relais connected to GPIO 26 (R1) and 27 (R2)
    #undef isFan           // Fan ist connected to RELAYPIN1
        
    #undef isWindowOpenDetector        // run routine to check window open alert
        #undef isSendBlynkWindowOpenAlert  // if alert, send it to another device via Blynk
        #undef isBeeperWindowOpenAlert     // if alert, activate beeper
        #undef isReceiveBlynkWindowOpenAlert    // receive alerts from other Blynk connected units
        #undef isStartupBeepTest  // if enabled, 500 ms Beep during setup
        #define tempSwitchSensorSelector 0  // select DS18B20 for temperature sensing

    #undef sendSERIAL       // enable if data from external sensors (temp, humdity) to be received via serial2
    #undef receiveSERIAL    // enable if data to be sent via serial2 (temp, humidity) from Arduino
    #undef serialMonitor    // debugging routine - program does nothing but listen to serial and log it
    #define getNTPTIME       // get time from NTP server
    #undef isSD             // SD Card reader attached

    #undef logSD            // logging to SD card enabled. needs "isSD"
    #define logSerial       // logging to serial enabled
    #undef isLEDHeartbeat      // LED heartbeat on, Pin 14
    #undef isBluetoothCredentials  // get credentials via bluetooth
    #undef isCaptivePortal  // Code for captive portal
       #undef debugCaptivePortal // if defined, captive portal is always used.
    #undef isSyslog        // syslog logging is enabled   
        #define SYSLOG_SERVER "192.168.178.42" //"syslog-server"
        #define SYSLOG_PORT 514
        #define DEVICE_HOSTNAME infoStringShort
        #define APP_NAME PROGNAME
    #define isMeasureRSSI   // measure and transmit the Wifi signal strength (RSSI)    
    #define isThingspeakRSSI // transmit RSSI via Thingspeak
    static char infoStringLong[] = " Small Sensor Bad: Small Box, BME280, 2 DS18B20";
    static char infoStringShort[] = " Small Sensor Bad";
#endif

#ifdef  blynkSchlafzimmer // Schlafzimmer: small box, OLED display
    #define isThingspeak      // Thingspeak connection enabled. Alternative to Blynk
    #undef isVirtuino      // Virtuno connection enabled. Alternative to Blynk
    #define isOTA           // allow OTA over te air updates    
    #undef isMHZ14A        // CO2 Sensor present. communication via serial2
    #undef isSENSEAIR_S8    // alternate CO2 sensor present, communication via serial2
    #define isBME280         // BME 280 Sensor (P, T, %) present
    #undef isBME680        // BME 680 sensor (P, T, %, Gas) present auf I2C, Zanshin_BME680.h Lib
    #undef isBME680_BSECLib // BME 680 Sensor present, use with BSEC Lib
    #define isOneDS18B20    // one or more DS18B20 OneWire temperature sensor present
        #define noDS18B20Sensors 2  // number of DS18B20 expected
    #define isDisplay       // Adafruit SSD 1306 display is present
    #undef isLCD            // LCD display present
    #undef isInfactory433   // Infactory 433 MHz Sender (Type NV-5849, black). Internal connection to ESP32
    #undef isRelay         // relais connected to GPIO 26 (R1) and 27 (R2)
    #undef isFan           // Fan ist connected to RELAYPIN1
    
    #undef isWindowOpenDetector        // run routine to check window open alert
        #undef isSendBlynkWindowOpenAlert  // if alert, send it to another device via Blynk
        #undef isBeeperWindowOpenAlert     // if alert, activate beeper
        #undef isReceiveBlynkWindowOpenAlert    // receive alerts from other Blynk connected units
        #undef isStartupBeepTest  // if enabled, 500 ms Beep during setup
        #define tempSwitchSensorSelector 0  // select DS18B20 for temperature sensing

    #undef sendSERIAL       // enable if data from external sensors (temp, humdity) to be received via serial2
    #undef receiveSERIAL    // enable if data to be sent via serial2 (temp, humidity) from Arduino
    #undef serialMonitor    // debugging routine - program does nothing but listen to serial and log it
    #define getNTPTIME       // get time from NTP server
    #undef isSD             // SD Card reader attached

    #undef logSD            // logging to SD card enabled. needs "isSD"
    #define logSerial       // logging to serial enabled
    #undef isLEDHeartbeat      // LED heartbeat on, Pin 14
    #undef isBluetoothCredentials  // get credentials via bluetooth
    #undef isCaptivePortal  // Code for captive portal
       #undef debugCaptivePortal // if defined, captive portal is always used.
    #undef isSyslog        // syslog logging is enabled   
        #define SYSLOG_SERVER "192.168.178.42" //"syslog-server"
        #define SYSLOG_PORT 514
        #define DEVICE_HOSTNAME infoStringShort
        #define APP_NAME PROGNAME
    #define isMeasureRSSI   // measure and transmit the Wifi signal strength (RSSI)    
    #define isThingspeakRSSI // transmit RSSI via Thingspeak
    static char infoStringLong[] = " Small Sensor Schlafzimmer: Small Box, OLED, BME280, 2 DS18B20";
    static char infoStringShort[] = "Schlafzimmer";
#endif


#ifdef  blynkExPapaKleineBox // Ex Papa kleine Box : small box, OLED display, 2 DS18B20, BME280
    #define isThingspeak      // Thingspeak connection enabled. Alternative to Blynk
    #undef isVirtuino      // Virtuno connection enabled. Alternative to Blynk
    #define isOTA           // allow OTA over te air updates    
    #undef isMHZ14A        // CO2 Sensor present. communication via serial2
    #undef isSENSEAIR_S8    // alternate CO2 sensor present, communication via serial2
    #define isBME280         // BME 280 Sensor (P, T, %) present
    #undef isBME680        // BME 680 sensor (P, T, %, Gas) present auf I2C, Zanshin_BME680.h Lib
    #undef isBME680_BSECLib // BME 680 Sensor present, use with BSEC Lib
    #define isOneDS18B20    // one or more DS18B20 OneWire temperature sensor present
        #define noDS18B20Sensors 2  // number of DS18B20 expected
    #define isDisplay       // Adafruit SSD 1306 display is present
    #undef isLCD            // LCD display present
    #undef isInfactory433   // Infactory 433 MHz Sender (Type NV-5849, black). Internal connection to ESP32
    #undef isRelay         // relais connected to GPIO 26 (R1) and 27 (R2)
    #undef isFan           // Fan ist connected to RELAYPIN1

    #undef isWindowOpenDetector        // run routine to check window open alert
        #undef isSendBlynkWindowOpenAlert  // if alert, send it to another device via Blynk
        #undef isBeeperWindowOpenAlert     // if alert, activate beeper
        #undef isReceiveBlynkWindowOpenAlert    // receive alerts from other Blynk connected units
        #undef isStartupBeepTest  // if enabled, 500 ms Beep during setup
        #define tempSwitchSensorSelector 0  // select DS18B20 for temperature sensing

    #undef sendSERIAL       // enable if data from external sensors (temp, humdity) to be received via serial2
    #undef receiveSERIAL    // enable if data to be sent via serial2 (temp, humidity) from Arduino
    #undef serialMonitor    // debugging routine - program does nothing but listen to serial and log it
    #define getNTPTIME       // get time from NTP server
    #undef isSD             // SD Card reader attached

    #undef logSD            // logging to SD card enabled. needs "isSD"
    #define logSerial       // logging to serial enabled
    #undef isLEDHeartbeat      // LED heartbeat on, Pin 14
    #undef isBluetoothCredentials  // get credentials via bluetooth
    #undef isCaptivePortal  // Code for captive portal
       #undef debugCaptivePortal // if defined, captive portal is always used.
    #undef isSyslog        // syslog logging is enabled   
        #define SYSLOG_SERVER "192.168.178.42" //"syslog-server"
        #define SYSLOG_PORT 514
        #define DEVICE_HOSTNAME infoStringShort
        #define APP_NAME PROGNAME
    #define isMeasureRSSI   // measure and transmit the Wifi signal strength (RSSI)  
    #define isThingspeakRSSI // transmit RSSI via Thingspeak  
    static char infoStringLong[] = " Small Sensor ExHans: Small Box, OLED, BME280, 2 DS18B20";
    static char infoStringShort[] = "Small Sensor ExHans";
#endif

#ifdef blynkBME680BreadBoard  // EnvMonitor BME680 Testbead on Breadboard
    #define isThingspeak      // Thingspeak connection enabled. Alternative to Blynk
    #undef isVirtuino      // Virtuno connection enabled. Alternative to Blynk
    #define isOTA           // allow OTA over te air updates    
    #undef isMHZ14A        // CO2 Sensor present. communication via serial2
    #undef isSENSEAIR_S8    // alternate CO2 sensor present, communication via serial2
    #undef isBME280         // BME 280 Sensor (P, T, %) present
    #undef isBME680        // BME 680 sensor (P, T, %, Gas) present auf I2C, Zanshin_BME680.h Lib
    #define isBME680_BSECLib // BME 680 Sensor present, use with BSEC Lib
    #undef BME_Secondary_Address   // if defined, use secondary address for BME680
    #define isOneDS18B20    // one or more DS18B20 OneWire temperature sensor present
        #define noDS18B20Sensors 1  // number of DS18B20 expected
    #undef isDisplay       // Adafruit SSD 1306 display is present
    #undef isLCD            // LCD display present
    #undef isInfactory433   // Infactory 433 MHz Sender (Type NV-5849, black). Internal connection to ESP32
    #define isRelay         // relais connected to GPIO 26 (R1) and 27 (R2)
    #undef isFan           // Fan ist connected to RELAYPIN1

    #undef isWindowOpenDetector        // run routine to check window open alert
        #define isSendBlynkWindowOpenAlert  // if alert, send it to another device via Blynk (auth: authAlertReceiver[])
        #undef isBeeperWindowOpenAlert     // if alert, activate beeper
        #undef isReceiveBlynkWindowOpenAlert    // receive alerts from other Blynk connected units
        #undef isStartupBeepTest  // if enabled, 500 ms Beep during setup
        #define tempSwitchSensorSelector 0  // select DS18B20 for temperature sensing

    #undef sendSERIAL       // enable if data from external sensors (temp, humdity) to be received via serial2
    #undef receiveSERIAL    // enable if data to be sent via serial2 (temp, humidity) from Arduino
    #undef serialMonitor    // debugging routine - program does nothing but listen to serial and log it
    #define getNTPTIME       // get time from NTP server
    #undef isSD             // SD Card reader attached

    #undef logSD            // logging to SD card enabled. needs "isSD"
    #define logSerial       // logging to serial enabled
    #define isLEDHeartbeat      // LED heartbeat on, Pin 14
    #undef isBluetoothCredentials  // get credentials via bluetooth
    #define isCaptivePortal  // Code for captive portal
        #undef debugCaptivePortal // if defined, captive portal is always used.
    #define isSyslog        // syslog logging is enabled   
        #define SYSLOG_SERVER "192.168.178.42" //"syslog-server"
        #define SYSLOG_PORT 514
        #define DEVICE_HOSTNAME infoStringShort
        #define APP_NAME PROGNAME
    #define isMeasureRSSI   // measure and transmit the Wifi signal strength (RSSI)   
    #undef isThingspeakRSSI // transmit RSSI via Thingspeak. not here, air quality on field 4
    static char infoStringLong[] = " BME680 Breadboard: BME680, 1 DS18B20 auf Breadboard";
    static char infoStringShort[] = "BME680 Breadboard";
#endif

#ifdef blynkRedBoxYellowButton    // was blynkGeneralTestbed
    #define isThingspeak      // Thingspeak connection enabled. Alternative to Blynk
    #undef isVirtuino      // Virtuno connection enabled. Alternative to Blynk
    #define isOTA           // allow OTA over te air updates    
    #undef isMHZ14A        // CO2 Sensor present. communication via serial2
    #define isSENSEAIR_S8    // alternate CO2 sensor present, communication via serial2
    #define isBME280         // BME 280 Sensor (P, T, %) present
    #undef isBME680        // BME 680 sensor (P, T, %, Gas) present auf I2C, Zanshin_BME680.h Lib
    #undef isBME680_BSECLib // BME 680 Sensor present, use with BSEC Lib
    #undef BME_Secondary_Address   // if defined, use secondary address for BME680
    #define isOneDS18B20    // one or more DS18B20 OneWire temperature sensor present
        #define noDS18B20Sensors 3  // number of DS18B20 expected
    #define isDisplay       // Adafruit SSD 1306 display is present
    #undef isLCD            // LCD display present
    #undef isInfactory433   // Infactory 433 MHz Sender (Type NV-5849, black). Internal connection to ESP32
    #define isRelay         // relais connected to GPIO 26 (Fan via Transistor R1) and 27 (R2)
    #undef isFan           // Fan ist connected to RELAYPIN1

    #define isWindowOpenDetector        // run routine to check window open alert
        #define isSendBlynkWindowOpenAlert  // if alert, send it to another device via Blynk (auth: authAlertReceiver[])
        #define isBeeperWindowOpenAlert     // if alert, activate beeper
        #define isReceiveBlynkWindowOpenAlert    // receive alerts from other Blynk connected units
        #undef isStartupBeepTest  // if enabled, 500 ms Beep during setup
        #define tempSwitchSensorSelector 1  // select DS18B20 for temperature sensing

    #undef sendSERIAL       // enable if data from external sensors (temp, humdity) to be received via serial2
    #undef receiveSERIAL    // enable if data to be sent via serial2 (temp, humidity) from Arduino
    #undef serialMonitor    // debugging routine - program does nothing but listen to serial and log it
    #define getNTPTIME       // get time from NTP server
    #undef isSD             // SD Card reader attached

    #undef logSD            // logging to SD card enabled. needs "isSD"
    #define logSerial       // logging to serial enabled
    #undef isLEDHeartbeat      // LED heartbeat on, Pin 14
    #undef isBluetoothCredentials  // get credentials via bluetooth
    #define isCaptivePortal  // Code for captive portal
       #undef debugCaptivePortal // if defined, captive portal is always used.
    #undef isSyslog        // syslog logging is enabled   
        #define SYSLOG_SERVER "192.168.178.42" //"syslog-server"
        #define SYSLOG_PORT 514
        #define DEVICE_HOSTNAME  infoStringShort // "syslog_hostname" //
        #define APP_NAME PROGVERSION // "syslog_appname" //
    #define isMeasureRSSI   // measure and transmit the Wifi signal strength (RSSI)   
    #define isThingspeakRSSI // transmit RSSI via Thingspeak    
    static char infoStringLong[] = "Red Box Yellow Button with SenseAir";// " General Testbed for all kinds of stuff";
    static char infoStringShort[] = "RedBoxYellowBtn"; // "GeneralTestbed";
#endif 

#ifdef virtuinoTestbed  // Virtuino and MQTT Testbed
    #undef isBLYNK          // this one without Blynk
    #define isVirtuino      // this one is with Virtuino
    #undef isThingspeak      // Thingspeak connection enabled. Alternative to Blynk

    #define isOTA           // allow OTA over te air updates    
    #undef isMHZ14A        // CO2 Sensor present. communication via serial2
    #undef isSENSEAIR_S8    // alternate CO2 sensor present, communication via serial2
    #undef isBME280         // BME 280 Sensor (P, T, %) present
    #undef isBME680        // BME 680 sensor (P, T, %, Gas) present auf I2C, Zanshin_BME680.h Lib
    #define isBME680_BSECLib // BME 680 Sensor present, use with BSEC Lib
    #undef BME_Secondary_Address   // if defined, use secondary address for BME680
    #define isOneDS18B20    // one or more DS18B20 OneWire temperature sensor present
        #define noDS18B20Sensors 1  // number of DS18B20 expected
    #undef isDisplay       // Adafruit SSD 1306 display is present
    #undef isLCD            // LCD display present
    #undef isInfactory433   // Infactory 433 MHz Sender (Type NV-5849, black). Internal connection to ESP32
    #define isRelay         // relais connected to GPIO 26 (R1) and 27 (R2)
    #undef isFan           // Fan ist connected to RELAYPIN1
    #undef sendSERIAL       // enable if data from external sensors (temp, humdity) to be received via serial2
    #undef receiveSERIAL    // enable if data to be sent via serial2 (temp, humidity) from Arduino
    #undef serialMonitor    // debugging routine - program does nothing but listen to serial and log it
    #define getNTPTIME       // get time from NTP server
    #undef isSD             // SD Card reader attached

    #undef logSD            // logging to SD card enabled. needs "isSD"
    #define logSerial       // logging to serial enabled
    #define isLEDHeartbeat      // LED heartbeat on, Pin 14
    #undef isBluetoothCredentials  // get credentials via bluetooth
    #define isCaptivePortal  // Code for captive portal
       #undef debugCaptivePortal // if defined, captive portal is always used.
    #undef isSyslog        // syslog logging is enabled   
        #define SYSLOG_SERVER "192.168.178.42" //"syslog-server"
        #define SYSLOG_PORT 514
        #define DEVICE_HOSTNAME infoStringShort
        #define APP_NAME PROGNAME
    #define isMeasureRSSI   // measure and transmit the Wifi signal strength (RSSI) 
    #define isThingspeakRSSI // transmit RSSI via Thingspeak   
    static char infoStringLong[] = " Virtuino Testbed: BME680 und 1 DS18B20 auf V0.4 Platine";
    static char infoStringShort[] = "Virtuino Testbed";
#endif 
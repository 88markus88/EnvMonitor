/******************************************************
* Global defines Header file for EnvMonitor680.cpp
*******************************************************/
 
#define PROGNAME  "EnvMonitorBME680.cpp"
#define PROGVERSION "V0.45"
#define PROGDATE "2021-05-15"

// !!! use only one option that sends or receives data from serial!
#define isBLYNK         // BLYNK Connection enabled
    #undef blynkRestartHouly // if defined, Blynk is restarted on an hourly base, for unsteady connections
    #define blynkRegularCheck // if defined, checkBlynk called() regularly by timer to reconnect
    #undef blynkCloud       // define this to use blynk cloud, undef to use local server
    #define blynkLocalIP    192,168,178,64  // IP address for local Blynk server
    #define blynkTerminal   // terminal output to virtual pin V50 

 // defines to determine the correct HW configuration, incl. auth string and calibration values. ONE ONLY!
 // #define blynkWebHinkelhurz
  #define blynkBME680Kueche
 // #define blynkSchlafzimmer 
 // #define blynkEnvLocal2Bad
 // #define blynkInfactoryExternalS  // KombiSensorExt-LCD. LCD in Black Box 
 // #define blynkSenseAirRedBox
 // #define blynkKombinsensor1

//***********************************************
// hardware configurations defined here
//***********************************************
#ifdef blynkInfactoryExternalS  // KombiSensorExt-LCD. LCD in Black Eur Box 
    #define isOTA           // allow OTA over te air updates    
    #undef isMHZ14A        // CO2 Sensor present. communication via serial2
    #undef isSENSEAIR_S8    // alternate CO2 sensor present, communication via serial2
    #define isBME280         // BME 280 Sensor (P, T, %) present
    #undef isBME680        // BME 680 sensor (P, T, %, Gas) present auf I2C, Zanshin_BME680.h Lib
    #undef isBME680_BSECLib // BME 680 Sensor present, use with BSEC Lib
    #define isOneDS18B20    // one or more DS18B20 OneWire temperature sensor present
    #undef isDisplay       // Adafruit SSD 1306 display is present
    #define isLCD            // LCD display present
    #undef isInfactory433   // Infactory 433 MHz Sender (Type NV-5849, black). Internal connection to ESP32
    #undef isRelay         // relais connected to GPIO 26 (R1) and 27 (R2)
    #undef sendSERIAL       // enable if data from external sensors (temp, humdity) to be received via serial2
    #define receiveSERIAL    // enable if data to be sent via serial2 (temp, humidity) from Arduino
    #undef serialMonitor    // debugging routine - program does nothing but listen to serial and log it
    #define getNTPTIME       // get time from NTP server
    #undef isSD             // SD Card reader attached

    #undef logSD            // logging to SD card enabled. needs "isSD"
    #define logSerial       // logging to serial enabled
#endif

#ifdef blynkKombinsensor1  // KombiSensorExt-OLED. OLED in Black Box, Arduino, BME280, 3 DS18B20 
    #define isOTA           // allow OTA over te air updates    
    #undef isMHZ14A        // CO2 Sensor present. communication via serial2
    #undef isSENSEAIR_S8    // alternate CO2 sensor present, communication via serial2
    #define isBME280         // BME 280 Sensor (P, T, %) present
    #undef isBME680        // BME 680 sensor (P, T, %, Gas) present auf I2C, Zanshin_BME680.h Lib
    #undef isBME680_BSECLib // BME 680 Sensor present, use with BSEC Lib
    #define isOneDS18B20    // one or more DS18B20 OneWire temperature sensor present
    #define isDisplay       // Adafruit SSD 1306 display is present
    #unfine isLCD            // LCD display present
    #undef isInfactory433   // Infactory 433 MHz Sender (Type NV-5849, black). Internal connection to ESP32
    #undef isRelay         // relais connected to GPIO 26 (R1) and 27 (R2)
    #undef sendSERIAL       // enable if data from external sensors (temp, humdity) to be received via serial2
    #define receiveSERIAL    // enable if data to be sent via serial2 (temp, humidity) from Arduino
    #undef serialMonitor    // debugging routine - program does nothing but listen to serial and log it
    #define getNTPTIME       // get time from NTP server
    #undef isSD             // SD Card reader attached

    #undef logSD            // logging to SD card enabled. needs "isSD"
    #define logSerial       // logging to serial enabled
#endif

#ifdef blynkBME680Kueche  // Küche in black Euro Box 
    #define isOTA           // allow OTA over te air updates    
    #define isMHZ14A        // CO2 Sensor present. communication via serial2
    #undef isSENSEAIR_S8    // alternate CO2 sensor present, communication via serial2
    #undef isBME280         // BME 280 Sensor (P, T, %) present
    #undef isBME680        // BME 680 sensor (P, T, %, Gas) present auf I2C, Zanshin_BME680.h Lib
    #define isBME680_BSECLib // BME 680 Sensor present, use with BSEC Lib
    #define isOneDS18B20    // one or more DS18B20 OneWire temperature sensor present
    #define isDisplay       // Adafruit SSD 1306 display is present
    #undef isLCD            // LCD display present
    #undef isInfactory433   // Infactory 433 MHz Sender (Type NV-5849, black). Internal connection to ESP32
    #define isRelay         // relais connected to GPIO 26 (R1) and 27 (R2)
    #undef sendSERIAL       // enable if data from external sensors (temp, humdity) to be received via serial2
    #undef receiveSERIAL    // enable if data to be sent via serial2 (temp, humidity) from Arduino
    #undef serialMonitor    // debugging routine - program does nothing but listen to serial and log it
    #define getNTPTIME       // get time from NTP server
    #undef isSD             // SD Card reader attached

    #undef logSD            // logging to SD card enabled. needs "isSD"
    #define logSerial       // logging to serial enabled
#endif

#ifdef blynkSenseAirRedBox  // Red Euro Box. OLED, Senseair CO2 sensor, 3 DS18B20
    #define isOTA           // allow OTA over te air updates    
    #undef isMHZ14A        // CO2 Sensor present. communication via serial2
    #define isSENSEAIR_S8    // alternate CO2 sensor present, communication via serial2
    #define isBME280         // BME 280 Sensor (P, T, %) present
    #undef isBME680        // BME 680 sensor (P, T, %, Gas) present auf I2C, Zanshin_BME680.h Lib
    #undef isBME680_BSECLib // BME 680 Sensor present, use with BSEC Lib   
    #define isOneDS18B20    // one or more DS18B20 OneWire temperature sensor present
    #define isDisplay       // Adafruit SSD 1306 display is present
    #undef isLCD            // LCD display present
    #undef isInfactory433   // Infactory 433 MHz Sender (Type NV-5849, black). Internal connection to ESP32
    #undef isRelay         // relais connected to GPIO 26 (R1) and 27 (R2)
    #undef sendSERIAL       // enable if data from external sensors (temp, humdity) to be received via serial2
    #undef receiveSERIAL    // enable if data to be sent via serial2 (temp, humidity) from Arduino
    #undef serialMonitor    // debugging routine - program does nothing but listen to serial and log it
    #define getNTPTIME       // get time from NTP server
    #undef isSD             // SD Card reader attached

    #undef logSD            // logging to SD card enabled. needs "isSD"
    #define logSerial       // logging to serial enabled
#endif

#ifdef  blynkEnvLocal2Bad // Bad: small box, no display
    #define isOTA           // allow OTA over te air updates    
    #undef isMHZ14A        // CO2 Sensor present. communication via serial2
    #undef isSENSEAIR_S8    // alternate CO2 sensor present, communication via serial2
    #define isBME280         // BME 280 Sensor (P, T, %) present
    #undef isBME680        // BME 680 sensor (P, T, %, Gas) present auf I2C, Zanshin_BME680.h Lib
    #undef isBME680_BSECLib // BME 680 Sensor present, use with BSEC Lib   
    #define isOneDS18B20    // one or more DS18B20 OneWire temperature sensor present
    #undef isDisplay       // Adafruit SSD 1306 display is present
    #undef isLCD            // LCD display present
    #undef isInfactory433   // Infactory 433 MHz Sender (Type NV-5849, black). Internal connection to ESP32
    #undef isRelay         // relais connected to GPIO 26 (R1) and 27 (R2)
    #undef sendSERIAL       // enable if data from external sensors (temp, humdity) to be received via serial2
    #undef receiveSERIAL    // enable if data to be sent via serial2 (temp, humidity) from Arduino
    #undef serialMonitor    // debugging routine - program does nothing but listen to serial and log it
    #define getNTPTIME       // get time from NTP server
    #undef isSD             // SD Card reader attached

    #undef logSD            // logging to SD card enabled. needs "isSD"
    #define logSerial       // logging to serial enabled
#endif

#ifdef  blynkSchlafzimmer // Schlafzimmer: small box, OLED display
    #define isOTA           // allow OTA over te air updates    
    #undef isMHZ14A        // CO2 Sensor present. communication via serial2
    #undef isSENSEAIR_S8    // alternate CO2 sensor present, communication via serial2
    #define isBME280         // BME 280 Sensor (P, T, %) present
    #undef isBME680        // BME 680 sensor (P, T, %, Gas) present auf I2C, Zanshin_BME680.h Lib
    #undef isBME680_BSECLib // BME 680 Sensor present, use with BSEC Lib
    #define isOneDS18B20    // one or more DS18B20 OneWire temperature sensor present
    #define isDisplay       // Adafruit SSD 1306 display is present
    #undef isLCD            // LCD display present
    #undef isInfactory433   // Infactory 433 MHz Sender (Type NV-5849, black). Internal connection to ESP32
    #undef isRelay         // relais connected to GPIO 26 (R1) and 27 (R2)
    #undef sendSERIAL       // enable if data from external sensors (temp, humdity) to be received via serial2
    #undef receiveSERIAL    // enable if data to be sent via serial2 (temp, humidity) from Arduino
    #undef serialMonitor    // debugging routine - program does nothing but listen to serial and log it
    #define getNTPTIME       // get time from NTP server
    #undef isSD             // SD Card reader attached

    #undef logSD            // logging to SD card enabled. needs "isSD"
    #define logSerial       // logging to serial enabled
#endif
/******************************************************
* Main Header file for EnvMonitor680.cpp
*******************************************************/

//*** global defined constants
#define WDT_TIMEOUT_SECONDS 55  // 40 seconds watchdog timeout. Not too short, or the chip is dead!

//Message ID Codes
#define msgDefaultID          0

#define msgDS18B20NoMeasFlag  1
#define msgDS18B20Info        2
#define msgDS18B20Restart     3
#define msgDS18B20NotMeasuring 4

#define msgLCDInfo            10

#define msgOLEDInfo           20

#define msgTimeInfo           30

#define msgProgInfo           50

#define msgBlynkInfo          60
#define msgBlynkConnected     61
#define msgBlynkNotConnected  62
#define msgBlynkRestart       63
#define msgBlynkSend          64

#define msgWifiConnected      70
#define msgWifiNotConnected   71

#define msgThingspeakInfo     80

#define msgRelayInfo          90
#define msgTempOffsetChange   91

#define msgBME680Info         100
#define msgBME680NotFound     101
#define msgBME680Reset        102
#define msgBME680Error        103

#define msgSetupInfo          110

#define msgSerialReceived     120
#define msgSerialFaulty       121
#define msgReceiveSerialInfo  140

#define msgThingspeakSend     130

#define msgInfactoryInfo      150
#define msgInfactoryReadNVS   151
#define msgInfactorySyncing   152
#define msgInfactoryEvaluating 153
#define msgInfactoryNoData    154

#define msgVirtuinoSend       160

#define msgBME280Info         170

#define msgMHZ14aInfo         180
#define msgMHZ14aWarmup       181

#define msgSenseAirInfo       190
#define msgSenseAirMissing    191

#define msgAlertReceived      200
#define msgBeeperTriggered    201

#define msgWiFiRssiInfo       210

#define msgMQTTInfo           220
#define msgMQTTError          221
#define msgMQTTSend           222
#define msgMQTTSendDS10B20    223
#define msgMQTTReceive        225
#define msgMQTTConnect        226
#define msgMQTTSubscribe      227
#define msgMQTTState          228

//Message Severities
#define msgDefault  0
#define msgInfo     1
#define msgWarn     2
#define msgErr      3
#define msgStop     4

// defines for MQTT to build topic string
// sensor types and associates measurement values
#define mqttSensorBME280      "BME280"
#define mqttSensorBME680      "BME680"
#define mqttBMETemperature    "Temperature"
#define mqttBMEHumidity       "Humidity"
#define mqttBMEPressure       "Pressure"
#define mqttBMEAirQuality     "AirQuality"

#define mqttSensorDS18B20     "DS18B20"
#define mqttDS18B20Temperature "DSTemperature"
#define mqttDS18B20Temperature1 "DSTemperature1"
#define mqttDS18B20Temperature2 "DSTemperature2"
#define mqttDS18B20Temperature3 "DSTemperature3"

#define mqttSensorSenseAirS8  "SenseAirS8"
#define mqttSensorMHZ14A      "MHZ14A"
#define mqttCO2ppm            "CO2ppm"

#define mqttSensorInfactory433        "Infactory433"
#define mqttInfactory433Temperature1  "TempCh1"
#define mqttInfactory433Humidity1     "HumidCh1"
#define mqttInfactory433Temperature2  "TempCh2"
#define mqttInfactory433Humidity2     "HumidCh2"

// general rule for globals: defined in module defining their function,
// otherwise external.

//** module global variables
long startTime;
int NoReboots;
Preferences preferences;    // Nonvolatile storage on ESP32 - To store NoReboots

char printstring[180];
char printstring1[80];
char printstring2[180];
char printstring3[80];

//*** module global constants
const float  SEA_LEVEL_PRESSURE = 1013.25;         ///< Standard atmosphere sea level pressure
const int PushButton = 15;  // GPIO 15 for Pushbutton

#ifdef isLEDHeartbeat
  #define HEARTBEATPIN 14
  int heartbeatStatus = 0;
#endif

// global, since used by #isFan and #isWindowOpenDetector 
volatile float tempSwitchOffset = 2.5;     // at this offset the fan is switched off. 2.4 - 2.6 proven ok
// int tempSwitchSensorSelector = 0; // index of DS18B20 used for fan and beeper switching

#if defined isRelay || defined isBeeperWindowOpenAlert // relais or alert beeper connected to GPIO 26 (R1) and 27 (R2)
  #define RELAYPIN1 26
  #define RELAYPIN2 27

  #ifdef isFan
    SimpleTimer fanHandlerTimer;
    int fanTimerHandle;               // timer handle for fan handling        
    #define bme680FanHandlerInterval  500L
    int fanState = 0;                 // present state of fan
    float fanMaxPotential =0.3;       // maximum potential in % of tempSwitchOffset for fan to cool sensor   
          // if 0.3 and tempSwitch Offset = 2.5, fan is switched off at BME680 0.7*2.5 = 1,75°C above room temp  
   #endif // isFan       
#endif  // isRelay

#ifdef isWindowOpenDetector
  SimpleTimer windowOpenHandlerTimer;
  int windowOpenTimerHandle;               // timer handle for fan handling        
  // #define windowOpenHandlerInterval  1000
  int windowOpenHandlerInterval = 2000;
  int beeperState = 0;              // present state of beeper
  volatile int beeperQuietCounter = 0;       // does not beep if this one is above 0 (quiet if button pressed)
  int beeperQuietCycles = 60;       // keeps quiet for this number of cycles if button pressed
  float temperatureAverage = -111;   // long term average of temperature
  long int temperatureAverageNumber = 200;  // over this number of measurements the average is taken
  long int temperatureAverageCounter =0;     // counter for temperatures taken
  float temperatureDropTrigger = 4.0;       // if temperature drops more than this below average, beeper is triggered
#endif

#if defined isReceiveBlynkWindowOpenAlert || defined isWindowOpenDetector
  int externalAlertState = 0;         // indicates that there is an external alert via bridge
#endif

#ifdef getNTPTIME
  // get time from NTP server
  // https://randomnerdtutorials.com/esp32-date-time-ntp-client-server-arduino/
  // const char* ntpServer = "fritz.box";
  // const char* ntpServer2 = "192.168.178.1";
const char* ntpServer = "pool.ntp.org";
const char* ntpServer2 = "ptbtime3.ptb.de";
const char* ntpServer3 = "fritz.box";
  // const char* ntpServer = "192.168.178.1";
  // const char* ntpServer2 = "pool.ntp.org";
  // const char* ntpServer3 = "ptbtime3.ptb.de";

  const long  gmtOffset_sec = 3600;             // Germany_ UTC + 1 = +3600 sec
  const int   daylightOffset_sec = 3600;
  // improved for setting to the correct time zone directly
  // https://github.com/espressif/arduino-esp32/issues/3797 
  // https://remotemonitoringsystems.ca/time-zone-abbreviations.php
  const char* defaultTimezone = "CET-1CEST,M3.5.0/2,M10.5.0/3";
  int TimeIsInitialized = false;

  SimpleTimer NTPTimer;
  int NTPTimerHandle;               // timer handle for NTP time calls
  int NTPTimerInterval = 3600000;   // call this every hour
#endif // getNPTTIME
 
#if defined sendSERIAL || defined receiveSERIAL
  #define RXD2 16
  #define TXD2 17
  float serialTemp=0, serialHumidity=0; // data to be transmitted via serial
  int serialChannel;
  float InfactoryT[3];
  float InfactoryH[3];
  float last_InfactoryT[3] = {-111.22, -111.22, -111.22};
  float last_InfactoryH[3] = {-111.22, -111.22, -111.22};
  static int serialSentCh1Count=0, serialSentCh2Count=0, serialSentCh3Count=0, serialFailCount=0;
#endif 

#ifdef isOneDS18B20
  // Include file for DS18B20
  #include <OneWire.h>
  #include <DallasTemperature.h>
  // 1-Wire connection for temperature (Dallas DS18B20) is plugged into GPIO13 on the ESP32
  #define ONE_WIRE_BUS 13
  // power for One Wire Bus via GPIO 32
  #define POWER_ONEWIRE_BUS 32    

  // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
  OneWire oneWire(ONE_WIRE_BUS);
  // Pass our oneWire reference to Dallas Temperature.
  DallasTemperature sensors(&oneWire);
  
  // more flexible in Array: to store DS18B20 Temperatures
  #define MAX_NO_DS18B20 10
  int noDS18B20Connected = 0;       // number of sensors that are actually connected
  volatile double DS18B20Temperature[MAX_NO_DS18B20] = {-111.11, -111.11,-111.11,-111.11,-111.11,
                                            -111.11, -111.11, -111.11, -111.11, -111.11};
  volatile double calDS18B20Temperature[MAX_NO_DS18B20]={-111.11, -111.11,-111.11,-111.11,-111.11,
                                            -111.11, -111.11, -111.11, -111.11, -111.11}; 
  double sum_ThSp_calDS18B20Temperature[MAX_NO_DS18B20]= {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int n_ThSp_calDS18B20Temperature[MAX_NO_DS18B20]  = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  double sum_MQTT_calDS18B20Temperature[MAX_NO_DS18B20]= {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int n_MQTT_calDS18B20Temperature[MAX_NO_DS18B20]  = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

                                                // calibrated temperature values for output
  #define DS18B20RestartLimit 25              // these two to store how long no valid measurements, do restart of above limit
  int noDS18B20Restarts = 0;                  // Counter for DS18B20 Restarts
  
  volatile unsigned int notMeasuredCount = 0; // count for measurements not taken with DS18B20
  volatile unsigned int notChangedCount = 0;  // count for measurements not changed with DS18B20
  unsigned int manualDS18B20Restart = 0;      // flag for manual restart of DS18B20
  
  // address storage added since sometimes one sensor drops out, disturbing the sequence
  volatile uint8_t DS18B20Address[MAX_NO_DS18B20][8]; // my store for the device addresses of the attached sensors
  // Task handle for OneWire read (Core 0 on ESP32)
  TaskHandle_t Task1;
  volatile unsigned long GetOneDS18B20Counter = 0;     // loop counter for detached procedure GetOneDS18B20Temperature
  unsigned long previousGetOneDS18B20Counter=0;         // comparison value for loop counter
  unsigned long notMeasuredDS18B20=0;                   // counter for not measuring DS18B20

  bool stopDS18B20MeasureFlag = false;        // if this flag is set, no measurements are taken
#endif  

#ifdef isBME680
  // Include files for BME280 using Zanshin Lib, available as source code
  #include <SPI.h>              // < Include the SPI standard library
  #include "Zanshin_BME680.h"   // < The BME680 sensor library

  float air_quality_score, air_quality_score_sum = 0; 
  unsigned long air_quality_score_n = 0;
  char air_quality_string[80];
  char air_quality_shortstring[80];
#endif  

// stuff for RSSI measurements transfer to Thiingspeak
float rssi, last_rssi=0;
float rssi_sum=0;
unsigned long rssi_n=0; 

#ifdef isBME680_BSECLib
  // Include files for BME280 using BSEC original lib, available as object code only
  #include <SPI.h>              // < Include the SPI standard library
  #include "bsec.h"   // < The BME680 sensor library

  float air_quality_score, air_quality_score_sum = 0; 
  unsigned long air_quality_score_n = 0;
  char air_quality_string[80];
  char air_quality_shortstring[80];

  /* Configure the BSEC library with information about the sensor
    18v/33v = Voltage at Vdd. 1.8V or 3.3V
    3s/300s = BSEC operating mode, BSEC_SAMPLE_RATE_LP or BSEC_SAMPLE_RATE_ULP
    4d/28d = Operating age of the sensor in days
    generic_18v_3s_4d
    generic_18v_3s_28d
    generic_18v_300s_4d
    generic_18v_300s_28d
    generic_33v_3s_4d
    generic_33v_3s_28d
    generic_33v_300s_4d
    generic_33v_300s_28d
  */
  const uint8_t bsec_config_iaq[] = {
    #include "config/generic_33v_3s_4d/bsec_iaq.txt"
  };

  // #define STATE_SAVE_PERIOD	UINT32_C(360 * 60 * 1000) // 360 minutes - 4 times a day
  #define STATE_SAVE_PERIOD	UINT32_C(120 * 60 * 1000) // 120 minutes - 12 times a day

  // Create an object of the class Bsec
  Bsec iaqSensor;
  uint8_t bsecState[BSEC_MAX_STATE_BLOB_SIZE] = {0};
  uint16_t stateUpdateCounter = 0;

  Preferences permstorage;    // permanent storage object for BSEC BME 680 sensor parameter

  unsigned int countBME680Resets = 0;       // counter for resets of BME680
  unsigned int countBME680PowerToggles = 0; // counter for Power Toggles of BME680

  // Helper functions declarations for BSEC library handling
  void checkIaqSensorStatus(void);
  void loadBsecState(void);
  void updateBsecState(void);

  float Temperature, Humidity, Pressure, Altitude; // converted values in °C, %, mbar
#endif  

#ifdef isBME280
  // Include files for BME280
  #include <Wire.h>
  #include <Adafruit_Sensor.h>
  #include <Adafruit_BME280.h>

  Adafruit_BME280 bme; // I2C
  // Connect VIN of the BME280 sensor to 3.3V (NOT 5.0V!)
  // Connect GND to Ground
  // Connect SCL GPIO 22
  // Connect SDA GPIO 21

    float Temperature, Humidity, Pressure, Altitude; // converted values in °C, %, mbar
#endif

// wifi stuff. also used if no Blynk, for time and Virtuino
#include <WiFi.h>
#include <WiFiClient.h>
// credentials information: Blynk Auth tokens, wifi credentials etc.
#include "Credentials.h"

// timer for blynk check and restart
#include <BlynkSimpleEsp32.h>
BlynkTimer MyBlynkTimer;
#ifdef isBLYNK
  // Stuff for Blynk
  #define BLYNK_PRINT Serial
  // EXPDis #include <BlynkSimpleEsp32.h>

  // timer for blynk check and restart
  // EXPDis BlynkTimer MyBlynkTimer;
  int checkTimerHandle;
  #define checkTimerInterval 5000L
  int restartTimerHandle;
  #define restartTimerInterval 3*3600L*1000L

  int restartCount = 0;

  // terminal object
  WidgetTerminal myBlynkTerminal(V50);

  // bridge object to transfer alert data via V71
  WidgetBridge bridge1(V71);
  WidgetBridge bridge2(V81);
#endif

#ifdef isVirtuino
  //--- SETTINGS ------------------------------------------------
  // ssid and pass are in "credentials.h"
  //const char* ssid = "Fritz7390-MP2.4";         // enter the name (SSID) of your WIFI network
  //const char* password = "Aa-v465-*U3P";         // enter your WIFI network PASSWORD
  WiFiServer server(8000);                   // Default Virtuino Server port 
  IPAddress ip(192, 168, 178, 200);            // where 150 is the desired IP Address. The first three numbers must be the same as the router IP
  IPAddress gateway(192, 168, 178, 1);         // set gateway to match your network. Replace with your router IP
  //---

  //---VirtuinoCM  Library settings --------------
  #include "VirtuinoCM.h"
  VirtuinoCM virtuino;               
  #define V_memory_count 62          // the size of V memory. You can change it to a number <=255)
  float V[V_memory_count];           // This array is synchronized with Virtuino V memory. You can change the type to int, long etc.
  //---
  String Text_0="";   // Text string 0 synchronized
  String Text_1="";   // Text string 1 synchronized
  String Text_2="";   // Text string 2 synchronized
  String Text_3="";   // Text string 3 synchronized
#endif

#ifdef isThingspeak
  #include <HTTPClient.h>
  unsigned long myChannelNumber = 1553116;
  const char * myWriteAPIKey = "Z56RS9AXT2VCZCV0";
  // String readApiKey = "NMUECWL090S543MU" // not needed here, but for reference

  String ThingspeakServerName = "https://api.thingspeak.com/update?api_key=";
#endif

// includes for OTA over the air Updates 
#ifdef isOTA
  #include <WiFi.h>
  #include <ESPmDNS.h>
  #include <WiFiUdp.h>
  #include <ArduinoOTA.h>
#endif

#ifdef isBME680
  //*** global variables for BME 680
  BME680_Class BME680;          // < Create an instance of the BME680 class
  int32_t      raw_temperature;               ///< BME680 temperature value
  int32_t      raw_humidity;                  ///< BME680 humidity value
  int32_t      raw_pressure;                  ///< BME680 pressure value
  int32_t      raw_gas;                       ///< BME680 gas resistance value
  int32_t      start_pressure;                ///< Initial pressure reading
  
  float        altitude;                      // calculated altitude
#endif

float temperature, humidity, pressure, gas; // converted values in °C, %, mbar, ???
float temperature_sum, humidity_sum, pressure_sum;  // sums for averaging
long temperature_n, humidity_n, pressure_n; // counters for averaging

#ifdef isMHZ14A
  //*** PINs for serial communication via UART with CO2 Sensor
  #define RXD2 16
  #define TXD2 17
  //*** stuff for MH-Z14a CO2 monitor
  byte cmd[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};  // get gas command
  byte cmdCal[9] = {0xFF, 0x01, 0x87, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78};  // calibrate command
  char response[9];  // holds the recieved data
  int CO2ppm = 0;
  unsigned long CO2ppm_sum = 0, CO2ppm_n = 0, last_CO2ppm = 0;
  unsigned long warmingTimer = 0;
  unsigned long timer1 = 0;

  unsigned long MHZ14AWarmingTime;
  unsigned long MZH14AWarmupWait = 90;    // wait time until warmed up in sec
  unsigned long MZH14AMeasureWait = 15;   // time between two measurements in sec
#endif

#ifdef isSENSEAIR_S8
  //--- SenseAir S8 stuff. Based on https://github.com/SFeli/ESP32_S8

  #define TXD2 17
  #define RXD2 16

  byte CO2req[] = {0xFE, 0X04, 0X00, 0X03, 0X00, 0X01, 0XD5, 0XC5};
  byte Response[20];
  uint16_t crc_02;
  int ASCII_WERT;
  int int01, int02, int03;
  unsigned long ReadCRC;      // CRC Control Return Code 

  // Auslesen ABC-Periode / FirmWare / SensorID 
  byte ABCreq[] = {0xFE, 0X03, 0X00, 0X1F, 0X00, 0X01, 0XA1, 0XC3};   // 1f in Hex -> 31 dezimal
  byte FWreq[] = {0xFE, 0x04, 0x00, 0x1C, 0x00, 0x01, 0xE4, 0x03};    // FW      ->       334       1 / 78
  byte ID_Hi[] = {0xFE, 0x04, 0x00, 0x1D, 0x00, 0x01, 0xB5, 0xC3};    // Sensor ID hi    1821       7 / 29
  byte ID_Lo[] = {0xFE, 0x04, 0x00, 0x1E, 0x00, 0x01, 0x45, 0xC3};    // Sensor ID lo   49124     191 / 228 e.g. in Hex 071dbfe4
  //byte Tstreq[] = {0xFE, 0x03, 0x00, 0x01, 0x00, 0x01, 0xE4, 0x03};   // Read the holding register
  //byte Tstreq[] = {0xFE, 0x04, 0x00, 0x01, 0x00, 0x01, 0xE4, 0x03}; // Read the input Register
  byte Tstreq[] = {0xFE, 0x44, 0X00, 0X01, 0X02, 0X9F, 0X25};       // undocumented function 44
  int Test_len = 7;               // length of Testrequest in case of function 44 only 7 otherwise 8
    
  int CO2ppm = 0;
  unsigned long CO2ppm_sum = 0, CO2ppm_n = 0, last_CO2ppm = 0;

  bool senseAirPresentFlag = 0;  // is set to 1 if sensor is present during setup
#endif

#ifdef isSD
  #include "FS.h"
  #include "SD.h"
  #include <SPI.h>

  #define SD_CS 5
  char logfilename[80];
#endif

#ifdef isDisplay
  // for OLED Display
  #include <SPI.h>
  #include <Wire.h>
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>
  // Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
  #define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
  // Stuff for OLED Display
  #define SCREEN_WIDTH 128 // OLED display width, in pixels
  #define SCREEN_HEIGHT 64 // OLED display height, in pixels

  volatile int displayMode = 1;  // determines mode for display. volatile since change by interrupt
  volatile int displayDone = 0;  // zum Entprellen des Interrupthandlers
  volatile bool displayDimmed = false; // wenn true ist das display aus
  volatile unsigned long lastButtonTime = 0; // time when button was last pressed, for dimming of display
  unsigned int displayDimmDelay = 30000; // time delay in ms after which the display is switched off
                                        // on again with next button press
  int maxDisplayMode = 2;

  // timer for oled_handler, and interval for it
  #define oledHandlerInterval 400L
  int oledTimerHandle;
#endif  // isDisplay

// for LCD display 4 rows, 20 characters
#ifdef isLCD
  #include <LiquidCrystal_I2C.h>
  #include "LCDFunctions.h"

  volatile int lcdDisplayMode = 1;
    
  volatile int lcdDisplayDone = 0;
  static int maxLcdisplayMode = 3;

  // timer for lcd_handler, and interval for it
  #define lcdHandlerInterval 400L
  int lcdTimerHandle;
#endif

#if defined isLCD || defined isDisplay
  // timer for darkening display, and interval for it
  #define displayOffTimerInterval 30000L // 900000 ms= 900 sec
  int displayOffTimerHandle;
#endif


//** global stuff 
// timer for main_handler, and interval for it
#define mainHandlerInterval 2000L
#ifdef isThingspeak
  #define mainHandlerInterval 2000L
#else
  #define mainHandlerInterval 2000L  
#endif

SimpleTimer mainHandlerTimer;
int mainHandlerTimerHandle;

#ifdef isThingspeak
  #define thingspeakHandlerInterval 30000L // was 60000L

  SimpleTimer thingspeakHandlerTimer;
  int thingspeakHandlerTimerHandle=1;
#endif  

#ifdef isBluetoothCredentials
  // Bluetooth
  #include "BluetoothSerial.h"    // Header File for Serial Bluetooth, will be added by default into Arduino
  BluetoothSerial ESP_BT; //Object for Bluetooth
  Preferences credentialstorage;    // permanent storage object for BSEC BME 680 sensor parameter
#endif  

#ifdef isCaptivePortal
  Preferences credentialstorage2;    // permanent storage object for BSEC BME 680 sensor parameter
#endif

#ifdef isSyslog
  #include <WiFiUdp.h>
  #include <Syslog.h>

  // A UDP instance to let us send and receive packets over UDP
  WiFiUDP udpClient;
  // Create a new syslog instance with LOG_KERN facility
  Syslog syslog(udpClient, SYSLOG_SERVER, SYSLOG_PORT, DEVICE_HOSTNAME, APP_NAME, LOG_KERN);
#endif

#ifdef isMQTT
  // also needs WiFi.h - is always included
  #include <PubSubClient.h>
  WiFiClient espClient;
  PubSubClient mqttClient(espClient);

  // timer stuff
  #define mqttHandlerInterval 15000L 

  SimpleTimer mqttHandlerTimer;
  int mqttHandlerTimerHandle=1;
#endif

/************************************************************
* Forward declarations
*************************************************************/

//*** global forward declarations
void logOut(char* printstring);
void GetGasReference();
void CalculateIAQ(float score, char* printstring, char* shortstring);
void getBME680SensorData(); 
void checkBlynk();
void restartBlynk();
void main_handler();
void thingspeak_handler();
void lcd_handler();
void oled_handler();
void bme680FanHandler(void);
void windowOpenHandler(void);
void resetBME680(int sensorStatus, int bme680Status);
void restartDS18B20MeasurementFunction();
bool connectToWiFi(char* ssid, char* pass, int noRetries);
String toStringIp(IPAddress ip);
void windowSetBeeper(int desiredValue); 

//*** specific forward declarations
#ifdef getNTPTIME
  void printLocalTime(char* timestring, int mode);
  void getNTPTime();
#endif

#ifdef isSENSEAIR_S8
  // forward declarations for SenseAir S8 CO2 sensor handling
  void send_Request (byte * Request, int Re_len);
  void read_Response (int RS_len);
  unsigned short int ModBus_CRC(unsigned char * buf, int len);
  unsigned long get_Value(int RS_len);
#endif

#ifdef isMQTT
  void mqttCallbackFunction(char* topic, byte* message, unsigned int length); 
  void mqttHandler();
#endif
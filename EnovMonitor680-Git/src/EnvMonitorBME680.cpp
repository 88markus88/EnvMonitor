// EnvMonitor for ESP32 with BME680, MH-Z14A CO2 Detektor, BME280, 2 DS18B20
// Deplay via Blynk on Smartphone / Tablet, also via #define
// Configurable via #defines, which Sensors are present
//
// Infos BME680:
// https://wiki.dfrobot.com/Gravity__I2C_BME680_Environmental_Sensor__VOC,_Temperature,_Humidity,_Barometer__SKU__SEN0248
// https://github.com/Zanduino/BME680 
// Pinblegung (I2C): (+) = VCC (3,3-5V) / (-) = GND / C = SCL / D = SDA
// ESP32: 21= SDA, 22 = SCL
// Connect + of the BME680 sensor to 3.3V 
// Connect - to Ground
// Connect C GPIO 22
// Connect D GPIO 21
//**************************************************************************************
// serial communication with winston CO2 sensor via UART
// https://circuits4you.com/2018/12/31/esp32-hardware-serial2-example/
// https://www.hackster.io/kritch83/getting-started-with-the-mh-z14a-co2-detector-e96234
// RX: GPIO 16 / TX: GPIO 17
//**************************************************************************************
// Blynk: Blynk uses Core 0 of the ESP32. Therefore, parallel processing to read the values of
// DS18B20 temp-sensors leads to faulty results, since blynk interrupts the measurements. 
// solution was to run the detached measurement procedure on core1 as all the rest
// and to use non-blocking delays (300 ms: vTaskDelay(300 / portTICK_PERIOD_MS);)
// which allow the measurement routint to run with no issues. Now almost each measurement is successful
//
// Blynk collides with data collection interrupt routine for 433MHz sensor temp/hygro. 
// all data for interrupt routine need to be "volatile" - otherwise the system crashes in data access
// interrupt routines should have the attribute IRAM_ATTR to ensure that they run in RAM = faster
// Blynk Needs to be switched off while collecting 433 data

// Device selection stuff. if these flags are set, all relevant code is included. 
// this is used to enable / disable the code for various sensors as part of the env-station
// (use define or undef)


//*** general libraries. Libraries that multiple modules use should reside here 
#include <Arduino.h>
#include "time.h"
#include <Preferences.h>        // used to permanently store data
#include <esp_task_wdt.h>       // Load Watchdog-Library
#include "GlobalDefines.h"      // global defines that determine what to compile
#include "HelperFunctions.h"    // prototypes for global helper functions
#include "EnvMonitorBME680.h"   // My main include file
#include "Infactory433.h"       // Infactory 433 Temp/Humid sensor
#include "SDFunctions.h"        // functions to handle SD memory card
#include "LCDFunctions.h"       // functions to handle LCD display

  // Write string on the SD card
#if defined logSD && defined isSD
  void logSDCard(char *printstring) 
  {
    // Serial.print("Save to SD: ");
    // Serial.println(printstring);
    appendFile(SD, logfilename, printstring);
  }
#endif  

// function to handle all logging output. To serial or into file on SD
void logOut(char* printstring)
  {
    char timestring[50]="";      
    char outstring[120];

    #ifdef getNTPTIME
      if(TimeIsInitialized)
      {
        printLocalTime(timestring, 5);
      }  
    #endif 

    #ifdef logSerial
      Serial.print(printstring);
    #endif

    #if defined logSD && defined isSD      
      strcpy(outstring, timestring);
      strcat(outstring, printstring);      
      logSDCard(outstring);
    #endif

    #if defined isBLYNK && defined blynkTerminal
      strcpy(outstring, timestring);
      strcat(outstring, printstring);
      myBlynkTerminal.print(outstring);
      myBlynkTerminal.flush();
    #endif
  }

#ifdef isDisplay

  // Display class based on AdafruitGFX, special type SSD1306
  Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

  void Display(char *string, int size, int x, int y, boolean clear)
  {
    if (clear) display.clearDisplay();
    display.setTextSize(size);
    display.setCursor(x, y);
    display.println(string);
    display.display();
  }
#endif

#ifdef isOneDS18B20
  //*****************************************************************************
  // This task runs isolated on core 0 because sensors.requestTemperatures() is slow and blocking for about 750 ms
  // gets temperature from DS18B20 via OneWire
  volatile unsigned long LastMeasTimer;
  const long ds18b20MeasInterval = 1000; // measure every xxx ms

  // mutex needed for critical section
  // static portMUX_TYPE my_mutex = portMUX_INITIALIZER_UNLOCKED;

  void GetOneDS18B20Temperature( void * parameter) 
  {
    float tmp1 = 0;
    // float tmp2 = 0;
    static int i,j;
    int localInfactoryFlag = false;
    uint8_t addr[8];          // uint8_t = unsigned char, 8 bit integer

    #ifdef isInfactory433
      localInfactoryFlag = measuringInfactoryOngoing; // if infactory sensor connected, use global flag - otherwise always false
    #endif


    for (;;) {   // Endless loop
      if(millis()-LastMeasTimer > ds18b20MeasInterval)
      // there should be noDS18B20Connected sensors attached
      {
        if(localInfactoryFlag == false && stopDS18B20MeasureFlag == false)   // do not take measurements if infactory 433 MHz sensor is queried
        {
          sensors.requestTemperatures(); // Send the command to get temperatures
          // vTaskDelay(100);
          LastMeasTimer = millis();
          for(i=0;i<noDS18B20Connected;i++)
          {
            for(j=0;j<8;j++){
              addr[j] = DS18B20Address[i][j];
              // if(addr[j]<16) Serial.print("0");
              // Serial.print(addr[j],HEX);
            }  
            // Serial.println("");

            //*** alternative: by index: tmp1 = sensors.getTempCByIndex(i);
            tmp1 = sensors.getTempC(addr); 
            if(abs(DS18B20Temperature[i] - tmp1) < 0.01) // check: did the value change?
              notChangedCount++;              // global counter for not changed values
            else
              notChangedCount = 0;  
            if (tmp1 != -127 && tmp1 != 85)   //-127: bad reading. 85: no measurement yet 
            {
              DS18B20Temperature[i] = tmp1;
              Serial.print(i); 
              notMeasuredCount = 0;     // note that a measurement has been taken, set counter to zero
            }  else
            {
              DS18B20Temperature[i] = -111.11;
              Serial.print(":"); 
              LastMeasTimer = 0;
              notMeasuredCount++;       // note that measurements was not possible, increase counter
            }
          }
        }  
        else
        {
          sprintf(printstring,"No DS18B20 Measurement due to flag %d %d\n",
            localInfactoryFlag,stopDS18B20MeasureFlag);
          logOut(printstring);  
        }
      }
      else
      {
        vTaskDelay(200 / portTICK_PERIOD_MS); // delay for 200 ms
        // sprintf(printstring,"W %ld %ld ", millis(),LastMeasTimer);    
        // logOut(printstring);
        GetOneDS18B20Counter ++;  
      }  
    }
  }


  //---- the following procedure determines the number of DS18B20, sets their precision to 12 bit and stores their addresses
  DeviceAddress tempDeviceAddress;
  #define TEMPERATURE_PRECISION 12   // precision 9..12 Bit

   void adresseAusgeben(void) {
    byte i;
    uint8_t addr[8];          // uint8_t = unsigned char, 8 bit integer
    int numberOfDevices;

    // code 1: devices finden.
    // this method does not work for ESP32
    delay(1000);
    numberOfDevices = sensors.getDeviceCount();
 
    Serial.printf("Found %d Devices \n", numberOfDevices);

    numberOfDevices = 3; /// temporary
    // Setzen der Genauigkeit
    for(i=0 ;i<numberOfDevices; i++) {
      if(sensors.getAddress(tempDeviceAddress, i)) {
         sensors.setResolution(tempDeviceAddress, TEMPERATURE_PRECISION);
         Serial.print("Sensor ");
         Serial.print(i);
         Serial.print(" had a resolution of ");
         Serial.println(sensors.getResolution(tempDeviceAddress), DEC);
      }
    }
    Serial.println("");
    numberOfDevices = sensors.getDeviceCount();   // does not function with OneWire library 2.3.5 (claimed to be ok with 2.3.3)
    Serial.printf("Found %d sensors\n", numberOfDevices);
    esp_task_wdt_reset();   // keep watchdog happy

    // code 2: find devices by searching on the onewire bus across all addresses. 
    // workaround, works apparently for ESP32

    noDS18B20Connected = 0;
    Serial.print("Searching 1-Wire-Devices...\n\r");// "\n\r" is NewLine
    while(sensors.getAddress(addr, noDS18B20Connected)) {  
      Serial.printf("1-Wire-Device %d found with Adress:\n\r", noDS18B20Connected);
      esp_task_wdt_reset();   // keep watchdog happy

      for( i = 0; i < 8; i++) {
        DS18B20Address[noDS18B20Connected][i] = addr[i];
        // Serial.print("0x");
        if (addr[i] < 16) {
          Serial.print('0');
        }
        Serial.print(addr[i], HEX);
        if (i < 7) {
          Serial.print(" ");
        }
      }
      Serial.printf("\n");
      if ( OneWire::crc8( addr, 7) != addr[7]) {
        Serial.print("CRC is not valid!\n\r");
        return;
      }
      noDS18B20Connected ++;    // one more device found
    }
    for( i = 0; i < 8; i++) {
      Serial.print(DS18B20Address[noDS18B20Connected][i], HEX);
      Serial.print(" ");
    }
    Serial.printf("Found 1-Wire-Devices: %d \n\r", noDS18B20Connected);
    oneWire.reset_search();
    return;
  }

  // manual restart via V60
  BLYNK_WRITE(V60) 
  {
    int x = param.asInt();
    sprintf(printstring,"Manual restart of DS18B20 initiated %d\n", x);
    logOut(printstring);
    if(x==1)  
      manualDS18B20Restart = 1;
    else  
      manualDS18B20Restart = 1;
  }

  //----- a short routine to restart the sensors 
  void restartDS18B20()
  {
    // Toggle power on 32
    // POWER_ONEWIRE_BUS
    noDS18B20Restarts++;
    sprintf(printstring,"Restarting OneWire Bus for DS18B20: %d \n", noDS18B20Restarts);
    logOut(printstring);

    stopDS18B20MeasureFlag = true;   // use this flag to stop measurements with DS18B20
    esp_task_wdt_reset();   // keep watchdog happy
    vTaskDelay(1000 / portTICK_PERIOD_MS); // delay for 1000 ms

    pinMode(POWER_ONEWIRE_BUS, OUTPUT);
    digitalWrite(POWER_ONEWIRE_BUS, LOW);
    vTaskDelay(3000 / portTICK_PERIOD_MS); // delay for 3000 ms
    esp_task_wdt_reset();
    digitalWrite(POWER_ONEWIRE_BUS, HIGH);
    vTaskDelay(300 / portTICK_PERIOD_MS); // delay for 300 ms
    // Start OneWire for DS18B20
    sensors.begin();
    vTaskDelay(200 / portTICK_PERIOD_MS); // delay for 200 ms
      // Create GetTemperature task for core 0, loop() runs on core 1
    adresseAusgeben();    // adressen onewire devices ausgeben, devices finden

    esp_task_wdt_reset();   // keep watchdog happy
    vTaskDelay(1000 / portTICK_PERIOD_MS); // delay for 1000 ms
    stopDS18B20MeasureFlag = false; // reset flag to start measurements
  }  
#endif  // DS18B20

#ifdef isDisplay
  // interrupt handler for PushButton pressed. only needed for display
  void changeDisplayMode(void)
  {
    if(displayDone ==1 )
    {
      if(displayDimmed == true)
      {
        displayDimmed = false;
        displayDone = 0;
        lastButtonTime = millis();
      }
      else
      {
        displayMode ++;
        if (displayMode > maxDisplayMode)    // tbd: variable adaptation to number of sensors
          displayMode = 0;
        displayDone=0;    
        lastButtonTime = millis();   
      }
    }  
    // Serial.printf("************** Button Pressed %d *************************%d %d \n",  displayMode, displayDone);  
  }
#endif

#ifdef getNTPTIME
  /****************************************************************************************/
  /* get and print local time, once obtained from time server */
  /* http://www.cplusplus.com/reference/ctime/strftime/ */
  /****************************************************************************************/
  void printLocalTime(char* printstring, int mode)
  {
    char timeHour[3];
    char timeMinute[3];
    char timeSecond[3];
    char timeWeekDay[10];
    char timeDay[3];
    char timeMonth[7];
    char timeMonthShort[5];
    char timeYear[5];
    char timeYearShort[3];
    char timeISODate[12];
    char timeISOTime[10];


    struct tm timeinfo;
    if(!getLocalTime(&timeinfo)){
      Serial.println("Failed to obtain time");
      strcpy(printstring,"<?time?>");
      return;
    }

    strftime(timeHour,3, "%H", &timeinfo);  
    strftime(timeMinute,3, "%M", &timeinfo);  
    strftime(timeSecond,3, "%S", &timeinfo);  
    strftime(timeWeekDay,10, "%A", &timeinfo);
    strftime(timeDay,3, "%d", &timeinfo);  
    strftime(timeMonth,4, "%b", &timeinfo);  
    strftime(timeMonthShort,3, "%m", &timeinfo);  
    strftime(timeYear,5, "%Y", &timeinfo);  
    strftime(timeYearShort,5, "%y", &timeinfo);  
    strftime(timeISODate,12, "%F", &timeinfo);  
    strftime(timeISOTime,10, "%T", &timeinfo); 

    // format modiefies for strftime()
    // http://www.cplusplus.com/reference/ctime/strftime/ 
    switch(mode)
    {
      // output time and date to printstring and / or serial port
      case 1: 
        Serial.println(&timeinfo,"%e.%m.%G - %H:%M:%S"); 
        break;
      case 2: 
        sprintf(printstring,"%s.%s.%s - %s:%s:%s\n",
          timeDay,timeMonth,timeYear,timeHour,timeMinute,timeSecond);
        // Serial.print(printstring);
      case 3: 
        sprintf(printstring,"%s - %s\n", timeISODate, timeISOTime);
        // Serial.print(printstring); 
        break;
      case 4: 
        sprintf(printstring,"%s.%s.%s-%s:%s:%s ", 
          timeDay,timeMonthShort,timeYearShort,timeHour,timeMinute,timeSecond);
      case 5: 
        sprintf(printstring,"%s%s%s-%s:%s:%s ", 
          timeDay,timeMonth,timeYearShort,timeHour,timeMinute,timeSecond);    
        // Serial.print(printstring); 
        break;  
      case 6: 
        sprintf(printstring,"%s:%s:%s ",timeHour,timeMinute,timeSecond);    
        // Serial.print(printstring); 
        break;    
      default: Serial.println("Invalid time print mode");
    }
    
    /*
    Serial.print("Day of week: ");
    Serial.println(&timeinfo, "%A");
    Serial.print("Month: ");
    Serial.println(&timeinfo, "%B");
    Serial.print("Day of Month: ");
    Serial.println(&timeinfo, "%d");
    Serial.print("Year: ");
    Serial.println(&timeinfo, "%Y");
    Serial.print("Hour: ");
    Serial.println(&timeinfo, "%H");
    Serial.print("Hour (12 hour format): ");
    Serial.println(&timeinfo, "%I");
    Serial.print("Minute: ");
    Serial.println(&timeinfo, "%M");
    Serial.print("Second: ");
    Serial.println(&timeinfo, "%S");

    Serial.println("Time variables");
    char timeHour[3];
    strftime(timeHour,3, "%H", &timeinfo);
    Serial.println(timeHour);
    char timeWeekDay[10];
    strftime(timeWeekDay,10, "%A", &timeinfo);
    Serial.println(timeWeekDay);
    Serial.println();
    */
  }


  // get time from NTP server
  // https://randomnerdtutorials.com/esp32-date-time-ntp-client-server-arduino/
  void getNTPTime()
  {
    char printstring[80];

    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, pass);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    esp_task_wdt_reset();   // keep watchdog happy
    Serial.println("");
    Serial.println("WiFi connected.");
    if (WiFi.status() == WL_CONNECTED)
    {
      Serial.println("WiFi connected.");
      // Init and get the time
      configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
      printLocalTime(printstring, 3);
      logOut(printstring);
      //disconnect WiFi as it's no longer needed
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
      TimeIsInitialized = true;
    }
    else
    {
      Serial.println("WiFi NOT connected - time connection not possible.");
      TimeIsInitialized = false;
    }  
  }
#endif

#ifdef serialMonitor
// routine to read all characters on the serial input and display it, later to write to SD
  #define serBufSize 1024

  void monitorSerial()
  {
    
    char buffer[serBufSize];
    int cnt=0;
    char rc;

    while(1)
    {
      // delete read buffer
      strcpy(buffer,"");
      cnt = 0;

      while (Serial2.available() > 0 && cnt < serBufSize) 
      {
        rc = Serial2.read();
        buffer[cnt] = rc;
        cnt++;
        delay(1); /// wait a little in case sender still active
      } // while

      if(cnt>0)
      {
        buffer[cnt] = '\0'; // terminate the string
        if(strlen(buffer) > 0)
        {
    	    // Serial.print("ser: ");
          Serial.println(buffer);
          #ifdef logSD  
            appendFile(SD, logfilename, buffer);
          #endif
        } 
      }
      delay(50);
      esp_task_wdt_reset(); // reset watchdog
    }
  } // monitorSerial

#endif //serial monitor

//*****************************************************************************
// outputProgramInfo 
// helper function to output all infos about the program, at startup
//*****************************************************************************

void outputProgramInfo()
{
  sprintf(printstring,"\n with ");
  sprintf(printstring2,":");
  #ifdef isOneDS18B20  
    strcat(printstring, " DS18B20 (T) -");
    // strcat(printstring2, "DS18B20-");
  #endif
  #ifdef isBME680  
    strcat(printstring, " BME680 (P/T/H/G) -");
    // strcat(printstring2, "BME680-");
  #endif
  #ifdef isBME280  
    strcat(printstring, " BME280 (P/T/H) -");
  #endif
  #ifdef isMHZ14A  
    strcat(printstring, " MHZ14A (CO2) -");
  #endif
  #ifdef isSENSEAIR_S8  
    strcat(printstring, " SenseAir S8 (CO2) -");
  #endif

  #ifdef isInfactory433 
    sprintf(printstring2,"Infactory NV5849 - ");
    //strcat(printstring,printstring2);
  #endif

  strcat(printstring, " Sensors");
  #ifdef isBLYNK  
    strcat(printstring, " , BLYNK");
    // strcat(printstring2, "BLYNK");
  #endif
  #ifdef isDisplay
    strcat(printstring, " +Display");
  #endif
  strcat( printstring, " \n");
 
  strcpy(printstring2,"\n******************************************************************************");
  logOut(printstring2);
  strcpy(printstring2,"\n This is EnvMonitorBME680.cpp \n"); 
  logOut(printstring2);
  logOut(infoStringLong);
  logOut(printstring);
  #ifdef isBLYNK
    sprintf(printstring," Blynk Auth Code: %s \n", auth); 
    logOut(printstring);
    sprintf(printstring," Blynk Server IP: %s \n", "blynkLocalIP"); 
    logOut(printstring);
  #endif
  sprintf(printstring," %s  %s  %s \n",PROGNAME, PROGVERSION, PROGDATE);
  logOut(printstring);
  strcpy(printstring2,"******************************************************************************\n");
  logOut(printstring2);
  delay(200);
}

#ifdef isLCD
  // interrupt handler for PushButton pressed. only needed for LCD display
  void lcdChangeDisplayMode(void)
  {
    // Serial.printf("************** LCD Button Pressed before ********************** %d %d \n",  
    //   lcdDisplayMode, lcdDisplayDone);  
    if(lcdDisplayDone ==1 )
    {
      lcdDisplayMode ++;
      if (lcdDisplayMode > maxLcdisplayMode)    // tbd: variable adaptation to number of sensors
         lcdDisplayMode = 0;
      lcdDisplayDone=0;    
      //lastButtonTime = millis();   
    }  
    // Serial.printf("************** LCD Button Pressed after ********************** %d %d \n",  
    //  lcdDisplayMode, lcdDisplayDone);  
  }
#endif

//*****************************************************************************
// setup function
//*****************************************************************************
void setup() 
{
  // Init USB serial port
  Serial.begin(115200);
  delay(10);

  // Set Watchdog
  pinMode(0, INPUT_PULLUP); // Digital-Pin 0 as input
  esp_task_wdt_init(WDT_TIMEOUT_SECONDS,true); //Init Watchdog with 10 seconds timeout and panic (hardware rest if watchdog acts)
  esp_task_wdt_add(NULL); //No special task needed

  startTime = millis(); // remember the start time

  #ifdef getNTPTIME
    // get time from NTP server
    // https://randomnerdtutorials.com/esp32-date-time-ntp-client-server-arduino/
    getNTPTime();
    esp_task_wdt_reset();   // keep watchdog happy
  #endif 

  // get number of reboots so far, is used to set SD filename
  preferences.begin("nvs", false);                 // Open nonvolatile storage (nvs)
  NoReboots = preferences.getInt("NoReboots", 0);  // Read stored last NoReboots, default 0
  if(NoReboots > 9998)
    NoReboots = 0;
  preferences.putInt("NoReboots", NoReboots+1);    // and increment it - new start.
  preferences.end();
  sprintf(printstring,"NoReboots=%d\n", NoReboots);
  logOut(printstring);
  
  // set timer for main_handler()
  //mainHandlerTimer.setInterval(mainHandlerInterval, main_handler);
  MyBlynkTimer.setInterval(mainHandlerInterval, main_handler);

  #ifdef isSD
    // Create a file on the SD card and write the data labels
    sprintf(logfilename,"/log%04d.txt",NoReboots);
    initSDCard(logfilename);
  #endif  

  #ifdef isDisplay
    // Start the display
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // Initialize with the I2C addr 0x3C (for the 128x64 from Conrad else 3D)
    display.setTextColor(WHITE);
    display.clearDisplay(); 
    display.setTextSize(1);

    sprintf(printstring,"Display initialized ");
    display.setCursor(0, 0);
    display.println(printstring);
    sprintf(printstring,"%s",PROGNAME);
    display.setCursor(0, 12);
    display.println(printstring);
    sprintf(printstring,"Version: %s",PROGVERSION);
    display.setCursor(0, 24);
    display.println(printstring);
    sprintf(printstring,"Date:    %s",PROGDATE);
    display.setCursor(0, 36);
    display.println(printstring);

    display.display();          // transfer buffer content to display
    // Display(printstring, 1,0,0,true);
    // Display(printstring2, 1,0,12,false);
    delay(1000);
  #endif 

  // PushButton setup, only for display
  #ifdef isDisplay
    pinMode(PushButton, RISING);
    attachInterrupt(digitalPinToInterrupt(PushButton),changeDisplayMode, CHANGE);
    displayMode = 1;
    displayDone = 1;

    // set the maximum display Mode for use in switchDisplay(), depending on sensors present
    #if defined isBME280 || defined isBME680
      maxDisplayMode =3;
    #endif
    #if defined isOneDS18B20
      maxDisplayMode =4;
    #endif
    #if defined isMHZ14A || defined isSENSEAIR_S8 || defined receiveSERIAL
      maxDisplayMode =5;
    #endif
    // set timer for oled_handler()
    //oledHandlerTimer.setInterval(lcdHandlerInterval, oled_handler);
    MyBlynkTimer.setInterval(oledHandlerInterval, oled_handler);
  #endif  // isDisplay

  // setup functions for LCD display 4 rows 20 characters
  #ifdef isLCD
    // push button setup for LCD display
    pinMode(PushButton, RISING);
    attachInterrupt(digitalPinToInterrupt(PushButton),lcdChangeDisplayMode, CHANGE);
    lcdDisplayMode = 1;
    lcdDisplayDone = 1;
    maxLcdisplayMode = 5;
    initLCD();                  // initialize the LCD
    displayLCDProgInfo(infoStringShort);  // and display program Info

    // set timer for lcd_handler()
    //lcdHandlerTimer.setInterval(lcdHandlerInterval, lcd_handler);
    MyBlynkTimer.setInterval(lcdHandlerInterval, lcd_handler);
  #endif // isLCD

  // Program Info to serial port
  outputProgramInfo(); 

  #ifdef isRelay
    pinMode(RELAYPIN1,OUTPUT);
    pinMode(RELAYPIN2,OUTPUT);
    delay(10);
    /*
    Serial.printf("Relay1 ON\n");
    digitalWrite(RELAYPIN1, HIGH);
    delay(200);
    Serial.printf("Relay1 OFF\n");
    digitalWrite(RELAYPIN1, LOW);
    Serial.printf("Relay2 ON\n");
    digitalWrite(RELAYPIN2, HIGH);
    delay(200);
    Serial.printf("Relay2 OFF\n");
    digitalWrite(RELAYPIN2, LOW);
    */
  #endif

  #ifdef isInfactory433     // set input pin and interrupt handler for 433 MHz sensor (conflict with button!)
    doInitializeInfactory();
  #endif  

  #ifdef isBME280
    // Init BME280 I2C address depends on sensor 0x76 or 0x77.
    if (!bme.begin(0x76)) {
      Serial.println("Could not find a valid BME280 sensor, check wiring!");
      delay(2000);
    }

    // recommended settings for weather monitoring
    Serial.println("-- Weather Station Scenario --");
    Serial.println("forced mode, 1x temperature / 1x humidity / 1x pressure oversampling,");
    Serial.println("filter off");
    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1, // temperature
                  Adafruit_BME280::SAMPLING_X1, // pressure
                  Adafruit_BME280::SAMPLING_X1, // humidity
                  Adafruit_BME280::FILTER_OFF   );                  
    // suggested rate is 1/60Hz (1m)
    // delayTime = 60000; // in milliseconds
  #endif
  
  #ifdef isSENSEAIR_S8
    Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);      // UART to Sensair CO2 Sensor
    
    Serial.println();
    Serial.print("SenseAir S8 ABC-Value: ");  
    send_Request(ABCreq, 8);                     // Request ABC-Information from the Sensor
    read_Response(7);
    Serial.printf("%02ld", get_Value(7));
    Serial.println("");

    Serial.print("SenseAir S8 Sensor ID : ");                // 071dbfe4
    send_Request(ID_Hi, 8);
    read_Response(7);
    Serial.printf("%02x%02x", Response[3], Response[4]);
    send_Request(ID_Lo, 8);
    read_Response(7);
    Serial.printf("%02x%02x", Response[3], Response[4]);
    Serial.println("");

    Serial.print("SenseAir S8 FirmWare  : ");
    send_Request(FWreq, 8);                // Send Request for Firmwar to the Sensor
    read_Response(7);                   // get Response (Firmware 334) from the Sensor
    Serial.printf("%02d%02d", Response[3], Response[4]);
    Serial.println("");
  #endif

  #ifdef isMHZ14A
    // establish serial communication to CO2 sensor
    Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
    warmingTimer = millis();  // initiAlize warmup timer
    timer1 = millis();
  #endif

  #ifdef isBME680
    Serial.print("- Initializing BME680 sensor\n");
    while (!BME680.begin(I2C_STANDARD_MODE, 0x77)) {  // Start using I2C, use address 0x77 (could also be 0x76)
      Serial.print("-  Unable to find BME680. Trying again in 5 seconds.\n");
      delay(5000);
    }  // of loop until device is located
    Serial.print("- Setting 16x oversampling for all sensors\n");
    Serial.print("- Setting IIR filter to a value of 4 samples\n");
    Serial.print("- Turning on gas measurements\n");
    BME680.setOversampling(TemperatureSensor, Oversample16);
    BME680.setOversampling(HumiditySensor, Oversample16);
    BME680.setOversampling(PressureSensor, Oversample16);
    BME680.setIIRFilter(IIR4);
    BME680.setGas(320, 150);  // Setting either to 0 turns off gas. 1: temp in C, 2: heatin gin ms
    // BME680.setGas(0, 0);  
    BME680.getSensorData(raw_temperature, raw_humidity, start_pressure, raw_gas);  // Get most recent readings
    delay(100);

    // Now run the sensor for a burn-in period, then use combination of relative humidity and gas resistance to estimate indoor air quality as a percentage.
    GetGasReference();
  #endif

  #ifdef isBLYNK
    sprintf(printstring,"Blynk setup section entered\n");
    logOut(printstring);
    // Blynk initialization for Blynk web account - MP internet
    #ifdef blynkCloud
      Blynk.begin(auth, ssid, pass);
    #else
      // Blynk initialization for local blynk server on raspi - MP Home
      // Blynk.begin(auth, ssid, pass, IPAddress(192,168,178,31), 8080);
      // Blynk.begin(auth, ssid, pass, IPAddress(192,168,178,64), 8080);
      Blynk.begin(auth, ssid, pass, IPAddress(blynkLocalIP), 8080);
    #endif  

    // Original example, never use in reality: You can also specify server:
    // Blynk.begin(auth, ssid, pass, "blynk-cloud.com", 80);
    // Blynk.begin(auth, ssid, pass, IPAddress(192,168,1,100), 8080);

    #ifdef blynkTerminal
      myBlynkTerminal.clear();
    #endif  
    sprintf(printstring,"Blynk connected\n");
    logOut(printstring);
    #ifdef isDisplay
      sprintf(printstring,"Blynk connected");
      Display(printstring, 1,0,48,false); // string, size, x,y,clear
    #endif  

    #if defined sendSERIAL || defined receiveSERIAL
      Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
      #ifdef isBLYNK
        // reset some values to default
        Blynk.virtualWrite(V35, serialSentCh1Count);
        Blynk.virtualWrite(V36, serialSentCh2Count);
        Blynk.virtualWrite(V37, serialSentCh3Count);
        Blynk.virtualWrite(V38, serialFailCount);
        Blynk.run();
        vTaskDelay(500 / portTICK_PERIOD_MS); // wait 500ms
        
        Blynk.disconnect();  // we do not want Blynk to disturb the arduino measuring data
        sprintf(printstring,"Blynk disconnected\n");
        //Serial.println("A");
        logOut(printstring);
      #endif  
    #endif

    // handle OTA over the air Updates 
    #ifdef isOTA
      ArduinoOTA.setHostname("EnvMonitor680");
      // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  // ArduinoOTA.setHostname("myesp32");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
    .onStart([]() {
      String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
        else // U_SPIFFS
          type = "filesystem";

          // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
          Serial.println("Start updating " + type);
      })
      .onEnd([]() {
        Serial.println("\nEnd");
      })
      .onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
      })
      .onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
      });

      ArduinoOTA.begin();
    #endif
  
    //Serial.println("A1 ");
    #ifdef isInfactory433
    if(InfactoryTempC > -110 && InfactoryHumidity > -110) // data already measured during setup? send them
    {         
      Blynk.virtualWrite(V15, InfactoryTempC); //sending to Blynk, if other than the default -111.11
      Serial.printf("Setup direct >s>s>s>s>s> Sending TempC %3.1f ", InfactoryTempC);
      Blynk.virtualWrite(V16, InfactoryHumidity); //sending to Blynk
      Serial.printf(">s>s>s>s>s> Sending Humidity %3.1f \n", InfactoryHumidity);
      InfactoryTempC = -111.11;     // set to default to avoid resend later
      InfactoryHumidity = -111.11;
    }  
    else
    {
      // get last data collected in non-volatile storage, stored to survive reboot
      preferences.begin("nvs", false);                          // Open nonvolatile storage (nvs)
      InfactoryTempC = preferences.getFloat("InfTempC", -111.1);
      InfactoryHumidity = preferences.getFloat("InfHumidity", -111.11);
      preferences.end();
      if(InfactoryTempC > -110 && InfactoryHumidity > -110)    // valid data reiceived from nvs
      {
        Blynk.virtualWrite(V15, InfactoryTempC); //sending to Blynk, if other than the default -111.11
        Serial.printf("Setup NVS >+>+>+>+>+> Sending TempC %3.1f ", InfactoryTempC);
        Blynk.virtualWrite(V16, InfactoryHumidity); //sending to Blynk
        Serial.printf(">+>+>+>+>+> Sending Humidity %3.1f \n", InfactoryHumidity);
        preferences.begin("nvs", false);                    // Open nonvolatile storage (nvs)
        preferences.putFloat("InfTempC", -111.1);           // reset nvs to defaults 
        preferences.putFloat("InfHumidity", -111.11);       // to only read once
        preferences.end();
      }  
    }
    #endif

    #ifdef blynkRegularCheck
      // MyBlynkCheckTimer.setInterval(5000L, checkBlynk); // check if connected to Blynk server every 5 seconds. Not necessary, left out
      MyBlynkTimer.setInterval(5000L, checkBlynk); // check if connected to Blynk server every 5 seconds. Not necessary, left out
    #endif  
    // Serial.println("B");
    #ifdef blynkRestartHouly
      // MyBlynkRestartTimer.setInterval(1*3600L*1000L, restartBlynk); // attempt to restart Blynk every 1 hours
      MyBlynkTimer.setInterval(1*3600L*1000L, restartBlynk); // attempt to restart Blynk every 1 hours
    #endif  
    // MyBlynkTimer.setInterval(1*60*1000L, restartBlynk); // attempt to restart Blynk every 1 minute
  #endif

  #ifdef isOneDS18B20
    // switch on Power (via GPIO 32)
    pinMode(POWER_ONEWIRE_BUS, OUTPUT);
    digitalWrite(POWER_ONEWIRE_BUS, HIGH);
    // Start OneWire for DS18B20
    sensors.begin();
    delay(1000);
      // Create GetTemperature task for core 0, loop() runs on core 1
    adresseAusgeben();    // adressen onewire devices ausgeben, devices finden

    xTaskCreatePinnedToCore(
      GetOneDS18B20Temperature, /* Function to implement the task */
      "Task1", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      0,  /* Priority of the task. higher number is higher priority, https://esp32.com/viewtopic.php?t=10629 */
      &Task1,  /* Task handle. */
      1); /* Core where the task should run */
  #endif
  // Serial.println("C");
  // delay(200);

  #ifdef serialMonitor
    logOut("Entering Serial Monitor mode - all other is off\n");
    monitorSerial();
  #endif
  sprintf(printstring,"end setup\n");
  logOut(printstring);
} // setup

#ifdef isBME680

float hum_weighting = 0.25; // so hum effect is 25% of the total air quality score
float gas_weighting = 0.75; // so gas effect is 75% of the total air quality score

float hum_score, gas_score;
float gas_reference = 250000;
float hum_reference = 40;
int getgasreference_count = 0;

float air_quality_score;
char air_quality_string[80];
char air_quality_shortstring[80];

void getAirQuality()
{
  // char printstring[80];
  int gas_lower_limit = 5000; // Bad air quality limit
  int gas_upper_limit = 50000; // Good air quality limit
  float current_humidity;
  
  //Calculate humidity contribution to IAQ index
  current_humidity = humidity;
  if (current_humidity >= 38 && current_humidity <= 42)
    hum_score = 0.25*100; // Humidity +/-5% around optimum
  else
  { //sub-optimal
    if (current_humidity < 38)
      hum_score = 0.25 / hum_reference * current_humidity * 100;
    else
    {
      hum_score = ((-0.25/(100-hum_reference)*current_humidity)+0.416666)*100;
    }
  }

  //Calculate gas contribution to IAQ index
  if (gas_reference > gas_upper_limit) gas_reference = gas_upper_limit;
  if (gas_reference < gas_lower_limit) gas_reference = gas_lower_limit;
  gas_score = (0.75/(gas_upper_limit-gas_lower_limit)*gas_reference -(gas_lower_limit*(0.75/(gas_upper_limit-gas_lower_limit))))*100;
  // Serial.printf("gas_reference %f gas_score %f ,gas_lower_limit %d, gas_upper_limit %d\n",
  //  gas_reference,gas_score,gas_lower_limit,gas_upper_limit);

  //Combine results for the final IAQ index value (0-100% where 100% is good quality air)
  air_quality_score = hum_score + gas_score;

  // Serial.println("Air Quality = "+String(air_quality_score,1)+"% derived from 25% of Humidity reading and 75% of Gas reading - 100% is good quality air");
  // Serial.println("Air Quality = "+String(air_quality_score,1)+ 
  // Serial.println("Humidity element was : "+String(hum_score/100)+" of 0.25");
  // Serial.println(" Gas element was : "+String(gas_score/100)+" of 0.75");
  
  // if (raw_gas < 120000) Serial.println("***** Poor air quality *****");
  // if ((getgasreference_count++)%10==0) GetGasReference();
  GetGasReference();
  // Serial.println(CalculateIAQ(air_quality_score));
  CalculateIAQ(air_quality_score, air_quality_string, air_quality_shortstring);
  Serial.printf("Air Quality: %3.1f %s (Humidity: %3.1f, Gas: %3.1f)\n",
      air_quality_score, air_quality_string, hum_score, gas_score);
}

  // Now run the sensor for a burn-in period, then use combination of relative humidity and gas resistance to estimate indoor air quality as a percentage.
  void GetGasReference()
  {
    int readings = 3;
    Serial.printf("============================================================\n");
    Serial.printf("Getting a new gas reference value. Before: %3.1f ", gas_reference);
    for (int i = 1; i <= readings; i++)   // read gas for 10 x 0.150mS = 1.5secs
    { 
      getBME680SensorData(); 
      gas_reference += raw_gas;
    }
    gas_reference = gas_reference / readings;  // mean of old value plus new values
    Serial.printf(" --- after: %3.1f \n", gas_reference);
  }

  void CalculateIAQ(float score, char* printstring, char* shortstring)
  {
    strcpy(printstring, "Air quality is ") ;

    if (score<=40){
      strcat(printstring, "Hazardous");
      strcpy(shortstring,"Hazard");
    }
    if(score> 40 && score<=60){    // 40..60
      strcat(printstring, "Very Unhealthy");
      strcpy(shortstring,"VeryUnhy");
    }
    if( score>60 && score<=65){    // 60..65
      strcat(printstring, "Unhealthy");
      strcpy(shortstring,"Unhealthy");
    }
    if(score>65 && score<=70){    // 65..70
      strcat(printstring, "Unhealthy for Sensitive Groups");
      strcpy(shortstring,"Unh Sensi");
    }
    if(score > 70 && score<=90){    // 70..90
      strcat(printstring, "Moderate");
      strcpy(shortstring,"Moderate");
    }
    if(score > 90){     // > 90
      strcat(printstring, "Good");
      strcpy(shortstring,"Good");                   
    }  
  }

  /**************************************************!
    @brief    Function to read the BME680 data 
    @details  The BME680 data is read in 
    @return   void
  ***************************************************/
  void getBME680SensorData() 
  {
    BME680.getSensorData(raw_temperature, raw_humidity, raw_pressure, raw_gas);  // Get readings
    temperature = raw_temperature/100.0;
    pressure = raw_pressure/100.0;
    altitude = 44330.0 * (1.0 - pow((pressure) / SEA_LEVEL_PRESSURE, 0.1903));
    humidity = raw_humidity/1000.0;
    gas = raw_gas/1000.0;
  }  // of method "getSensorData()"
#endif

#ifdef isMHZ14A
  /**************************************************!
   * calibrate CO2 sensor
  ***************************************************/
  void calibrateCO2Sensor()
  {
    Serial2.write(cmdCal, 9);
    // delay(3000);
    vTaskDelay(3000 / portTICK_PERIOD_MS); // non-blocking delay
  }

  // calibration started via virtual pin V20 of Blynk app
  BLYNK_WRITE(V20) 
  {
    int x = param.asInt();
    Serial.printf("Calibrate button pushed %d \n", x);
    calibrateCO2Sensor();
  }

  /**************************************************!
   * get data from serial port of CO2 sensor
   ***************************************************/
  void getCO2Data()
  {
    //int garbage;

    while (Serial2.available())  // this clears out any garbage in the RX buffer
      //garbage = Serial2.read();
      Serial2.read();

    Serial2.write(cmd, 9);  // Sent out read command to the sensor
    Serial2.flush();  // this pauses the sketch and waits for the TX buffer to send all its data to the sensor

    while (!Serial2.available())  // this pauses the sketch and waiting for the sensor responce
      delay(0);

    Serial2.readBytes(response, 9);  // once data is avilable, it reads it to a variable
    int responseHigh = (int)response[2];
    int responseLow = (int)response[3];
    CO2ppm = (256 * responseHigh) + responseLow;
  }
#endif

#ifdef isSENSEAIR_S8
  // support functions for SenseAir S8, based on code from https://github.com/SFeli/ESP32_S8
  void send_Request (byte * Request, int Re_len)
  {
    while (!Serial2.available())
    {
      Serial2.write(Request, Re_len);   // Send request to S8-Sensor
      delay(50);
    }
  }

  void read_Response (int RS_len)
  {
    int01 = 0;
    while (Serial2.available() < 7 ) 
    {
      int01++;
      if (int01 > 10)
      {
        while (Serial2.available())
          Serial2.read();
        break;
      }
      delay(50);
    }
    for (int02 = 0; int02 < RS_len; int02++)    // Empfangsbytes
    {
      Response[int02] = Serial2.read();
    }
  }

  unsigned short int ModBus_CRC(unsigned char * buf, int len)
  {
    unsigned short int crc = 0xFFFF;
    for (int pos = 0; pos < len; pos++) {
      crc ^= (unsigned short int)buf[pos];   // XOR byte into least sig. byte of crc
      for (int i = 8; i != 0; i--) {         // Loop over each bit
        if ((crc & 0x0001) != 0) {           // If the LSB is set
          crc >>= 1;                         // Shift right and XOR 0xA001
          crc ^= 0xA001;
        }
        else                            // else LSB is not set
          crc >>= 1;                    // Just shift right
      }
    }  // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
    return crc;  
  }

  unsigned long get_Value(int RS_len)
  /* Extract the Value from the response (byte 3 and 4)  
      and check the CRC if the response is compleat and correct */
  {
    /* Loop only for test and developtment purpose */
  //  for (int i=0; i<RS_len-2; i++)
  //  {
  //    int03 = Response[i];
  //    Serial.printf("Wert[%i] : %i / ", i, int03);
  //  }

  // Check the CRC //
    ReadCRC = (uint16_t)Response[RS_len-1] * 256 + (uint16_t)Response[RS_len-2];
    if (ModBus_CRC(Response, RS_len-2) == ReadCRC) {
      // Read the Value //
      unsigned long val = (uint16_t)Response[3] * 256 + (uint16_t)Response[4];
      return val * 1;       // S8 = 1. K-30 3% = 3, K-33 ICB = 10
    }
    else {
      Serial.print("Error");
      return 99;
    }
  }
#endif

#ifdef isBME280
  /**************************************************!
    @brief    Function to read the BME280 data 
    @details  The BME280 data is read in 
    @return   void
  ***************************************************/
  void getBME280SensorData() 
  {
    // Only needed in forced mode! Forced mode lets the sensor sleep between measurements.
    // normal mode: sensor warming +1,5°C vs. DS18B20. Forced mode: +0,9°C
    bme.takeForcedMeasurement(); // has no effect in normal mode

    Pressure = bme.readPressure()/100;
    // Serial.printf("Pressure: %3.1f mBar ", Pressure);
    Humidity = bme.readHumidity();
    // Serial.printf("Humidity: %3.1f %% ", Humidity);
    Temperature = bme.readTemperature();
    // Serial.printf("Temperature: %3.1f °C \n", Temperature);

    Altitude = 44330.0 * (1.0 - pow((Pressure) / SEA_LEVEL_PRESSURE, 0.1903));
  }  // of method "getSensorData()"
#endif

#ifdef isDisplay
//--------------- shows data on display in larger print, according to displayMode. 
//--------------- displayMode can be toggled by a button. 
  void specialDisplay(int displayMode)
  {
    static float TempC0=-111.11, TempC1 = -111.11, TempC2=-111.11;
    char printstring[80];
    display.clearDisplay(); 
    switch (displayMode)
    {
      #ifdef isBME680 
      case 2:
        sprintf(printstring,"Pressure [mbar]");
        display.setTextSize(1);
        display.setCursor(0, 0);
        display.println(printstring);      
        sprintf(printstring,"%3.1f mB ",pressure);
        display.setTextSize(2);
        display.setCursor(0, 12);
        display.println(printstring);
        sprintf(printstring,"Temperature [C]");
        display.setTextSize(1);
        display.setCursor(0, 32);
        display.println(printstring); 
        sprintf(printstring,"%3.1f C ",temperature);
        display.setTextSize(2);
        display.setCursor(0, 44);
        display.println(printstring);
        break;
      case 3:
        sprintf(printstring,"Humidity ");
        display.setTextSize(1);
        display.setCursor(0, 0);
        display.println(printstring);  
        sprintf(printstring,"%3.1f %% ",humidity);
        display.setTextSize(2);
        display.setCursor(0, 12);
        display.println(printstring);  
        sprintf(printstring,"Air Quality: %s",air_quality_shortstring);
        display.setTextSize(1);
        display.setCursor(0, 32);
        display.println(printstring);  
        sprintf(printstring,"%3.1f %% ",air_quality_score);
        display.setTextSize(2);
        display.setCursor(0, 44);
        display.println(printstring);  
        break;
      #endif   // BME680
      #ifdef isBME280 
      case 2:
        sprintf(printstring,"Pressure [mbar]");
        display.setTextSize(1);
        display.setCursor(0, 0);
        display.println(printstring);      
        sprintf(printstring,"%3.1f mB ",Pressure);
        display.setTextSize(2);
        display.setCursor(0, 12);
        display.println(printstring);
        sprintf(printstring,"Temperature [C]");
        display.setTextSize(1);
        display.setCursor(0, 32);
        display.println(printstring); 
        sprintf(printstring,"%3.1f C ",Temperature);
        display.setTextSize(2);
        display.setCursor(0, 44);
        display.println(printstring);
        break;
      case 3:
        sprintf(printstring,"Humidity ");
        display.setTextSize(1);
        display.setCursor(0, 0);
        display.println(printstring);  
        sprintf(printstring,"%3.1f %% ",Humidity);
        display.setTextSize(2);
        display.setCursor(0, 12);
        display.println(printstring);  
        break;
      #endif   // BME280
      case 4:
      #ifdef isOneDS18B20
        sprintf(printstring,"Temp DS18B20 1/2/3");
        display.setTextSize(1);
        display.setCursor(0, 0);
        display.println(printstring);  
        if(DS18B20Temperature[0]>-110)    // this to avoid flickering if no fresh temp value. use last one.
          TempC0 = calDS18B20Temperature[0];    
        if(TempC0 > -110)  
          sprintf(printstring,"T1: %3.1f C ",TempC0);
        else 
          sprintf(printstring,"T1: ------");        
        display.setTextSize(2);
        display.setCursor(0, 10);
        display.println(printstring);   

        if(noDS18B20Connected > 1 )
        {
          if(DS18B20Temperature[1]>-110)
            TempC1 = calDS18B20Temperature[1]; 
          if(TempC1 > -110)  
            sprintf(printstring,"T2: %3.1f C ",TempC1);
          else 
            sprintf(printstring,"T2: ------");  
          display.setTextSize(2);
          display.setCursor(0, 30);
          display.println(printstring);  
        }  
        if(noDS18B20Connected > 2 )
        {       
          if(DS18B20Temperature[2]>-110)
            TempC2 = calDS18B20Temperature[2]; 
          if(TempC2 > -110)  
            sprintf(printstring,"T3: %3.1f C ",TempC2);
          else 
            sprintf(printstring,"T3: ------");
          display.setTextSize(2);
          display.setCursor(0, 50);
          display.println(printstring);  
        }
        break;    
      #endif  
      case 5:
      #if defined isMHZ14A || defined isSENSEAIR_S8
        #ifdef isMHZ14A
          sprintf(printstring,"CO2 Sensor MZH14A");
        #endif  
        #ifdef isSENSEAIR_S8
          sprintf(printstring,"CO2 - SenseAir S8");
        #endif 
        display.setTextSize(1);
        display.setCursor(0, 0);
        display.println(printstring);  
        sprintf(printstring,"CO2: ");
        display.setTextSize(2);
        display.setCursor(0, 16);
        display.println(printstring);  
        sprintf(printstring,"%d ppm ",CO2ppm);
        display.setTextSize(2);
        display.setCursor(0, 36);
        display.println(printstring);  
        break;
      #endif
      #if defined receiveSERIAL
        sprintf(printstring,"External - 433MHz");
        display.setTextSize(1);
        display.setCursor(0, 0);
        display.println(printstring);  

        sprintf(printstring,"%3.1f C %1.0f%%",InfactoryT[0], InfactoryH[0]);
        display.setTextSize(2);
        display.setCursor(0, 12);
        display.println(printstring);  
        sprintf(printstring,"%3.1f C %1.0f%%",InfactoryT[1], InfactoryH[1]);
        display.setCursor(0, 36);
        display.println(printstring);  
        break;
      #endif
      default:
        sprintf(printstring,"No sensors to show");
        display.setTextSize(1);
        display.setCursor(0, 0);
        display.println(printstring); 
        break; 
    }
    displayDone = 1;    // yes, display has been done (entprellen button im interrupthandler)
  }
#endif

#ifdef isBLYNK
  int blynkDisconnects; // number of times disconnected from server

  void(* resetFunc) (void) = 0; //declare reset function @ address 0 THIS IS VERY USEFUL

  // function to check if blynk is still connected
  void checkBlynk()
  {  // called every 3 seconds by SimpleTimer MyBlynkCheckTimer
    bool isconnected = Blynk.connected();
    if (isconnected == false)
    {
      blynkDisconnects++;  
      if(blynkDisconnects > 10)   
      {
        sprintf(printstring,"checkBlynk: Resetting - no Blynk connection %d %d\n", isconnected, blynkDisconnects);
        logOut(printstring);
        resetFunc();
      }  
      sprintf(printstring,"checkBlynk: Reconnecting Blynk %d %d\n", isconnected, blynkDisconnects);
      logOut(printstring);  
      Blynk.connect();
    }
    else
    {
      blynkDisconnects = 0;
      sprintf(printstring,"checkBlynk: Blynk is connected %d %d\n", isconnected, blynkDisconnects);
      logOut(printstring);       
    }
  }
  /*
  void checkBlynk()
  {  // called every 3 seconds by SimpleTimer MyBlynkCheckTimer
    bool isconnected = Blynk.connected();
    if (isconnected == false)
      disconnects++;         
    if(disconnects > 0)
    {
      Serial.printf("Resetting since not connection to Blynk %d\n", isconnected); 
      resetFunc();
    }
  }
  */

  // function to stop and then restart Blynk
  // intended as workaround, since after a few hours no data are transmitted. 
  void restartBlynk()
  {
    char printstring[80];

    // logics to check if a reboot has been done
    if (restartCount == 0) {            
      NoReboots ++;                     // Increment number of reboots
      preferences.begin("nvs", false);  // Save changed NoReboots to NVS memory
      preferences.putInt("NoReboots", NoReboots);
      preferences.end();
      sprintf(printstring,"Incremented Number of Reboots to: %d\n", NoReboots);
      logOut(printstring); 
    }

    restartCount ++;
    Serial.printf("=== Restarting Blynk routinely. No: %d ===\n", restartCount);
 
    sprintf(printstring, "Restart Blynk No %d. Reboots: %d  ", restartCount, NoReboots);
    logOut(printstring); 
    Blynk.disconnect();
    delay(1500);
    // Blynk.begin(auth, ssid, pass, IPAddress(192,168,178,31), 8080);
    Blynk.begin(auth, ssid, pass, IPAddress(blynkLocalIP), 8080);
    delay(600);
    Blynk.virtualWrite(V12, restartCount); 
  }

  #ifdef isRelay
  // Relay 1 is switched via virtual Pin 18
  BLYNK_WRITE(V18) 
  {
    int x = param.asInt();
    Serial.printf("Relay 1 switched to  %d \n", x);
    if(x==1)  
      digitalWrite(RELAYPIN1, HIGH);
    else  
      digitalWrite(RELAYPIN1, LOW);

  }

// Relay 2 is switched via virtual Pin 19
  BLYNK_WRITE(V19) 
  {
    int x = param.asInt();
    Serial.printf("Relay 2 switched to  %d \n", x);
    if(x==1)  
      digitalWrite(RELAYPIN2, HIGH);
    else  
      digitalWrite(RELAYPIN2, LOW);
  }
  #endif  // relay
#endif  //blynk

#if defined sendSERIAL || defined receiveSERIAL
  // serial stuff for sending / receiving external sensor data

  //******************** send a sentence to serial
  void sendSerial()
  {
    int bytesSent=0;
    char sendString[80];
    sprintf(sendString,"<%4.2f,%4.2f>",serialTemp,serialHumidity);
    if(Serial2.availableForWrite()>=strlen(sendString))
    {
      bytesSent= Serial2.write(sendString);
      Serial.printf("Serial2 %d bytes sent: %s\n",bytesSent, sendString);
    } else 
    {
      Serial.printf("Serial2 write not possible, not available for write\n");
    }  
  }

  boolean newData = false;
  const byte numChars=80;
  char receivedChars[numChars];

  //******************* receive a sentence from serial
  int receiveSerial()
  {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<', endMarker = '>', rc;

    // delete read buffer
    strcpy(receivedChars,"");

    while (Serial2.available() > 0 && newData == false) 
    {
      rc = Serial2.read();

      if (recvInProgress == true) 
      {
        if (rc != endMarker) 
        {
          receivedChars[ndx] = rc;
          ndx++;
          if (ndx >= numChars) 
          {
            ndx = numChars - 1;
          }
        }
        else 
        {
          receivedChars[ndx] = '\0'; // terminate the string
          recvInProgress = false;
          ndx = 0;
          newData = true;
        }
      }
      else if (rc == startMarker) 
      {
        recvInProgress = true;
      }
    } // while

    Serial.printf(" - Serial read: '%s' \n", receivedChars);
    return(strlen(receivedChars));

  } // receiveSerial

  boolean parseData(char *tempChars, float* serialT, float* serialH, int* serialCh) {      // split the data into its parts

    char * strtokIndx;                    // this is used by strtok() as an index
    //char message[numChars];               // buffer

    strtokIndx = strtok(tempChars,",");   // get the first part - the string
    if (strtokIndx != NULL) 
      *serialCh = atoi(strtokIndx);    // convert this part to an integer
    else
      return false;  
 
    strtokIndx = strtok(NULL, ",");  // this continues where the previous call left off
    if (strtokIndx != NULL) 
      *serialT = atof(strtokIndx);     // convert this part to an integer
    else
      return false;  

    strtokIndx = strtok(NULL, ",");
    if (strtokIndx != NULL) 
      *serialH = atof(strtokIndx);     // convert this part to a float
    else
      return false;

    return true;
  }

  // process a set of serial data from Infactory433 sensor connected to an arduino
  boolean processSerialData(float* sTempC, float* sHumidity, int* sChannel, int* numCharsReceived)
  {
    char tempChars[80];
    // char printstring[80];
    int ret=0;
    
    *numCharsReceived = receiveSerial();  // receives string into char array receivedChars
    if (numCharsReceived > 0) {
      strcpy(tempChars, receivedChars);
            // this temporary copy is necessary to protect the original data
            //   because strtok() used in parseData() replaces the commas with \0

      const char failure[10] = "Fail";     
      if(!parseData(tempChars, &serialTemp, &serialHumidity, &serialChannel)
          || (NULL != strstr(receivedChars,failure)) )
      {
        if (NULL != strstr(receivedChars,failure))
          serialFailCount++;
        // sprintf(printstring,"faulty data received, Total Count: %d\n",serialFailCount);
        // logOut(printstring);  
        serialTemp=-111.11;
        serialHumidity=-111;
        serialChannel=0;
        ret = false;
      }  
      else
      {
        *sTempC =  serialTemp;
        *sHumidity = serialHumidity;
        // sprintf(printstring,"converted Infactory (Ch: %d) T: %3.1f %3.1f %%\n",
        //   *sChannel, *sTempC, *sHumidity);
        // logOut(printstring);  
        ret=true;
      }
      newData = false;
    }
    else
    {
      // Serial.println("No serial data");    
      *sTempC =  -111.11;
      *sHumidity = -111.11;
      *sChannel =0;
      ret=false;
    }  
    return(ret);
  } // processSerialData()

#endif // serial stuff  

//*****************************************************************************
// main handler, was loop
//*****************************************************************************
void main_handler() 
{
  char printstring[180]="";
  // char printstring1[80]="", printstring2[80]="", printstring3[80]="";
  float time_sec;

  // float time_sec;
  long start_loop_time, end_loop_time;
  static long lastBME680Time;

  time_sec = (float)millis()/1000;
  start_loop_time = millis();
  // Serial.printf(" ******* Main Loop start at %3.1f sec ********** \n",time_sec);
  sprintf(printstring, "M %3.1f ",time_sec);
  logOut(printstring);
  //sprintf(printstring,"\n");
  //if(((int)(time_sec+0.5))%10 == 0)
  //  logOut(printstring);

  #ifdef isInfactory433
    // if( (millis() > lastInfactoryReception + 45000) || (lastInfactoryReception < 1))
    if(millis() > lastInfactoryReception + 40000)
    {
      measuringInfactoryOngoing = true;     // let others know: measurements are being done
      // stop Blynk, since it interferes with the interrupt routine that collects the radio data from sensor
      #ifdef isBLYNK
        Blynk.disconnect();
        vTaskDelay(1000 / portTICK_PERIOD_MS); // wait 1000ms
      #endif  

      doInfactoryStuff(70000);   // try to read infactory sensor for 70 sec. intended to block loop
      sprintf(printstring,"\n===== Main Loop after doInfactoryStuff: %d  ", successInfactoryCalc);
      logOut(printstring);
      sprintf(printstring,"TempC %3.1f Humidity %3.1f %%  Channel %d:\n", 
          InfactoryTempC, InfactoryHumidity, InfactoryChannel);
      logOut(printstring);
      trialsInfactory++;
      if(InfactoryTempC > -110 && InfactoryHumidity > -110) 
        successInfactory++;
      sprintf(printstring,"Infactory Readings %d of %d attempts\n", successInfactory, trialsInfactory);  
      logOut(printstring);

      // restart Blynk
      #ifdef isBLYNK
        esp_task_wdt_reset(); // reset watchdog in case it takes longer
        // testxxxxx Blynk.begin(auth, ssid, pass, IPAddress(blynkLocalIP), 8080);
        Blynk.connect();
        if((InfactoryTempC > -110) && successInfactoryCalc)
        {         
          Blynk.virtualWrite(V15, InfactoryTempC); //sending to Blynk, if other than the default -111.11
          sprintf(printstring,"Main >>>>>> Sending TempC %3.1f ", InfactoryTempC);
          logOut(printstring);
          Blynk.virtualWrite(V16, InfactoryHumidity); //sending to Blynk
          sprintf(printstring,">>>>>> Sending Humidity %3.1f \n", InfactoryHumidity);
          logOut(printstring);
        }  
        else
        {
          // get last data collected in non-volatile storage, stored to survive reboot
          preferences.begin("nvs", false);                          // Open nonvolatile storage (nvs)
          InfactoryTempC = preferences.getFloat("InfTempC", -111.1);
          InfactoryHumidity = preferences.getFloat("InfHumidity", -111.11);
          preferences.end();
          if(InfactoryTempC > -110 && InfactoryHumidity > -110)    // valid data reiceived from nvs
          {
            Blynk.virtualWrite(V15, InfactoryTempC); //sending to Blynk, if other than the default -111.11
            sprintf(printstring,"Main NVS >->->->->-> Sending TempC %3.1f ", InfactoryTempC);
            logOut(printstring);
            Blynk.virtualWrite(V16, InfactoryHumidity); //sending to Blynk
            sprintf(printstring,">->->->->-> Sending Humidity %3.1f \n", InfactoryHumidity);
            logOut(printstring);
            preferences.begin("nvs", false);                    // Open nonvolatile storage (nvs)
            preferences.putFloat("InfTempC", -111.1);           // reset nvs to defaults 
            preferences.putFloat("InfHumidity", -111.11);       // to only read once
            preferences.end();
          }  
        }
      #endif  
      received = false;   // reset reception flag
      successInfactoryCalc = false; // reset for next round
      measuringInfactoryOngoing = false; // 433 MHz measuring done, allow DS18B20 measurements again
    }
  #endif  // isInfactory433

  #ifdef sendSERIAL  
    serialTemp=InfactoryTempC;
    serialHumidity=InfactoryHumidity;

    sendSerial();
  #endif  // sendSERIAL

  #ifdef receiveSERIAL  
    boolean serialReceived = false;
    int charsReceived = 0;
    serialReceived = processSerialData(&InfactoryTempC, &InfactoryHumidity, &serialChannel, &charsReceived);
    if(serialReceived)
    {
      sprintf(printstring,"converted Infactory (Ch: %d) T: %3.1f %3.1f %%\n",
        serialChannel, InfactoryTempC, InfactoryHumidity);
      logOut(printstring);  
    }
    else
    {
      if(charsReceived > 0)
      {  
        sprintf(printstring,"faulty data received, Total Count: %d\n",serialFailCount);
        logOut(printstring);  
      }  
    }

    #ifdef isBLYNK
      if(serialReceived)
      {         
        if(lastInfactoryTempC[serialChannel]<-110) 
          lastInfactoryTempC[serialChannel] = InfactoryTempC; // get out of default
        if(abs(lastInfactoryTempC[serialChannel]-InfactoryTempC) < 5.0)  // too large difference-improbable value
        {
          lastInfactoryTempC[serialChannel] = InfactoryTempC;
          // restart Blynk
          esp_task_wdt_reset(); // reset watchdog in case it takes longer
          // testxxxxx Blynk.begin(auth, ssid, pass, IPAddress(blynkLocalIP), 8080);
          Blynk.connect();
          sprintf(printstring,"Blynk Connected, serialReceived: %d \n",serialReceived);
          logOut(printstring);  
          checkBlynk();   // test to ensure that blynk is running

          do{
            Blynk.run(); 
            Blynk.virtualWrite(V38, serialFailCount);
            Blynk.run();
            // Blynk.virtualWrite(V38, serialFailCount);
            sprintf(printstring,">>>>>> Send TempC %3.1f >>>> Humidity %3.1f Ch: %d (fail: %d)\n",
                InfactoryTempC, InfactoryHumidity, serialChannel,serialFailCount);
            logOut(printstring); 
            // store the channel related data for specialDisplay();
            InfactoryT[serialChannel] = InfactoryTempC;
            InfactoryH[serialChannel] = InfactoryHumidity;
            switch(serialChannel){
              case 0:
                Blynk.virtualWrite(V15, InfactoryTempC); //sending to Blynk, if other than the default -111.11
                Blynk.virtualWrite(V16, InfactoryHumidity); //sending to Blynk
                serialSentCh1Count++;
                Blynk.virtualWrite(V35, serialSentCh1Count);
              break;
              case 1:
                Blynk.virtualWrite(V17, InfactoryTempC); //sending to Blynk, if other than the default -111.11
                Blynk.virtualWrite(V18, InfactoryHumidity); //sending to Blynk
                serialSentCh2Count++;
                Blynk.virtualWrite(V36, serialSentCh2Count);
              break;
              case 2:
                Blynk.virtualWrite(V19, InfactoryTempC); //sending to Blynk, if other than the default -111.11
                Blynk.virtualWrite(V20, InfactoryHumidity); //sending to Blynk
                serialSentCh3Count++;
                Blynk.virtualWrite(V37, serialSentCh3Count);
              break;
              default: 
              Serial.printf("Unknown Channel %d\n",serialChannel);  
            }
            esp_task_wdt_reset(); // reset watchdog in case it takes longer
            Blynk.run();
            vTaskDelay(200 / portTICK_PERIOD_MS); // wait 200ms
            serialReceived = processSerialData(&InfactoryTempC, &InfactoryHumidity, &serialChannel, &charsReceived);
          } while(serialReceived);  
          Blynk.run();

          // send data from other sensors before disconnecting Blynk
          #ifdef isOneDS18B20
            if(calDS18B20Temperature[0] > -110)
              Blynk.virtualWrite(V13, calDS18B20Temperature[0]); //sending to Blynk
            if(calDS18B20Temperature[1] > -110)  
              Blynk.virtualWrite(V10, calDS18B20Temperature[1]); //sending to Blynk
            if(calDS18B20Temperature[2] > -110)  
              Blynk.virtualWrite(V11, calDS18B20Temperature[2]); //sending to Blynk  
            Blynk.run();  
            sprintf(printstring,">>>>> Ds18B20 Temp's: T1: %3.1f T2: %3.1f T3: %3.1f\n",calDS18B20Temperature[0],calDS18B20Temperature[1],calDS18B20Temperature[2]);
            logOut(printstring);  
            Blynk.run();  
          #endif
          #ifdef isBME280
            if(Pressure > 950) // prevent sending zeroes
            {
              Blynk.virtualWrite(V5, Temperature); 
              Blynk.virtualWrite(V6, Pressure); 
              Blynk.virtualWrite(V7, Humidity); 
              Blynk.run();  
              sprintf(printstring,">>>>> BME280 Sensor values: %3.1f °C %3.1f %% %3.1f mBar %3.1f m\n", 
              Temperature, Humidity, Pressure, Altitude); 
              logOut(printstring);    
              Blynk.run();  
            }  
          #endif
          vTaskDelay(500 / portTICK_PERIOD_MS); // wait 500ms before disconnecting
          Blynk.disconnect();
        }
      }  
    #endif  // serialReceived
  #endif  //receiveSERIAL

  #ifdef isDisplay
    // clear the display - only once!
  /*  
    display.clearDisplay(); 
  */  
  #endif

  // DS18B20 Data are received in parallel running procedure "GetOneDS18B20Temperature()"
  #ifdef isOneDS18B20
    // check state of parallel task
    // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/freertos.html
    // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/freertos.html#_CPPv46eReady
    eTaskState state;
    state = eTaskGetState(Task1); // eRunning=0, eReady=1, eBlocked=2, eSuspended=3, eDeleted=4, eInvalid
    // GetOneDS18B20Counter is incremented every time the loop runs to get DS18B20

    // correct with compensation factors which are specific to each sensor module, defined near auth codes
    for(int iii=0; iii<noDS18B20Connected; iii++)
      calDS18B20Temperature[iii] = DS18B20Temperature[iii] + corrDS18B20[iii];
      
    sprintf(printstring,"\nBase DS18B20 Temp1 %3.1f  Temp2 %3.1f Temp3 %3.1f %d %d %d - %d %ld\n", 
      DS18B20Temperature[0], DS18B20Temperature[1], DS18B20Temperature[2], 
      notMeasuredCount, notChangedCount, noDS18B20Restarts, state, GetOneDS18B20Counter);
     logOut(printstring);
    //sprintf(printstring,"Cal. DS18B20 Temp1 %3.1f  Temp2 %3.1f Temp3 %3.1f \n", 
    //   calDS18B20Temperature[0], calDS18B20Temperature[1], calDS18B20Temperature[2]);
    //logOut(printstring);
    #ifdef isDisplay
    /*
    if(displayMode == 1)
    {
      display.setCursor(0, 36);
      display.setTextSize(1);
      if (DS18B20Temperature[0]>=-110)
        sprintf(printstring1, "T1:%3.1f", calDS18B20Temperature[0]);
      if (DS18B20Temperature[1]>=-110)
        sprintf(printstring2, "2:%3.1f", calDS18B20Temperature[1]); 
      if (DS18B20Temperature[2]>=-110)
        sprintf(printstring3, "3:%3.1f\n", calDS18B20Temperature[2]);   
      sprintf(printstring,"%s %s %s ", printstring1, printstring2, printstring3);
      display.println(printstring);

      display.setCursor(80, 48);
      sprintf(printstring,"1W.R: %d",noDS18B20Restarts);
      display.println(printstring);
    } 
    */ 
    #endif  
    
    #ifdef isBLYNK
      if(DS18B20Temperature[0] > -110)
        Blynk.virtualWrite(V13, calDS18B20Temperature[0]); //sending to Blynk
      if(DS18B20Temperature[1] > -110)  
        Blynk.virtualWrite(V10, calDS18B20Temperature[1]); //sending to Blynk
      if(DS18B20Temperature[2] > -110)  
        Blynk.virtualWrite(V11, calDS18B20Temperature[2]); //sending to Blynk  
      vTaskDelay(300 / portTICK_PERIOD_MS); // non-blocking delay instead
    #endif  

    // checks for problems with measurements of DS18B20
    if(notMeasuredCount > DS18B20RestartLimit || notChangedCount > 3*DS18B20RestartLimit || manualDS18B20Restart >= 1)  // in GetOneDS18B20Temperature this count is handled if faulty checksum
    {
      sprintf(printstring,"\nRestarting DS18B20 Sensors since not measurement taken in %d %d cycles \n",
        notMeasuredCount, notChangedCount);
      logOut(printstring);
      restartDS18B20();
      notMeasuredCount = 0; // reset the counter
      notChangedCount = 0;  // reset the counter
      manualDS18B20Restart = 0; // reset the manual switch
    }

  #endif  // isOneDS18B20

  //*** get sensor data from BME680 P/T/%/Gas sensor
  #ifdef isBME680
    // measure only every 60 seconds
    if(start_loop_time > lastBME680Time + 60000)
    {
      lastBME680Time = start_loop_time;
      getBME680SensorData();
      // correct with compensation factors which are specific to each sensor module, defined near auth codes
      temperature += corrBME680Temp;
      getAirQuality();    // get air quality index
      Serial.printf(" %3.1f Temp %3.1f Humid %3.1f Pressure %3.1f Gas %5.0f (Alt %3.1f) Air %f %s \n", 
                    time_sec, temperature, humidity, pressure, gas, altitude, air_quality_score, air_quality_string);

      #ifdef isBLYNK
        // get the data to blynk
        Blynk.virtualWrite(V5, temperature); 
        Blynk.virtualWrite(V6, pressure); 
        Blynk.virtualWrite(V7, humidity); 
        Blynk.virtualWrite(V8, air_quality_score); 
        Blynk.virtualWrite(V12, air_quality_string); 
      #endif
      
      // delay(300); // give blynk time to send the stuff
      vTaskDelay(300 / portTICK_PERIOD_MS); // non-blocking delay instead
    } // if start_loop_time  
    #ifdef isDisplay
    /*
      if(displayMode == 1)
      {
        sprintf(printstring,"ESP32-EnvMonitor");
        //Display(printstring, 1,0,0,true);
        display.setCursor(0, 0);
        display.setTextSize(1);
        display.println(printstring);

        sprintf(printstring,"%3.1f mB %3.1f C",pressure,temperature);
        display.setCursor(0, 12);
        display.println(printstring);
        sprintf(printstring,"%3.1f%% Q:%3.1f %s",humidity,air_quality_score, air_quality_shortstring);
        display.setCursor(0, 24);
        display.println(printstring);
      }  
    */
    #endif
  #endif  // isBME680

  #ifdef isBME280
    getBME280SensorData();
    // correct with compensation factors which are specific to each sensor module, defined near auth codes
    Temperature += corrBME280Temp;
    sprintf(printstring,"BME280 Sensor values: %3.1f °C %3.1f %% %3.1f mBar %3.1f m\n", 
        Temperature, Humidity, Pressure, Altitude); 
    logOut(printstring);    
    #ifdef isBLYNK
      // get the data to blynk
      Blynk.virtualWrite(V5, Temperature); 
      Blynk.virtualWrite(V6, Pressure); 
      Blynk.virtualWrite(V7, Humidity); 
      // Blynk.virtualWrite(V8, Altitude); 
    #endif

    #ifdef isDisplay
    /*
      if(displayMode == 1)
      {
        sprintf(printstring,"EnvMonitor680 %s", PROGVERSION);
        //Display(printstring, 1,0,0,true);
        display.setCursor(0, 0);
        display.setTextSize(1);
        display.println(printstring);

        sprintf(printstring,"%3.1f mB %3.1f C",Pressure,Temperature);
        display.setCursor(0, 12);
        display.println(printstring);
        sprintf(printstring,"%3.1f %%",Humidity);
        display.setCursor(0, 24);
        display.println(printstring);
      }  
    */  
    #endif

    vTaskDelay(100 / portTICK_PERIOD_MS); // non-blocking delay instead
  #endif  // isBME280
	
  //*** read data from CO2 sensor
  #ifdef isMHZ14A
    MHZ14AWarmingTime = time_sec - warmingTimer/1000;
    if(MHZ14AWarmingTime > MZH14AWarmupWait)
    {
      if (time_sec - timer1/1000 > MZH14AMeasureWait)  // runs every 15 sec
      {
        getCO2Data();
        Serial.printf("\n %3.1f MH-Z14A CO2 Sensor value: %d PPM \n", time_sec, CO2ppm);
        timer1 = millis();
      }
      #ifdef isDisplay
      /*
        if(displayMode == 1)
        {
          display.setCursor(0, 48);
          display.setTextSize(1);
          sprintf(printstring, "CO2 %d ppm \n", CO2ppm);
          display.println(printstring);  
        }  
      */  
      #endif  
      #ifdef isBLYNK 
        Blynk.virtualWrite(V9, CO2ppm);
      #endif  
      Serial.printf("%3.1f [sec] ", time_sec);
    }
    else
      Serial.printf(" CO2 Sensor still warming up %3.1f of %3.1f\n", 
        (float)MHZ14AWarmingTime, (float)MZH14AWarmupWait);
  #endif  // isMHZ14A

  #ifdef isSENSEAIR_S8
    send_Request(CO2req, 8);               // send request for CO2-Data to the Sensor
    read_Response(7);                      // receive the response from the Sensor
    CO2ppm = get_Value(7);
    Serial.printf("\n %3.1f SenseAir CO2 Sensor value: %d PPM \n", time_sec, CO2ppm);
    #ifdef isDisplay
    /*
      if(displayMode == 1)
      {
        display.setCursor(0, 48);
        display.setTextSize(1);
        sprintf(printstring, "CO2 %d ppm \n", CO2ppm);
        display.println(printstring);  
      }  
    */  
    #endif  // display
    
    #ifdef isBLYNK 
      Blynk.virtualWrite(V9, CO2ppm);
    #endif  // Blynk
  #endif    // SENSEAIR

  #ifdef isDisplay
  /*
    if(displayMode > 1)
      specialDisplay(displayMode);

    // clear display in mode 0
    if(displayMode == 0)
       display.clearDisplay(); 

    // dim display 10 sec after last button push  
    if(millis() > lastButtonTime + displayOffDelay)
    {
      display.dim(true);
      displayDimmed = true;
    }  
    else
    {
      display.dim(false);  
      displayDimmed = false;
    }  

    // transfer buffer to display  
    display.display();
    displayDone = 1;
  */  
  #endif  // isDisplay

  // loop timing - no more needed, with main_handler controlled by timer
  /*
  end_loop_time= millis();
  int delaytime= 2000-(end_loop_time-start_loop_time);
  // Serial.printf(".%ld %ld %d|",interruptCount, interruptFlagger, delaytime);
  if (delaytime < 0) delaytime = 0;
  if (delaytime > 2000) delaytime = 2000;
  vTaskDelay(delaytime/ portTICK_PERIOD_MS); // delay for 100 ms, non-blocking
  */

  // main handler successfully finished, reset watchdog
  esp_task_wdt_reset();

  // Serial.printf("End main_handler\n");
} // main_handler

// handler for LCD display functions, called via timer
#ifdef isLCD
void lcd_handler()
{
  if(lcdDisplayMode==0)
  {
    sprintf(printstring,"\ndisplayLCD Mode  is zero: %d\n", lcdDisplayMode);
    logOut(printstring);
    lcd.clear();
    lcd.noBacklight();
    lcdDisplayDone = 1;   // makes ready to receive button press again
  }
  if(lcdDisplayMode > 0)
  {
    sprintf(printstring,"\ndisplayLCD Mode %d\n", lcdDisplayMode);
    logOut(printstring);
    printLocalTime(printstring, 6);
    displayLCD(lcdDisplayMode, 
      Pressure, Temperature, Humidity, 
      calDS18B20Temperature[0], calDS18B20Temperature[1], calDS18B20Temperature[2],
      1234, // CO2ppm,
      InfactoryT[0], InfactoryH[0],InfactoryT[1], InfactoryH[1],
      printstring, infoStringShort
    );
    lcdDisplayDone = 1; // makes ready to receive button press again
  }  
}
#endif //#ifdef isLCD

// handler for OLED display functions, called via timer
#ifdef isDisplay
  void oled_handler()
  {
    // clear the display - only once!
    display.clearDisplay(); 

    #ifdef isOneDS18B20
      if(displayMode == 1)
      {
        display.setCursor(0, 36);
        display.setTextSize(1);
        if (DS18B20Temperature[0]>=-110)
          sprintf(printstring1, "T1:%3.1f", calDS18B20Temperature[0]);
        if (DS18B20Temperature[1]>=-110)
          sprintf(printstring2, "2:%3.1f", calDS18B20Temperature[1]); 
        if (DS18B20Temperature[2]>=-110)
          sprintf(printstring3, "3:%3.1f\n", calDS18B20Temperature[2]);   
        sprintf(printstring,"%s %s %s ", printstring1, printstring2, printstring3);
        display.println(printstring);

        display.setCursor(80, 48);
        sprintf(printstring,"1W.R: %d",noDS18B20Restarts);
        display.println(printstring);
      }  
    #endif  //isOneDS18B20

    #ifdef isBME680
    if(displayMode == 1)
    {
      sprintf(printstring,"ESP32-EnvMonitor");
      //Display(printstring, 1,0,0,true);
      display.setCursor(0, 0);
      display.setTextSize(1);
      display.println(printstring);

      sprintf(printstring,"%3.1f mB %3.1f C",pressure,temperature);
      display.setCursor(0, 12);
      display.println(printstring);
      sprintf(printstring,"%3.1f%% Q:%3.1f %s",humidity,air_quality_score, air_quality_shortstring);
      display.setCursor(0, 24);
      display.println(printstring);
    }  
    #endif  //isBME680

    #ifdef isBME280
      if(displayMode == 1)
      {
        sprintf(printstring,"EnvMonitor680 %s", PROGVERSION);
        //Display(printstring, 1,0,0,true);
        display.setCursor(0, 0);
        display.setTextSize(1);
        display.println(printstring);
        sprintf(printstring,"%3.1f mB %3.1f C",Pressure,Temperature);
        display.setCursor(0, 12);
        display.println(printstring);
        sprintf(printstring,"%3.1f %%",Humidity);
        display.setCursor(0, 24);
        display.println(printstring);
      }  
    #endif //#ifdef isBME280
  
    #ifdef isMHZ14A
      if(displayMode == 1)
      {
        display.setCursor(0, 48);
        display.setTextSize(1);
        sprintf(printstring, "CO2 %d ppm \n", CO2ppm);
        display.println(printstring);  
      }  
    #endif //#ifdef isMHZ14A

    #ifdef isSENSEAIR_S8
      if(displayMode == 1)
      {
        display.setCursor(0, 48);
        display.setTextSize(1);
        sprintf(printstring, "CO2 %d ppm \n", CO2ppm);
        display.println(printstring);  
      } 
    #endif // #ifdef isSENSEAIR_S8

    if(displayMode > 1)
        specialDisplay(displayMode);

    // clear display in mode 0
    if(displayMode == 0)
      display.clearDisplay(); 

    // dim display 10 sec after last button push  
    if(millis() > lastButtonTime + displayOffDelay)
    {
      display.dim(true);
      displayDimmed = true;
    }  
    else
    {
      display.dim(false);  
      displayDimmed = false;
    }  

    // transfer buffer to display  
    display.display();
    displayDone = 1;
  }
#endif // isDisplay

//*****************************************************************************
// main loop
//*****************************************************************************
void loop()
{
  //mainHandlerTimer.run();   // timer for main handler
  MyBlynkTimer.run();
  
  #ifdef isLCD
    //lcdHandlerTimer.run();  // timer for LCD display functions
  #endif

  //*** Blynk communication
  #ifdef isBLYNK
    Blynk.run(); 
    // check if Blynk is connected
    #ifdef blynkRegularCheck
      //MyBlynkCheckTimer.run();
    #endif
    // and restart it every 3 hours, since after 200 min M5Stack stops sending data
    //MyBlynkRestartTimer.run();
  #endif  
 
  // handle OTA over the air Updates 
   #ifdef isOTA
    ArduinoOTA.handle();
  #endif
}
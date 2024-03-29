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
#include <SimpleTimer.h>        // Simple Timer, used instead of Blynk timer for virtuino
#include <Preferences.h>        // used to permanently store data
#include <esp_task_wdt.h>       // Load Watchdog-Library
#include "GlobalDefines.h"      // global defines that determine what to compile
#include "HelperFunctions.h"    // prototypes for global helper functions
#include "EnvMonitorBME680.h"   // My main include file
#include "Infactory433.h"       // Infactory 433 Temp/Humid sensor
#include "SDFunctions.h"        // functions to handle SD memory card
#include "LCDFunctions.h"       // functions to handle LCD display
#include "ESP32Ping.h"          // ping library
#include "CaptivePortal.h"      // Captive Portal functions

//#include <cstring>
//#include <string>

void(* resetFunc) (void) = 0; //declare reset function @ address 0 THIS IS VERY USEFUL

// Write string on the SD card
#if defined logSD && defined isSD
  void logSDCard(char *printstring) 
  {
    // Serial.print("Save to SD: ");
    // Serial.println(printstring);
    appendFile(SD, logfilename, printstring);
  }
#endif  

// function to handle all logging output. To serial, into file on SD, to Blynk terminal
void logOut(char* printstring, unsigned int MsgID, unsigned int MsgSeverity)
  {
    char timestring[50]="";      
    char outstring[230];

    #ifdef isLEDHeartbeat
      heartbeatStatus = !heartbeatStatus;
      // digitalWrite (HEARTBEATPIN, heartbeatStatus);
      digitalWrite (HEARTBEATPIN, HIGH);
    #endif

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

    #ifdef isLEDHeartbeat
      heartbeatStatus = !heartbeatStatus;
      digitalWrite (HEARTBEATPIN, LOW);
    #endif

    #ifdef isSyslog
      if(WiFi.status() == WL_CONNECTED)
      {
        sprintf(outstring, "%d %d %s", MsgID, MsgSeverity, printstring);
        syslog.log(LOG_INFO | LOG_USER, outstring);
        // Log message can be formated like with printf function.
        // syslog.logf(LOG_ERR,  "This is error message no. %d", iteration);  
        // Log Levels: LOG_INFO, LOG_ERR, LOG_DAEMON 
      }
    #endif

    #ifdef isMQTTLog
      char topicStr[200];
      sprintf(topicStr,"esp32/%s/log/%d/%d",mqttRoomString, MsgID, MsgSeverity);
      strcpy(outstring, timestring);
      strcat(outstring, printstring); 

      //sprintf(printstring, "test logOut 2. Topic: _%s_ _%s_ %d",topicStr, outstring, strlen(outstring));
      //Serial.print(printstring);
      if(mqttClient.connected())
        mqttClient.publish(topicStr, outstring, strlen(outstring)); //payload: outstring
      //sprintf(printstring, "test logOut 2");  
      //Serial.print(printstring);
    #endif

  }

#ifdef isDisplay

  // Display class based on AdafruitGFX, special type SSD1306
  Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

  /**************************************************!
  @brief    Display one string on OLED display
  @details 
  @param string String to be displayed
  @param size Text size in multiples of standard. 1: normal, 2: double ....
  @param x  x-position of text
  @param y  y-position of text
  @param clear boolean flag, determines if display is cleared before text output
  @return   void
  ***************************************************/
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

  volatile unsigned long LastMeasTimer;
  const long ds18b20MeasInterval = 1000; // measure every xxx ms

  // mutex could be needed for critical section, not implemented
  // static portMUX_TYPE my_mutex = portMUX_INITIALIZER_UNLOCKED;

  /**************************************************!
  @brief    Function to measure DS18B20 temperatures in endless loop. Designed to run in parallel
  @details  This task runs isolated on core 0 because sensors.requestTemperatures() is slow and blocking for about 750 ms
            gets temperature from DS18B20 via OneWire
            Temperature is measured by address, not by indes. This ensures that there is no mismatch, if a sensor
            goes temporarily offline (as fake sensors occasionally do)
            Does some checking. Sensor values indicate errors if out of range:
              -127: bad reading. 85: no measurement yet (to early after a restart)
            Uses global variables:
            - measuringInfactoryOngoing : If this flag is set, no measurements are taken (to not disturb timing)
            - stopDS18B20MeasureFlag : If this flag is set, no measurements are taken (to not disturb timing)
            - noDS18B20Connected : Number of DS18B20 which are connected
            - DS18B20Address[i][j] :  Address of Sensor i, each 8 bytes long (j= 0..7)
            Returns global variables
            - DS18B20Temperature[i] : raw measured temperature in °C for each sensor
            - LastMeasTimer : timer mark when last measurement has been done
            - notChangedCount : global counter for not changed values. Used to determine if measurements stopped
            - GetOneDS18B20Counter : global counter for cycles through this procedure. Used to determine if measurements stopped
  @param    parameter   Task input parameter, not used. Required for starting function
  @return   void
  ***************************************************/
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
          logOut(printstring, msgDS18B20NoMeasFlag, msgWarn);  
        }
      }
      else
      {
        vTaskDelay(200 / portTICK_PERIOD_MS); // delay for 200 ms
        // sprintf(printstring,"W %ld %ld ", millis(),LastMeasTimer);    
        // logOut(printstring, msgDefaultID, msgDefault);
        GetOneDS18B20Counter ++;  
      }  
    }
  }

  //---- the following procedure determines the number of DS18B20, sets their precision to 12 bit and stores their addresses
  DeviceAddress tempDeviceAddress;
  #define TEMPERATURE_PRECISION 12   // precision 9..12 Bit

  /**************************************************!
  @brief    Determine number of DS18B20 sensors on 1Wire Bus and determine their addresses
  @details  First attempts sensors.getDeviceCount() to determine the number of devices, which does not work presently
            Then sets temperature precision to 12 bits for 3 devices 
            Finally uses sensors.getAddress() to count how many devices actually present.
            Fills the following global variables:
            noDS18B20Connected                      : Number of DS18B20 actually present
            DS18B20Address[noDS18B20Connected][i]   : their addresses
  @param    none
  @return   void
  ***************************************************/
   void adresseAusgeben(void) {
    byte i, j, k;
    uint8_t addr[8];          // uint8_t = unsigned char, 8 bit integer
    int numberOfDevices, noRepeats;
    char ps[80];
    long checksum[5], sum1, sum2;         // address checksums
    bool check, crcCheck;

    // loop to determine the addresses of the DS18B20 sensors. 
    // Repeat in case of crc check failure or duplicate addresses
    noRepeats = 0;
    do{
      esp_task_wdt_reset();   // keep watchdog happy
      // code 1: devices finden.
      // this method does not work for ESP32
      delay(1000);
      oneWire.reset_search(); // reset search of oneWire devices
      numberOfDevices = sensors.getDeviceCount();
  
      sprintf(printstring,"sensors.getDeviceCount found %d Devices \n", numberOfDevices);
      logOut(printstring, msgDS18B20Info, msgInfo);

      numberOfDevices = noDS18B20Sensors;   // number of DS18B20 expected; /// temporary
      // Setzen der Genauigkeit
      for(i=0; i<numberOfDevices; i++) {
        if(sensors.getAddress(tempDeviceAddress, i)) {
          sensors.setResolution(tempDeviceAddress, TEMPERATURE_PRECISION);
          Serial.print(F("Sensor "));
          Serial.print(i);
          Serial.print(F(" had a resolution of "));
          Serial.println(sensors.getResolution(tempDeviceAddress), DEC);
        }
      }
      Serial.println("");
      numberOfDevices = sensors.getDeviceCount();   // does not function with OneWire library 2.3.5 (claimed to be ok with 2.3.3)
      sprintf(printstring,"Found %d sensors\n", numberOfDevices);
      logOut(printstring, msgDS18B20Info, msgInfo);
      esp_task_wdt_reset();   // keep watchdog happy

      // code 2: find devices by searching on the onewire bus across all addresses. 
      // workaround, works apparently for ESP32

      sprintf(printstring,"Searching 1-Wire-Devices...\n\r");// "\n\r" is NewLine
      logOut(printstring, msgDS18B20Info, msgInfo);
      
      //critical section. Could help with incorrect sensor detection
      // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/freertos-smp.html#critical-sections-disabling-interrupts
      //portMUX_TYPE myMutex = portMUX_INITIALIZER_UNLOCKED;
      //portENTER_CRITICAL(&myMutex);
      #ifdef isBLYNK
        Blynk.disconnect();
        delay(1500);
      #endif  

      crcCheck = true;  // reset crc Check flag
      for(j=1; j<=20 ; j++)
      {
        noDS18B20Connected = 0;
        oneWire.reset_search(); // reset search of oneWire devices
        delay(100);
        while(sensors.getAddress(addr, noDS18B20Connected)) {  

          sprintf(printstring,"1-Wire-Device %d found with Adress: ", noDS18B20Connected);
          // logOut(printstring, msgDS18B20Info, msgInfo);
          esp_task_wdt_reset();   // keep watchdog happy

          // Fletcher checksum algorithm https://de.wikipedia.org/wiki/Fletcher%E2%80%99s_Checksum 
          checksum[noDS18B20Connected] = 0;
          sum1=0;
          sum2=0;
          for( i = 0; i < 8; i++) {
            DS18B20Address[noDS18B20Connected][i] = addr[i];
            sum1 = (sum1 + addr[i]) % 255;
            sum2 = (sum2 + sum1) % 255;
            //checksum[noDS18B20Connected] += (i+1)*addr[i];
            // Serial.print("0x");
            /*
            if (addr[i] < 16) {
            strcat(printstring,"0");
            }
            sprintf(printstring,"%s%d",printstring, addr[i]);
            if (i < 7) {
              strcat(printstring," ");
            }
            */
          }
          checksum[noDS18B20Connected] = 256 * sum1 + sum2; // checksum is two sub-sums combined
          // strcat(printstring,"\n");
          //logOut(printstring, msgDefaultID, msgInfo);
          if ( OneWire::crc8( addr, 7) != addr[7]) {
            // sprintf(printstring,"CRC is not valid!\n\r");
            // logOut(printstring, msgDS18B20Info, msgWarn);
            crcCheck = false;
            //return;
          }
          noDS18B20Connected ++;    // one more device found
        } // while

        // check if no two addresses are identical
        check = true;
        /*
        for(i = 1; i<noDS18B20Connected; i++){
          if(checksum[i] == checksum[i-1])
            check = false; 
        }
        */
        for(i = 1; i<noDS18B20Connected; i++)
          for(k = 0; k<i; k++)
            if(checksum[i] == checksum[k])
              check = false; 

        if ((noDS18B20Connected >= noDS18B20Sensors) && (check == true)) // exit for loop if expected number of sensors has been found
          break;
        delay(j * 100);
      } // for j
      #ifdef isBLYNK
        Blynk.connect();
        delay(600);
      #endif  
      //portEXIT_CRITICAL(&myMutex); // exit critical section
      noRepeats++;
    }while((check==false || crcCheck==false) && noRepeats < 3); // repeat until the check is true

    sprintf(printstring,"Check: %d \n", check);
    logOut(printstring, msgDS18B20Info, msgInfo);
    sprintf(printstring,"Found 1-Wire-Devices: %d in %d loop runs\n", noDS18B20Connected, j);
    logOut(printstring, msgDS18B20Info, msgInfo);

    for(j=0 ; j< noDS18B20Connected; j++) 
    {
      sprintf(printstring, "Addr [%d] (Checksum: %ld): ", j, checksum[j]);
      for( i = 0; i < 8; i++) {
        // Serial.print(DS18B20Address[noDS18B20Connected][i], HEX);
        // Serial.print(" ");
        itoa(DS18B20Address[j][i], ps, 16);
        strcat(printstring, ps); 
        strcat(printstring," ");
      }
      strcat(printstring,"\n");
      logOut(printstring, msgDS18B20Info, msgInfo);
    }  
    oneWire.reset_search(); // reset search of oneWire devices
    
    return;
  }

  /**************************************************!
  @brief    Complete restart of the DS18B20 measurement function, incl. parallel running meas. function
  @details  Terminates the detached task (Task1) running to do DS10B20 measurements in parallel
            Switches power for the 1Wire bus off for 2 seconds, then on again.
            Then determines the number and addresses of DS18B20 sensors
            And finally restarts the measuring function running in parallel as Task1 (GetOneDS18B20Temperature)
  @param    none
  @return   void
  ***************************************************/
  void restartDS18B20MeasurementFunction()
  {
    noDS18B20Restarts++;
    sprintf(printstring,"XXXX Restarting DS18B20 measuring function XXXX !!! %d \n", noDS18B20Restarts);
    logOut(printstring, msgDS18B20Restart, msgWarn);  
    stopDS18B20MeasureFlag = true;   // use this flag to stop measurements with DS18B20
    esp_task_wdt_reset();   // keep watchdog happy

    vTaskDelete(Task1);              // delete the measurement task

    // switch off Power (via GPIO 32)
    pinMode(POWER_ONEWIRE_BUS, OUTPUT);
    digitalWrite(POWER_ONEWIRE_BUS, LOW);
    vTaskDelay(2000 / portTICK_PERIOD_MS); // delay for 2000 ms
    esp_task_wdt_reset();   // keep watchdog happy

    // switch on Power (via GPIO 32)
    pinMode(POWER_ONEWIRE_BUS, OUTPUT);
    digitalWrite(POWER_ONEWIRE_BUS, HIGH);
    vTaskDelay(300 / portTICK_PERIOD_MS); // delay for 1000 ms
    // Start OneWire for DS18B20
    sensors.begin();
    vTaskDelay(700 / portTICK_PERIOD_MS); // delay for 1000 ms

    adresseAusgeben();       // find addresses of DS18B20 devices
    esp_task_wdt_reset();    // keep watchdog happy

    // Create GetTemperature task for core 0, loop() runs on core 1
    xTaskCreatePinnedToCore(
      GetOneDS18B20Temperature, /* Function to implement the task */
      "Task1", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      0,  /* Priority of the task. higher number is higher priority, https://esp32.com/viewtopic.php?t=10629 */
      &Task1,  /* Task handle. */
      1); /* Core where the task should run */
    stopDS18B20MeasureFlag = false; // reset flag to start measurements
  }

  // manual restart via V60
  #ifdef isBLYNK
    BLYNK_WRITE(V60) 
    {
      int x = param.asInt();
      sprintf(printstring,"Manual restart of DS18B20 initiated %d\n", x);
      logOut(printstring, msgDS18B20Restart, msgWarn);
      if(x==1)  
        manualDS18B20Restart = 1;
      else  
        manualDS18B20Restart = 0;
    }
  #endif // isBLYNK  

#endif  // DS18B20

#if defined isLCD || defined isDisplay
  /**************************************************!
  @brief    Timer Handler Function to set display modes for both OLED and LCD to switch display off
  @details  called via timer (displayOffTimerHandle), 
            If triggered, switch display off. Uses displayMode setting for this.
            does set for OLED: displayMode =0 for LCD: lcdDisplayMode = 0
  @param    none
  @return   void
  ***************************************************/
  void displayoff_handler(void)
  {
    #ifdef isDisplay  
      displayMode = 0;  // for OLED
      sprintf(printstring, "Switching off OLED display since no button pressed. %d\n", displayMode);
      logOut(printstring, msgLCDInfo, msgInfo);
    #endif
    #ifdef isLCD
       #ifdef allowLCDOff
         lcdDisplayMode = 0; // for LCD. If this line is enabled: display off after displayOffTimerInterval [ms]
       #endif // allowLCDOff
      sprintf(printstring, "Switching off LCD display since no button pressed. %d\n", lcdDisplayMode);
      logOut(printstring, msgLCDInfo, msgInfo);
    #endif

  }
#endif  

#ifdef isDisplay
  // interrupt handler for PushButton pressed. only needed for display
  /**************************************************!
  @brief    interrupt handler for PushButton pressed. needed for OLED display
  @details  interrupt handler for PushButton pressed. only needed for OLED display
  @param none
  @return   void
  ***************************************************/
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
    // restart display off timer
    MyBlynkTimer.restartTimer(displayOffTimerHandle);

    #ifdef isBeeperWindowOpenAlert
      beeperQuietCounter = beeperQuietCycles; // button is pressed: keep beeper quiet for this number of cacles
    #endif  
    
    // Serial.printf("************** Button Pressed %d *************************%d %d \n",  displayMode, displayDone);  
  }
#endif

#ifdef isLCD
   /**************************************************!
   @brief    interrupt handler for PushButton pressed. needed for LCD display
   @details  interrupt handler for PushButton pressed. only needed for LCD display
   @param none
   @return   void
  ***************************************************/
  void lcdChangeDisplayMode(void)
  {
    // Serial.printf("************** LCD Button Pressed before ********************** %d %d \n",  
    //   lcdDisplayMode, lcdDisplayDone);  
    if(lcdDisplayDone ==1 )
    {
      lcdDisplayMode ++;
      if (lcdDisplayMode > maxLcdisplayMode)    // tbd: variable adaptation to number of sensors
      #ifdef allowLCDOff
         lcdDisplayMode = 0;
      #else
        lcdDisplayMode = 1;
      #endif   
      lcdDisplayDone=0;    
      //lastButtonTime = millis();   
    }  
    // Serial.printf("************** LCD Button Pressed after ********************** %d %d \n", lcdDisplayMode, lcdDisplayDone);  
    
    // restart display off timer
    MyBlynkTimer.restartTimer(displayOffTimerHandle);
      
    #ifdef isBeeperWindowOpenAlert  
      beeperQuietCounter = beeperQuietCycles; // button is pressed: keep beeper quiet for this number of cacles
    #endif  
  }
#endif

#ifdef getNTPTIME
  /**************************************************!
  @brief    get local system date/time and convert it to proper string format
  @details  gets local time, and converts it to strings using "strftime"
  @details  http://www.cplusplus.com/reference/ctime/strftime/
  @param    printstring returned string with the date/time info
  @param    mode  deterines format to be returned. 1: only print
  @return   void
  ***************************************************/
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
      Serial.println(F("Failed to obtain time"));
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
        sprintf(printstring,"%d %s%s%s-%s:%s:%s ", timeinfo.tm_isdst,
          timeDay,timeMonth,timeYearShort,timeHour,timeMinute,timeSecond);    
        // Serial.print(printstring); 
        break;  
      case 7: 
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

  /**************************************************!
  @brief    getNTPTime: start WLAN and get time from NTP server
  @details  routine to get time from an network time server via NTP protocol. Set system time.
  @details  https://randomnerdtutorials.com/esp32-date-time-ntp-client-server-arduino/
  @param    none
  @return   void
  ***************************************************/
 // Issue: incorrect setting of day when switching to daylight saving time
 // See: https://github.com/espressif/arduino-esp32/issues/3797

  void getNTPTime()
  {
    //char printstring[80];
    // int tryCount = 0;

    Serial.print(F("Connecting to "));
    Serial.println(ssid);

    // new 22.11.21
    if(WiFi.status() != WL_CONNECTED)
      connectToWiFi(ssid, pass, 7);

    /* old until 22.11.21
    // while ((!wifiClient.connected()) || (WiFi.status() != WL_CONNECTED)) 
    while(WiFi.status() != WL_CONNECTED)
    {
      tryCount++;
      WiFi.disconnect();      // new 11.10.21: disconnect and connect in the loop
      WiFi.begin(ssid, pass);
    
      delay(500*tryCount);
      Serial.print(".");
    }
    */
    esp_task_wdt_reset();   // keep watchdog happy

    if (WiFi.status() == WL_CONNECTED)
    {
      Serial.println(F("WiFi connected."));
      // check Ping, to determine if internet connection and DNS server are running OK.
      // if DNS server not working: may be Pi-Hole which needs reset.
      bool success = Ping.ping("www.google.com", 3);
      if(!success)
        Serial.println(F("Ping Google.com failed"));
      else 
        Serial.println(F("Ping Google.com successful."));

      const IPAddress remote_ip1(8, 8, 8, 8); 
      success = Ping.ping(remote_ip1, 3);
      if(!success)
        Serial.println(F("Ping 8.8.8.8 failed"));
      else 
        Serial.println(F("Ping 8.8.8.8 successful."));
      /*  
      const IPAddress remote_ip2(23, 23, 23, 23); 
      success = Ping.ping(remote_ip2, 3);
      if(!success)
        Serial.println("Ping 23.23.23.23 failed");
      else 
        Serial.println("Ping 23.23.23.23 successful.");
      */ 

      esp_task_wdt_reset();   // keep watchdog happy  
      
      struct tm timeinfo;
      // Init and get the time. try all 3 time servers in sequence, if first not successful
      // configTime(gmtOffset_sec, daylightOffset_sec, ntpServer, ntpServer2, ntpServer3);
      // works for one server:
      // configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

      // improved for setting to the correct time zone directly
      // https://github.com/espressif/arduino-esp32/issues/3797 
      // https://remotemonitoringsystems.ca/time-zone-abbreviations.php
      configTzTime( defaultTimezone, ntpServer); //sets TZ and starts NTP sync

      if(getLocalTime(&timeinfo))
      {
        TimeIsInitialized = true;
        sprintf(printstring,"Successfully obtained time from first server %s",ntpServer);
        Serial.println(printstring);  
      }  
      else{  
        sprintf(printstring,"Failed to obtain time from first server %s",ntpServer);  
        Serial.println(printstring);   
        // configTime(gmtOffset_sec, daylightOffset_sec, ntpServer2);
        configTzTime( defaultTimezone, ntpServer2); //sets TZ and starts NTP sync
        if(getLocalTime(&timeinfo)){
          TimeIsInitialized = true;
          sprintf(printstring,"Successfully obtained time from 2nd server %s",ntpServer2);
          Serial.println(printstring);  
        }  
        else{
          sprintf(printstring,"Failed to obtain time from 2nd server %s",ntpServer2);  
          Serial.println(printstring);  
          // configTime(gmtOffset_sec, daylightOffset_sec, ntpServer3);
          configTzTime( defaultTimezone, ntpServer3); //sets TZ and starts NTP sync
          if(getLocalTime(&timeinfo)){
            sprintf(printstring,"Successfully obtained time from 3rd server %s",ntpServer3);
            Serial.println(printstring);  
            TimeIsInitialized = true;
          } 
          else{
            sprintf(printstring,"Failed to obtain time from 3rd server %s",ntpServer3);  
            Serial.println(printstring);  
            TimeIsInitialized = false;  
          }
        }
      }

      if(TimeIsInitialized){
        printLocalTime(printstring, 3);
        logOut(printstring, msgTimeInfo, msgInfo);
      }  
      //disconnect WiFi as it's no longer needed
      //in EnvMonitor we do not disconnect
      // WiFi.disconnect(true);
      // WiFi.mode(WIFI_OFF);
    }
    else
    {
      Serial.println(F("WiFi NOT connected - time server connection not possible."));
      TimeIsInitialized = false;
    }  
  }
#endif

#ifdef serialMonitor
  #define serBufSize 1024

  /**************************************************!
  @brief    monitorSerial
  @details  routine to monitor ALL characters on the serial input and display it, later to write to SD
  @param none
  @return   void
  ***************************************************/
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

/**************************************************!
 @brief    outputProgramInfo
 @details  helper function to output all infos about the program, at startup
 @param none
 @return   void
***************************************************/
void outputProgramInfo()
{
  sprintf(printstring," with ");
  sprintf(printstring2,":");
  #ifdef isOneDS18B20  
    strcat(printstring, " DS18B20 (T) -");
    // strcat(printstring2, "DS18B20-");
  #endif
  #if defined  isBME680  || defined isBME680_BSECLib
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
 
  strcpy(printstring2,"\n******************************************************************************\n");
  logOut(printstring2, msgProgInfo, msgInfo);
  strcpy(printstring2," This is EnvMonitorBME680.cpp \n"); 
  logOut(printstring2, msgProgInfo, msgInfo);
  sprintf(printstring2," %s\n", infoStringShort);
  logOut(printstring2, msgProgInfo, msgInfo);
  sprintf(printstring2,"%s\n", infoStringLong);
  logOut(printstring2, msgProgInfo, msgInfo);
  logOut(printstring, msgProgInfo, msgInfo);
  #ifdef isBLYNK
    // sprintf(printstring," Blynk Auth Code: %s \n", auth); 
    sprintf(printstring," Blynk Auth Code: xxxxxxxxxxx \n"); 
    logOut(printstring, msgProgInfo, msgInfo);
    sprintf(printstring," Blynk Server IP: %s \n", "blynkLocalIP"); 
    logOut(printstring, msgProgInfo, msgInfo);
  #endif
  #ifdef isThingspeak 
    // sprintf(printstring," Thingspeak enabled. WriteApiKey: %s \n", thingspeakWriteAPIKey); 
    sprintf(printstring," Thingspeak enabled. WriteApiKey: ................. \n"); 
    logOut(printstring, msgProgInfo, msgInfo);
  #endif
  sprintf(printstring," %s  %s  %s \n",PROGNAME, PROGVERSION, PROGDATE);
  logOut(printstring, msgProgInfo, msgInfo);
  strcpy(printstring2,"******************************************************************************\n");
  logOut(printstring2, msgProgInfo, msgInfo);
  delay(200);
}

#ifdef isBLYNK
 /**************************************************!
   @brief    extended BLYNK setup function, attempts restarts in increasing intervals
   @details  Sometimes local Blynk server does not connect after a restart, needs longer pause. This routine attempts to overcome that
   @param none
   @return   void
  ***************************************************/
  void extendedBlynkConnect()
  {
    int i, delayTime = 300;  // delay time in seconds

    // Start the display
    /* done in setup
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // Initialize with the I2C addr 0x3C (for the 128x64 from Conrad else 3D)
    display.setTextColor(WHITE);
    */

    sprintf(printstring,"extended Blynk setup function entered\n");
    logOut(printstring, msgBlynkInfo, msgInfo);
    #ifdef isDisplay
      display.clearDisplay(); 
      display.setTextSize(1);
      sprintf(printstring,"Extended Blynk Setup");
      display.setCursor(0, 0);
      display.println(printstring);
      display.display();          // transfer buffer content to display
    #endif  

    do{
      // Blynk initialization for Blynk web account - MP internet
      // Original example, never use in reality: You can also specify server:
      // Blynk.begin(auth, ssid, pass, "blynk-cloud.com", 80);
      // Blynk.begin(auth, ssid, pass, IPAddress(192,168,1,100), 8080);
      #ifdef blynkCloud
        // Blynk.begin(auth, ssid, pass);
        if(WiFi.status() != WL_CONNECTED)
          Blynk.connectWiFi(ssid,pass);
        
        Blynk.config(auth);   // Blynk.config(auth, "blynk-cloud.com", 80);
        Blynk.connect();
      #else
        // Blynk initialization for local blynk server on raspi - MP Home
        // original approach, does not always work
        // Blynk.begin(auth, ssid, pass, IPAddress(blynkLocalIP), 8080);
        // alternate approach to start Blynk
        if(WiFi.status() != WL_CONNECTED)
          Blynk.connectWiFi(ssid,pass);
        Blynk.config(auth, IPAddress(blynkLocalIP), 8080);
        Blynk.connect();
      #endif  

      #ifdef blynkTerminal
        myBlynkTerminal.clear();
      #endif  
      if(Blynk.connected()){
        sprintf(printstring,"Blynk connected\n");
        logOut(printstring, msgBlynkInfo, msgInfo);
        #ifdef isDisplay        
          display.setCursor(0, 12);
          sprintf(printstring,"Blynk connected");
          display.println(printstring);
        #endif  
        Blynk.syncAll();  // synchronize device with server
      }
      else{
        sprintf(printstring,"Blynk NOT connected. Retry in %d [s]\n", delayTime);
        logOut(printstring, msgBlynkInfo, msgWarn);
        #ifdef isDisplay
          sprintf(printstring,"Blynk NOT connected");
          display.setCursor(0, 24);
          display.println(printstring);
          sprintf(printstring,"Retry in %d [s]", delayTime);
          display.setCursor(0, 36);
          display.println(printstring);
          display.display();          // transfer buffer content to display
        #endif  
        // esp_task_wdt_init(delayTime/1000 + WDT_TIMEOUT_SECONDS,true); // change watchdog to ensure that it does not kill us.
        for(i=0; i<delayTime; i++){
          vTaskDelay(1000 / portTICK_PERIOD_MS);  // delay for 1000 ms
          esp_task_wdt_reset(); // reset watchdog 
          #ifdef isDisplay
            display.clearDisplay();
            sprintf(printstring,"Retry in %d / %d s", i, delayTime);
            display.setCursor(0, 36);
            display.println(printstring);
            display.display();
          #endif  
        }
        delayTime += 100;  // increment delay time for next round, if necessary
      }
    } while(!Blynk.connected());  // exit if Blynk is connected
    esp_task_wdt_init(WDT_TIMEOUT_SECONDS,true); // reset the watchdog time to defualt
  }
#endif

#ifdef isVirtuino
  //================= Virtuino Code ==============================

//============================================================== connectToWiFiNetwork
void connectToWiFiNetwork(){
  Serial.println("Connecting to "+String(ssid));
   // If you don't want to config IP manually disable the next two lines
   IPAddress subnet(255, 255, 255, 0);        // set subnet mask to match your network
  //Wifi.hostname("VirtuinoHostName");        // so kann man direkt den Hostnamen setzen
  WiFi.config(ip, gateway, subnet);          // If you don't want to config IP manually disable this line
  WiFi.mode(WIFI_STA);                       // Config module as station only.

  // new 22.11.21
  connectToWiFi(ssid, pass, 7);

  /* old until 22.11.21
  WiFi.begin(ssid, pass);
   while (WiFi.status() != WL_CONNECTED) {
     delay(500);
     Serial.print(".");
    }
   Serial.println("");
   Serial.println(F("WiFi connected"));
   Serial.println(WiFi.localIP());
  */ 
}


//============================================================== onCommandReceived
//==============================================================
/* This function is called every time Virtuino app sends a request to server to change a Pin value
 * The 'variableType' can be a character like V, T, O  V=Virtual pin  T=Text Pin    O=PWM Pin 
 * The 'variableIndex' is the pin number index of Virtuino app
 * The 'valueAsText' is the value that has sent from the app   */
boolean debug = true;

void onReceived(char variableType, uint8_t variableIndex, String valueAsText){     
  //if(debug) Serial.print(valueAsText);
  if (variableType=='V'){
    float value = valueAsText.toFloat();        // convert the value to float. The valueAsText have to be numerical
    if (variableIndex<V_memory_count) V[variableIndex]=value;              // copy the received value to arduino V memory array
  }
  else  if (variableType=='T'){
    if (variableIndex==0) Text_0=valueAsText;          // Store the text to the text variable Text_0
    else if (variableIndex==1) Text_1=valueAsText;     // Store the text to the text variable Text_1
    else if (variableIndex==2) Text_2=valueAsText;     // Store the text to the text variable T2
    else if (variableIndex==3) Text_3=valueAsText;     // Store the text to the text variable T3
  }  
}

//==============================================================
/* This function is called every time Virtuino app requests to read a pin value*/
String onRequested(char variableType, uint8_t variableIndex){    
  //if(debug) {Serial.print(variableType); Serial.print(" "); Serial.println(variableIndex); }
  if (variableType=='V') {
    if (variableIndex<V_memory_count) return  String(V[variableIndex]);   // return the value of the arduino V memory array
  }
  else if (variableType=='T') {
   if (variableIndex==0) return Text_0;  
   else if (variableIndex==1) return Text_1;   
   else if (variableIndex==2) return Text_2;   
   else if (variableIndex==3) return Text_3;   
  }
  return "";
}

//==============================================================
  void virtuinoRun(){
   WiFiClient client = server.available();
   if (!client) return;
   if (debug) Serial.println("Connected");
   unsigned long timeout = millis() + 3000;
   while (!client.available() && millis() < timeout) delay(1);
   if (millis() > timeout) {
    Serial.println("timeout");
    client.flush();
    client.stop();
    return;
  }
    virtuino.readBuffer="";    // clear Virtuino input buffer. The inputBuffer stores the incoming characters
      while (client.available()>0) {        
        char c = client.read();         // read the incoming data
        virtuino.readBuffer+=c;         // add the incoming character to Virtuino input buffer
        if (debug) Serial.write(c);
      }
     client.flush();
     if (debug) Serial.println("\nReceived data: "+virtuino.readBuffer);
     String* response= virtuino.getResponse();    // get the text that has to be sent to Virtuino as reply. The library will check the inptuBuffer and it will create the response text
     if (debug) Serial.println("Response : "+*response);
     client.print(*response);
     client.flush();
     delay(10);
     client.stop(); 
    if (debug) Serial.println("Disconnected");
}

//============================================================== vDelay
void vDelay(int delayInMillis){long t=millis()+delayInMillis;while (millis()<t) virtuinoRun();}

#endif

#ifdef isBluetoothCredentials 

String NetworkID[50];
/**************************************************!
   @brief    function scan all networks available within Wifi
   @details  
   @param none
   @return   void
  ***************************************************/
  int scanWifiNetworks()
  {
    Serial.println("scan start");

    // WiFi.scanNetworks will return the number of networks found
    int n = WiFi.scanNetworks();
    Serial.println("scan done");
    if(n!=WIFI_SCAN_FAILED){
      if (n == 0) {
        Serial.println("no networks found");
      } else {
        Serial.print(n);
        Serial.println(" networks found");
        for (int i = 0; i < n; ++i) {
          NetworkID[i] = (String)WiFi.SSID(i);
          // Print SSID and RSSI for each network found
          Serial.print(i + 1);
          Serial.print(": ");
          Serial.print((String)WiFi.SSID(i));
          Serial.print(" (");
          Serial.print((String)WiFi.RSSI(i));
          Serial.print(") ");
          Serial.print(NetworkID[i]);
          Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN)?" ":"*");
          delay(10);
        }
      } 
    }
    WiFi.mode ( WIFI_STA );
    // WiFi.begin();
    // delay(500);
    // WiFi.disconnect();
    delay(1000);
    WiFi.disconnect( true );
    delay(5000);
    return(n);  
  }

 /**************************************************!
   @brief    function to get wifi credentials via bluetooth at startup
   @details  Bluetooth connection is opened by caller. Then listens for ssid:[ID] and pass:[pw], and returns these
             can also use a scan, but since that makes wifi unreliable on ESP32: better not.
   @param char* ssid: wifi network ssid to be returned
   @param char* pass: wifi password to be returned
   @param int* noOfNetwork: alternative to ssid, the index of the list scanned is returned
   @param int noOfNetworks: Input parameter, the number of networks that are detectred  
   @return bool : true = successfully got wifi parameters.
  ***************************************************/
bool checkBluetoothCredentials(char* ssid, char* pass, int* noOfNetwork, int noNetworks)
{
  long startMillis, intervalMillis=60000;
  String buffer_in, Ssid, Pass; // https://www.arduino.cc/reference/en/language/variables/data-types/stringobject/ 
  boolean ssidFlag=false, passFlag=false, noFlag=false;
  int a,b;

  startMillis = millis();
  while(millis() < startMillis + intervalMillis)
  {
    if (ESP_BT.available()) // Check if we receive anything from Bluetooth
    {
      buffer_in = ESP_BT.readStringUntil('\n'); //Read what we recevive 
      Serial.print("Received: "); Serial.println(buffer_in);
	    delay(20);
      if(buffer_in.indexOf("ssid") != -1)
      {
        a=buffer_in.indexOf("[");
        b=buffer_in.indexOf("]");
        if(a>-1 && b>-1 && (b-a)>1) // start and end character found, and something between them
        {
          Ssid = buffer_in.substring(a+1, b);
          strcpy(ssid, Ssid.c_str());
          sprintf(printstring,"ssid is: _%s_\n",ssid);
          Serial.print(printstring);
          ssidFlag=true;
        }  
        startMillis = millis(); // reset timer
      }
      else if(buffer_in.indexOf("pass") != -1)
      {
        a=buffer_in.indexOf("[");
        b=buffer_in.indexOf("]");
        if(a>-1 && b>-1 && (b-a)>1) // start and end character found, and something between them
        {
          Pass = buffer_in.substring(a+1, b);
          sprintf(printstring,"pass is: _%s_\n",Pass.c_str());
          strcpy(pass,Pass.c_str());
          Serial.print(printstring);
          passFlag=true;
        }
        startMillis = millis(); // reset timer
      }  
      else if(buffer_in.toInt() > 0)
      {
        a = buffer_in.toInt();

        if(a>0 && a<noNetworks) 
        {
          *noOfNetwork = a;
          sprintf(printstring,"Network number is: _%d_\n",*noOfNetwork);
          Serial.print(printstring);
          Serial.println(WiFi.SSID(*noOfNetwork - 1));
          noFlag=true;
        }
        startMillis = millis(); // reset timer
      }  
    }
    delay(100);
    esp_task_wdt_reset();   // keep watchdog happy  

    if ((ssidFlag && passFlag) || (passFlag && noFlag)) // leave loop if both are available
      break;
  }
  return ((ssidFlag && passFlag) || (passFlag && noFlag));
} 
#endif

  /**************************************************!
  @brief    Connect or reconnect to WiFi
  @details  gets ssid and password for the network to connect to.
            function attempts 6 times to connect to the network
            If not successfull after the sixth time, returns false.
  @param    char* ssid  - SSID of the network to connect to
  @param    char* pass  - password of the network to connect to
  @param    int noRetries - number of retries in connecting. Default is 7.
  @return   bool : true if connected, false if not.
  ***************************************************/
// Connect or reconnect to WiFi
bool connectToWiFi(char* ssid, char* pass, int noRetries)
{
  int i=0;
  long starttime, endtime;
  WiFi.disconnect();   
  delay(500);  
  // if((!wifiClient.connected()) || (WiFi.status() != WL_CONNECTED))
  //{
    sprintf(printstring,"Attempting to connect to SSID: _%s_ PASS: _%s_\n", ssid, pass);
    Serial.println(printstring);

    #ifdef isDisplay
      display.clearDisplay(); 
      display.setTextSize(1);
      sprintf(printstring,"Connecting to SSID: ");
      display.setCursor(0, 0);
      display.println(printstring);
      sprintf(printstring,"%s ", ssid);
      display.setCursor(0, 12);
      display.println(printstring);
      display.display();          // transfer buffer content to display
    #endif 

    // 0 = WL_IDLE_STATUS
    // 1 = WL_NO_SSID_AVAIL 
    // 2 = WL_SCAN_COMPLETED
    // 3 = WL_CONNECTED 
    // 4 = WL_CONNECT_FAILED 
    // 5 = WL_CONNECTION_LOST
    // 6 = WL_DISCONNECTED 

    // see following link for discussion of connection issues
    // https://github.com/espressif/arduino-esp32/issues/2501
    // der entscheidende Punkt, war nach dem WiFi.begin() lange genug zu warten
    // da hilft die wartende Funktion WiFi.waitForConnectResult();

    starttime = millis();
    // WiFi.mode(WIFI_STA);             // Config module as station only.
    int status = WiFi.status();
    while((status != WL_CONNECTED) && (i<=noRetries))
    {
      i++;
      if(status == WL_CONNECT_FAILED)
      {
        WiFi.disconnect(true);      // new 11.10.21: disconnect and connect in the loop. New 19.11.: only if connect failed
        delay(500);
      }  
      esp_task_wdt_reset();   // keep watchdog happy

      WiFi.begin(ssid, pass);
      delay(i*500);  
      // status = WiFi.status();      // this call returns the result right away
      status = WiFi.waitForConnectResult(); // this call may wait a long time

      sprintf(printstring,".%d ",status);
      Serial.print(printstring);
       #ifdef isDisplay
        display.setCursor(15*(i-1), 24);
        display.println(printstring);
        display.display();          // transfer buffer content to display
      #endif  
    } 
    esp_task_wdt_reset();   // keep watchdog happy
  //}
  endtime = millis();
  if(WiFi.status() == WL_CONNECTED)
  {
    sprintf(printstring,"Connected after %5.2f sec IP: %s\n",(float)(endtime-starttime)/1000, toStringIp(WiFi.localIP()).c_str());
    Serial.println(printstring);
    logOut(printstring, msgWifiConnected, msgInfo);
    #ifdef isDisplay
      sprintf(printstring,"Connected in %5.2f s ",(float)(endtime-starttime)/1000);
      display.setCursor(0, 36);
      display.println(printstring);
      sprintf(printstring,"IP: %s ",toStringIp(WiFi.localIP()).c_str());
      display.setCursor(0, 48);
      display.println(printstring);
      display.display();          // transfer buffer content to display
    #endif  
    return(true);
  }  
  else
  {
    sprintf(printstring,"NOT Connected after %5.2f sec",(float)(endtime-starttime)/1000);
    Serial.println(printstring);
    logOut(printstring, msgWifiConnected, msgInfo);
    #ifdef isDisplay
      display.setCursor(0, 48);
      display.println(printstring);
      display.display();          // transfer buffer content to display
    #endif  
    return(false);
  }  
}  

 /**************************************************!
   @brief    setup function
   @details  main setup for all functions of the software
   @param none
   @return   void
  ***************************************************/
void setup() 
{
  // Init USB serial port
  Serial.begin(115200);
  delay(10);

  // Set Watchdog
  Serial.print("1");
  pinMode(0, INPUT_PULLUP); // Digital-Pin 0 as input
  esp_task_wdt_init(WDT_TIMEOUT_SECONDS,true); //Init Watchdog with 40 seconds timeout and panic (hardware rest if watchdog acts)
  esp_task_wdt_add(NULL); //No special task needed

  // Set digital Output 14 as output for LED
  #ifdef isLEDHeartbeat
    pinMode(HEARTBEATPIN, OUTPUT); // Digital-Pin 0 as input
  #endif   

  startTime = millis(); // remember the start time
  Serial.print("2");

  #ifdef isDisplay
    // Start the display
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // Initialize with the I2C addr 0x3C (for the 128x64 from Conrad else 3D)
    display.setTextColor(WHITE);
    display.clearDisplay(); 
    display.setTextSize(1);
  #endif  

  #ifdef isBluetoothCredentials
    // get ssid and pass via bluetooth
    int noNetworks=0;
    char ss[50], pw[50]; 
    int noOfNetwork=0;
    String Ss, Pw;

    // this often prevents a successful connection later!!
    // noNetworks = scanWifiNetworks();

    Serial.println("\nBluetooth Device is ready to pair");
    Serial.println("Waiting For Wifi Credential Updates 60 seconds");
    Serial.println("Enter: 'ssid:[myssid]' AND 'pass:[password]' ");

    ESP_BT.begin("ESP32_BLUETOOTH"); //Name of your Bluetooth Signal
    ESP_BT.setPin("1234");           // Set the bluetooth PIN of the device
    if(checkBluetoothCredentials(ss,  pw, &noOfNetwork, noNetworks) )
    {
      sprintf(printstring,"Raw Credentials: _%s_ _%s_ %d\n",
          ss, pw, noOfNetwork);
      Serial.print(printstring);
      if(noOfNetwork < 1)
        strcpy(ssid,ss);
      else
        strcpy(ssid,WiFi.SSID(noOfNetwork-1).c_str());
      strcpy(pass,pw);
      sprintf(printstring,"checkBluetoothCredentials returned TRUE. Credentials: _%s_ _%s_ %d\n",
          ssid, pass, noOfNetwork);
      Serial.print(printstring);
      credentialstorage.begin("credentials", false);    
      int ssid_len = credentialstorage.putBytes("ssid", ssid, strlen(ssid)+1); 
      int pass_len = credentialstorage.putBytes("pass", pass, strlen(pass)+1); 
      credentialstorage.end();
      sprintf(printstring,"Wrote to EEPROM: ssid: %d _%s_ | %d pass: _%s_ \n", 
          ssid_len, ssid, pass_len, pass);
      Serial.print(printstring);
      delay(500);
    }
    else
    {
      // no credentials received via bluetooth. 
      sprintf(printstring,"checkBluetoothCredentials returned FALSE. Credentials: _%s_ _%s_\n",ssid, pass);
      Serial.print(printstring);
      // try to get from credential storage (EEPROM)
      credentialstorage.begin("credentials", false);    
      // ssid = credentialstorage.getString("ssid", ""); 
      // pass = credentialstorage.getString("pass", ""); 
      int ssid_len = credentialstorage.getBytes("ssid", ssid, 50); 
      int pass_len = credentialstorage.getBytes("pass", pass, 50); 
      credentialstorage.end();
      sprintf(printstring,"Read from EEPROM: ssid: %d _%s_ | %d pass: _%s_ \n", ssid_len, ssid, pass_len, pass);
      Serial.print(printstring);
      delay(500);

      if (strlen(ssid) <2 || strlen(pass) < 2){
        Serial.println("No values saved in EEPROM for ssid or password, using defaults");
        strcpy(ssid,default_ssid);
        strcpy(pass,default_pass);
      }
    }
    ESP_BT.end();
  #else // ensure that there are wifi credentials if not taken from bluetooth
    strcpy(ssid,default_ssid);
    strcpy(pass,default_pass);  
  #endif // isBluetoothCredentials

  #ifdef isCaptivePortal // captive portal is credentials are not present
    #ifdef isDisplay
      display.clearDisplay(); 
      display.setTextSize(1);
      sprintf(printstring,"Credentials from Captive Portal");
      display.setCursor(0, 0);
      display.println(printstring);
      display.display();          // transfer buffer content to display
    #endif 
    // try to get from credential storage (EEPROM)
    credentialstorage2.begin("credentials", false);    
    int ssid_len = 0; 
    int pass_len = 0; 
    ssid_len = credentialstorage2.getBytes("ssid", ssid, 50); 
    pass_len = credentialstorage2.getBytes("pass", pass, 50); 
    credentialstorage2.end();
    sprintf(printstring,"Read from Preferences EEPROM: ssid: %d _%s_ | %d pass: _%s_ \n", ssid_len, ssid, pass_len, pass);
    Serial.print(printstring);
    
    // Check if connection with these data is possible
    #ifdef debugCaptivePortal
      bool connectPossible = false; //debug: always go beyond EEPROM. REMOVE for productive software!!!
    #else  
      bool connectPossible = connectToWiFi(ssid,  pass, 7);
    #endif  
    if(connectPossible)
    {
      sprintf(printstring,"Connection successful (Preferences) for ssid _%s_ pass _%s_ => Proceeding\n", ssid, pass);
      logOut(printstring, msgWifiConnected, msgInfo);
    }
    else
    {
      sprintf(printstring,"Connection NOT successful (Preferences) for ssid _%s_ pass _%s_ => Captive Portal\n", ssid, pass);
      logOut(printstring, msgWifiNotConnected, msgErr);
      
      setupCaptivePortal(); // start the captive portal
      loopCaptivePortal(ssid, pass); // and it's main event loop
      connectPossible = connectToWiFi(ssid, pass, 7);
      if(connectPossible)
      { 
        sprintf(printstring,"Connection successful (Data from Captive Portal) for ssid _%s_ pass _%s_ => Proceeding\n", ssid, pass);
        logOut(printstring, msgWifiConnected, msgInfo);
        // write credentials to EEPROM
        credentialstorage2.begin("credentials", false);    
        ssid_len = credentialstorage2.putBytes("ssid", ssid, strlen(ssid)+1); 
        pass_len = credentialstorage2.putBytes("pass", pass, strlen(pass)+1); 
        credentialstorage2.end();     
        sprintf(printstring,"Credentials written to Preferences: _%s_ (%d) _%s_ (%d)\n", 
            ssid, ssid_len, pass, pass_len);
        Serial.print(printstring);
      }  
      else
      {
        sprintf(printstring,"Connection NOT successful (Data from Captive Portal) for ssid _%s_ pass _%s_ => Proceeding\n", ssid, pass);
        logOut(printstring, msgWifiNotConnected, msgErr);
      }
    }  
    if(!connectPossible)  // did not work, use data from default
    {
      strcpy(ssid,default_ssid);
      strcpy(pass,default_pass);  
      connectPossible = connectToWiFi(ssid, pass, 7);
      if(connectPossible)
      {
        sprintf(printstring,"Connection successful (using Defaults) for ssid _%s_ pass _%s_ => Proceeding\n", ssid, pass);
        logOut(printstring, msgWifiConnected, msgInfo);
      }
      else
      {
        sprintf(printstring,"Connection NOT successful (using Defaults) for ssid _%s_ pass _%s_ => Proceeding\n", ssid, pass);
        logOut(printstring, msgWifiNotConnected, msgErr);
      }
    }
  #endif // isCaptivePortal

  #ifdef getNTPTIME
    // get time from NTP server
    // https://randomnerdtutorials.com/esp32-date-time-ntp-client-server-arduino/
    getNTPTime();
    esp_task_wdt_reset();   // keep watchdog happy

    NTPTimerHandle = NTPTimer.setInterval(NTPTimerInterval, getNTPTime);
    Serial.print("6b ");
    sprintf(printstring, "NTPTimerHandle: %d\n", NTPTimerHandle);
    logOut(printstring, msgTimeInfo, msgInfo);
    Serial.print(NTPTimerHandle);
  #endif // getNPTTIME
  Serial.print("3");

  #ifdef isSyslog
    // prepare syslog configuration here (can be anywhere before first call of 
    // log/logf method)
    syslog.server(SYSLOG_SERVER, SYSLOG_PORT);
    syslog.deviceHostname(DEVICE_HOSTNAME);
    syslog.appName(APP_NAME);
    syslog.defaultPriority(LOG_KERN);
  #endif // isSyslog 

  // get number of reboots so far, is used to set SD filename
  preferences.begin("nvs", false);                 // Open nonvolatile storage (nvs)
  NoReboots = preferences.getInt("NoReboots", 0);  // Read stored last NoReboots, default 0
  if(NoReboots > 9998)
    NoReboots = 0;
  preferences.putInt("NoReboots", NoReboots+1);    // and increment it - new start.
  preferences.end();

  
  Serial.print("4");
  #ifdef isSD
    // Create a file on the SD card and write the data labels
    sprintf(logfilename,"/log%04d.txt",NoReboots);
    initSDCard(logfilename);
  #endif  // isSD

  #ifdef isVirtuino
    virtuino.begin(onReceived,onRequested,512);  //Start Virtuino. Set the buffer to 512. With this buffer Virtuino can control about 50 pins (1 command >= 9bytes) The T(text) commands with 20 characters need 20+6 bytes
    //virtuino.key="1234";                       //This is the Virtuino password. Only requests the start with this key are accepted from the library
    connectToWiFiNetwork();
    server.begin();
  #endif // isVirtuino

  #ifdef isThingspeak
    WiFi.mode(WIFI_STA);  

    thingspeakHandlerTimerHandle = thingspeakHandlerTimer.setInterval(thingspeakHandlerInterval, thingspeak_handler);
    // Blynk
    // mainHandlerTimerHandle = MyBlynkTimer.setInterval(mainHandlerInterval, main_handler);
    Serial.print("7a ");
    sprintf(printstring, "thingspeakHandlerTimerHandle: %d\n", thingspeakHandlerTimerHandle);
    logOut(printstring, msgThingspeakInfo, msgInfo);
    Serial.print(thingspeakHandlerTimerHandle);
  #endif // isThingspeak

 // https://randomnerdtutorials.com/esp32-mqtt-publish-subscribe-arduino-ide/
  #ifdef isMQTT // library dafür: pubsubclient
     // check wifi status and connect if not yet done
    if(WiFi.status() != WL_CONNECTED)
      connectToWiFi(ssid, pass, 7);

    // create the timer for mqtt handling
    mqttHandlerTimerHandle = mqttHandlerTimer.setInterval(mqttHandlerInterval, mqttHandler);
    
    // set MQTT server and MQTT callback function
    strcpy(mqttActualServer, mqttDefaultServer);
    mqttClient.setServer(mqttActualServer, 1883);
    //mqttClient.setServer("192.168.178.64", 1883);
    mqttClient.setCallback(mqttCallbackFunction);
    mqttClient.setKeepAlive(120); // keep the client alive for 120 sec if no action occurs
    mqttClient.setBufferSize(1024); // increase buffer size from standard 1024
  #endif //isMQTT

  #ifdef isBLYNK
    sprintf(printstring,"Blynk setup section entered\n");
    logOut(printstring, msgBlynkInfo, msgInfo);
    #ifdef isDisplay
      display.clearDisplay(); 
      display.setTextSize(1);
      sprintf(printstring,"Standard Blynk Setup");
      display.setCursor(0, 0);
      display.println(printstring);
      display.display();          // transfer buffer content to display
    #endif 

    #ifdef isDisplay
      display.setCursor(0, 0);
      display.println(printstring);
    #endif  

    // Blynk initialization for Blynk web account - MP internet
    // Original example, never use in reality: You can also specify server:
    // Blynk.begin(auth, ssid, pass, "blynk-cloud.com", 80);
    // Blynk.begin(auth, ssid, pass, IPAddress(192,168,1,100), 8080);
    #ifdef blynkCloud
      // Bei Fritzbox beachteh: Unbegrenztes Zugangsprofil, sonst blockt sie!
      // Blynk.begin(auth, ssid, pass);
      if(WiFi.status() != WL_CONNECTED)
        Blynk.connectWiFi(ssid,pass);
      
      Blynk.config(auth);   // Blynk.config(auth, "blynk-cloud.com", 80);
      Blynk.connect();     
    #else
      // Blynk initialization for local blynk server on raspi - MP Home
      // original approach, does not always work
      // Blynk.begin(auth, ssid, pass, IPAddress(blynkLocalIP), 8080);
      // alternate approach to start Blynk
      if(WiFi.status() != WL_CONNECTED)
        Blynk.connectWiFi(ssid,pass);
      Blynk.config(auth, IPAddress(blynkLocalIP), 8080);
      Blynk.connect();
    #endif  

    #ifdef blynkTerminal
      myBlynkTerminal.clear();
    #endif  
    if(Blynk.connected()){
      sprintf(printstring,"Blynk connected\n");
      logOut(printstring, msgBlynkConnected, msgInfo);
      Blynk.syncAll();  // synchronize device with server
    }
    else{
      sprintf(printstring,"Blynk NOT connected\n");
      logOut(printstring, msgBlynkNotConnected, msgInfo);
      extendedBlynkConnect(); // try reconnects with increasing time intervals
    }
  #endif // isBLYNK  
  Serial.print("5");
  // log number of reboots. Do this here, after SD opened and Blynk connection operational (no reboots make up logfilename...)
  sprintf(printstring,"NoReboots=%d\n", NoReboots);
  logOut(printstring, msgDefaultID, msgInfo);

  Serial.print("6");
  // set timer for main_handler()

  #ifdef isBLYNK
    mainHandlerTimerHandle = MyBlynkTimer.setInterval(mainHandlerInterval, main_handler);
  #else  
    // Virtuino, Thingspeak
    mainHandlerTimer.setInterval(mainHandlerInterval, main_handler);
  #endif  


  Serial.print("7 ");
  sprintf(printstring, "mainHandlerTimerHandle: %d\n", mainHandlerTimerHandle);
  logOut(printstring, msgDefaultID, msgInfo);
  Serial.print(mainHandlerTimerHandle);

  #ifdef isDisplay
    // Start the display
    /* done earlier in setup
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // Initialize with the I2C addr 0x3C (for the 128x64 from Conrad else 3D)
    display.setTextColor(WHITE);
    display.clearDisplay(); 
    display.setTextSize(1);
    */

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
    if(Blynk.connected()){
      sprintf(printstring,"Blynk connected");
      Display(printstring, 1,0,48,false); // string, size, x,y,clear
    }else
    {
      sprintf(printstring,"Blynk NOT connected");
      Display(printstring, 1,0,48,false); // string, size, x,y,clear
    }

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
    #if defined isBME280 || defined isBME680 || defined isBME680_BSECLib
      maxDisplayMode =3+1;
    #endif
    #if defined isOneDS18B20
      maxDisplayMode =4+1;
    #endif
    #if defined isMHZ14A || defined isSENSEAIR_S8 || defined receiveSERIAL
      maxDisplayMode =5+1;
    #endif
    // set timer for oled_handler()
    //oledHandlerTimer.setInterval(lcdHandlerInterval, oled_handler);
    oledTimerHandle = MyBlynkTimer.setInterval(oledHandlerInterval, oled_handler);
    sprintf(printstring, "oledTimerHandle: %d\n", oledTimerHandle);
    logOut(printstring, msgOLEDInfo, msgInfo);
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
    lcdTimerHandle = MyBlynkTimer.setInterval(lcdHandlerInterval, lcd_handler);
    sprintf(printstring, "lcdTimerHandle: %d\n", lcdTimerHandle);
    logOut(printstring, msgLCDInfo, msgInfo);
  #endif // isLCD

  #if defined isLCD || defined isDisplay
    // this timer is triggered when display is to go off. restarted every time a button is pressed
    displayOffTimerHandle = MyBlynkTimer.setInterval(displayOffTimerInterval, displayoff_handler);
    sprintf(printstring, "displayOffTimerHandle: %d\n", displayOffTimerHandle);
    logOut(printstring, msgOLEDInfo, msgInfo);
  #endif

  // Program Info to serial Monitor
  outputProgramInfo(); 

  #if defined isRelay || defined isBeeperWindowOpenAlert
    pinMode(RELAYPIN1,OUTPUT);
    pinMode(RELAYPIN2,OUTPUT);
    delay(10);

    #ifdef isStartupBeepTest
      // digitalWrite(RELAYPIN1, HIGH);
      windowSetBeeper(1);
      digitalWrite(RELAYPIN2, HIGH);
      delay(500);
      windowSetBeeper(0);
    #endif
    // digitalWrite(RELAYPIN1, LOW);
    digitalWrite(RELAYPIN2, LOW);

  #endif

  #if defined isRelay && defined isFan
    // set timer for bme680FanHandler()
    // fanTimerHandle = MyBlynkTimer.setInterval(bme680FanHandlerInterval, bme680FanHandler);
    fanTimerHandle = fanHandlerTimer.setInterval(bme680FanHandlerInterval, bme680FanHandler);
    sprintf(printstring, "fanTimerHandle: %d\n", fanTimerHandle);
    logOut(printstring, msgRelayInfo, msgInfo);
  #endif

  #if defined isWindowOpenDetector && defined isBLYNK
    // set timer for windowOpenHandler()
    // windowOpenTimerHandle = MyBlynkTimer.setInterval(windowOpenHandlerInterval, windowOpenHandler);
    
    windowOpenTimerHandle = windowOpenHandlerTimer.setInterval(windowOpenHandlerInterval, windowOpenHandler);
    sprintf(printstring, "windowOpenTimerHandle: %d\n", windowOpenTimerHandle);
    logOut(printstring, msgRelayInfo, msgInfo);
    #ifdef isSendBlynkWindowOpenAlert // reset alert via bridge
      bridge1.virtualWrite(V70, 0); // bridge1 uses V70. 1: alert on, 0: alert off
      bridge2.virtualWrite(V80, 0); // bridge2 uses V80. 1: alert on, 0: alert off
    #endif
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
    unsigned long retVal;
    Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);      // UART to Sensair CO2 Sensor
    
    send_Request(ABCreq, 8);                     // Request ABC-Information from the Sensor
    read_Response(7);
    retVal = get_Value(7);

    Serial.println();
    Serial.print("SenseAir S8 ABC-Value: ");
    Serial.printf("%02ld", retVal);
    Serial.println("");

    if(retVal != 99)  // no error returned
    {
      senseAirPresentFlag = true;

      sprintf(printstring,"SenseAir S8 ABC-Value: %02ld\n", retVal);
      logOut(printstring, msgSenseAirInfo, msgInfo);
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
    }  
    else
    {
      senseAirPresentFlag = false;
      sprintf(printstring,"SenseAir S8 ABC-Value Error: %02ld", retVal);
      logOut(printstring, msgSenseAirMissing, msgErr);
    }  
  #endif

  #ifdef isMHZ14A
    // establish serial communication to CO2 sensor
    Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
    warmingTimer = millis();  // initiAlize warmup timer
    timer1 = millis();
  #endif

  #ifdef isBME680
    sprintf(printstring,"- Initializing BME680 sensor with Zanshin Library\n");
    logOut(printstring, msgBME680Info, msgInfo);
    while (!BME680.begin(I2C_STANDARD_MODE, 0x77)) {  // Start using I2C, use address 0x77 (could also be 0x76)
      sprintf(printstring,"-  Unable to find BME680. Trying again in 5 seconds.\n");
      logOut(printstring, msgBME680NotFound, msgErr);
      delay(5000);
    }  // of loop until device is located
    sprintf(printstring,"- Setting 16x oversampling for all sensors\n");
    logOut(printstring, msgBME680Info,msgInfo);
    sprintf(printstring,"- Setting IIR filter to a value of 4 samples\n");
    logOut(printstring, msgBME680Info, msgInfo);
    sprintf(printstring,"- Turning on gas measurements\n");
    logOut(printstring, msgBME680Info, msgInfo);
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

  #ifdef isBME680_BSECLib
    int iaqSensorStatus, iaqBME680Status;
    do{
      sprintf(printstring,"- Initializing BME680 sensor with BSEC Library\n");
      logOut(printstring, msgBME680Info, msgInfo);

      Wire.begin();
      permstorage.begin("BME680", false);         // open namespace BME680 in permanent storage

      #ifdef BME_Secondary_Address   // if defined, use secondary address for BME680
        iaqSensor.begin(BME680_I2C_ADDR_SECONDARY, Wire);
      #else
        iaqSensor.begin(BME680_I2C_ADDR_PRIMARY, Wire);
      #endif
      //output = "\nBSEC library version " + String(iaqSensor.version.major) + "." + String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) + "." + String(iaqSensor.version.minor_bugfix);
      //Serial.println(output);
      sprintf(printstring,
        "\nBSEC library version %d.%d.%d.%d", 
          iaqSensor.version.major, iaqSensor.version.minor, 
          iaqSensor.version.major_bugfix, iaqSensor.version.minor_bugfix);
      logOut(printstring, msgBME680Info, msgInfo);
      checkIaqSensorStatus();

      iaqSensor.setConfig(bsec_config_iaq);
      checkIaqSensorStatus();

      loadBsecState();

      bsec_virtual_sensor_t sensorList[7] = {
        BSEC_OUTPUT_RAW_TEMPERATURE,
        BSEC_OUTPUT_RAW_PRESSURE,
        BSEC_OUTPUT_RAW_HUMIDITY,
        BSEC_OUTPUT_RAW_GAS,
        BSEC_OUTPUT_IAQ,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
      };

      iaqSensor.updateSubscription(sensorList, 7, BSEC_SAMPLE_RATE_LP);

      iaqSensorStatus = iaqSensor.status;
      iaqBME680Status = iaqSensor.bme680Status;
      checkIaqSensorStatus(); // may reset iaqSensor.status
      if(iaqSensorStatus != BSEC_OK || iaqBME680Status != BSEC_OK) 
        resetBME680(iaqSensor.status, iaqBME680Status);
      esp_task_wdt_reset();   // keep watchdog happy  
    } while(iaqSensorStatus != BSEC_OK || iaqBME680Status != BSEC_OK);  
  #endif 

  // EXTDis #ifdef isBLYNK
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
        logOut(printstring, msgBlynkNotConnected, msgInfo);
      #endif  // isBLYNK 
    #endif // Serial stuff  
    // EXTDis #endif // isBLYNK

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
          Serial.println("Start OTA updating " + type);
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
    #endif // isOTA
  
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
    #endif // isInfactory

    #ifdef isBLYNK
      #ifdef blynkRegularCheck
        // MyBlynkCheckTimer.setInterval(5000L, checkBlynk); // check if connected to Blynk server every 5 seconds. Not necessary, left out
        checkTimerHandle = MyBlynkTimer.setInterval(checkTimerInterval, checkBlynk); // check if connected to Blynk server every 5 seconds. Not necessary, left out
        sprintf(printstring, "checkTimerHandle: %d\n", checkTimerHandle);
        logOut(printstring, msgBlynkInfo, msgInfo);
      #endif  
      // Serial.println("B");
      #ifdef blynkRestartHouly
        // MyBlynkRestartTimer.setInterval(1*3600L*1000L, restartBlynk); // attempt to restart Blynk every 1 hours
        restartTimerHandle = MyBlynkTimer.setInterval(restartTimerInterval, restartBlynk); // attempt to restart Blynk every 3 hours
        sprintf(printstring, "restartTimerHandle: %d\n", restartTimerHandle);
        logOut(printstring, msgBlynkInfo, msgInfo);
      #endif  
    #endif // isBLYNK  

  Serial.print("8");
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
  Serial.print("9");

  #ifdef serialMonitor
    logOut("Entering Serial Monitor mode - all other is off\n", msgDefaultID, msgInfo);
    monitorSerial();
  #endif
  sprintf(printstring,"end setup\n");
  logOut(printstring, msgSetupInfo, msgInfo);
  Serial.print(" 10 ");
} // setup


#ifdef isBME680

  float hum_weighting = 0.25; // so hum effect is 25% of the total air quality score
  float gas_weighting = 0.75; // so gas effect is 75% of the total air quality score

  float hum_score, gas_score;
  float gas_reference = 250000;
  float hum_reference = 40;
  int getgasreference_count = 0;

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
      strcpy(printstring, "AQ is ") ;

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

#ifdef isBME680_BSECLib
  // Reset the BME680 sensor (if we can...)
  void resetBME680(int sensorStatus, int bme680Status)
  {
    bool ret;
    int i=0;

    countBME680Resets++;  // increment reset counter

    if(countBME680Resets > 500)   
    {
      sprintf(printstring,"resetBME680: Resetting %d times not successful, restarting ESP32\n", countBME680Resets-1);
      logOut(printstring, msgBME680Reset, msgErr);
      delay(2000);
      resetFunc();
    }  

    #ifdef isRelay
      // toggle power if Wire.begin does not help (as it does not in  most cases)
      if(countBME680Resets > 5)   
      {
        sprintf(printstring,"resetBME680: Resetting %d times not successful, toggling power to BME680\n", countBME680Resets-1);
        logOut(printstring, msgBME680Reset, msgErr);
        digitalWrite(RELAYPIN2, HIGH);
        delay(2000);
        digitalWrite(RELAYPIN2, LOW);
        countBME680PowerToggles++;  // increment power toggle counter
        countBME680Resets = 0;      // restart reset counter
      }  
    #endif  

    sprintf(printstring,"Resetting BME680 #%d since sensor status: %d BME680 Status: %d\n", 
      countBME680Resets, sensorStatus, bme680Status);
    logOut(printstring, msgBME680Reset, msgInfo);

    do{
      ret = Wire.begin(); 
      sprintf(printstring,"%d Wire.begin() returned: %d\n", i, ret);
      logOut(printstring, msgBME680Reset, msgInfo);
      i++;
     }while(!ret && i<5);

    permstorage.begin("BME680", false);         // open namespace BME680 in permanent storage

    #ifdef BME_Secondary_Address   // if defined, use secondary address for BME680
      iaqSensor.begin(BME680_I2C_ADDR_SECONDARY, Wire);
    #else
      iaqSensor.begin(BME680_I2C_ADDR_PRIMARY, Wire);
    #endif
    iaqSensor.getState(bsecState);

    sprintf(printstring,"Sensor and BME680 status after iaqSensor.begin(): %d %d\n", 
        iaqSensor.status, iaqSensor.bme680Status);
    logOut(printstring, msgBME680Reset, msgInfo);

    //output = "\nBSEC library version " + String(iaqSensor.version.major) + "." + String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) + "." + String(iaqSensor.version.minor_bugfix);
    //Serial.println(output);
    sprintf(printstring,
      "\nBSEC library version %d.%d.%d.%d\n", 
        iaqSensor.version.major, iaqSensor.version.minor, 
        iaqSensor.version.major_bugfix, iaqSensor.version.minor_bugfix);
    logOut(printstring, msgBME680Reset, msgInfo);
    // checkIaqSensorStatus();

    iaqSensor.setConfig(bsec_config_iaq);
    // checkIaqSensorStatus();

    loadBsecState();

    bsec_virtual_sensor_t sensorList[7] = {
      BSEC_OUTPUT_RAW_TEMPERATURE,
      BSEC_OUTPUT_RAW_PRESSURE,
      BSEC_OUTPUT_RAW_HUMIDITY,
      BSEC_OUTPUT_RAW_GAS,
      BSEC_OUTPUT_IAQ,
      BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
      BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
    };

    iaqSensor.updateSubscription(sensorList, 7, BSEC_SAMPLE_RATE_LP);
    checkIaqSensorStatus();
  }

  // check the status if the iaq (internal air quality) sensor
  void checkIaqSensorStatus(void)
  {
    if (iaqSensor.status != BSEC_OK) {
      if (iaqSensor.status < BSEC_OK) {
        sprintf(printstring,"BSEC error code : %d\n", iaqSensor.status);
        logOut(printstring, msgBME680Error, msgErr);
        //for (;;)  // we do not want to halt this program, just log output
        //  errLeds(); /* Halt in case of failure */
      } else {
        sprintf(printstring,"BSEC warning code : %d\n", iaqSensor.status);
        logOut(printstring, msgBME680Error, msgWarn);
      }
    }

    if (iaqSensor.bme680Status != BME680_OK) {
      if (iaqSensor.bme680Status < BME680_OK) {
        // output = "BME680 error code : " + String(iaqSensor.bme680Status);
        // Serial.println(output);
        sprintf(printstring,"BME680 error code : %d\n", iaqSensor.bme680Status);
        logOut(printstring, msgBME680Error, msgErr);
        //for (;;)  // we do not want to halt this program, just log output
        //  errLeds(); /* Halt in case of failure */
      } else {
        // output = "BME680 warning code : " + String(iaqSensor.bme680Status);
        // Serial.println(output);
        sprintf(printstring,"BME680 warning code : %d\n", iaqSensor.bme680Status);
        logOut(printstring, msgBME680Error, msgWarn);
      }
      iaqSensor.status = iaqSensor.status;
    } 
    else
        iaqSensor.status = BSEC_OK;
  }

  // read the state of the BSEC sensor from permanent storage. 139 Bytes in char field.
  void loadBsecState(void)
  {
    // besser als mit EEPROM: preferences. https://randomnerdtutorials.com/esp32-save-data-permanently-preferences/
    size_t storesize = permstorage.getBytesLength("bsecstate");
    if (storesize == BSEC_MAX_STATE_BLOB_SIZE) {
      // Existing state in permstorage
      sprintf(printstring,"Reading BME680 state from permstorage\n");
      logOut(printstring, msgBME680Info, msgInfo);

      size_t readsize = permstorage.getBytes("bsecstate", bsecState, BSEC_MAX_STATE_BLOB_SIZE);
      Serial.printf("bsecstate expected: %d read: %d\n", storesize, readsize);
      /*
      for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++) {
        sprintf(printstring, " %3d 0x%02X  ",i, bsecState[i]);
        logOut(printstring, msgBME680Info, msgInfo);
        if((i+1)%10==0) {
          sprintf(printstring,"\n");
          logOut(printstring, msgBME680Info, msgInfo);
        }  
      }
      sprintf(printstring,"\n");
      logOut(printstring, , msgBME680Info, msgInfo);
      */
      iaqSensor.setState(bsecState);
      checkIaqSensorStatus();
    } else {
      // Erase the key "bsecstate" 
      sprintf(printstring,"Erasing bsecstate");
      logOut(printstring, msgBME680Info, msgInfo);
      permstorage.remove("bsecstate");  // remove the key
      // permstorage.clear(); // erase the whole namespace
    }
  }

  // write the state of the BSEC sensor to permanent storage if required
  void updateBsecState(void)
  {
    char outstring[100];

    bool update = false, printflag = false;
    /* Set a trigger to save the state. Here, the state is saved every STATE_SAVE_PERIOD with the first state being saved once the algorithm achieves full calibration, i.e. iaqAccuracy = 3 */
    if (stateUpdateCounter == 0) {
      if (iaqSensor.iaqAccuracy >= 3) {
        update = true;
        stateUpdateCounter++;
      }
    } else {
      /* Update every STATE_SAVE_PERIOD milliseconds */
      if ((stateUpdateCounter * STATE_SAVE_PERIOD) < millis()) {
        update = true;
        stateUpdateCounter++;
      }
    }

    if (update) {
      iaqSensor.getState(bsecState);
      checkIaqSensorStatus();

      // use preferences instead of deprecated eeprom method
      sprintf(printstring,"Writing state into permstorage. Size: %d \n", BSEC_MAX_STATE_BLOB_SIZE);
      logOut(printstring, msgBME680Info, msgInfo);
      /*
      for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE ; i++) {
        sprintf(outstring," %3d 0x%02X  ",i, bsecState[i]);
        strcat(printstring, outstring);
        // Serial.printf(" %3d ",i);
        // Serial.print(bsecState[i], HEX);
        if((i+1)%10==0) {
          strcat(printstring,"\n");
          logOut(printstring, msgBME680Info, msgInfo);
          strcpy(printstring,"");
          printflag = true;
        }  
      }
      if(!printflag) 
        logOut(printstring, msgBME680Info, msgInfo);
      */  

      //critical section. Could help with hanging putBytes.
      // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/freertos-smp.html#critical-sections-disabling-interrupts
      //portMUX_TYPE myMutex = portMUX_INITIALIZER_UNLOCKED;
      //portENTER_CRITICAL(&myMutex);

      permstorage.putBytes("bsecstate", bsecState, BSEC_MAX_STATE_BLOB_SIZE);
      //portEXIT_CRITICAL(&myMutex);
      sprintf(printstring,"\npermstorage length after writing %d\n",permstorage.getBytesLength("bsecstate"));
      logOut(printstring, msgBME680Info, msgInfo);
    }
  }
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
  /**************************************************!
   @brief    Display handing routine for OLED display, based on displayMode
   @details  determines which exactly is presently visible on the display. displayMode toggled by button
   @param displayMode determines the present content of the display. 0: off, 1: all, 2: versin etc.
   @return   void
  ***************************************************/
  void specialDisplay(int displayMode)
  {
    static float TempC0=-111.11, TempC1 = -111.11, TempC2=-111.11;
    char timestring[50]="unknown";    
    //char printstring[80];
    display.clearDisplay(); 
    switch (displayMode)
    {
      case 2:     // Version stuff etc. 
        display.setTextSize(1);
        // sprintf(printstring,"%s",PROGNAME);
        sprintf(printstring,"%s",infoStringShort);
        display.setCursor(0, 0); display.println(printstring);
        sprintf(printstring,"Version  : %s",PROGVERSION);
        display.setCursor(0, 12); display.println(printstring);
        sprintf(printstring,"Prog Date: %s",PROGDATE);
        display.setCursor(0, 24); display.println(printstring);
        #ifdef getNTPTIME
          if(TimeIsInitialized)
            // printLocalTime(timestring, 5); // with DST Info (1=DST, 0: not DST)
            printLocalTime(timestring, 7);
        #endif 
        sprintf(printstring,"Time:%s",timestring);
        display.setCursor(0, 36); display.println(printstring);
        sprintf(printstring,"Sec: %3.1f ",(float)millis()/1000);
        display.setCursor(0, 48); display.println(printstring); 
        if(WiFi.status() == WL_CONNECTED)
        {
          // int8_t rssi;
          // rssi = WiFi.RSSI();
          sprintf(printstring,"RS:%2.0f",rssi);
          display.setCursor(88, 48); display.println(printstring); 
        }  
        break;
      #if defined isBME680 || defined isBME680_BSECLib
      case 3:
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
      case 4:
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
        sprintf(printstring,"%3.1f  ",air_quality_score);
        display.setTextSize(2);
        display.setCursor(0, 44);
        display.println(printstring);  
        break;
      #endif   // BME680
      #ifdef isBME280 
      case 3:
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
      case 4:
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
      case 5:
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
      case 6:
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

  // function to check if blynk is still connected
  void checkBlynk()
  {  // called every 3 seconds by SimpleTimer MyBlynkCheckTimer
    bool isconnected;
    isconnected = Blynk.connected();
    
    if(WL_CONNECTED != WiFi.status())
    {   
      sprintf(printstring,"checkBlynk 5: Resetting - no Blynk connection. isConnected: %d Wifi.Status: %d\n", isconnected,  WiFi.status());
      logOut(printstring, msgBlynkNotConnected, msgWarn);
      isconnected = false;
    }

    if (isconnected == false)
    {
      blynkDisconnects++;  

      if(blynkDisconnects > 5)   
      {
        sprintf(printstring,"checkBlynk 1a: Restart Blynk - no Blynk connection. isconnected: %d blynkDisconnects: %d\n", isconnected, blynkDisconnects);
        logOut(printstring, msgBlynkNotConnected, msgWarn);
        restartBlynk();
      }  
      if(blynkDisconnects > 10)   
      {
        sprintf(printstring,"checkBlynk 1b: Resetting - no Blynk connection. isconnected: %d blynkDisconnects: %d\n", isconnected, blynkDisconnects);
        logOut(printstring, msgBlynkNotConnected, msgWarn);
        resetFunc();
      }  

      sprintf(printstring,"checkBlynk 2: Reconnecting Blynk. isconnected: %d blynkDisconnects: %d\n", isconnected, blynkDisconnects);
      logOut(printstring, msgBlynkNotConnected, msgWarn);  
      if(WiFi.status() != WL_CONNECTED)  
      {
        sprintf(printstring,"checkBlynk 3: Reconnecting Wifi. Wifi.status: %d\n", WiFi.status());
        logOut(printstring, msgBlynkNotConnected, msgWarn);  
        Blynk.connectWiFi(ssid,pass);
        #ifdef blynkCloud
           Blynk.config(auth);
        #else
          Blynk.config(auth, IPAddress(blynkLocalIP), 8080);
        #endif  
      }  
      sprintf(printstring,"checkBlynk 4: after Blynk.conectWifi. Wifi.status: (3:connected, 6: disconnected) %d\n", WiFi.status());
      logOut(printstring, msgBlynkNotConnected, msgWarn);  
      Blynk.connect();
      Blynk.syncAll();
      sprintf(printstring,"checkBlynk 6: After Blynk.connect. Wifi.status: %d Blynk.connected: %d\n", WiFi.status(), Blynk.connected() );
      logOut(printstring, msgBlynkNotConnected, msgWarn);  
    }
    else
    {
      blynkDisconnects = 0;
      sprintf(printstring,"checkBlynk 0: Blynk is connected. isconnected: %d blynkDisconnects: %d\n", isconnected, blynkDisconnects);
      logOut(printstring, msgBlynkConnected, msgInfo);      
    }
  }

  /* outdated version
  void checkBlynk()
  { // called every 3 seconds by SimpleTimer MyBlynkCheckTimer
    bool isconnected;
    isconnected = Blynk.connected();
    if (isconnected == false)
    {
      blynkDisconnects++;  
      if(blynkDisconnects > 10)   
      {
        sprintf(printstring,"checkBlynk: Resetting - no Blynk connection %d %d\n", isconnected, blynkDisconnects);
        logOut(printstring, msgBlynkNotConnected, msgWarn);
        resetFunc();
      }  
      sprintf(printstring,"checkBlynk: Reconnecting Blynk %d %d\n", isconnected, blynkDisconnects);
      logOut(printstring, msgBlynkNotConnected, msgWarn);  
      Blynk.connect();
    }
    else
    {
      blynkDisconnects = 0;
      sprintf(printstring,"checkBlynk: Blynk is connected %d %d\n", isconnected, blynkDisconnects);
      logOut(printstring, msgBlynkConnected, msgInfo);       
    }
  }
  */ 

  // function to stop and then restart Blynk
  // intended as workaround, since often after a few hours no data are transmitted. 
  void restartBlynk()
  {
    // logics to check if a reboot has been done
    if (restartCount == 0) {            
      NoReboots ++;                     // Increment number of reboots
      preferences.begin("nvs", false);  // Save changed NoReboots to NVS memory
      preferences.putInt("NoReboots", NoReboots);
      preferences.end();
      sprintf(printstring,"restartBlynk 1: Incremented Number of Reboots to: %d\n", NoReboots);
      logOut(printstring, msgBlynkRestart, msgInfo); 
    }
    restartCount ++;
    sprintf(printstring, "restartBlynk 2: Restart Blynk No %d. Reboots: %d  ", restartCount, NoReboots);
    logOut(printstring, msgBlynkRestart, msgInfo); 

    Blynk.disconnect();
    delay(1500);
    if(WiFi.status() != WL_CONNECTED)
    {
      sprintf(printstring,"restartBlynk 3: Reconnecting Wifi. Status: %d\n", WiFi.status());
      logOut(printstring, msgBlynkRestart, msgInfo);  
      Blynk.connectWiFi(ssid,pass);
      #ifdef blynkCloud
        Blynk.config(auth);
      #else  
        Blynk.config(auth, IPAddress(blynkLocalIP), 8080);
      #endif  
    }  
    sprintf(printstring,"restartBlynk 4: Blynk.connect(). Status: %d\n", WiFi.status());
    logOut(printstring, msgBlynkRestart, msgInfo); 
    Blynk.connect();
    delay(600);
    Blynk.virtualWrite(V12, restartCount); 
    Blynk.syncAll();
  }
  // outdated version
  /*
  void restartBlynk()
  {
    //char printstring[80];

    // logics to check if a reboot has been done
    if (restartCount == 0) {            
      NoReboots ++;                     // Increment number of reboots
      preferences.begin("nvs", false);  // Save changed NoReboots to NVS memory
      preferences.putInt("NoReboots", NoReboots);
      preferences.end();
      sprintf(printstring,"Incremented Number of Reboots to: %d\n", NoReboots);
      logOut(printstring, msgBlynkRestart, msgInfo); 
    }

    restartCount ++;
    Serial.printf("=== Restarting Blynk routinely. No: %d ===\n", restartCount);
 
    sprintf(printstring, "Restart Blynk No %d. Reboots: %d  ", restartCount, NoReboots);
    logOut(printstring, msgBlynkRestart, msgInfo); 
    Blynk.disconnect();
    delay(1500);
    #ifdef blynkCloud
      Blynk.begin(auth, ssid, pass, "blynk-cloud.com", 80);
    #else
      // Blynk.begin(auth, ssid, pass, IPAddress(192,168,178,31), 8080);
      Blynk.begin(auth, ssid, pass, IPAddress(blynkLocalIP), 8080);
    #endif  
    delay(600);
    Blynk.virtualWrite(V12, restartCount); 
  }
  */ 

  #ifdef isRelay
  // Relay 1 is switched via virtual V40 (wasPin 18)
    BLYNK_WRITE(V40) 
    {
      int x = param.asInt();
      // Serial.printf("Relay 1 switched to  %d \n", x);
      if(x==1){  
        digitalWrite(RELAYPIN1, HIGH);
        #ifdef isFan
          fanState=1;
        #endif  
      } else  
      {
        digitalWrite(RELAYPIN1, LOW);
        #ifdef isFan
          fanState=0;
        #endif  
      }  
    }

  // Relay 2 is switched via virtual Pin 41 (was 19)
    BLYNK_WRITE(V41) 
    {
      int x = param.asInt();
      // Serial.printf("Relay 2 switched to  %d \n", x);
      if(x==1)  
        digitalWrite(RELAYPIN2, HIGH);
      else  
        digitalWrite(RELAYPIN2, LOW);
    }
  // Temperature Offset is switched via virtual Pin 42
    BLYNK_WRITE(V42) 
    {
      tempSwitchOffset = param.asFloat();
      sprintf(printstring, "tempSwitchOffset changed to  %4.2f \n", tempSwitchOffset);
      logOut(printstring, msgTempOffsetChange, msgInfo);
      digitalWrite(RELAYPIN1, LOW); // fan off and defined startup state
      #ifdef isFan
        fanState=0;
      #endif  
      Blynk.virtualWrite(V40,0) ;   // defined state at app: fan off
    }
  #endif  // relay

  #ifdef isReceiveBlynkWindowOpenAlert
    BLYNK_WRITE(V70) 
    {
      int pinData = param.asInt();
      sprintf(printstring, "Message received via V70  %d \n", pinData);
      logOut(printstring, msgAlertReceived, msgInfo);
      /* beeperState = pinData; */
      externalAlertState = pinData;
      #ifdef isBeeperWindowOpenAlert
        windowSetBeeper(pinData); // sets beeperState, switches beeper on if beeperQuietCounter not running
        /*
        if(beeperState == 1)
          digitalWrite(RELAYPIN1, HIGH);  
        else  
          digitalWrite(RELAYPIN1, LOW);  
        */  
      #endif    
    }
    
    BLYNK_WRITE(V80) 
    {
      int pinData = param.asInt();
      sprintf(printstring, "Message received via V80  %d \n", pinData);
      logOut(printstring, msgAlertReceived, msgInfo);
      /* beeperState = pinData; */
      externalAlertState = pinData;
      #ifdef isBeeperWindowOpenAlert
        windowSetBeeper(pinData); // sets beeperState, switches beeper on if beeperQuietCounter not running
        /*
        if(beeperState == 1)
          digitalWrite(RELAYPIN1, HIGH);  
        else  
          digitalWrite(RELAYPIN1, LOW);  
        */  
      #endif  
    }
  #endif

  // this function is called every time that blynk connection is established, and re-syncs widgets if first connection
  bool isFirstConnect = true;
  BLYNK_CONNECTED()
  {
    sprintf(printstring,"Blynk detected connection. isFirstConnect: %d \n", isFirstConnect);
    logOut(printstring,msgBlynkConnected, msgInfo); 
    if(isFirstConnect)
      Blynk.syncAll();
    isFirstConnect = false;

    // ensure that bridge is present which can be used to send alerts to another blynk device
    #ifdef isSendBlynkWindowOpenAlert
      bridge1.setAuthToken(authAlertReceiver[0]); 
      bridge2.setAuthToken(authAlertReceiver[1]); 
    #endif
  }  
#endif  //blynk

#if defined isOneDS18B20 && defined isWindowOpenDetector
  /**************************************************!
  @brief    helper function to actually set the beeper
  @details  based on desiredValue. Sets beeperState, respects beeperQuietCounter
  @param  int desiredValue
  @return void
  ***************************************************/
  void windowSetBeeper(int desiredValue)
  {
      #ifdef isBeeperWindowOpenAlert
      sprintf(printstring,"Beeper set to: %d \n", desiredValue);
      logOut(printstring, msgBeeperTriggered, msgInfo);
      // turn on beeper if desiredValue==1 and beeper quiet counter not running
        if((desiredValue == 1) && (beeperQuietCounter < 1)){
          digitalWrite(RELAYPIN1, HIGH);  
          beeperState = 1;
        }  
        else{
          digitalWrite(RELAYPIN1, LOW);  
          beeperState = 0;
        }  
      #endif 
  }

  /**************************************************!
  @brief    window open handler
  @details  checks if window is open, and if yes turns on the alert as #defined
  @param none
  @return void
  ***************************************************/
  void windowOpenHandler(void)
  {
    if(calDS18B20Temperature[tempSwitchSensorSelector] < -110) // invalid DS18B20 Temp, do not try
      return;
    float localTemp = calDS18B20Temperature[tempSwitchSensorSelector];

    // temperature dropped enough and beeper off: turn beeper on.
    if( (localTemp < temperatureAverage - temperatureDropTrigger)
      && (temperatureAverageCounter > 0.2*temperatureAverageNumber)
      && beeperState == 0
      )     
    {
      #ifdef isBLYNK
        Blynk.virtualWrite(V40, 1);   
        #ifdef isBeeperWindowOpenAlert
          windowSetBeeper(1); // sets beeper, respects quiet counter
        #endif  
        #ifdef isSendBlynkWindowOpenAlert
          bridge1.virtualWrite(V70, 1); // bridge1 uses V70. 1: alert on, 0: alert off
          bridge2.virtualWrite(V80, 1); // bridge2 uses V80. 1: alert on, 0: alert off
        #endif
      #endif  
      sprintf(printstring, "ON_ON_ON_ON Alert switched ON. BME: %4.2f DS18B20: %4.2f\n", 
          localTemp, calDS18B20Temperature[tempSwitchSensorSelector]);
      logOut(printstring, msgRelayInfo, msgInfo);
    } 
      // Beeper is on, and condition no longer met, and no external alert: turn it off
    if(localTemp >= temperatureAverage - temperatureDropTrigger
      && beeperState == 1
      && externalAlertState == 0)     
    {  
      windowSetBeeper(0); // beeper off, beeperState off
      // beeperState = 0;
      #ifdef isBLYNK
        Blynk.virtualWrite(V40, 0); 
      #endif   
      /*
      #ifdef isBeeperWindowOpenAlert
        digitalWrite(RELAYPIN1, LOW);
      #endif  
      */ 
      #if defined isSendBlynkWindowOpenAlert && defined isBLYNK
        bridge1.virtualWrite(V70, 0); // bridge1 uses V70. 1: alert on, 0: alert off
        bridge2.virtualWrite(V80, 0); // bridge2 uses V80. 1: alert on, 0: alert off
      #endif
      sprintf(printstring, "OFF_OFF_OFF Alert switched OFF. BME: %4.2f DS18B20: %4.2f\n", 
          localTemp, calDS18B20Temperature[tempSwitchSensorSelector]);
      logOut(printstring, msgRelayInfo, msgInfo);
    }  
    // button has been pushed recently => Beeper off
    if(beeperQuietCounter > 0)  
    {
      windowSetBeeper(0); // beeper off, beeperState off
      // beeperState = 0;
      // digitalWrite(RELAYPIN1, LOW); 
      beeperQuietCounter--;  // decrement counter for "button recently pressed"
      #if defined isSendBlynkWindowOpenAlert && defined isBLYNK
        bridge1.virtualWrite(V70, 0); // bridge1 uses V70. 1: alert on, 0: alert off
        bridge2.virtualWrite(V80, 0); // bridge2 uses V80. 1: alert on, 0: alert off
      #endif
    }  

    #ifdef isBLYNK
      Blynk.syncVirtual(V40); // synchronize app and sketch.  
    #endif

    #if defined isReceiveBlynkWindowOpenAlert
      if(externalAlertState == 1) // if there is an external alert as well, overwrite
      {
        windowSetBeeper(0); // beeper on, beeperState on
        Blynk.virtualWrite(V40, 0);   // set in app
        // digitalWrite(RELAYPIN1, HIGH);// set port for beeper
      }  
    #endif

    // initiate temperature average to speed up start phase. Use present temp minus 
    if(temperatureAverage < -110)    // startup value is -111
      temperatureAverage = localTemp - 5;
    
    // update temperature average
    temperatureAverageCounter ++;
    float tmp = (temperatureAverageNumber-1)*temperatureAverage/(temperatureAverageNumber);
    temperatureAverage = tmp + localTemp / temperatureAverageNumber;
    sprintf(printstring, "ActTemp: %4.2f TAvg: %4.2f Cntr: %ld beepSt: %d bQuietCnt: %d extAlertSt: %d\n", 
          localTemp,temperatureAverage, temperatureAverageCounter, beeperState, beeperQuietCounter, externalAlertState);
      logOut(printstring, msgRelayInfo, msgInfo);
  }
#endif  // DS18B20 & relay  & beeper


#if defined isOneDS18B20 && defined isRelay && defined isFan
  /**************************************************!
  @brief    fan handler
  @details  handles the function of the fan, if built in, based on temp difference 
  @param none
  @return void
  ***************************************************/
  void bme680FanHandler(void)
  {
    if(calDS18B20Temperature[tempSwitchSensorSelector] < -110) // invalid DS18B20 Temp, do not try
      return;
    // temperature high enough and fan off: turn fan on.
    if(temperature > calDS18B20Temperature[tempSwitchSensorSelector] + tempSwitchOffset 
      && fanState == 0)     
    {
      fanState = 1;
      #ifdef isBLYNK
        Blynk.virtualWrite(V40, 1);   
        digitalWrite(RELAYPIN1, HIGH);  
      #endif  
      sprintf(printstring, "ON_ON_ON_ON Fan switched ON. BME: %4.2f DS18B20: %4.2f Offset: %4.2f Max Pot: %3.1f\n", 
          temperature, calDS18B20Temperature[tempSwitchSensorSelector], tempSwitchOffset, fanMaxPotential);
      logOut(printstring, msgRelayInfo, msgInfo);
    } 
      // fan is on, and temperature has dropped to max. potential for fan: turn it off
    if(temperature < calDS18B20Temperature[tempSwitchSensorSelector] + tempSwitchOffset * (1-fanMaxPotential)
      && fanState == 1)     
    {  
      fanState = 0;
      #ifdef isBLYNK
        Blynk.virtualWrite(V40, 0); 
      #endif   
      digitalWrite(RELAYPIN1, LOW);
      sprintf(printstring, "OFF_OFF_OFF Fan switched OFF. BME: %4.2f DS18B20: %4.2f Offset: %4.2f Max Pot: %3.1f\n", 
          temperature, calDS18B20Temperature[tempSwitchSensorSelector], tempSwitchOffset, fanMaxPotential);
      logOut(printstring, msgRelayInfo, msgInfo);
    }  
    #ifdef isBLYNK
      Blynk.syncVirtual(V40); // synchronize app and sketch.  
    #endif
  }
#endif  // DS18B20 & relay  & fan

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
    if(strlen(receivedChars)>0)
      Serial.printf(" - Serial read: '%s' len: %d \n", receivedChars, strlen(receivedChars));
    return(strlen(receivedChars));

  } // receiveSerial

  boolean parseData(char *tempChars, float* serialT, float* serialH, int* serialCh) {      // split the data into its parts

    char * strtokIndx;                    // this is used by strtok() as an index
    char *pEnd;                           // end pointer for strtod  
    //char message[numChars];               // buffer

    strtokIndx = strtok(tempChars,",");   // get the first part - the string
    if (strtokIndx != NULL) 
      // *serialCh = atoi(strtokIndx);    // convert this part to an integer
      *serialCh = strtol(strtokIndx,&pEnd, 10); // strol and strod are more robust than atoi and atof
    else
      return false;  
 
    strtokIndx = strtok(NULL, ",");  // this continues where the previous call left off
    if (strtokIndx != NULL) 
      // *serialT = atof(strtokIndx);     // convert this part to an float
      *serialT = strtod(strtokIndx,&pEnd);
    else
      return false;  

    strtokIndx = strtok(NULL, ",");
    if (strtokIndx != NULL) 
      // *serialH = atof(strtokIndx);     // convert this part to a float
      *serialH = strtod(strtokIndx,&pEnd);
    else
      return false;

    return true;
  }

  /**************************************************!
   @brief    handles date received via serial2 from Infactory 433 sensor attached to Aruino
   @details  gets data from "receiveSerial(), parses them and returns temp/humidity/channel via parameters
   @param sTempC returned temperature in Celsius
   @param sHumidity returned humidity in %
   @param sChannel returned  channel for which temp and humidity have been received
   @param numCharsReceived  returned number of characters received
   @return   boolean. true if data received, otherwise false
  ***************************************************/
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
        // sprintf(printstring,"%d faulty data received, Total Count: %d - %s\n",numCharsReceived,serialFailCount, receivedChars);
        //  logOut(printstring, msgSerialFaulty, msgErr);  
        serialTemp=-111.11;
        serialHumidity=-111;
        serialChannel=0;
        ret = false;
      }  
      else
      {
        *sTempC =  serialTemp;
        *sHumidity = serialHumidity;
        sprintf(printstring,"converted Infactory (Ch: %d) T: %3.1f %3.1f %%\n",
          *sChannel, *sTempC, *sHumidity);
        logOut(printstring, msgSerialReceived, msgInfo);  
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

#ifdef isThingspeak
  /**************************************************!
    @brief    function to send data to thingspeak 
    @details  sends the completed URL string to the thingspeak web API
    @param    String url : completed url string to be sent via internet to Thingspeak api 
    @return   int httpResponseCode. OK if above zero.
  ***************************************************/
  char lastThingspeakSendSuccessString[50]="unknown";
  
  int sendThingspeakData(String url)
  {
    // int i=0;
    static int httpErrorCounter = 0;
    static int sendThingspeakDataErrors = 0;
    char timestring[50]="";    

    //Serial.println(url);
    sprintf(printstring,"sendThingspeakData() URL: %s\n",url.c_str());
    logOut(printstring, msgThingspeakSend, msgInfo);

    // Connect or reconnect to WiFi
    if(WiFi.status() != WL_CONNECTED)
      connectToWiFi(ssid, pass, 7);  // new 22.11.21
    /* old until 22.11.21
    // if((!wifiClient.connected()) || (WiFi.status() != WL_CONNECTED))
    {
      Serial.print("Attempting to connect to SSID: ");
      Serial.println(ssid);
      while((WiFi.status() != WL_CONNECTED) && (i<10))
      {
        i++;
        WiFi.disconnect();      // new 11.10.21: disconnect and connect in the loop
        WiFi.begin(ssid, pass);
        Serial.print(".");
        delay(i*500);     
      } 
      esp_task_wdt_reset();   // keep watchdog happy
      Serial.println("\nConnected.");
    }
    */

    HTTPClient http; // Initialize our HTTP client
    int httpResponseCode;
    do {      
      esp_task_wdt_reset();   // keep watchdog happy   
      bool success = Ping.ping("https://api.thingspeak.com", 3);
      if(!success)
        sprintf(printstring,"Ping https://api.thingspeak.com failed\n");
      else 
        sprintf(printstring,"Ping https://api.thingspeak.com successful\n");
      logOut(printstring, msgThingspeakSend, msgInfo);

      delay(500 + 5000 * httpErrorCounter);
      httpResponseCode = http.begin(url.c_str()); // Initialize our HTTP request
      sprintf(printstring, "HTTP Begin Response code: %d \n", httpResponseCode);
      logOut(printstring, msgThingspeakSend, msgInfo);

      // http.setConnectTimeout(10000); // does not work: set connect timeout: to establish the connection
      http.setTimeout(10000);  // set timeout for http to 10 sec: to transfer data
      httpResponseCode = http.GET(); // Send HTTP request          

      #ifdef getNTPTIME
        if(TimeIsInitialized)
        {
          printLocalTime(timestring, 5);  // get the present time
          sprintf(printstring, "printLocalTime returned: %s \n", timestring);
          logOut(printstring, msgThingspeakSend, msgInfo);
        }  
        else
          strcpy(timestring,"time not initialized");
      #endif 
      if (httpResponseCode > 0){ // Check for good HTTP status code
        sprintf(printstring, "HTTP Resp: %d Prev OK send: %s\n", httpResponseCode, lastThingspeakSendSuccessString);
        logOut(printstring, msgThingspeakSend, msgErr);
        httpErrorCounter = 0;
        strcpy(lastThingspeakSendSuccessString, timestring);
        // sprintf(printstring, "HTTP Resp: %d Last OK send: %s timestring: %s\n", httpResponseCode, lastThingspeakSendSuccessString, timestring);
        // logOut(printstring, msgThingspeakSend, msgErr);
      }else{
        sprintf(printstring, "HTTP Error code: %d Last OK send: %s\n", httpResponseCode, lastThingspeakSendSuccessString);
        logOut(printstring, msgThingspeakSend, msgErr);
        httpErrorCounter++;
      }
      http.end();             // free resouces
      // if too many fails: reboot.
      // if(httpErrorCounter > 4)
      //  ESP.restart();
    } while (httpResponseCode <=0 && httpErrorCounter <= 1 );  // was 3 for 3 repeats
    httpErrorCounter = 0;
    if(httpResponseCode <=0)
      sendThingspeakDataErrors++;
    sprintf(printstring, "HTTP Error Counter: %d \n", sendThingspeakDataErrors);
    logOut(printstring, msgThingspeakSend, msgErr);
    return(httpResponseCode);
  }
#endif  // isThingspeak

#ifdef isThingspeak
  bool isEqual(double a, double b, double limit)
  {
    if(abs(a-b) < limit)
      return (true);
    else
      return (false);  
  }

  /**************************************************!
    @brief    thingspeak handler. Handles all data sending to thingspeak, called via timer
    @details  called via timer, builds thingspeak string and does output for all sensors actually present
    @return   void
  ***************************************************/
  void thingspeak_handler() 
  {
    #define minDiffTemperature  0.05
    #define minDiffHumidity     0.1
    #define minDiffPressure     0.05
    #define minDiffAirQuality   1.0
    #define minDiffDS18B20      0.05
    #define minDiffCO2ppm       10.0
    #define minDiffExtTemp      0.05
    #define minDiffExtHumid     0.5
    #define maxDiffExtTemp      5.0
    #define maxDiffExtHumid     5.0
    #define minDiffRSSI         0.3

    #define minimumRepeatCounter 30   // at least every 30 calls the data are sent for items without own counter

    
    // build thingspeak string and then send it 
    double limit = -110.0;
    static float last_temperature=-111, last_humidity=0, last_pressure=0; 
    int httpResponseCode;
    static bool repeatFlag = false; // first time is never a repeat
    #if defined isBME680 || defined isBME680_BSECLib
      static float last_air_quality_score=0;
    #endif   
    static double last_DSTemp0=-111, last_DSTemp1=-111, last_DSTemp2=-111;
    static long thingspeakSendItemCounter = 0, thingspeakCallCounter = 0;
    int thingspeakItemsCollected = 0;
    String url=ThingspeakServerName + thingspeakWriteAPIKey; 
    String secondary_url=ThingspeakServerName + thingspeakWriteAPIKey; 
    // average since last transfer
    float avg;

    thingspeakCallCounter++;      // counter for how often this function has been called
    thingspeakItemsCollected = 0; // counter for the number of items to be sent during this call
    sprintf(printstring,"ToThingspeak: ");
    // BME 680 & BME280 stuff
    #if defined isBME280 || defined isBME680 || defined isBME680_BSECLib
      // temperature to field 1 in thingspeak string
      #ifndef receiveSERIAL // if no serial data from Infactory 433 Sensor received, send BME temp to field 1 (otherwise humidity of 2nd channel)
        if(temperature_n > 0)
          avg = temperature_sum / temperature_n;
        else
          // avg = temperature;
          avg = last_temperature;  // Test 11.10.22 to avoid zeroes
        if(!isEqual(avg,last_temperature,minDiffTemperature) 
            || (temperature_n > 999) 
            || (repeatFlag==true) 
            || (thingspeakCallCounter % minimumRepeatCounter == 0)
          )
        {  
          // insert here if large temp jump: send last_temperature again to avoid unrealistical curve form
          if(!isEqual(avg,last_temperature,minDiffTemperature*10) && (last_temperature > -110)) // if jump larger than 10 x minimum recognized temp difference, and valid last temp.
          {
            secondary_url =ThingspeakServerName + thingspeakWriteAPIKey + "&field1=" + last_temperature;
            thingspeakSendItemCounter++;
            thingspeakCallCounter++;
            sprintf(printstring2," Sent temperature % 4.1f before strong rise  Call#: %ld \n", 
                last_temperature, thingspeakCallCounter);
            strcat(printstring, printstring2);
            logOut(printstring, msgThingspeakSend, msgInfo);

            httpResponseCode = sendThingspeakData(secondary_url);
            sprintf(printstring,"sendThingspeakData() returned: %d repeatFlag: %d \n",
              httpResponseCode, repeatFlag);
            logOut(printstring, msgThingspeakSend, msgInfo);           
          }
          
          last_temperature = avg;
          sprintf(printstring2," temp: %5.2f",avg);
          strcat(printstring, printstring2);
          temperature_sum=0; temperature_n=0;
          url = url+ "&field1=" + avg;
          thingspeakSendItemCounter++;
          thingspeakItemsCollected++;
        }  
        else{
          sprintf(printstring2," temp: notMeas ");
          strcat(printstring, printstring2);
          temperature_sum=0; temperature_n=0;
        }
      #endif //   #ifndef sendSERIAL
      // humidity to thingspeak
      if(humidity_n > 0)
        avg = humidity_sum / humidity_n;
      else
        // avg = humidity;
        avg = last_humidity;  // Test 11.10.22
      if(!isEqual(avg,last_humidity,minDiffHumidity) || (humidity_n > 999) || (repeatFlag==true) || (thingspeakCallCounter % minimumRepeatCounter == 0)){  
        last_humidity = avg;
        sprintf(printstring2," hum: %5.2f",avg);
        strcat(printstring, printstring2);
        humidity_sum=0; humidity_n=0;
        url = url+ "&field2=" + avg;
        thingspeakSendItemCounter++;
        thingspeakItemsCollected++;
      }  
      else{
        sprintf(printstring2," hum: notMeas ");
        strcat(printstring, printstring2);
        humidity_sum=0; humidity_n=0;
      }
      // pressure to thingspeak
      if(pressure_n > 0)
        avg = pressure_sum / pressure_n;
      else
        // avg = pressure;
        avg = last_pressure;  // Test 11.10.22
      if(!isEqual(avg,last_pressure,minDiffPressure) || (pressure_n > 999) || (repeatFlag==true)|| (thingspeakCallCounter % minimumRepeatCounter == 0)){  
        last_pressure = avg;
        sprintf(printstring2," pres: %5.2f",avg);
        strcat(printstring, printstring2);
        pressure_sum=0; pressure_n=0;
        url = url+ "&field3=" + avg;
        thingspeakSendItemCounter++;
        thingspeakItemsCollected++;
      }  
      else{
        sprintf(printstring2," pres: notMeas ");
        strcat(printstring, printstring2);
        pressure_sum=0; pressure_n=0;
      }

      #if defined isBME680 || defined isBME680_BSECLib
        // air quality score to thingspeak
        if(air_quality_score_n > 0)
          avg = air_quality_score_sum / air_quality_score_n;
        else
          avg = air_quality_score;
        // send datum is sufficiently changed or 1000 measurments collected (to get an occasional data point to thingspeak)  
        if(!isEqual(avg,last_air_quality_score,minDiffAirQuality) || air_quality_score_n > 999 || (repeatFlag==true) || (thingspeakCallCounter % minimumRepeatCounter == 0))
        {  
          sprintf(printstring2," airq : %5.2f %5.2f %ld ",avg,air_quality_score_sum,air_quality_score_n);
          strcat(printstring, printstring2);
          last_air_quality_score = avg;
          air_quality_score_sum = 0;  // reset sum for average
          air_quality_score_n = 0;    // rest counter for average
          url = url+ "&field4=" + avg;
          thingspeakSendItemCounter++;      
          thingspeakItemsCollected++;    
      }  
        else{
          sprintf(printstring2," airq: notMeas (avg: %5.3f) ",avg);
          strcat(printstring, printstring2);
          air_quality_score_sum = 0;  // reset sum for average
          air_quality_score_n = 0;    // rest counter for average
        }
      #endif  // defined isBME680 || defined isBME680_BSECLib
    #endif // BME280 or BME680  

    #ifdef isThingspeakRSSI  // on field4. NOT if air quality, or external 466MHz sensors with ext humidity on field 4
      // Wifi RSSI to Thingspeak 
      if(rssi_n > 0)
        avg = rssi_sum / rssi_n;
      else
        avg = rssi;
      if(!isEqual(avg,last_rssi,minDiffRSSI) || (rssi_n > 999) || (repeatFlag==true) || (thingspeakCallCounter % minimumRepeatCounter == 0)){  
        last_rssi = avg;
        sprintf(printstring2," RSSI: %5.2f",avg);
        strcat(printstring, printstring2);
        rssi_sum=0; rssi_n=0;
        url = url+ "&field4=" + avg;
        thingspeakSendItemCounter++;
        thingspeakItemsCollected++;
      }  
    #endif 

    #ifdef isOneDS18B20
      // DS18B20 data 
      double temp;
      sprintf(printstring,"DS18B20[0] sum: %f n: %d \n",sum_ThSp_calDS18B20Temperature[0],n_ThSp_calDS18B20Temperature[0]);
      logOut(printstring,msgDS18B20Info, msgInfo);
      if(n_ThSp_calDS18B20Temperature[0] > 0)
        temp = sum_ThSp_calDS18B20Temperature[0] / n_ThSp_calDS18B20Temperature[0];
      else
        temp = -111.11;  
      if( 
          ((!isEqual(temp,last_DSTemp0,minDiffDS18B20)) || (repeatFlag==true) || (thingspeakCallCounter % minimumRepeatCounter == 0))
          && (temp > limit)
        )
      {    
        // insert here if large temp jump: send last_temperature again to avoid unrealistical curve form
        if(!isEqual(temp,last_DSTemp0,minDiffDS18B20*10) && (last_DSTemp0 > -110)) // if jump larger than 10 x minimum recognized temp difference
          {
            secondary_url = ThingspeakServerName + thingspeakWriteAPIKey + "&field5=" + last_DSTemp0;
            thingspeakSendItemCounter++;
            thingspeakCallCounter++;
            sprintf(printstring2," Sent DS18B20 0 temperature % 4.1f before strong rise  Call#: %ld \n", 
                last_DSTemp0, thingspeakCallCounter);
            strcat(printstring, printstring2);
            logOut(printstring, msgThingspeakSend, msgInfo);

            httpResponseCode = sendThingspeakData(secondary_url);
            sprintf(printstring,"sendThingspeakData() returned: %d repeatFlag: %d \n",
              httpResponseCode, repeatFlag);
            logOut(printstring, msgThingspeakSend, msgInfo);           
          }
        sprintf(printstring2," Tmp0: notMeas cal: %5.2f last: %5.2f act: %5.2f sum: %5.2f n: %d",
          calDS18B20Temperature[0], last_DSTemp0, temp, sum_ThSp_calDS18B20Temperature[0], n_ThSp_calDS18B20Temperature[0]);
        strcat(printstring, printstring2);
        last_DSTemp0 = temp;
        sum_ThSp_calDS18B20Temperature[0] = 0;
        n_ThSp_calDS18B20Temperature[0] = 0;
        url = url+ "&field5=" + temp;
        thingspeakSendItemCounter++;
        thingspeakItemsCollected++;       
      }  
      else{
        sprintf(printstring2," Tmp0: notMeas cal: %5.2f last: %5.2f act: %5.2f sum: %5.2f n: %d",
          calDS18B20Temperature[0], last_DSTemp0, temp, sum_ThSp_calDS18B20Temperature[0], n_ThSp_calDS18B20Temperature[0]);
        strcat(printstring, printstring2);
        sum_ThSp_calDS18B20Temperature[0] = 0;
        n_ThSp_calDS18B20Temperature[0] = 0;
      }

      if(n_ThSp_calDS18B20Temperature[1] > 0)
        temp = sum_ThSp_calDS18B20Temperature[1] / n_ThSp_calDS18B20Temperature[1];
      else
        temp = -111.11;  
      //if( ((temp > limit) && !isEqual(temp,last_DSTemp1,minDiffDS18B20)) || (repeatFlag==true)
      //  || (thingspeakSendItemCounter % minimumRepeatCounter == 0))
      if( 
          ((!isEqual(temp,last_DSTemp1,minDiffDS18B20)) || (repeatFlag==true) || (thingspeakCallCounter % minimumRepeatCounter == 0))
          && (temp > limit)
        )  
      {  
        // insert here if large temp jump: send last_temperature again to avoid unrealistical curve form
        if(!isEqual(temp,last_DSTemp1,minDiffDS18B20*10) && (last_DSTemp1 > -110)) // if jump larger than 10 x minimum recognized temp difference
          {
            secondary_url = ThingspeakServerName + thingspeakWriteAPIKey + "&field6=" + last_DSTemp1;
            thingspeakSendItemCounter++;
            thingspeakCallCounter++;
            sprintf(printstring2," Sent DS18B20 1 temperature % 4.1f before strong rise  Call#: %ld \n", 
                last_DSTemp1, thingspeakCallCounter);
            strcat(printstring, printstring2);
            logOut(printstring, msgThingspeakSend, msgInfo);

            httpResponseCode = sendThingspeakData(secondary_url);
            sprintf(printstring,"sendThingspeakData() returned: %d repeatFlag: %d \n",
              httpResponseCode, repeatFlag);
            logOut(printstring, msgThingspeakSend, msgInfo);           
          }
        sprintf(printstring2," Tmp1: %5.2f ",temp);
        strcat(printstring, printstring2);
        last_DSTemp1 = temp;
        sum_ThSp_calDS18B20Temperature[1] = 0;
        n_ThSp_calDS18B20Temperature[1] = 0;
        url = url+ "&field6=" + temp;
        thingspeakSendItemCounter++;
        thingspeakItemsCollected++;
      }  
      else{
        sprintf(printstring2," Tmp1: notMeas ");
        strcat(printstring, printstring2);
        sum_ThSp_calDS18B20Temperature[1] = 0;
        n_ThSp_calDS18B20Temperature[1] = 0;
      }

      if(n_ThSp_calDS18B20Temperature[2] > 0)
        temp = sum_ThSp_calDS18B20Temperature[2] / n_ThSp_calDS18B20Temperature[2];
      else
        temp = -111.11;        
      //if( ((temp > limit) && !isEqual(temp,last_DSTemp2,minDiffDS18B20)) || (repeatFlag==true)
      //  || (thingspeakSendItemCounter % minimumRepeatCounter == 0))
      if( 
          ((!isEqual(temp,last_DSTemp2,minDiffDS18B20)) || (repeatFlag==true) || (thingspeakCallCounter % minimumRepeatCounter == 0))
          && (temp > limit)
        )  
      {  
        // insert here if large temp jump: send last_temperature again to avoid unrealistical curve form
        if(!isEqual(temp,last_DSTemp2,minDiffDS18B20*10) && (last_DSTemp2 > -110)) // if jump larger than 10 x minimum recognized temp difference
          {
            secondary_url = ThingspeakServerName + thingspeakWriteAPIKey + "&field7=" + last_DSTemp2;
            thingspeakSendItemCounter++;
            thingspeakCallCounter++;
            sprintf(printstring2," Sent DS18B20 2 temperature % 4.1f before strong rise  Call#: %ld \n", 
                last_DSTemp2, thingspeakCallCounter);
            strcat(printstring, printstring2);
            logOut(printstring, msgThingspeakSend, msgInfo);

            httpResponseCode = sendThingspeakData(secondary_url);
            sprintf(printstring,"sendThingspeakData() returned: %d repeatFlag: %d \n",
              httpResponseCode, repeatFlag);
            logOut(printstring, msgThingspeakSend, msgInfo);           
          }
        sprintf(printstring2," Tmp2: %5.2f",temp);
        strcat(printstring, printstring2);
        last_DSTemp2 = temp;
        sum_ThSp_calDS18B20Temperature[2] = 0;
        n_ThSp_calDS18B20Temperature[2] = 0;
        url = url+ "&field7=" + temp;
        thingspeakSendItemCounter++;
        thingspeakItemsCollected++;
      }  
      else{
        sprintf(printstring2," Tmp2: notMeas ");
        strcat(printstring, printstring2);
        sum_ThSp_calDS18B20Temperature[1] = 0;
        n_ThSp_calDS18B20Temperature[1] = 0;
      }
    #endif // isOneDS18B20  

    // CO2ppm transfer if one of the sensors is present
    #if defined isMHZ14A || defined isSENSEAIR_S8
      if(CO2ppm_n > 0)
          avg = (float)CO2ppm_sum / (float)CO2ppm_n;
        else
          avg = (float)CO2ppm;
        if(!isEqual(avg,last_CO2ppm,minDiffCO2ppm) || (CO2ppm_n > 99) || (repeatFlag==true)){  
          last_CO2ppm = avg;
          sprintf(printstring2," CO2ppm: %5.2f",avg);
          strcat(printstring, printstring2);
          CO2ppm_sum=0; CO2ppm_n=0;
          url = url+ "&field8=" + avg;
          thingspeakSendItemCounter++;
          thingspeakItemsCollected++;
        }  
        else{
          sprintf(printstring2," CO2ppm: notMeas ");
          strcat(printstring, printstring2);
          CO2ppm_sum=0; CO2ppm_n=0;
        }
    #endif

    // external Infactory 433MHz sensors for temp and pressure
    #ifdef receiveSERIAL
      int i;
      for(i=0;i<3;i++)
      {
        sprintf(printstring2,"<<<<< SerialData %5.2f (%5.2f) %5.2f %5.2f (%5.2f) %5.2f\n",
          InfactoryT[0],last_InfactoryT[0],InfactoryH[0], InfactoryT[1], last_InfactoryT[1], InfactoryH[1]);
        logOut(printstring2, msgReceiveSerialInfo, msgInfo);
        switch(i){
          case 0: // attach temperature and humidity for Channel 1 (Index 0)
              if( // check for validity of data
                  (isEqual(InfactoryT[i],last_InfactoryT[i],maxDiffExtTemp)   // jump between act and last data not too large (would be a spike)
                    || last_InfactoryT[i] < -100                              // excemption: last value not yet out of default
                  )  
                  && InfactoryT[i] > -100                                   // act value is out of default (-111.11)
                  && abs(InfactoryT[i]) > 0.01                              // value is above zero: otherwise atoi conversion error when reading likely
                )  
                if( // check for sufficient difference of last and act data, or repeat flag
                  !isEqual(InfactoryT[i],last_InfactoryT[i],minDiffExtTemp) // sufficient difference in data
                    || (repeatFlag==true)                                   // repeat flag, since sending to thingspeak not successful  
                    || (thingspeakCallCounter % minimumRepeatCounter == 0)  // we want a minimum difference, but no wait forever
                  ) 
                  {  
                    sprintf(printstring,"Infactory T Ch1: %5.2f ",InfactoryT[i]);
                    url = url+ "&field7=" + InfactoryT[i];
                    thingspeakSendItemCounter++;
                    thingspeakItemsCollected++;
                    last_InfactoryT[i] = InfactoryT[i]; 
                  }              
            /*  
            if(
                (isEqual(InfactoryT[i],last_InfactoryT[i],maxDiffExtTemp) || last_InfactoryT[i] < -100)    // too large jump (unless not yet out of default): probably a spike
                && (!isEqual(InfactoryT[i],last_InfactoryT[i],minDiffExtTemp) || (repeatFlag==true) || (thingspeakCallCounter % minimumRepeatCounter == 0)) // we want a minimum difference, but no wait forever
                && (InfactoryT[i] > -100) // got out of default (-111.11)
                && (abs(InfactoryT[i]) > 0.01)
              )
            
            {  
              sprintf(printstring,"Infactory T Ch1: %5.2f ",InfactoryT[i]);
              url = url+ "&field7=" + InfactoryT[i];
              thingspeakSendItemCounter++;
              thingspeakItemsCollected++;
              last_InfactoryT[i] = InfactoryT[i]; 
            }
              
            if(
              (isEqual(InfactoryH[i],last_InfactoryH[i],maxDiffExtHumid) || last_InfactoryH[i] < -100)    // too large jump (unless not yet out of default): probably a spike
              && (!isEqual(InfactoryH[i],last_InfactoryH[i],minDiffExtHumid) || (repeatFlag==true) || (thingspeakCallCounter % minimumRepeatCounter == 0))
              && (InfactoryH[i] > -100)
              && (abs(InfactoryH[i]) > 0.01)
            )
            */  
            if( // check for validity of data
                  (isEqual(InfactoryH[i],last_InfactoryH[i],maxDiffExtHumid)   // jump between act and last data not too large (would be a spike)
                    || last_InfactoryH[i] < -100                              // excemption: last value not yet out of default
                  )  
                  && InfactoryH[i] > -100                                   // act value is out of default (-111.11)
                  && abs(InfactoryH[i]) > 0.01                              // value is above zero: otherwise atoi conversion error when reading likely
                )  
                if( // check for sufficient difference of last and act data, or repeat flag
                  !isEqual(InfactoryH[i],last_InfactoryH[i],minDiffExtHumid) // sufficient difference in data
                    || (repeatFlag==true)                                   // repeat flag, since sending to thingspeak not successful  
                    || (thingspeakCallCounter % minimumRepeatCounter == 0)  // we want a minimum difference, but no wait forever
                  ) 

            {  
              sprintf(printstring2," Infactory H Ch1: %5.2f",InfactoryH[i]);
              strcat(printstring, printstring2);
              url = url+ "&field4=" + InfactoryH[i];
              thingspeakSendItemCounter++;
              thingspeakItemsCollected++;
              last_InfactoryH[i] = InfactoryH[i];
            }  
          break;
          case 1: // attach temperature only for Channel 2 (Index 1). Later added: Humidity for channel 2 into field 1 (that is BME Temp if no infactory present)
            if( // check for validity of data
                  (isEqual(InfactoryT[i],last_InfactoryT[i],maxDiffExtTemp)   // jump between act and last data not too large (would be a spike)
                    || last_InfactoryT[i] < -100                              // excemption: last value not yet out of default
                  )  
                  && InfactoryT[i] > -100                                   // act value is out of default (-111.11)
                  && abs(InfactoryT[i]) > 0.01                              // value is above zero: otherwise atoi conversion error when reading likely
              )  
                if( // check for sufficient difference of last and act data, or repeat flag
                  !isEqual(InfactoryT[i],last_InfactoryT[i],minDiffExtTemp) // sufficient difference in data
                    || (repeatFlag==true)                                   // repeat flag, since sending to thingspeak not successful  
                    || (thingspeakCallCounter % minimumRepeatCounter == 0)  // we want a minimum difference, but no wait forever
                  ) 
                  {  
                    sprintf(printstring,"Infactory T Ch2: %5.2f ",InfactoryT[i]);
                    url = url+ "&field8=" + InfactoryT[i];
                    thingspeakSendItemCounter++;
                    thingspeakItemsCollected++;
                    last_InfactoryT[i] = InfactoryT[i]; 
                  }
            /*     
            if(
                (isEqual(InfactoryT[i],last_InfactoryT[i],maxDiffExtTemp) || last_InfactoryT[i] < -100)    // too large jump (unless not yet out of default): probably a spike
                && (!isEqual(InfactoryT[i],last_InfactoryT[i],minDiffExtTemp) || (repeatFlag==true) || (thingspeakCallCounter % minimumRepeatCounter == 0))
                && (InfactoryT[i] > -100)
                && (abs(InfactoryT[i]) > 0.01)
              )
            {  
              sprintf(printstring, "Infactory T Ch2: %5.2f",InfactoryT[i]);
              strcat(printstring, printstring2);
              url = url+ "&field8=" + InfactoryT[i];
              thingspeakSendItemCounter++;
              thingspeakItemsCollected++;
              last_InfactoryT[i] = InfactoryT[i];
            } 
            */
            // humidity 
            if( // check for validity of data
                  (isEqual(InfactoryH[i],last_InfactoryH[i],maxDiffExtHumid)   // jump between act and last data not too large (would be a spike)
                    || last_InfactoryH[i] < -100                              // excemption: last value not yet out of default
                  )  
                  && InfactoryH[i] > -100                                   // act value is out of default (-111.11)
                  && abs(InfactoryH[i]) > 0.01                              // value is above zero: otherwise atoi conversion error when reading likely
              )  
                if( // check for sufficient difference of last and act data, or repeat flag
                  !isEqual(InfactoryH[i],last_InfactoryH[i],minDiffExtHumid) // sufficient difference in data
                    || (repeatFlag==true)                                   // repeat flag, since sending to thingspeak not successful  
                    || (thingspeakCallCounter % minimumRepeatCounter == 0)  // we want a minimum difference, but no wait forever
                  ) 
            {  
              sprintf(printstring2," Infactory H Ch2: %5.2f",InfactoryH[i]);
              strcat(printstring, printstring2);
              url = url+ "&field1=" + InfactoryH[i];
              thingspeakSendItemCounter++;
              thingspeakItemsCollected++;
              last_InfactoryH[i] = InfactoryH[i];
            }  
          break;
          default:
            ;  
        }
        logOut(printstring, msgSerialReceived, msgInfo);
      }
    #endif

    sprintf(printstring2," Sent Item# this time: %d overall: %ld Call#: %ld \n", 
        thingspeakItemsCollected,thingspeakSendItemCounter, thingspeakCallCounter);
    strcat(printstring, printstring2);
    logOut(printstring, msgThingspeakSend, msgInfo);
    // send data in collected string to Thingspeak, but only if data are available
    if(thingspeakItemsCollected > 0){
      httpResponseCode = sendThingspeakData(url);
      if(httpResponseCode == 200) 
        repeatFlag=false;
      else 
        repeatFlag = true;
      sprintf(printstring,"sendThingspeakData() returned: %d repeatFlag: %d \n",
        httpResponseCode, repeatFlag);
      logOut(printstring, msgThingspeakSend, msgInfo);
    }
    else
    {
      sprintf(printstring,"No data to Thingspeak. Item this time: %d Total: %ld Call# %ld\n",
        thingspeakItemsCollected, thingspeakSendItemCounter, thingspeakCallCounter);
      logOut(printstring, msgThingspeakSend, msgInfo);
    }  
  }
#endif // isThingspeak

#ifdef isMQTT
  /**************************************************!
    @brief    MQTT connect / reconned function. 
    @details  New 2022-11-12.
    @details  https://randomnerdtutorials.com/esp32-mqtt-publish-subscribe-arduino-ide/
    @details  API description: https://pubsubclient.knolleary.net/api
    @details  pubsubclient library is used
    @return   void
  ***************************************************/
  void mqttReconnect() 
  { int i=0;
    char topic1[50], topic2[50];
    // Loop until we're reconnected
    while (!mqttClient.connected() && i<2) {
      sprintf(printstring, "Attempting MQTT connection...");
      logOut(printstring, msgMQTTConnect, msgInfo);

      // Attempt to connect
      if (mqttClient.connect(mqttRoomString, mqttDefaultUser, mqttDefaultPaSSWORD)) {
        sprintf(topic1,"esp/%s",mqttRoomString);
        // sprintf(topic1,"esp32/KombiExt");
        sprintf(topic2,"esp/cmnd/%s",mqttRoomString);
        sprintf(printstring, "MQTT connected, subscribed: %s %s\n",topic1, topic2);
        logOut(printstring, msgMQTTSubscribe, msgInfo);
        
        // Subscribe. Multiple topics are possible.
        mqttClient.subscribe("esp32/KombiExt");
        mqttClient.loop();
        mqttClient.subscribe(topic2);
        mqttClient.loop();
      } 
      else 
      {
        Serial.print("failed, rc=");
        Serial.print(mqttClient.state());
        Serial.println(" try again in 2 seconds");
        sprintf(printstring, "MQTT connection failed. Retry in 2 seconds");
        logOut(printstring, msgMQTTError, msgWarn);

        // Wait 2 seconds before retrying
        delay(2000);
      }
      i++;
    } // while
  }

  /**************************************************!
    @brief    MQTT callback function. 
    @details  New 2022-11-12.
    @details  called when a MQTT message arrives, and is parsed here.
    @return   void
  ***************************************************/
  void mqttCallbackFunction(char* topic, byte* message, unsigned int length) 
  {
    sprintf(printstring, "MQTT msg received: [%s] [%s]",topic, message);
    logOut(printstring, msgMQTTReceive, msgInfo);

    Serial.print("Message arrived on topic: ");
    Serial.print(topic);
    Serial.print(". Message: ");
    
    String messageTemp;
    char myTopic [50];
    
    sprintf(myTopic,"esp32/%s",mqttRoomString);
    //sprintf(myTopic,"esp32/RedBoxYeBtn/output");

    for (int i = 0; i < length; i++) {
      Serial.print((char)message[i]);
      messageTemp += (char)message[i];
    }
    Serial.println();

    // If a message is received on the subscribed topics 
    // Do what is needed
    // if (String(topic) == "esp32/output") 
    if(strstr(topic,myTopic))  
    {
      sprintf(printstring,"action: esp32 %s receivedMQTT %s",mqttRoomString, message);
      logOut(printstring, msgMQTTReceive, msgInfo);
    }
  } // mqttCallbackFunction


  /**************************************************!
    @brief    send one DS18B20 Temperature, called from MQTT handler
    @details  New 2022-12-18
    @param int sNo: Number of sensor, 0..2
    @return   void
  ***************************************************/
  void mqttSendDS18B20(int sNo)
  {
    char payloadStr[50];
    char topicStr[50];
    // DS18B20 data 
    double temp;
    double limit = -110.0;
    static double last_DSTemp[MAX_NO_DS18B20] = {-111, -111, -111, -111, -111, -111, -111, -111, -111, -111};
    float time_sec;
    static float last_TimeDSSent[MAX_NO_DS18B20] = {0, 0, 0, 0, 0,  0, 0, 0, 0, 0};

    time_sec = (float)millis()/1000;

    sprintf(printstring,"DS18B20[%d] sum: %f n: %d \n",sNo, sum_MQTT_calDS18B20Temperature[sNo],n_MQTT_calDS18B20Temperature[sNo]);
    logOut(printstring,msgMQTTSendDS10B20, msgInfo);
    if(n_MQTT_calDS18B20Temperature[sNo] > 0)
      temp = sum_MQTT_calDS18B20Temperature[sNo] / n_MQTT_calDS18B20Temperature[sNo];
    else
      temp = -111.11;  
    if( 
        (!isEqual(temp,last_DSTemp[sNo],minDiffDS18B20)   // sufficiently large change
        || (time_sec > last_TimeDSSent[sNo] + 120))           // enough time elapsed
        && (temp > limit)                                 // and valid data
      )
      {    
        // insert here if large temp jump: send last_temperature again to avoid unrealistical curve form
        if(!isEqual(temp,last_DSTemp[sNo],minDiffDS18B20*10) && (last_DSTemp[sNo] > -110)) // if jump larger than 10 x minimum recognized temp difference
        {
          sprintf(topicStr,"esp32/%s/%s/%s%d",mqttRoomString, mqttSensorDS18B20, mqttDS18B20Temperature, sNo+1);
          printf(payloadStr,"%3.2f",last_DSTemp[sNo]);
          // caller! mqttSendItemCounter++;
          sprintf(printstring,"Strings to MQTT: [%s] [%s]\n", topicStr, payloadStr);
          logOut(printstring, msgMQTTSendDS10B20, msgInfo);
          mqttClient.publish(topicStr, payloadStr);

          sprintf(printstring2," MQTT Sent DS18B20 %d temperature % 4.1f before strong rise \n", sNo, last_DSTemp[sNo]);
          strcat(printstring, printstring2);
          logOut(printstring, msgMQTTSendDS10B20, msgInfo);          
        }
        sprintf(printstring2," Tmp%d: toMQTT cal: %5.2f last: %5.2f act: %5.2f sum: %5.2f n: %d\n",
          sNo, calDS18B20Temperature[0], last_DSTemp[sNo], temp, sum_MQTT_calDS18B20Temperature[0], n_MQTT_calDS18B20Temperature[0]);
        strcat(printstring, printstring2);
        logOut(printstring, msgMQTTSendDS10B20, msgInfo);

        last_DSTemp[sNo] = temp;
        sum_MQTT_calDS18B20Temperature[sNo] = 0;
        n_MQTT_calDS18B20Temperature[sNo] = 0;

        sprintf(topicStr,"esp32/%s/%s/%s%d",mqttRoomString, mqttSensorDS18B20, mqttDS18B20Temperature,sNo+1);
        sprintf(payloadStr,"%3.2f",temp);
        last_TimeDSSent[sNo] = time_sec; 
        // caller! mqttSendItemCounter++;
        sprintf(printstring,"Strings to MQTT: [%s] [%s] at time %5.2f\n", topicStr, payloadStr, time_sec);
        logOut(printstring, msgMQTTSendDS10B20, msgInfo);
        mqttClient.publish(topicStr, payloadStr);
      }  
      else{
        sprintf(printstring2,"\n Tmp%d: notMQTT cal: %5.2f last: %5.2f act: %5.2f sum: %5.2f n: %d last sent:%5.3f[s] now:%5.3f[s]",
          sNo,calDS18B20Temperature[0], last_DSTemp[sNo], temp, sum_MQTT_calDS18B20Temperature[0], n_MQTT_calDS18B20Temperature[0], last_DSTemp[sNo], time_sec);
        strcat(printstring, printstring2);
        logOut(printstring, msgMQTTSendDS10B20, msgWarn);
        sum_MQTT_calDS18B20Temperature[sNo] = 0;
        n_MQTT_calDS18B20Temperature[sNo] = 0;
      }
  }

  /**************************************************!
    @brief    MQTT handler, handles sending of data to MQTT broker, called via timer
    @details  New 2022-11-12.
    @details  called via timer, does measurements and output for all sensors actually present
    @details  uses global variables:
    @details  char msg[50];
    @details  int value = 0;
    @return   void
  ***************************************************/
  void mqttHandler()
  {
    char payloadStr[50];
    char topicStr[50];

    double limit = -110.0;
    static double last_DSTemp0=-111, last_DSTemp1=-111, last_DSTemp2=-111;
    static long mqttSendItemCounter = 0, mqttCallCounter = 0;
    int i;

    mqttCallCounter++;      // counter for how often this function has been called
    mqttSendItemCounter = 0; // counter for the number of items to be sent during this call

    int tst = mqttClient.connected();
    if (!tst)
    {
      sprintf(printstring,"MQTT client not connected, return: %d State: %d\n",tst, mqttClient.state());
      logOut(printstring, msgMQTTConnect, msgWarn);
      mqttReconnect();
    }  
    else{
      sprintf(printstring,"MQTT client is still connected, return: %d\n",tst);
      logOut(printstring, msgMQTTConnect, msgWarn);
    }
    sprintf(printstring,"MQTT client after attempt to connect, State: %d\n", mqttClient.state());
    logOut(printstring, msgMQTTConnect, msgWarn);
    
    if (mqttClient.connected()) 
    {
      //mqttClient.loop();
      #if defined isBME280 || defined isBME680 || defined isBME680_BSECLib
        sprintf(topicStr,"esp32/%s/%s/%s",mqttRoomString, mqttSensorBME280, mqttBMETemperature);
        sprintf(payloadStr,"%3.2f",Temperature);
        mqttSendItemCounter++;
        sprintf(printstring,"Strings to MQTT: [%s] [%s]\n", topicStr, payloadStr);
        logOut(printstring, msgMQTTInfo, msgInfo);
        mqttClient.publish(topicStr, payloadStr);

        sprintf(topicStr,"esp32/%s/%s/%s",mqttRoomString, mqttSensorBME280, mqttBMEPressure);
        sprintf(payloadStr,"%3.2f",Pressure);
        mqttSendItemCounter++;
        sprintf(printstring,"Strings to MQTT: [%s] [%s]\n", topicStr, payloadStr);
        logOut(printstring, msgMQTTInfo, msgInfo);
        mqttClient.publish(topicStr, payloadStr);

        sprintf(topicStr,"esp32/%s/%s/%s",mqttRoomString, mqttSensorBME280, mqttBMEHumidity);
        sprintf(payloadStr,"%3.2f",Humidity);
        mqttSendItemCounter++;
        sprintf(printstring,"Strings to MQTT: [%s] [%s]\n", topicStr, payloadStr);
        logOut(printstring, msgMQTTInfo, msgInfo);
        mqttClient.publish(topicStr, payloadStr);
      
      #endif // isBME280 || is BME680

      #if defined isBME680 || defined isBME680_BSECLib
        sprintf(topicStr,"esp32/%s/%s/%s",mqttRoomString, mqttSensorBME680, mqttBMEAirQuality);
        sprintf(payloadStr,"%3.2f",air_quality_score);
        mqttSendItemCounter++;
        sprintf(printstring,"Strings to MQTT: [%s] [%s]\n", topicStr, payloadStr);
        logOut(printstring, msgMQTTInfo, msgInfo);
        mqttClient.publish(topicStr, payloadStr);
      #endif // isBME680  || isBME680_BSECLib

      #ifdef isOneDS18B20
        // DS18B20 data 
        for(i = 0; i < noDS18B20Connected; i++)
          mqttSendDS18B20(i);
      #endif // isOneDS18B20      

      #if defined isMHZ14A || defined isSENSEAIR_S8
        #ifdef isMHZ14A
          sprintf(topicStr,"esp32/%s/%s/%s",mqttRoomString, mqttSensorMHZ14A, mqttCO2ppm);
        #endif
        #ifdef isSENSEAIR_S8
          sprintf(topicStr,"esp32/%s/%s/%s",mqttRoomString, mqttSensorSenseAirS8, mqttCO2ppm);
        #endif  
        sprintf(payloadStr,"%d",CO2ppm);
        mqttSendItemCounter++;
        sprintf(printstring,"Strings to MQTT: [%s] [%s]\n", topicStr, payloadStr);
        logOut(printstring, msgMQTTSend, msgInfo);
        mqttClient.publish(topicStr, payloadStr);
      #endif // defined isMHZ14A || defined isSENSEAIR_S8

      #ifdef receiveSERIAL
        sprintf(topicStr,"esp32/%s/%s/%s",mqttRoomString, mqttSensorInfactory433, mqttInfactory433Temperature1);
        sprintf(payloadStr,"%3.1f",InfactoryT[0]);
        mqttSendItemCounter++;
        sprintf(printstring,"Strings to MQTT: [%s] [%s]\n", topicStr, payloadStr);
        logOut(printstring, msgMQTTSend, msgInfo);
        mqttClient.publish(topicStr, payloadStr);      

        sprintf(topicStr,"esp32/%s/%s/%s",mqttRoomString, mqttSensorInfactory433, mqttInfactory433Humidity1);
        sprintf(payloadStr,"%3.1f",InfactoryH[0]);
        mqttSendItemCounter++;
        sprintf(printstring,"Strings to MQTT: [%s] [%s]\n", topicStr, payloadStr);
        logOut(printstring, msgMQTTSend, msgInfo);
        mqttClient.publish(topicStr, payloadStr);    

        sprintf(topicStr,"esp32/%s/%s/%s",mqttRoomString, mqttSensorInfactory433, mqttInfactory433Temperature2);
        sprintf(payloadStr,"%3.1f",InfactoryT[1]);
        mqttSendItemCounter++;
        sprintf(printstring,"Strings to MQTT: [%s] [%s]\n", topicStr, payloadStr);
        logOut(printstring, msgMQTTSend, msgInfo);
        mqttClient.publish(topicStr, payloadStr);      

        sprintf(topicStr,"esp32/%s/%s/%s",mqttRoomString, mqttSensorInfactory433, mqttInfactory433Humidity2);
        sprintf(payloadStr,"%3.1f",InfactoryH[1]);
        mqttSendItemCounter++;
        sprintf(printstring,"Strings to MQTT: [%s] [%s]\n", topicStr, payloadStr);
        logOut(printstring, msgMQTTSend, msgInfo);
        mqttClient.publish(topicStr, payloadStr);    
        
      #endif // receiveSERIAL

    } // if mqttClient connected
  } // mqttHandler
#endif // isMQTT

/**************************************************!
  @brief    main handler, was loop. Handles all sensors, called via timer
  @details  called via timer, does measurements and output for all sensors actually present
  @return   void
***************************************************/
void main_handler() 
{
  // char printstring[180]="";
  // char printstring1[80]="", printstring2[80]="", printstring3[80]="";
  float time_sec;

  // float time_sec;
  long start_loop_time, end_loop_time = 0;
  int i; // for loops
  
  #if defined  isBME680  || defined isBME680_BSECLib
    static long lastBME680Time;
  #endif 

  time_sec = (float)millis()/1000;

  start_loop_time = millis();
  // Serial.printf(" ******* Main Loop start at %3.1f sec ********** \n",time_sec);
  sprintf(printstring, " M %3.1f ",time_sec);
  // logOut(printstring, msgDefaultID, msgInfo);

  // transfer Wifi signal strengths, for diagnostics
  #ifdef isMeasureRSSI
    if(WiFi.status() == WL_CONNECTED)
    {
       rssi = WiFi.RSSI();
       rssi_sum += rssi;
       rssi_n++;
       #ifdef isBLYNK
        if(Blynk.connected())
          sprintf(printstring,"%s Blynk connected ", printstring);
        else   
          sprintf(printstring,"%s Blynk NOT connected ", printstring);
        Blynk.virtualWrite(V1, rssi);
        // Blynk.syncAll();
        // delay(25);
        // Blynk.run();  

        sprintf(printstring,"%s RSSI: %3.0f\n ",printstring,rssi);
        // sprintf(printstring,"%s toBlynk: %d\n ",printstring,rssi);
      #endif
    }  
    logOut(printstring, msgWiFiRssiInfo, msgInfo);
  #endif // isMeasureRSSI

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
      logOut(printstring, msgInfactoryInfo, msgInfo);
      sprintf(printstring,"TempC %3.1f Humidity %3.1f %%  Channel %d:\n", 
          InfactoryTempC, InfactoryHumidity, InfactoryChannel);
      logOut(printstring, msgInfactoryInfo, msgInfo);
      trialsInfactory++;
      if(InfactoryTempC > -110 && InfactoryHumidity > -110) 
        successInfactory++;
      sprintf(printstring,"Infactory Readings %d of %d attempts\n", successInfactory, trialsInfactory);  
      logOut(printstring, msgInfactoryInfo, msgInfo);

      // restart Blynk
      #ifdef isBLYNK
        esp_task_wdt_reset(); // reset watchdog in case it takes longer
        // testxxxxx Blynk.begin(auth, ssid, pass, IPAddress(blynkLocalIP), 8080);
        Blynk.connect();
        if((InfactoryTempC > -110) && successInfactoryCalc)
        {         
          Blynk.virtualWrite(V15, InfactoryTempC); //sending to Blynk, if other than the default -111.11
          sprintf(printstring,"Main >>>>>> Sending TempC %3.1f ", InfactoryTempC);
          logOut(printstring, msgInfactoryInfo, msgInfo);
          Blynk.virtualWrite(V16, InfactoryHumidity); //sending to Blynk
          sprintf(printstring,">>>>>> Sending Humidity %3.1f \n", InfactoryHumidity);
          logOut(printstring, msgInfactoryInfo, msgInfo);
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
            logOut(printstring, msgInfactoryReadNVS, msgInfo);
            Blynk.virtualWrite(V16, InfactoryHumidity); //sending to Blynk
            sprintf(printstring,">->->->->-> Sending Humidity %3.1f \n", InfactoryHumidity);
            logOut(printstring, msgInfactoryReadNVS, msgInfo);
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
      logOut(printstring, msgSerialReceived, msgInfo);  
    }
    else
    {
      if(charsReceived > 0)
      {  
        sprintf(printstring,"faulty data received (chars: %d), Total Count: %d\n",charsReceived,serialFailCount);
        logOut(printstring, msgSerialReceived, msgWarn);  
      }  
    }

    // EXPDis #ifdef isBLYNK
      if(serialReceived)
      {         
        if(lastInfactoryTempC[serialChannel]<-110) 
          lastInfactoryTempC[serialChannel] = InfactoryTempC - 4.5; // get out of default
        // too large difference-improbable value.
        // 0.0 and -111.11 are errors
        if( (abs(lastInfactoryTempC[serialChannel]-InfactoryTempC) < 5.0) && InfactoryTempC>-110 && abs(InfactoryTempC) > 0.01 )  
        {
          // restart Blynk
          esp_task_wdt_reset(); // reset watchdog in case it takes longer
          #ifdef isBLYNK
            // testxxxxx Blynk.begin(auth, ssid, pass, IPAddress(blynkLocalIP), 8080);
            Blynk.connect();
            sprintf(printstring,"Blynk Connected, serialReceived: %d \n",serialReceived);
            logOut(printstring, msgBlynkConnected, msgInfo);  
            checkBlynk();   // test to ensure that blynk is running
          #endif  

          do{
            #ifdef isBLYNK
              Blynk.run(); 
              Blynk.virtualWrite(V38, serialFailCount);
              Blynk.run();
              // Blynk.virtualWrite(V38, serialFailCount);
            #endif   
            sprintf(printstring,">>>>>> Send TempC %3.1f >>>> Humidity %3.1f Ch: %d (fail: %d)\n",
                InfactoryTempC, InfactoryHumidity, serialChannel,serialFailCount);
            logOut(printstring, msgReceiveSerialInfo, msgInfo); 
            // store the channel related data for specialDisplay() and for sending via Thingspeak;
            InfactoryT[serialChannel] = InfactoryTempC;
            InfactoryH[serialChannel] = InfactoryHumidity;
            #ifdef isBLYNK
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
            #endif  // isBLYNK 
            esp_task_wdt_reset(); // reset watchdog in case it takes longer
            #ifdef isBLYNK
              Blynk.run();
              vTaskDelay(200 / portTICK_PERIOD_MS); // wait 200ms
            #endif  
            lastInfactoryTempC[serialChannel] = InfactoryTempC;
            serialReceived = processSerialData(&InfactoryTempC, &InfactoryHumidity, &serialChannel, &charsReceived);
          } while(serialReceived);  
          #ifdef isBLYNK
            Blynk.run();
          #endif  

          #ifdef isBLYNK
            // send data from other sensors before disconnecting Blynk
            #ifdef isOneDS18B20
              if((calDS18B20Temperature[0]) > (-110.0))
                Blynk.virtualWrite(V13, calDS18B20Temperature[0]); //sending to Blynk
              if((calDS18B20Temperature[1]) > (-110.0))  
                Blynk.virtualWrite(V10, calDS18B20Temperature[1]); //sending to Blynk
              if((calDS18B20Temperature[2]) > (-110.0))  
                Blynk.virtualWrite(V11, calDS18B20Temperature[2]); //sending to Blynk  
              Blynk.run();  
              sprintf(printstring,">>>>> Ds18B20 Temp's: T1: %3.1f T2: %3.1f T3: %3.1f\n",calDS18B20Temperature[0],calDS18B20Temperature[1],calDS18B20Temperature[2]);
              logOut(printstring, msgReceiveSerialInfo, msgInfo);  
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
                logOut(printstring, msgReceiveSerialInfo, msgInfo);    
                Blynk.run();  
              }  
            #endif
            vTaskDelay(500 / portTICK_PERIOD_MS); // wait 500ms before disconnecting
            Blynk.disconnect();
          #endif //  isBLYNK 
        }
        else
        {
          //if( (abs(lastInfactoryTempC[serialChannel]-InfactoryTempC) < 5.0) && InfactoryTempC>-110 && abs(InfactoryTempC) > 0.01 )
          sprintf(printstring,"##### Serial data rejected: Ch: %d TempC: %3.1f lastTempC: %3.1f\n",serialChannel, InfactoryTempC, lastInfactoryTempC[serialChannel]);
          logOut(printstring, msgBlynkConnected, msgInfo);  
          lastInfactoryTempC[serialChannel] = InfactoryTempC; // needs to be updated also in "else" case, otherwise one outlier stops all new data.
          InfactoryT[serialChannel] = -111.11;
          InfactoryH[serialChannel] = -111.11;
        }
      }  
    // EXPDis #endif  // isBLYNK
  #endif  //receiveSERIAL

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
    /*  
    sprintf(printstring,"BaseDS18B20 Tmp1 %3.2f Tmp2 %3.2f Tmp3 %3.2f %d %d %d - %d %ld\n", 
      DS18B20Temperature[0], DS18B20Temperature[1], DS18B20Temperature[2], 
      notMeasuredCount, notChangedCount, noDS18B20Restarts, state, GetOneDS18B20Counter);
     logOut(printstring, msgDS18B20Info, msgInfo);
    */ 
     
    /*
    sprintf(printstring,"CalDS18B20 Tmp1  %3.2f Tmp2 %3.2f Tmp3 %3.2f\n", 
      calDS18B20Temperature[0], calDS18B20Temperature[1], calDS18B20Temperature[2]);
    logOut(printstring, msgDS18B20Info, msgInfo);
    sprintf(printstring,"%d %d %d - %d %ld\n", 
      notMeasuredCount, notChangedCount, noDS18B20Restarts, state, GetOneDS18B20Counter);
    logOut(printstring, msgDS18B20Info, msgInfo);
    */
    if(GetOneDS18B20Counter <= previousGetOneDS18B20Counter)  // DS18B20 routine not counting
    {
      notMeasuredDS18B20 ++;
      sprintf(printstring,"!!!! DS18B20 not measuring !!! %ld %ld %ld \n",
        GetOneDS18B20Counter, previousGetOneDS18B20Counter, notMeasuredDS18B20);
      logOut(printstring, msgDS18B20NotMeasuring, msgWarn);  
      vTaskDelay(notMeasuredDS18B20 * 1000 / portTICK_PERIOD_MS); // progressive delay to give the measuring routine more time
    }  
    else
      notMeasuredDS18B20 = 0;
    //if (notMeasuredDS18B20 > 5)
    //  restartDS18B20MeasurementFunction();

    previousGetOneDS18B20Counter = GetOneDS18B20Counter;

  //  sprintf(printstring,"Cal.DS18B20 Tmp1 %3.1f Tmp2 %3.1f Tmp3 %3.1f \n", 
  //     calDS18B20Temperature[0], calDS18B20Temperature[1], calDS18B20Temperature[2]);
  //  logOut(printstring, msgDS18B20Info, msgInfo);
    
    double limit = -110.0; // EXPDis
    #ifdef isBLYNK

      // EXPDis double limit = -110.0;
      sprintf(printstring,"ToBlynk: ");
      if((calDS18B20Temperature[0]) > (limit)){
        Blynk.virtualWrite(V13, calDS18B20Temperature[0]); //sending to Blynk
        sprintf(printstring2,"Tmp1: %5.2f ",calDS18B20Temperature[0]);
        strcat(printstring, printstring2);
      }  
      else{
        sprintf(printstring2,"Tmp1: notMeas ");
        strcat(printstring, printstring2);
      }
      if((calDS18B20Temperature[1]) > (limit)){  
        Blynk.virtualWrite(V10, calDS18B20Temperature[1]); //sending to Blynk
        sprintf(printstring2,"Tmp2: %5.2f ",calDS18B20Temperature[1]);
        strcat(printstring, printstring2);
      }  
      else{
        sprintf(printstring2,"Tmp2: notMeas ");
        strcat(printstring, printstring2);
      }
      if((calDS18B20Temperature[2]) > (limit)){  
        Blynk.virtualWrite(V11, calDS18B20Temperature[2]); //sending to Blynk  
        sprintf(printstring2,"Tmp3: %5.2f",calDS18B20Temperature[2]);
        strcat(printstring, printstring2);
      }  
      else{
        sprintf(printstring2,"Tmp3: notMeas ");
        strcat(printstring, printstring2);
      }
      strcat(printstring,"\n");
      logOut(printstring, msgBlynkSend, msgInfo);
      
      vTaskDelay(300 / portTICK_PERIOD_MS); // non-blocking delay instead
    #endif  

    #ifdef isVirtuino
      double limit = -110.0;
      sprintf(printstring,"ToVirtuino: ");
      if((calDS18B20Temperature[0]) > (limit)){
        V[13] = calDS18B20Temperature[0]; //sending to Virtuino
        sprintf(printstring2,"Tmp1: %5.2f ",calDS18B20Temperature[0]);
        strcat(printstring, printstring2);
      }  
      else{
        sprintf(printstring2,"Tmp1: notMeas ");
        strcat(printstring, printstring2);
      }
      if((calDS18B20Temperature[1]) > (limit)){  
        V[10] = calDS18B20Temperature[1]; //sending to Virtuino
        sprintf(printstring2,"Tmp2: %5.2f ",calDS18B20Temperature[1]);
        strcat(printstring, printstring2);
      }  
      else{
        sprintf(printstring2,"Tmp2: notMeas ");
        strcat(printstring, printstring2);
      }
      if((calDS18B20Temperature[2]) > (limit)){  
        V[11] = calDS18B20Temperature[2]; //sending to Virtuino  
        sprintf(printstring2,"Tmp3: %5.2f",calDS18B20Temperature[2]);
        strcat(printstring, printstring2);
      }  
      else{
        sprintf(printstring2,"Tmp3: notMeas ");
        strcat(printstring, printstring2);
      }
      strcat(printstring,"\n");
      logOut(printstring, msgVirtuinoSend, msgInfo);
      vTaskDelay(300 / portTICK_PERIOD_MS); // non-blocking delay 
    #endif  

    #ifdef isThingspeak
       if((calDS18B20Temperature[0]) > (limit)){
         sum_ThSp_calDS18B20Temperature[0] += calDS18B20Temperature[0];
         n_ThSp_calDS18B20Temperature[0] += 1;
       }  
       if((calDS18B20Temperature[1]) > (limit)){
         sum_ThSp_calDS18B20Temperature[1] += calDS18B20Temperature[1];
         n_ThSp_calDS18B20Temperature[1] += 1;
       }  
       if((calDS18B20Temperature[2]) > (limit)){
         sum_ThSp_calDS18B20Temperature[2] += calDS18B20Temperature[2];
         n_ThSp_calDS18B20Temperature[2] += 1;
       }  
    #endif

    #ifdef isMQTT
      for(i=0;i<3;i++)
      {
       if((calDS18B20Temperature[i]) > (limit)){
         sum_MQTT_calDS18B20Temperature[i] += calDS18B20Temperature[i];
         n_MQTT_calDS18B20Temperature[i] += 1;
       }
      } 
    #endif

    // checks for problems with measurements of DS18B20
    if(notMeasuredCount > DS18B20RestartLimit || notChangedCount > 3*DS18B20RestartLimit || // in GetOneDS18B20Temperature this count is handled if faulty checksum
      manualDS18B20Restart >= 1 ||  // manual restart via Blynk app
      notMeasuredDS18B20 > 5)       // in GetOneDS18B20Temperature this count isf function is running
    {
      sprintf(printstring,"\nRestarting DS18B20. No measurement taken in %d %d cycles - counter not incr: %ld Manual: %d\n",
        notMeasuredCount, notChangedCount, notMeasuredDS18B20, manualDS18B20Restart);
      logOut(printstring, msgDS18B20Restart, msgWarn);
      restartDS18B20MeasurementFunction();
      notMeasuredCount = 0; // reset the counter
      notChangedCount = 0;  // reset the counter
      manualDS18B20Restart = 0; // reset the manual switch
      notMeasuredDS18B20 = 0; // reset the counter for activity of the detached, parallel measurement function
    }
  #endif  // isOneDS18B20

  //*** get sensor data from BME680 P/T/%/Gas sensor using Zanshin Library
  #ifdef isBME680
    // measure only every 60 seconds
    if(start_loop_time > lastBME680Time + 60000)
    {
      lastBME680Time = start_loop_time;
      getBME680SensorData();
      // correct with compensation factors which are specific to each sensor module, defined near auth codes
      temperature += corrBMETemp;
      pressure += corrBMEPressure; 
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
      vTaskDelay(100 / portTICK_PERIOD_MS); // non-blocking delay instead
    } // if start_loop_time  
  #endif  // isBME680

  //*** get sensor data from BME680 P/T/%/Gas sensor using BSEC Library
  #ifdef isBME680_BSECLib
    // measure only every 60 seconds
    if(start_loop_time > lastBME680Time + 1000)   // was 60000. now essentially disabled
    {
      lastBME680Time = start_loop_time;
      /*
      getBME680SensorData();
      // correct with compensation factors which are specific to each sensor module, defined near auth codes
      temperature += corrBMETemp;
      pressure += corrBMEPressure; 
      getAirQuality();    // get air quality index
      Serial.printf(" %3.1f Temp %3.1f Humid %3.1f Pressure %3.1f Gas %5.0f (Alt %3.1f) Air %f %s \n", 
                    time_sec, temperature, humidity, pressure, gas, altitude, air_quality_score, air_quality_string);
      */
      char iaqString[10];
      if (iaqSensor.run()) { // If new data is available
        switch(iaqSensor.iaqAccuracy)
        {
          case 0:
            sprintf(iaqString,"Start");
            break;
          case 1:
            sprintf(iaqString,"Uncrt");
            break;
          case 2:
            sprintf(iaqString,"CalON");
            break;
          case 3:
            sprintf(iaqString,"CalOK");
            break;
          default:
            sprintf(iaqString,"-----");
            break;
        }
        sprintf(printstring,"t:%3.1f rT:%3.2f P:%3.0f rH:%3.2f R:%3.1f IAQ %5.1f (%d %s) T:%3.2f h:%3.1f\n",
            time_sec,iaqSensor.rawTemperature,iaqSensor.pressure, iaqSensor.rawHumidity, 
            iaqSensor.gasResistance, iaqSensor.iaq, iaqSensor.iaqAccuracy, iaqString, iaqSensor.temperature, iaqSensor.humidity);
        logOut(printstring, msgBME680Info, msgInfo);  
        // sprintf(printstring,"BME680 power toggles: %d\n", countBME680PowerToggles);
        // logOut(printstring, msgBME680Info, msgInfo);  
        sprintf(printstring," ||");
        for (int i=1;i<iaqSensor.iaq/10;i++)
          strcat(printstring,"=");
        strcat(printstring,"\n");  
        logOut(printstring, msgBME680Info, msgInfo);  

        updateBsecState();    // write sensor date to preferences, if time
      } else {

          int iaqSensorStatus = iaqSensor.status;
          int iaqBME680Status = iaqSensor.bme680Status;
          checkIaqSensorStatus(); // may reset iaqSensor.status

          if(iaqSensorStatus != BSEC_OK || iaqBME680Status != BSEC_OK) 
            resetBME680(iaqSensor.status, iaqBME680Status);
      }

      // get the data, and collect the required sums for averaging 
      temperature = iaqSensor.temperature;    // sensor temperature in C
      temperature_sum += temperature;         // for averaging in thingspeak
      temperature_n++;                        // for averaging in thingspeak

      pressure    = (iaqSensor.pressure / 100) + corrBMEPressure; // sensor air pressure in mbar
      pressure_sum += pressure;               // for averaging in thingspeak
      pressure_n++;                           // for averaging in thingspeak

      humidity    = iaqSensor.humidity;       // humidity in %
      humidity_sum += humidity;               // for averaging in thingspeak
      humidity_n++;                           // for averaging in thingspeak

      air_quality_score = iaqSensor.iaq;      // air quality score. here 0(perfect)..500(worst)
      air_quality_score_sum += air_quality_score; // for averaging in thingspeak
      air_quality_score_n++;                  // for averaging in thingspeak
      
      if(air_quality_score <= 50){
        strcpy(air_quality_string," Air Quality is good");
        strcpy(air_quality_shortstring,"Good");
      }
      if(air_quality_score > 50 && air_quality_score <= 100){
        strcpy(air_quality_string," Air quality is moderate");
        strcpy(air_quality_shortstring,"Moderate");
      }
      if(air_quality_score > 100 && air_quality_score <= 150){
        strcpy(air_quality_string," AQ unhealthy for Sensitive Groups");
        strcpy(air_quality_shortstring,"UnhSensi");
      }
      if(air_quality_score > 150 && air_quality_score <= 200){
        strcpy(air_quality_string," AQ is unhealthy");
        strcpy(air_quality_shortstring,"Unhelthy");
      }
      if(air_quality_score > 200 && air_quality_score <= 300){
        strcpy(air_quality_string," AQ is very unhealthy");
        strcpy(air_quality_shortstring,"VeryUnhy");
      }
      if(air_quality_score > 300 && air_quality_score <= 500){
        strcpy(air_quality_string," Air quality is hazardous!");
        strcpy(air_quality_shortstring,"Hazard!");
      }

      #ifdef isBLYNK
        // get the data to blynk
        Blynk.virtualWrite(V5, temperature); 
        Blynk.virtualWrite(V6, pressure); 
        Blynk.virtualWrite(V7, humidity); 
        Blynk.virtualWrite(V8, air_quality_score); 
        Blynk.virtualWrite(V12, air_quality_string); 
      #endif

      #ifdef isVirtuino
        // get the data to Virtuino
        V[5] = temperature; 
        V[6] = pressure; 
        V[7] = humidity; 
        V[8] = air_quality_score; 
        // V[12] = air_quality_string; 
      #endif

      // 26.01.24: copy to proper global variables to transfer to MQTT as well
      Temperature = temperature;
      Pressure = pressure;
      Humidity = humidity;
    
      // delay(300); // give blynk time to send the stuff
      vTaskDelay(100 / portTICK_PERIOD_MS); // non-blocking delay instead
    } // if start_loop_time  
  #endif  // isBME680_BSECLib

  #ifdef isBME280
    getBME280SensorData();
    // correct with compensation factors which are specific to each sensor module, defined near auth codes
    Temperature += corrBMETemp;
    Pressure += corrBMEPressure; 
    //sprintf(printstring,"BME280 Sensor values: %3.1f °C %3.1f %% %3.1f mBar %3.1f m\n", 
    //    Temperature, Humidity, Pressure, Altitude); 
    // logOut(printstring, msgBME280Info, msgInfo);    

    // get the data, and collect the required sums for averaging for Thingspeak
    // sprintf(printstring,"test1\n");
    // logOut(printstring, msgMQTTState, msgInfo);
    
    #ifdef isThingspeak
      temperature = Temperature;    // sensor temperature in C
      temperature_sum += temperature;         // for averaging in thingspeak
      temperature_n++;                        // for averaging in thingspeak

      pressure    = Pressure;                 // sensor air pressure in mbar
      pressure_sum += pressure;               // for averaging in thingspeak
      pressure_n++;                           // for averaging in thingspeak

      humidity    = Humidity;       // humidity in %
      humidity_sum += humidity;               // for averaging in thingspeak
      humidity_n++;                           // for averaging in thingspeak
    #endif // Thingspeak  

    #ifdef isBLYNK
      // get the data to blynk
      Blynk.virtualWrite(V5, Temperature); 
      Blynk.virtualWrite(V6, Pressure); 
      Blynk.virtualWrite(V7, Humidity); 
      // Blynk.virtualWrite(V8, Altitude); 
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
        sprintf(printstring,"--------- %3.1f MH-Z14A CO2 Sensor value: %d PPM \n", time_sec, CO2ppm);
        logOut(printstring, msgMHZ14aInfo, msgInfo); 
        timer1 = millis();
        CO2ppm_n ++;
        CO2ppm_sum += CO2ppm;
      } 
      #ifdef isBLYNK 
        Blynk.virtualWrite(V9, CO2ppm);
      #endif  
      sprintf(printstring,"%3.1f [sec] ", time_sec);
      logOut(printstring, msgMHZ14aInfo, msgInfo); 
    }
    else{
      sprintf(printstring," CO2 Sensor still warming up %3.1f of %3.1f\n", (float)MHZ14AWarmingTime, (float)MZH14AWarmupWait);
      logOut(printstring, msgMHZ14aWarmup, msgInfo); 
    }  
  #endif  // isMHZ14A

  #ifdef isSENSEAIR_S8
    if(senseAirPresentFlag)
    {
      send_Request(CO2req, 8);               // send request for CO2-Data to the Sensor
      read_Response(7);                      // receive the response from the Sensor
      CO2ppm = get_Value(7);
      if(CO2ppm > 32767)                     // can overflow, needs to be considered
        CO2ppm = CO2ppm-65535;               // signed int, overflow converted to negative
      sprintf(printstring,"\n %3.1f SenseAir CO2 Sensor value: %d PPM \n", time_sec, CO2ppm);
      logOut(printstring, msgSenseAirInfo, msgInfo); 
      #ifdef isBLYNK 
        Blynk.virtualWrite(V9, CO2ppm);
      #endif  // Blynk
    }
    else
      CO2ppm = -1;
  #endif    // SENSEAIR

  //sprintf(printstring,"test4\n");
  //logOut(printstring, msgMQTTState, msgInfo);
  #ifdef isMQTT
    sprintf(printstring,"before mqtt client loop\n");
    logOut(printstring, msgMQTTState, msgInfo);
    mqttClient.loop(); // ensure that callback function for mqtt is handled
    int tst = mqttClient.state();
    sprintf(printstring,"MQTT client state: %d\n",tst);
    logOut(printstring, msgMQTTState, msgInfo);
    if(tst!=MQTT_CONNECTED)
      mqttReconnect();
    mqttClient.loop(); // ensure that callback function for mqtt is handled  
  #endif

  sprintf(printstring,"%ld %ld",start_loop_time, end_loop_time);
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

#ifdef isLCD
/**************************************************!
  @brief    Function to handle LCD display data for various sensors on LCD display
  @details  called via timer, outputs data based on lcdDisplayMode, and for sensors actually present
  @return   void
  ***************************************************/
void lcd_handler()
{
  if(lcdDisplayMode == 0)
  {
    //sprintf(printstring,"displayLCD Mode  is zero: %d\n", lcdDisplayMode);
    //logOut(printstring, msgLCDInfo, msgInfo);
    lcd.clear();
    lcd.noBacklight();
    lcdDisplayDone = 1;   // makes ready to receive button press again
  }
  if(lcdDisplayMode > 0)
  {
    if(lcdDisplayMode==1 && lcdDisplayDone == 0) // button press just registered, predisplayOffTimerIntervalv. mode 0 (=off), no action taken yet
      initLCD();  // then initialize the display

    //sprintf(printstring,"displayLCD Mode %d\n", lcdDisplayMode);
    //logOut(printstring, , msgLCDInfo, msgInfo);
    printLocalTime(printstring, 6); // local time into printstring for display on LCD
    displayLCD(lcdDisplayMode, 
      Pressure, Temperature, Humidity, 
      calDS18B20Temperature[0], calDS18B20Temperature[1], calDS18B20Temperature[2],
      1234, // CO2ppm,
      InfactoryT[0], InfactoryH[0],InfactoryT[1], InfactoryH[1],
      printstring, infoStringShort
    );
    vTaskDelay(300 / portTICK_PERIOD_MS); // delay for 200 ms, to avoid double button presses
    lcdDisplayDone = 1; // makes ready to receive button press again
  }  
}
#endif //#ifdef isLCD

#ifdef isDisplay
  /**************************************************!
  @brief    Function to handle OLED display data for various sensors on OLED display
  @details  called via timer, outputs data based on displayMode, and for sensors actually present
  @return   void
  ***************************************************/
  void oled_handler()
  {
    // clear the display - only once!
    display.clearDisplay(); 

    if(displayMode == 0)  // clear display in mode 0
      display.clearDisplay(); 

    if(displayMode == 1)
    {
      #ifdef isOneDS18B20
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
      #endif  //isOneDS18B20

      #if defined isBME680 || defined isBME680_BSECLib
        sprintf(printstring,"EnvMonitor680 %s", PROGVERSION);
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
      #endif //isBME680

      #ifdef isBME280
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
      #endif //#ifdef isBME280
  
      #ifdef isMHZ14A
        display.setCursor(0, 48);
        display.setTextSize(1);
        sprintf(printstring, "CO2 %d ppm \n", CO2ppm);
        display.println(printstring);  
      #endif //#ifdef isMHZ14A

      #ifdef isSENSEAIR_S8
        display.setCursor(0, 48);
        display.setTextSize(1);
        sprintf(printstring, "CO2 %d ppm \n", CO2ppm);
        display.println(printstring);  
      #endif // #ifdef isSENSEAIR_S8
    } // if(displayMode == 1)

    if(displayMode > 1)
        specialDisplay(displayMode);

    // dim display 10 sec after last button push  
    if(millis() > lastButtonTime + displayDimmDelay)
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
    vTaskDelay(300 / portTICK_PERIOD_MS); // delay for 300 ms, to avoid double presses
    displayDone = 1;
  }
#endif // isDisplay

/*****************************************************************************
 @brief main loop
*****************************************************************************/
void loop()
{
  #if defined isThingspeak || defined isVirtuino || defined isBME280 || defined isBME680 || defined isOneDS18B20
    mainHandlerTimer.run();   // simple timer for main handler
  #endif
  #if defined isRelay && defined isFan
    fanHandlerTimer.run();    // simple timer for fan handler 
  #endif  
  #if defined isWindowOpenDetector
    windowOpenHandlerTimer.run();    // simple timer for window open handler 
  #endif  
  #ifdef isThingspeak
    thingspeakHandlerTimer.run(); // simple timer for thingspeak data sender handler
  #endif  
  #ifdef isMQTT
    mqttHandlerTimer.run(); // simple timer for MQTT data handler
  #endif  

  // and this is the version for Blynk timer. One for all, but needs Blynk
  MyBlynkTimer.run(); // EXPDis
  #ifdef isBLYNK 
    // EXPDis MyBlynkTimer.run();
  #endif  

  #ifdef isVirtuino 
    virtuinoRun(); // Necessary function to communicate with Virtuino. Client handler
  #endif  
  //*** Blynk communication
  #ifdef isBLYNK
    Blynk.run(); 
  #endif  
 
  // handle OTA over the air Updates 
   #ifdef isOTA
    ArduinoOTA.handle();
  #endif
}
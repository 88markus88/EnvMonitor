/******************************************************
* Functions for Infactory 433 MHz external 
* Temp/Humidity sensor for EnvMonitor680.cpp
*******************************************************/
//*** general libraries. Libraries that multiple modules use should reside here
#include "GlobalDefines.h"         // global defines that determine what to compile
#include <Arduino.h>
#include "time.h"
#include <Preferences.h>  // used to permanently store data
#include <esp_task_wdt.h> //Load Watchdog-Library
#include "HelperFunctions.h"       // prototypes for global helper functions
#include "Infactory433.h"          // Infactory 433 Temp/Humid sensor

// global variables for this module. However, these ones are also used for serial, therefore out of define
float InfactoryTempC = -111.11;       // global storage variable for temperature
float lastInfactoryTempC[3] = {-111.11,-111.11,-111.11};
 float InfactoryHumidity = -111.11;    // global storage variable for humidity

#ifdef isInfactory433
  /* Convert RF signal into bits for Infactory NC-5849-675 Sensor
  * Based on code by : Ray Wang (Rayshobby LLC)
  * http://rayshobby.net/?p=8998
  * https://rayshobby.net/reverse-engineer-wireless-temperature-humidity-rain-sensors-part-1/ 
  */
  // ATTENTION: this is in collision with Blynk! therefore measurement interrupt routine is switched on
  // appox every minute, and then switched off again. Only when off, Blynk is enabled.
  // If Blynk routines run in parallel, the interrupt routine cannot get the sensor data fast enough
  // consequently there is no reception.

  // ring buffer size has to be large enough to fit
  // data between two successive sync signals
  #define RING_BUFFER_SIZE  256
  #define SIGNAL_FILTER 400   // signals shorter than this (in us) are not stored = filtered out

  #define NO_SYNC_PULSES 6   // the sync sequence is 4 levels long, plus 1 bit (2 level after the signal = 6)
  #define NO_SIGNAL_PULSES 82 // signal is 41 bit long  

  #define SYNC_LENGTHI 540
  #define SYNC_LENGTH 4000

  #define SYNC_HIGH  540
  #define SYNC_LOW   8000
  #define BIT1_HIGH  540
  #define BIT1_LOW   4000
  #define BIT0_HIGH  540
  #define BIT0_LOW   2000

  //#define TOL1  70  // tolerance values for pulse lengths
  //#define TOL2 100  // standard until 1.3.21 
  //#define TOL3 200 

  #define TOL1  70  // tolerance values for pulse lengths
  #define TOL2 100   
  #define TOL3 200 

  #define DATAPIN  33  // was GPIO 15 forReceiver data, then 16. converted to 33: 

  volatile unsigned long timings[RING_BUFFER_SIZE];
  volatile unsigned int syncIndex1 = 0;  // index of the first sync signal
  volatile unsigned int syncIndex2 = 0;  // index of the second sync signal
  volatile bool received = false;
  volatile unsigned long interruptFlagger = 0;
  volatile unsigned long interruptCount = 0;

  unsigned char resultbuffer[10]; // byte buffer for payload
  int bitField[RING_BUFFER_SIZE];
  int bitsReceived = 0;
  long lastInfactoryReception = 0; // system time when last infactory reception took place, in ms
  int successInfactoryCalc = false;         // flag for successful temperature/humidity calculation
  volatile unsigned int changeCount;  // counter set in interrupt handler for number of bits received
  
  int InfactoryChannel = -1;            // channel-Kodierung, kann 00, 01 oder 10 sein 
  
  // flag to indicate that 433MHz measurements are ongoing, not to be disturbed. easier when always visible, therefore out of define
  unsigned int measuringInfactoryOngoing = 0; 

  int trialsInfactory=0, successInfactory=0;

  Preferences mypreferences; // Nonvolatile storage on ESP32 - To store values to survive a reset
#endif

// macros to set and to get bits in a bitfield
// http://www.mathcs.emory.edu/~cheung/Courses/255/Syllabus/1-C-intro/bit-array.html
#define SetBit(A,k)     ( A[(k/8)] |= (1 << (k%8)) )   
#define ClearBit(A,k)   ( A[(k/8)] &= ~(1 << (k%8)) )    
#define TestBit(A,k)    ( A[(k/8)] & (1 << (k%8)) )   

#ifdef isInfactory433
  //--- print a byte as bits
  void  print_bits(unsigned char octet)
  {
      int z = 128, oct = octet;

      while (z > 0)
      {
          if (oct & z)
              Serial.print("1");
          else
              Serial.print("0");
          z >>= 1;
      }
  }

  // reverse bit order in byte
  uint8_t reverse8(uint8_t x)
  {
      x = (x & 0xF0) >> 4 | (x & 0x0F) << 4;
      x = (x & 0xCC) >> 2 | (x & 0x33) << 2;
      x = (x & 0xAA) >> 1 | (x & 0x55) << 1;
      return x;
  }

  bool IRAM_ATTR isSync3489(unsigned int idx) {
  // check if we've received a short high followed by a long low
  unsigned long t0, tm1, tm2, tm3; 
  unsigned int i;
  i=idx+RING_BUFFER_SIZE;
  t0 = timings[(i) % RING_BUFFER_SIZE];
  tm1 = timings[(i-1) % RING_BUFFER_SIZE];
  tm2 = timings[(i-2) % RING_BUFFER_SIZE];
  tm3 = timings[(i-3) % RING_BUFFER_SIZE];   
  if(tm3<(SYNC_HIGH-TOL1) || tm3>(SYNC_HIGH+TOL1) ||
     tm2<(SYNC_LOW-TOL3)  || tm2>(SYNC_LOW+TOL3) ||
     tm1<(SYNC_HIGH-TOL1) || tm1>(SYNC_HIGH+TOL1) ||
     t0<(SYNC_LOW-TOL3)  || t0>(SYNC_LOW+TOL3)) 
  {
    return false;
  }
  return true;
}

/* Interrupt handler. therefore: global variables volatile, IRAM_ATTR set so that it runs faster*/
void IRAM_ATTR handler() {
    static volatile unsigned long duration = 0;
    static volatile unsigned long lastTime = 0;
    static volatile unsigned int ringIndex = 0;
    static volatile unsigned int syncCount = 0;

    interruptCount++;
    // Serial.print("_");
    // ignore if we haven't processed the previous received signal
    if (received == true) {
      return;
    }
    // calculating timing since last change
    long time = micros();
    duration = time - lastTime;
    lastTime = time;

    // check if signal is longer than filter value
    if(duration >= SIGNAL_FILTER)
    {
      // store data in ring buffer
      ringIndex = (ringIndex + 1) % RING_BUFFER_SIZE;
      timings[ringIndex] = duration;
      interruptFlagger++;
      // detect sync signal
      if (isSync3489(ringIndex)) {
        syncCount ++;
        interruptFlagger++;
        // first time sync is seen, record buffer index
        Serial.print("s1 ");
        if (syncCount == 1) {
          syncIndex1 = (ringIndex+1) % RING_BUFFER_SIZE;
        } 
        else if (syncCount == 2) {
          // second time sync is seen, start bit conversion
          Serial.print("s2 ");
          interruptFlagger+=1000;
          syncCount = 0;
          syncIndex2 = (ringIndex+1) % RING_BUFFER_SIZE;
          changeCount = (syncIndex2 < syncIndex1) ? (syncIndex2+RING_BUFFER_SIZE - syncIndex1) : (syncIndex2 - syncIndex1);
          // changeCount must be 88 -- 42 bits x 2 + 2 for sync
          Serial.printf("CC: %d \n",changeCount);
          
          if(false){  // xxxxxxxxx test
          //if (changeCount != 88) {
            received = false;
            syncIndex1 = 0;
            syncIndex2 = 0;
          } 
          else {
            received = true;
            // versuch
            detachInterrupt(digitalPinToInterrupt(DATAPIN));
            Serial.printf("Received=true\n");
          }
          
          interruptFlagger = changeCount;
        }

      }
      // else Serial.print("-");
    } // Signal filter  
  }
#endif  // isInfactory 433 

#ifdef isInfactory433
  void getInfactoryData()
  {
    unsigned int index;
    char printstring[100];

    if (received == true) {
      // Serial.printf("-");
      // disable interrupt to avoid new data corrupting the buffer
      detachInterrupt(digitalPinToInterrupt(DATAPIN));

      long processStartTime;
      processStartTime = micros();
          
      bool fail = false;
      bitsReceived = 0;
      for (unsigned int i=0; i<10; i++)
        resultbuffer[i]=0;

      // if wrong count, let's print it out to check what's going on
      if (changeCount > NO_SIGNAL_PULSES + NO_SYNC_PULSES) {
        sprintf(printstring,"changeCount: %d syncIndex1: %d syncIndex2: %d\n",changeCount,syncIndex1,syncIndex2);
        logOut(printstring, msgInfactoryInfo, msgInfo);
        for(unsigned int i=syncIndex1, c=0; i!=syncIndex2; i=(i+2)%RING_BUFFER_SIZE, c++) {
          unsigned long t0 = timings[i], t1 = timings[(i+1)%RING_BUFFER_SIZE];    
          sprintf(printstring,"c: %d (t0:%ld t1:%ld) ",c,t0,t1);
          logOut(printstring, msgInfactoryInfo, msgInfo);
          if (c%5==0) Serial.println(" ");
          if (c>=200) break;
        }
      }

      // special case: number of  bits counted by handler not meeting expectations
      if (changeCount > NO_SIGNAL_PULSES + NO_SYNC_PULSES)
      {
        sprintf(printstring,"\nWrong changeCount %d, trying to find sync. Was: %d - %d\n",changeCount, syncIndex1, syncIndex2);
        logOut(printstring, msgInfactorySyncing, msgInfo);
        for(index=syncIndex1; index!=syncIndex2; index=(index+1)%RING_BUFFER_SIZE)
        {
          //Serial.print("q");
          if(isSync3489(index) )
          {
            syncIndex1=(index+1)%RING_BUFFER_SIZE;
            sprintf(printstring,"\nNew syncIndex1 -2 : %d - %d\n",syncIndex1, syncIndex2);
            logOut(printstring, msgInfactorySyncing, msgInfo);
            break; 
          }
        }
      }

      // loop over buffer data
      unsigned int c=0;
      for(unsigned int i=syncIndex1; i!=syncIndex2; i=(i+2)%RING_BUFFER_SIZE, c++) {
        unsigned long t0 = timings[i], t1 = timings[(i+1)%RING_BUFFER_SIZE];
        if (t0>(BIT1_HIGH-TOL1) && t0<(BIT1_HIGH+TOL1) &&
            t1>(BIT1_LOW-TOL2) && t1<(BIT1_LOW+TOL2)) {
          // Serial.print("1");
           SetBit(resultbuffer, c);
          // Serial.printf(" %d ",c);
          bitsReceived ++;
        } else if (t0>(BIT0_HIGH-TOL1) && t0<(BIT0_HIGH+TOL1) &&
                  t1>(BIT0_LOW-TOL2) && t1<(BIT0_LOW+TOL2)){
          // Serial.print("0");
          ClearBit(resultbuffer, c);
          bitsReceived ++;
        } else if (t0>(SYNC_HIGH-TOL1) && t0<(SYNC_HIGH+TOL1) &&
                  t1>(SYNC_LOW-TOL3) && t1<(SYNC_LOW+TOL3)) {
          sprintf(printstring,"S%d (%ld %ld) \n", c, t0, t1);  // sync signal
          logOut(printstring, msgInfactoryEvaluating, msgInfo);
        } else if (t0>(BIT0_HIGH-TOL1) && t0<(BIT0_HIGH+TOL1) &&
                  t1>(SYNC_LENGTH-TOL3) && t1<(SYNC_LENGTH+TOL3)){
          sprintf(printstring,"Y%d (%ld %ld) \n", c, t0,t1); // long pause before sync
          logOut(printstring, msgInfactoryEvaluating, msgInfo);
        } else {
          sprintf(printstring,"?%d (%ld %ld) \n", c, t0,t1);      // undefined timing
          logOut(printstring, msgInfactoryEvaluating, msgInfo);
          // test 01.03.21 // fail = true;
        }

        if (c > 88){   // too many bits -> leave, still try to interpret later
          sprintf(printstring," c>88\n");
          logOut(printstring, msgInfactoryEvaluating, msgInfo);
          break;
        }  
        // if (c%8==7) Serial.print(" ");
      }

      int raw_temp=0, raw_humidity=0;
      float tempF= 0; 
      float tempC= 0;
      unsigned char nibble0, nibble1, nibble2;
      // my own interpretation routine
      if (!fail) {
        //for(unsigned int i=0; i<bitsReceived; i++) 
        //  Serial.print(bitField[i]);
        //Serial.printf("   %d  \n",bitsReceived);
        for(unsigned int i=0; i<6; i++) {
          resultbuffer[i] = reverse8(resultbuffer[i]);  // reverse bit order in byte, since stored in wrong order
          // Serial.printf("%d %d ",i,resultbuffer[i]);
          // print_bits(resultbuffer[i]);
          // Serial.printf(" ");
        }  
        // Serial.printf("   %d  \n",bitsReceived);

        // better with byte field
        nibble0 = (resultbuffer[1]<<2 & 0xf) | resultbuffer[2]>>6;
        nibble1 = resultbuffer[2]>>2 & 0xf;
        nibble2 = (resultbuffer[2]<<2 & 0xf) | resultbuffer[3]>>6;
        
        raw_temp = (nibble2 & 0xF)<<4;
        raw_temp = (raw_temp | nibble1)<<4;
        raw_temp = (raw_temp | nibble0);

        // raw_humidity = (resultbuffer[4]>>2 & 0x3f) | (resultbuffer[3] >> 2) & 0xf;
        raw_humidity = (resultbuffer[3] >> 2) & 0xf;
        // Serial.printf("H1: %d ",raw_humidity); 
        raw_humidity = raw_humidity | ((resultbuffer[4] >> 2) &  0x30);
        // Serial.printf("H3: %d",raw_humidity);

        tempF = (float)raw_temp / 10.0 - 90;    // Temperature in Fahrenheit
        tempC = (float)(tempF-32)*(float)5/9;   // converted to Celsius
       
        // Serial.printf("\nNew: Nibbles: %d %d %d raw_temp: %d raw_humidity: %d TempF %3.1f TempC %3.1f Humidity %d %:\n",
        //    nibble0, nibble1, nibble2, raw_temp, raw_humidity, tempF, tempC, raw_humidity);

        // store results in global variables
        InfactoryTempC = tempC;
        InfactoryHumidity = (float)raw_humidity;   
        InfactoryChannel =  (resultbuffer[1]>>2) && 0x3;  // channel in bits 5 and 6 of second byte
        
        sprintf(printstring,"Conversion Duration: %3.2f [ms]\n", float(micros()-processStartTime)/1000);  
        logOut(printstring, msgInfactoryInfo, msgInfo);
        if(raw_temp==0 || raw_humidity==0 || tempC < -40)  // faulty values check, no data 
        {
          InfactoryTempC=-111.11;
          InfactoryHumidity=-111.11;
          sprintf(printstring,"No data, temperature and humidity reset\n");
          logOut(printstring, msgInfactoryNoData, msgInfo);
        }
        successInfactoryCalc=true;    // flag to determine successfull measurement
      }

      // delay for 1 second to avoid repetitions
      vTaskDelay(1000 / portTICK_PERIOD_MS); // non-blocking delay instead
      sprintf(printstring,"---------------------------------------------------------------\n");
      logOut(printstring, msgInfactoryInfo, msgInfo);
      received = false;
      syncIndex1 = 0;
      syncIndex2 = 0;

      // re-enable interrupt
      attachInterrupt(digitalPinToInterrupt(DATAPIN), handler, CHANGE);
    }
  }

  void doInfactoryStuff(int maxTrialPeriod) 
  {
    char printstring[100];
    // enable interrupt for 433 MHz receiver
    measuringInfactoryOngoing = true;     // let others know: measurements are done
    attachInterrupt(digitalPinToInterrupt(DATAPIN), handler, CHANGE);
    //delay(100);

    int startingTime = millis();
    successInfactoryCalc = false;
    while(millis()<startingTime+maxTrialPeriod && successInfactoryCalc == false)   // try the specified number of milliseconds or until received
    {
      getInfactoryData();
      sprintf(printstring, ".");
      logOut(printstring, msgInfactoryInfo, msgInfo);

      // versuch, war 500
      vTaskDelay(3000/ portTICK_PERIOD_MS); // delay for 500 ms, non-blocking
        
      esp_task_wdt_reset();   // keep watchdog happy
    }
    if(successInfactoryCalc == true)
    {
      // remember data collected in non-volatile storage, to survive reboot and be used after ist
      mypreferences.begin("nvs", false);                          // Open nonvolatile storage (nvs)
      mypreferences.putFloat("InfTempC", InfactoryTempC);
      mypreferences.putFloat("InfHumidity", InfactoryHumidity);
      mypreferences.end();
    }  
    // Serial.printf("\ndoInfactoryStuff successInfactoryCalc: %d  time: %d\n",successInfactoryCalc,lastInfactoryReception);
    lastInfactoryReception = millis();
    // done in calling routine: 
    // successInfactoryCalc = false;
    // disable interrupt for receiver
    detachInterrupt(digitalPinToInterrupt(DATAPIN));
    measuringInfactoryOngoing = false;     // let others know: measurements are done
  }

  // to be used in "setup": initialize measurement with Infactory sensor
  void doInitializeInfactory()
  {
    pinMode(DATAPIN, INPUT);
    // machen wir später: attachInterrupt(digitalPinToInterrupt(DATAPIN), handler, CHANGE);

    // if stored data present from previous reboot: use them
    // get last data collected in non-volatile storage, stored to survive reboot
    mypreferences.begin("nvs", false);                          // Open nonvolatile storage (nvs)
    InfactoryTempC = mypreferences.getFloat("InfTempC", -111.1);
    InfactoryHumidity = mypreferences.getFloat("InfHumidity", -111.11);
    // and ensure that these are not red twice
    mypreferences.putFloat("InfTempC", -111.11);
    mypreferences.putFloat("InfHumidity", -111.11);
    mypreferences.end();

    // if no stored data: try to measure directly
    if(InfactoryTempC<-110 && InfactoryHumidity<-110) 
    {
      Serial.printf("\nWaiting for temperature and humidity from Infactory 433 MHz sensor\n");
      received=false;
      doInfactoryStuff(80000);   // try to get first infactory data set
      if(successInfactoryCalc==true) 
      {
        Serial.printf("Received temperature and humidity from Infactory 433 MHz sensor after %ld msec\n",
            millis()-lastInfactoryReception);
        Serial.printf("TempC %3.1f Humidity %3.1f %% :\n", InfactoryTempC, InfactoryHumidity);    
        lastInfactoryReception = millis();
      }  
      received = false; // reset reception flag
    }
  }

#endif
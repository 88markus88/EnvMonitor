//********************************************************************************************
// Convert 433MHz RF signal created by Temp/Hydro 433 MHz Sensor Infactory NC5849 
// and send them to serial output, for use by other devices (e.g. ESP32
// Written by Markus Preidel 
// Based on work by Ray Wang (Rayshobby LLC)
// http://rayshobby.net/?p=8998
// https://rayshobby.net/reverse-engineer-wireless-temperature-humidity-rain-sensors-part-1/ 
//********************************************************************************************

#include <SoftwareSerial.h>
#define sendSERIAL        // send data via serial
#undef receiveSERIAL      // not used, for future enhancement
#define sendSERIALERRORS  // only if this is set errors are sent via serial to ESP32

// ring buffer size has to be large enough to fit
// data between two successive sync signals
#define RING_BUFFER_SIZE  256
#define FILTER 200 // all smaller are ignored
int bitsReceived = 0;

//                          __                 __                  
// NC-3982 Sync:  |________| |________________| |________________|    data bits follow
// time in ms          4   ,5       8,03      .5     8.03 
//                    _
//  data bits: "1":  | |______|
//  time in ms       .6  4.0
//                    _
//  data bits: "0":  | |___| 
//  time in ms       .6  2.0

#define PROGNAME  "Sniffer433_real.cpp"
#define PROGDATE  "2021-04-10"
#define PROGVERSION "V0.10"

#undef outputDATA

#define NO_SYNC_PULSES 6   // the sync sequence is 4 levels long, plus 1 bit (2 level after the signal = 6)
#define NO_SIGNAL_PULSES 82 // signal is 41 bit long  

#define SYNC_LENGTHI 540
#define SYNC_LENGTH 4000

#define SYNC_HIGH  540
#define SYNC_LOW   8000
#define BIT1_HIGH  540
#define BIT1_LOW   4050
#define BIT0_HIGH  540
#define BIT0_LOW   2050

#define TOL1  80  // tolerance values
#define TOL2 150   
#define TOL3 250 

#define DATAPIN  3  // D3 is interrupt 1
#define TRIGGERPIN 12 // to trigger scope

volatile unsigned int timings[RING_BUFFER_SIZE];
volatile unsigned int syncIndex1 = 0;  // index of the first sync signal
volatile unsigned int syncIndex2 = 0;  // index of the second sync signal
volatile unsigned int changeCount = 0;
volatile unsigned int syncCount = 0;
volatile bool received = false;

unsigned char resultbuffer[10]; // byte buffer for payload

// Variables for automatic adaptation of pulse lengths
unsigned int Bit1_High_Lengths = BIT1_HIGH;
unsigned int Bit1_Low_Lengths = BIT1_LOW;
unsigned int Bit0_High_Lengths = BIT0_HIGH;
unsigned int Bit0_Low_Lengths = BIT0_LOW;

// Variables for pulse statistics
/*
unsigned long lastOutput=0;
unsigned int messagesReceived=0, messagesDropped=0;
unsigned int count1=0,  sum1HI=0,  sum1LO=0, max1HI=0,  min1HI=1000000,  max1LO=0, min1LO=1000000;
unsigned int count0=0,  sum0HI=0,  sum0LO=0, max0HI=0,  min0HI=1000000,  max0LO=0, min0LO=1000000;
unsigned int counts2=0, sums2HI=0, sums2LO=0,maxs2HI=0, mins2HI=1000000, maxs2LO=0, mins2LO=1000000;
unsigned int counts1=0, sums1HI=0, sums1LO=0,maxs1HI=0, mins1HI=1000000, maxs1LO=0, mins1LO=1000000;
unsigned int countsy=0, sumsyHI=0, sumsyLO=0,maxsyHI=0, minsyHI=1000000, maxsyLO=0, minsyLO=1000000;
*/ 

// other global ariables
char printstring[100];

// macros to set and to get bits in a bitfield
// http://www.mathcs.emory.edu/~cheung/Courses/255/Syllabus/1-C-intro/bit-array.html
#define SetBit(A,k)     ( A[(k/8)] |= (1 << (k%8)) )   
#define ClearBit(A,k)   ( A[(k/8)] &= ~(1 << (k%8)) )    
#define TestBit(A,k)    ( A[(k/8)] & (1 << (k%8)) )     

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

bool isSync3489(unsigned int idx) {
  // check if we've received a short high followed by a long low
  volatile unsigned long t0, tm1, tm2, tm3; 
  volatile unsigned int i;
  i=idx+RING_BUFFER_SIZE;
  t0 = timings[(idx+RING_BUFFER_SIZE) % RING_BUFFER_SIZE];
  tm1 = timings[(idx+RING_BUFFER_SIZE-1) % RING_BUFFER_SIZE];
  tm2 = timings[(idx+RING_BUFFER_SIZE-2) % RING_BUFFER_SIZE];
  tm3 = timings[(idx+RING_BUFFER_SIZE-3) % RING_BUFFER_SIZE];   
  if(tm3<(SYNC_HIGH-TOL1) || tm3>(SYNC_HIGH+TOL1) ||
     tm2<(SYNC_LOW-TOL3)  || tm2>(SYNC_LOW+TOL3) ||
     tm1<(SYNC_HIGH-TOL1) || tm1>(SYNC_HIGH+TOL1) ||
     // +++test     t0<(SYNC_LOW-TOL3)  || t0>(SYNC_LOW+TOL3)) 
     t0<(SYNC_LOW-TOL3)) 
  {
    return false;
  }
  digitalWrite(TRIGGERPIN, HIGH); // make sync signal for scope
  return true;
}

/* Interrupt 1 handler */
void handler() {
  volatile static unsigned long duration = 0;
  volatile static unsigned long lastTime = 0;
  volatile static unsigned int ringIndex = 0;

  // Serial.print(".");
  // ignore if we haven't processed the previous received signal
  if (received == true) {
    return;
  }
  // calculating timing since last change
  long time = micros();
  duration = time - lastTime;
  lastTime = time;

  if(duration > FILTER)
  {
    // store data in ring buffer
    ringIndex = (ringIndex + 1) % RING_BUFFER_SIZE;
    timings[ringIndex] = min(duration, 65535);    // safely cast long to int

    // detect sync signal
    if (isSync3489(ringIndex)) {
      syncCount ++;
      
      // first time sync is seen, record buffer index
      Serial.print("s1 ");
      if (syncCount == 1) {
        syncIndex1 = (ringIndex+1) % RING_BUFFER_SIZE;
      } 
      else if (syncCount == 2) {
        // second time sync is seen, start bit conversion
        Serial.print("s2 ");
        syncCount = 0;
        syncIndex2 = (ringIndex+1) % RING_BUFFER_SIZE;
        changeCount = (syncIndex2 < syncIndex1) ? (syncIndex2+RING_BUFFER_SIZE - syncIndex1) : (syncIndex2 - syncIndex1);
        // changeCount must be 88 -- 42 bits x 2 + 2 for sync
        Serial.print("CC: ");Serial.print(changeCount);Serial.print("\n");
        //+++ versuch if (changeCount != 88) {
        if (changeCount < 85 || changeCount >93) {
          received = false;
          syncIndex1 = syncIndex2;
          syncIndex2 = 0;
          syncCount = 0;
        } 
        else {
          received = true;
        }
      }
    }
  } // duration 
  // else Serial.print("-");
  digitalWrite(TRIGGERPIN, LOW);
}

#if defined sendSERIAL || defined receiveSERIAL
  #define RXD 5
  #define TXD 2
  SoftwareSerial mySerial(RXD, TXD); // RX, TX
#endif 

#if defined sendSERIAL || defined receiveSERIAL
  // serial stuff for sending / receiving external sensor data

  //******************** send a sentence to serial
  void sendSerial(int serialChannel,float serialTemp,int serialHumidity)
  {
    int bytesSent=0;
    char sendString[80];
    //sprintf(sendString,"<%d,%4.2f,%4.2f>",serialChannel,serialTemp,serialHumidity);
    // arduino cannot format %f. Alternative: dtostrf or sprintf
    char str1[6];
    dtostrf(serialTemp, 3, 1, str1);  //3 is mininum width, 1 is precision; float value is copied onto str_temp
    char str2[6];
    dtostrf(serialHumidity, 3, 1, str2); 
    sprintf(sendString,"<%d,%s,%s>",serialChannel,str1,str2);
    // if(mySerial.available()>=strlen(sendString))
    // if(true)
    if(mySerial.available())
    {
      bytesSent= mySerial.write(sendString);
      Serial.print("\n>>>>>>> Serial sent: "); Serial.print(bytesSent);Serial.print(" "); Serial.println(sendString);
    } else 
    {
      Serial.println("Serial write not possible, not available");
    }  
  }

  //******************** send a sentence to serial
  void sendSerialErr(char* sendString)
  {
    int bytesSent=0;
    #ifdef sendSERIALERRORS
      bytesSent= mySerial.write(sendString);
    #endif
    Serial.print("SerErr: "); Serial.print(bytesSent);Serial.print(" "); Serial.print(sendString);
  }
#endif // sendSERIAL

#if defined receiveSERIAL
  boolean newData = false;
  const byte numChars=80;
  char receivedChars[numChars];

  //******************* receive a sentence from serial
  void receiveSerial()
  {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<', endMarker = '>', rc;
    
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
    Serial.printf("Serial2 read: '%s' \n", receivedChars);
  } // readSerial

  void parseData(char *tempChars, float* serialT, float* serialH) {      // split the data into its parts
    char * strtokIndx;                    // this is used by strtok() as an index
    //char message[numChars];             // buffer
    strtokIndx = strtok(tempChars,",");   // get the first part - the string
    //strcpy(message, strtokIndx);        // copy it to messageFromPC
    //strtokIndx = strtok(NULL, ",");     // this continues where the previous call left off
    *serialT = atof(strtokIndx);          // convert this part to an integer
    strtokIndx = strtok(NULL, ",");
    *serialH = atof(strtokIndx);          // convert this part to a float
}
#endif // serial stuff  


void setup() {
  Serial.begin(115200);
  Serial.println("\n*************");
  sprintf(printstring," Decoder for 'Infactory NC-5849-675' on PIN %d\n",DATAPIN);
  Serial.print(printstring);
  sprintf(printstring, " %s %s %s \n",PROGNAME,PROGVERSION,PROGDATE);
  Serial.print(printstring);
  Serial.println("*************");

  // Empfänger und Interrupt handler aufsetzen
  pinMode(DATAPIN, INPUT);
  pinMode(TRIGGERPIN, OUTPUT);
  attachInterrupt(1, handler, CHANGE);

  #if defined sendSERIAL || defined receiveSERIAL
    pinMode(RXD, INPUT);
    pinMode(TXD, OUTPUT);
    mySerial.begin(9600);
  #endif
} 

// new routine to extract values exactly from this area within input buffer "timings"
bool calcValuesNew(int localsyncIndex1, int localsyncIndex2, int* Channel, float* tempC, float* tempF, int* raw_humidity)
{
  bool fail = false;
  bitsReceived = 0;
  for (unsigned int i=0; i<10; i++)
    resultbuffer[i]=0;

  unsigned long Bit1_High_Sum=0, Bit1_Count=0;
  unsigned long Bit1_Low_Sum=0; 
  unsigned long Bit0_High_Sum=0, Bit0_Count=0;
  unsigned long Bit0_Low_Sum=0; 
    
  if(abs(localsyncIndex2-localsyncIndex1)%2 != 0)
    localsyncIndex2++;
  if(localsyncIndex2 > RING_BUFFER_SIZE-1)
    localsyncIndex2 = 0;
  // loop over buffer data
  unsigned int c=12; // this is the actual start position, where the data starts within the telegram. localsyncIndex1 now directly points to this
  for(unsigned int i=localsyncIndex1; i!=localsyncIndex2; i=(i+2)%RING_BUFFER_SIZE, c++) 
  {
    unsigned int t0 = timings[i], t1 = timings[(i+1)%RING_BUFFER_SIZE];
    if (t0>(Bit1_High_Lengths - TOL1) && t0<(Bit1_High_Lengths + TOL1) &&
        t1>(Bit1_Low_Lengths - TOL3) && t1<(Bit1_Low_Lengths + TOL3)) {
      //Serial.print("1");
      SetBit(resultbuffer, c);
      // Serial.printf(" %d ",c);
      Bit1_Count +=1;
      Bit1_High_Sum += t0; 
      Bit1_Low_Sum += t1; 
      bitsReceived ++; 
    } else if (t0>(Bit0_High_Lengths - TOL1) && t0<(Bit0_High_Lengths + TOL1) &&
               t1>(Bit0_Low_Lengths - TOL3) && t1<(Bit0_Low_Lengths + TOL3)){
      //Serial.print("0");
      ClearBit(resultbuffer, c);
      Bit0_Count +=1;
      Bit0_High_Sum += t0; 
      Bit0_Low_Sum += t1; 
      bitsReceived ++;
    } else if (t0>(SYNC_HIGH-TOL1) && t0<(SYNC_HIGH+TOL1) &&
               t1>(SYNC_LOW-TOL3) && t1<(SYNC_LOW+TOL3)) {
      Serial.print("S");  // sync signal
      fail = true;
    } else if (t0>(BIT0_HIGH-TOL1) && t0<(BIT0_HIGH+TOL1) &&
               t1>(SYNC_LENGTH-TOL3) && t1<(SYNC_LENGTH+TOL3)){
      // Serial.print("Y");  // long low before sync
      sprintf(printstring,"Y t0:%d t1:%d  ",t0,t1);      // undefined timing
      Serial.print(printstring);
      fail = true;
    } else {
      sprintf(printstring,"? t0:%d t1:%d  ",t0,t1);      // undefined timing
      Serial.print(printstring);
      fail = true;
    }
    if (c%8==7) Serial.print(" ");
  }

  // collect data for bit lengths adaptation, moving average over 4 measurements
  if(Bit1_Count > 0 && Bit0_Count > 0)
  {
    Bit1_High_Lengths = (3 * Bit1_High_Lengths + Bit1_High_Sum / Bit1_Count)/4;
    Bit1_Low_Lengths =  (3 * Bit1_Low_Lengths  + Bit1_Low_Sum  / Bit1_Count)/4;
    Bit0_High_Lengths = (3 * Bit0_High_Lengths + Bit0_High_Sum / Bit0_Count)/4;
    Bit0_Low_Lengths =  (3 * Bit0_Low_Lengths  + Bit0_Low_Sum  / Bit0_Count)/4;
    sprintf(printstring,"Adapted to Bit1: %d %d - Bit0: %d  %d \n",Bit1_High_Lengths,Bit1_Low_Lengths,Bit0_High_Lengths,Bit0_Low_Lengths);     
    Serial.print(printstring);
  }  
  else
    Serial.print("No adaptation of Bit pulse lengths\n");

  int raw_temp=0;
  unsigned nibble0, nibble1, nibble2;

  if (!fail) {
    //sprintf(printstring," Bits:  %d  \n",bitsReceived);
    //Serial.print(printstring);
    for(unsigned int i=0; i<6; i++) {
      // print_bits(resultbuffer[i]);
      // Serial.print(" ");
      resultbuffer[i] = reverse8(resultbuffer[i]);  // reverse bit order in byte, since stored in wrong order
    } 
    //Serial.println(" ");
    for(unsigned int i=0; i<6; i++) {
      // print_bits(resultbuffer[i]);
      // Serial.print(" ");
    }  

    nibble0 = (resultbuffer[1]<<2 & 0xf) | resultbuffer[2]>>6;
    nibble1 = resultbuffer[2]>>2 & 0xf;
    nibble2 = (resultbuffer[2]<<2 & 0xf) | resultbuffer[3]>>6;
      
    raw_temp = (nibble2 & 0xF)<<4;
    raw_temp = (raw_temp | nibble1)<<4;
    raw_temp = (raw_temp | nibble0);

    // raw_humidity = (resultbuffer[4]>>2 & 0x3f) | (resultbuffer[3] >> 2) & 0xf;
    *raw_humidity = (resultbuffer[3] >> 2) & 0xf;
    *raw_humidity = *raw_humidity | ((resultbuffer[4] >> 2) &  0x30);
    *raw_humidity = *raw_humidity | ((resultbuffer[3] <<6) & 0xC0);  // new to address 128 and 64 as well

    *tempF = (float)raw_temp / 10.0 - 90.0;    // Temperature in Fahrenheit
    *tempC = (float)(*tempF-32.0)*(float)5.0/9.0;                 // converted to Celsius
    *Channel =  (resultbuffer[1]>>2) & 0x3;  // channel in bits 5 and 6 of second byte

    // arduino cannot format %f. Alternative: dtostrf
    char str1[6];
    dtostrf(*tempC, 3, 1, str1);  //3 is mininum width, 1 is precision; float value is copied onto str_temp
    char str2[6];
    dtostrf(*tempF, 3, 1, str2); 

    //sprintf(printstring,"\nCh: %d raw_temp: %d tempC: %s tempF: %s  raw_humidity: %d\n",
    //    *Channel,raw_temp, str1, str2, *raw_humidity);
    //Serial.print(printstring);
  } // fail  
  return(fail);
}

void loop() {
  bool fail;
  int raw_humidity =0;
  float tempF = 0; 
  float tempC = 0;
  int Channel =0;
  char str1[6];
  unsigned int localsyncIndex1, localsyncIndex2;
  
 // here starts improved method, working with changeCoung <> 88
  if(received==true)
  {
      // disable interrupt to avoid new data corrupting the buffer
      detachInterrupt(1);
      long processStartTime = micros(); 

      // copy indexes to sync pulses in "timings"
      localsyncIndex1 = syncIndex1;
      localsyncIndex2 = syncIndex2;
      // prepare for handler to find next sync pulse und use it properly
      syncIndex1 = syncIndex2;    // new "first found" index at beginning of next signal
      syncCount = 1;              // and only one sync found
      received = false;           // tell handler that he can start searching again
      attachInterrupt(1, handler, CHANGE);  // re-enable interrupt  
       
      //sprintf(printstring,"\nt: %ld CC:%d s1: %d s2: %d\n",millis(),changeCount,localsyncIndex1,localsyncIndex2);
      //sendSerialErr(printstring);
      //Serial.print(printstring);

      int lastDamageI=0,lastDamageC=0;
      int firstDamageI=RING_BUFFER_SIZE, firstDamageC=RING_BUFFER_SIZE;
      char known[12];
      for(unsigned int i=localsyncIndex1, c=0; i!=localsyncIndex2; i=(i+1)%RING_BUFFER_SIZE, c++) 
      {
        unsigned int t1=timings[i%RING_BUFFER_SIZE];
        if(
          (t1>(SYNC_HIGH-TOL1) && t1<(SYNC_HIGH+TOL1)) ||
          // test +++ (t1>(SYNC_LOW-TOL3) && t1<(SYNC_LOW+TOL3))   ||
          (t1>(SYNC_LOW-TOL3))   ||
          // end test +++
          (t1>(BIT1_HIGH-TOL1) && t1<(BIT1_HIGH+TOL1)) ||
          (t1>(BIT1_LOW-TOL3) && t1<(BIT1_LOW+TOL3))   ||
          (t1>(BIT0_HIGH-TOL1) && t1<(BIT0_HIGH+TOL1)) ||
          (t1>(BIT0_LOW-TOL3) && t1<(BIT0_LOW+TOL3))           
         )
        {strcpy(known,"ok");}  
        else
        {
          if(c < firstDamageC)
            {firstDamageC = c; firstDamageI = i;}
          if(c > lastDamageC)
            {lastDamageC = c; lastDamageI = i;}
          //strcpy(known,"NOT_KNOWN"); 
          //sprintf(printstring,"%d, %d, %d, %s\n",c,i,t1, known);
          // sendSerialErr(printstring);
          strcpy(printstring,"nk ");
          Serial.print(printstring);
        }
        // sprintf(printstring,"%d, %d, %d, %s\n",c,i,t1, known);
        // sendSerialErr(printstring);
        // Serial.print(printstring);
      }
      //sprintf(printstring,"FirstDamage: %d %d LastDamage: %d %d\n",firstDamageI,firstDamageC,lastDamageI,lastDamageC);
      //sendSerialErr(printstring);
      //Serial.print(printstring);
      
      // we can save the data if firstDamageC > 67 : valid data before, start from front : index 24..67
      // or lastDamageC < 24 : valid date in the end: index (localsyncIndex2-63) .. (localsyncIndex2-20) 
      if(firstDamageC > 67)
      {
        sprintf(printstring,"\nCase.gt.67 SI1 %d, SI2: %d\n",localsyncIndex1,localsyncIndex2);
        // sendSerialErr(printstring);
        Serial.print(printstring);
        int si1=localsyncIndex1;
        localsyncIndex1 = (si1 + 24)%RING_BUFFER_SIZE;   // data between 24 and 67, recalc to begin within buffer
        localsyncIndex2 = (si1 + 67)%RING_BUFFER_SIZE;
      }else if(lastDamageC < 24)
      {
        // localsyncIndex1 = localsyncIndex2 - 64;   // data between 24 and 67, recalc to begin within buffer
        // localsyncIndex2 = localsyncIndex2 - 21;
        sprintf(printstring,"\nCase.lt.24 SI1 %d, SI2: %d\n",localsyncIndex1,localsyncIndex2);
        // sendSerialErr(printstring);
        Serial.print(printstring);
        localsyncIndex1 = (localsyncIndex2 < 64) ? ((localsyncIndex2 - 64 + RING_BUFFER_SIZE)%RING_BUFFER_SIZE) : (localsyncIndex2 - 64);
        localsyncIndex2 = (localsyncIndex2 < 21) ? ((localsyncIndex2 - 21 + RING_BUFFER_SIZE)%RING_BUFFER_SIZE) : (localsyncIndex2 - 21);
      }
      sprintf(printstring,"New SI1: %d, SI2: %d\n",localsyncIndex1,localsyncIndex2);
      // sendSerialErr(printstring);
      Serial.print(printstring);

      if((firstDamageC > 67)||(lastDamageC < 24))   // saving is possible
      {
        sprintf(printstring,"### Calc-Range: %d-%d ###\n",localsyncIndex1,localsyncIndex2);
        // sendSerialErr(printstring);
        Serial.print(printstring);
        fail=calcValuesNew(localsyncIndex1, localsyncIndex2, &Channel, &tempC, &tempF, &raw_humidity);

        dtostrf(float(micros()-processStartTime)/1000.0, 3, 2, str1);
        sprintf(printstring,"\nConvT:%s [ms]", str1);  
        Serial.print(printstring);
        // and finally send data (takes time - slow serial connection)
        if(!fail)
        {
          #ifdef sendSERIAL  
            sendSerial(Channel, tempC, raw_humidity);
          #endif
        }  
        else
        {
          sprintf(printstring,"<Fail Conv sI1 %d sI2 %d Damage %d-%d>",localsyncIndex1,localsyncIndex2,firstDamageC,lastDamageC);
          sendSerialErr(printstring);      
          dtostrf(float(micros()-processStartTime)/1000.0, 3, 2, str1);
          sprintf(printstring,"\nConversion Duration: %s [ms]", str1);  
          Serial.print(printstring);
        }
      }
      else
      {
        // restart measuring also in case of failure. separate to be able to send serial after this in positive case
        sprintf(printstring,"<Fail Rng sI1 %d sI2 %d Damage %d-%d>",localsyncIndex1,localsyncIndex2,firstDamageC,lastDamageC);
        sendSerialErr(printstring);
      
        dtostrf(float(micros()-processStartTime)/1000.0, 3, 2, str1);
        sprintf(printstring,"\nConversion Duration: %s [ms]", str1);  
        Serial.print(printstring);
      }
  }    
  //Serial.print("-");
  //delay(100);
}

#ifdef outputDATA
// output the message information for statistics and analysis
// at the time this procedure is called, data are in timings[], and 
// syncIndex1 and syncIndex2 are pointing to the last plataeu of the sync sequence. 
// length of the sync sequence is NO_SYNC_PULSES. 
// message length = NO_SIGNAL_PULSES
// parameter fail is true if no conversion was possible == dropped
void outputMessageInfo5849(bool fail)
{
  unsigned int i, c, start, stop;
  unsigned int t0, t1;
  char printstring[80];

  // loop over buffer data 
    // for(i=syncIndex1-NO_SYNC_PULSES, c=0; i!=syncIndex2-NO_SYNC_PULSES; i=(i+2)%RING_BUFFER_SIZE, c++) 

    start= syncIndex1-NO_SYNC_PULSES;
    stop = (syncIndex2-NO_SYNC_PULSES) % RING_BUFFER_SIZE; 
    c=0;
    messagesReceived++;
    Serial.println("Output of collected data");
    for(i=start; i!= stop; i=(i+2)%RING_BUFFER_SIZE)
    {
      t0 = timings[i], t1 = timings[(i+1)%RING_BUFFER_SIZE];
      sprintf(printstring," t0: %4ld t1: %4ld ", t0, t1);
      Serial.print(printstring);

      // get statistics for "1"
      if (t0>(BIT1_HIGH-TOL1) && t0<(BIT1_HIGH+TOL1) &&
          t1>(BIT1_LOW-TOL3) && t1<(BIT1_LOW+TOL3)) {
        Serial.print("'1' ");
        if (c%8==7) Serial.println(" ");
        c++;
        sum1HI+=t0; sum1LO+=t1; count1++;
        if(t0 < min1HI) min1HI=t0;
        if(t0 > max1HI) max1HI=t0;
        if(t1 < min1LO) min1LO=t1;
        if(t1 > max1LO) max1LO=t1;
      }
      // get statistics for "0"
      else if (t0>(BIT0_HIGH-TOL1) && t0<(BIT0_HIGH+TOL1) &&
          t1>(BIT0_LOW-TOL3) && t1<(BIT1_LOW+TOL3)) {
        Serial.print("'0' ");
        if (c%8==7) Serial.println(" ");
        c++;
        sum0HI+=t0; sum0LO+=t1; count0++;
        if(t0 < min0HI) min0HI=t0;
        if(t0 > max0HI) max0HI=t0;
        if(t1 < min0LO) min0LO=t1;
        if(t1 > max0LO) max0LO=t1;
      }
      // sync2: short high, longer low
      else if (t0>(SYNC_HIGH-TOL1) && t0<(SYNC_HIGH+TOL1) &&
          t1>(SYNC_LOW-TOL2) && t1<(SYNC_LOW+TOL2)) {
        Serial.print("'s2' \n");
        sums2HI+=t0; sums2LO+=t1; counts2++;
        if(t0 < mins2HI) mins2HI=t0;
        if(t0 > maxs2HI) maxs2HI=t0;
        if(t1 < mins2LO) mins2LO=t1;
        if(t1 > maxs2LO) maxs2LO=t1;
      }
      // long sync before square wave, and before this a short pulse
      else if (t0>(SYNC_LENGTHI-TOL1) && t0<(SYNC_LENGTHI+TOL1) &&
          t1>(SYNC_LENGTH-TOL3) && t1<(SYNC_LENGTH+TOL3)) {
        Serial.print("'sy' ");
        sumsyHI+=t0; sumsyLO+=t1; countsy++;
        if(t0 < minsyHI) minsyHI=t0;
        if(t0 > maxsyHI) maxsyHI=t0;
        if(t1 < minsyLO) minsyLO=t1;
        if(t1 > maxsyLO) maxsyLO=t1;
      }  
      else Serial.print("'?'");   // unidentified signal
      
    }
    if (millis()-lastOutput>67000) 
      messagesDropped += ((millis()-lastOutput) / 68000)-1;
    if (fail)  messagesDropped++;
    sprintf(printstring,"Sync: %d Pulse. Time since last output: %5.1f [sec]. Messages received: %d dropped: %d FAIL flag: %d\n",
      NO_SYNC_PULSES, (float)(millis()-lastOutput)/1000, messagesReceived, messagesDropped, fail);
    Serial.print(printstring);  
    lastOutput = millis();
    if (countsy!=0){  
      sprintf(printstring,  "'sy' #: %2d meansyHI: %7.1f minsyHI %4d maxsyHI +%4d\n", countsy, (float)sumsyHI/countsy, minsyHI-sumsyHI/countsy, maxsyHI-sumsyHI/countsy);
      Serial.print(printstring);
      sprintf(printstring,  "'sy' #: %2d meansyLO: %7.1f minsyLO %4d maxsyLO +%4d\n", countsy, (float)sumsyLO/countsy, minsyLO-sumsyLO/countsy, maxsyLO-sumsyLO/countsy);
      Serial.print(printstring);
    }  
    //sprintf(printstring,  "'s1' #: %2d means1HI: %7.1f mins1HI %4d maxs1HI +%4d\n", counts1, (float)sums1HI/counts1, mins1HI-sums1HI/counts1, maxs1HI-sums1HI/counts1);
    //Serial.print(printstring);
    //sprintf(printstring, "'s1' #: %2d means1LO: %7.1f mins1LO %4d maxs1LO +%4d\n", counts1, (float)sums1LO/counts1, mins1LO-sums1LO/counts1, maxs1LO-sums1LO/counts1);
    //Serial.print(printstring);
    if (counts2!=0){  
      sprintf(printstring,  "'s2' #: %2d means2HI: %7.1f mins2HI %4d maxs2HI +%4d\n", counts2, (float)sums2HI/counts2, mins2HI-sums2HI/counts2, maxs2HI-sums2HI/counts2);
      Serial.print(printstring);
      sprintf(printstring,  "'s2' #: %2d means2LO: %7.1f mins2LO %4d maxs2LO +%4d\n", counts2, (float)sums2LO/counts2, mins2LO-sums2LO/counts2, maxs2LO-sums2LO/counts2);
      Serial.print(printstring);
    }  
    sprintf(printstring,"Signal: %d Pulse\n",NO_SIGNAL_PULSES);
    Serial.print(printstring);
    sprintf(printstring,  "'0'  #: %2d mean0HI:  %7.1f min0HI  %4d max0HI  +%4d\n", count0, (float)sum0HI/count0, min0HI-sum0HI/count0, max0HI-sum0HI/count0);
    Serial.print(printstring);
    sprintf(printstring,  "'0'  #: %2d mean0LO:  %7.1f min0LO  %4d max0LO  +%4d\n", count0, (float)sum0LO/count0, min0LO-sum0LO/count0, max0LO-sum0LO/count0);
    Serial.print(printstring);
    sprintf(printstring,  "'1'  #: %2d mean1HI:  %7.1f min1HI  %4d max1HI  +%4d\n", count1, (float)sum1HI/count1, min1HI-sum1HI/count1, max1HI-sum1HI/count1);
    Serial.print(printstring);
    sprintf(printstring,  "'1'  #: %2d mean1LO:  %7.1f min1LO  %4d max1LO  +%4d\n", count1, (float)sum1LO/count1, min1LO-sum1LO/count1, max1LO-sum1LO/count1);   
    Serial.print(printstring);
} //outputData
#endif

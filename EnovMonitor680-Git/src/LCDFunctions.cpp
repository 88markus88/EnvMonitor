/******************************************************
* LCD Display handling functions
*******************************************************/
#include <Arduino.h>        // general library
#include "GlobalDefines.h"  // needed here to convey the #defines

#ifdef isLCD
  #include <LiquidCrystal_I2C.h>  // liquid crystal library
  #include "LCDFunctions.h"       // header file for this .cpp file

  void initLCD()
  {
    // initialize LCD
    lcd.init();
    // turn on LCD backlight                      
    lcd.backlight();
  }

  /**************************************************!
  @brief    Function to display a string as scrolled text on LCD
  @details  Takes LCD row, message, delay time and number of columns as paramters
   The function accepts the following arguments:
  @param row: row number where the text will be displayed
  @param message: message to scroll
  @param delayTime: delay between each character shifting
  @param lcdColumns: number of columns of your LCD
  @return   void
  ***************************************************/
  void scrollText(int row, String message, int delayTime, int lcdColumns) 
  {
    for (int i=0; i < lcdColumns; i++) {
          message = " " + message;  
    } 
    message = message + " "; 
    for (int pos = 0; pos < message.length(); pos++) {
      lcd.setCursor(0, row);
      lcd.print(message.substring(pos, pos + lcdColumns));
      delay(delayTime);
    }
  }

  /**************************************************!
  @brief    Function to display a string on LCD
  @details  Takes LCD row, message, delay time and number of columns as paramters
  @param x: row number where the text will be displayed (0..3, 0 = top)
  @param y: column number where the text will be displayed (0..20, 0 = right)
  @param printstring: text to display. Should be max 20 characters.
  @return   void
  ***************************************************/
  void outLCD(int x, int y, char* printstring)
  {
    char outstring[40]="                    ";

    // strcpy(outstring,"                    ");
    strncpy(outstring, printstring,20);
    printstring[20] = 0; // ensure null-termination, which strncpy does not do if printstring is too long => Overflow
    lcd.setCursor(x, y);
    lcd.print(outstring);
  }

  /**************************************************!
  @brief    Function to display the program information on LCD
  @details  outputs constants PROGNAME, PROGVERSION, PROGDATE as well as paramter infoStringShort
  @param infoStringShort: displayed in row 3 (lowest). Short Info about the Box, from credentials.h
  @return   void
  ***************************************************/
  void displayLCDProgInfo(char *infoStringShort)
  {
    char printstring[40];
    // lcd.clear();  
    // lcd.backlight();
    
    sprintf(printstring,"* EnvMonitor by MP *");          outLCD(0,0,printstring);
    sprintf(printstring,"%s",PROGNAME);                   outLCD(0,1,printstring);
    // sprintf(printstring,"%s - %s",PROGVERSION,PROGDATE);  outLCD(0,2,printstring);
    sprintf(printstring,"%s     ",PROGVERSION);  outLCD(0,2,printstring);
    sprintf(printstring,"%s",PROGDATE);  outLCD(10,2,printstring);
    sprintf(printstring,"%s   ",infoStringShort);  outLCD(0,3,printstring);
  }

  /**************************************************!
  @brief    Function to display data for various sensors on LCD display
  @details  outputs data based on lcdDisplayMode, and for sensors actually present
  @param int lcdDisplayMode
  @param float p  : BME280 air pressure
  @param float t  : BME280 
  @param float h  : BME280 
  @param float T1  : DS18B20 temperature 1
  @param float T2  : DS18B20 temperature 2
  @param float T3  : DS18B20 temperature 3 (not used)
  @param int CO2ppm : CO2 concentration in ppm (not used)
  @param float iT0  : Infactory 433 MHz temperature Channel 1
  @param float iH0  : Infactory 433 MHz humidity Channel 1
  @param float iT1  : Infactory 433 MHz temperature Channel 2
  @param float iH1  : Infactory 433 MHz humidity Channel 2
  @param char* timestring : present local time 
  @param char* infoStringShort : short information about the hardware device, the box.
  @return   void
  ***************************************************/
  void displayLCD(int lcdDisplayMode,float p, float t, float h, 
    float T1, float T2, float T3, 
    int CO2ppm,
    float iT0, float iH0, float iT1, float iH1, 
    char* timestring, char* infoStringShort)
  {
    char printstring[40];
    char sT1[20], sT0[20], sH0[20], sH1[20]; // strings to hold converted Infactory temps and humidities 

    // ensure that 433MHz data are in permitted range, "--" otherwise
    #if defined receiveSERIAL
      if(iT0>-110) 
        sprintf(sT0,"%3.1f", iT0);
      else 
        sprintf(sT0,"---");

      if(iT1>-110) 
        sprintf(sT1,"%3.1f", iT1);
      else 
        sprintf(sT1,"---");

      if(iH0>-110) 
        sprintf(sH0,"%2.0f", iH0);
      else 
        sprintf(sH0,"--");

      if(iH1>-110) 
        sprintf(sH1,"%2.0f", iH1);
      else 
        sprintf(sH1,"--");
    #endif // receiveSERIAL  

    switch (lcdDisplayMode)
    {
      case 1: // mode 1: all info in condensed form
        //lcd.clear(); 
        lcd.backlight();
        #if defined getNTPTIME
          sprintf(printstring,"%s    ",timestring);
          outLCD(0,0,printstring);
        #else
          outLCD(0,0,"          ");         
        #endif
        #if defined isBME680 || defined isBME280
          sprintf(printstring,"T:%3.1f%cC", t, 223);    //223 is character for °C
          outLCD(12,0,printstring);
 
          sprintf(printstring,"P:%4.1f mB H:%3.1f%%  " ,p, h);  
          outLCD(0,1,printstring);
          //sprintf(printstring,"H:%3.1f%%",h);
          //outLCD(12,1,printstring);
        #endif

        #ifdef isOneDS18B20
          sprintf(printstring,"T1:%3.1f%cC   ",T1,223);
          outLCD(0,2,printstring);
          sprintf(printstring,"T2:%3.1f%cC ",T2,223);
          outLCD(10,2,printstring);
        #endif

        #if defined receiveSERIAL
          //sprintf(printstring,"%3.1f%c %2.0f%% %3.1f%c %2.0f%%  ",iT0,223, iH0, iT1,223, iH1);
          sprintf(printstring,"%s%c %s%% %s%c %s%%  ",sT0,223, sH0, sT1,223, sH1);
          outLCD(0,3,printstring); 
        #endif
        break;
      case 2: // BME 280 or 680 data: pressure, temperature, himidity
        //lcd.clear(); 
        lcd.backlight();
        #if defined isBME680 || defined isBME280
          sprintf(printstring,"%s- BME280   ",timestring);     outLCD(0,0,printstring);
          sprintf(printstring,"Luftdruck : %4.1f mbar", p);    outLCD(0,1,printstring);
          sprintf(printstring,"Temperatur: %3.1f%c C", t, 223);outLCD(0,2,printstring);    
          sprintf(printstring,"Luftfeucht: %3.1f%% " ,h);      outLCD(0,3,printstring);     
        #else
          sprintf(printstring,"displayLCD Mode %d", lcdDisplayMode);
          outLCD(0,0,printstring); 
        #endif
        break;
      case 3: // DS18B20 temperature data
        //lcd.clear(); 
        lcd.backlight();
          #ifdef isOneDS18B20
            sprintf(printstring,"%s-InnenTemp  ",timestring);     outLCD(0,0,printstring);
            sprintf(printstring,"1: Kabel  %3.1f%cC    ",T1,223); outLCD(0,1,printstring);
            sprintf(printstring,"2: Geraet %3.1f%cC    ",T2,223); outLCD(0,2,printstring);
            /*
            sprintf(printstring,"%s- DS18B20   ",timestring);     outLCD(0,0,printstring);
            sprintf(printstring,"Sensor 1: %3.1f%cC    ",T1,223); outLCD(0,1,printstring);
            sprintf(printstring,"Sensor 2: %3.1f%cC    ",T2,223); outLCD(0,2,printstring);
            */
            sprintf(printstring,"Sec: %3.1f            ",(float)millis()/1000);  outLCD(0,3,printstring);
            // sprintf(printstring,"                      ");        outLCD(0,3,printstring);
          #else
            sprintf(printstring,"displayLCD Mode %d", lcdDisplayMode);
            outLCD(0,0,printstring); 
          #endif
        break;  
        case 4: // info provided by external 433 MHz radio sensors
          //lcd.clear(); 
          lcd.backlight();
          #if defined receiveSERIAL
            sprintf(printstring,"%s Funksensor",timestring);                   outLCD(0,0,printstring);
            sprintf(printstring,"Sensor : CarPt GewHs ");                         outLCD(0,1,printstring);
            //sprintf(printstring,"Temp   : %4.1f%c %4.1f%c   ",iT0,223, iT1,223); outLCD(0,2,printstring);
            sprintf(printstring,"Temp   : %s%c %s%c   ",sT0,223, sT1,223); outLCD(0,2,printstring);
            /*
            sprintf(printstring,"%s Ext.433MHz",timestring);                   outLCD(0,0,printstring);
            sprintf(printstring,"Sensor : S1    S2    ");                         outLCD(0,1,printstring);
            sprintf(printstring,"Temp   : %4.1f%c %4.1f%c   ",iT0,223, iT1,223); outLCD(0,2,printstring);
            */
            //sprintf(printstring,"Feuchte:%3.0f%% %4.0f%%    ",iH0,iH1);           outLCD(0,3,printstring);
            sprintf(printstring,"Feuchte: %s%%   %s%%   ",sH0,sH1);           outLCD(0,3,printstring);
          #else
            sprintf(printstring,"displayLCD Mode %d", lcdDisplayMode);
            outLCD(0,0,printstring); 
          #endif
        break;   
        case 5: // program info
          //lcd.clear(); 
          lcd.backlight();
          displayLCDProgInfo(infoStringShort);
        break;   
      default:
        lcd.clear(); 
        lcd.backlight();
        sprintf(printstring,"Default. Mode %d", lcdDisplayMode);
        outLCD(0,0,printstring);
        break;  
    }  
    // lcdDisplayDone = 1;
    // lcd.backlight();
    // print scrolling message
    // scrollText(1, messageToScroll, 250, lcdColumns);
  }
#endif // isLCD
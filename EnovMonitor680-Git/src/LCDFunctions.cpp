/******************************************************
* LCD Display handling functions
*******************************************************/
#include <Arduino.h>
#include "GlobalDefines.h"

#ifdef isLCD
  #include <LiquidCrystal_I2C.h>
  #include "LCDFunctions.h"

  void initLCD()
  {
    // initialize LCD
    lcd.init();
    // turn on LCD backlight                      
    lcd.backlight();
  }

  // Function to scroll text
  // The function accepts the following arguments:
  // row: row number where the text will be displayed
  // message: message to scroll
  // delayTime: delay between each character shifting
  // lcdColumns: number of columns of your LCD
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

  void outLCD(int x, int y, char* printstring)
  {
    char outstring[40]="                    ";

    // strcpy(outstring,"                    ");
    strncpy(outstring, printstring,20);
    lcd.setCursor(x, y);
    lcd.print(outstring);
  }

  // display program info
  void displayLCDProgInfo(char *infoStringShort)
  {
    char printstring[40];
    lcd.clear();  
    lcd.backlight();
    
    sprintf(printstring,"* EnvMonitor by MP *");          outLCD(0,0,printstring);
    sprintf(printstring,"%s",PROGNAME);                   outLCD(0,1,printstring);
    // sprintf(printstring,"%s - %s",PROGVERSION,PROGDATE);  outLCD(0,2,printstring);
    sprintf(printstring,"%s",PROGVERSION);  outLCD(0,2,printstring);
    sprintf(printstring,"%s",PROGDATE);  outLCD(10,2,printstring);
    sprintf(printstring,"%s",PROGDATE);  outLCD(0,3,infoStringShort);
  }

  // display data for various sensors on LCD display
  void displayLCD(int lcdDisplayMode,float p, float t, float h, 
    float T1, float T2, float T3, 
    int CO2ppm,
    float iT0, float iH0, float iT1, float iH1, 
    char* timestring, char* infoStringShort)
  {
    char printstring[40];
    // lcd.clear();  
    // lcd.backlight();

    switch (lcdDisplayMode)
    {
      case 1:
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
          sprintf(printstring,"%3.1f%c %2.0f%% %3.1f%c %2.0f%%  ",iT0,223, iH0, iT1,223, iH1);
          outLCD(0,3,printstring); 
        #endif
        break;
      case 2:
        //lcd.clear(); 
        lcd.backlight();
        #if defined isBME680 || defined isBME280
          sprintf(printstring,"%s- BME280   ",timestring);                outLCD(0,0,printstring);
          sprintf(printstring,"Luftdruck : %4.1f mbar", p);    outLCD(0,1,printstring);
          sprintf(printstring,"Temperatur: %3.1f%c C", t, 223);outLCD(0,2,printstring);    
          sprintf(printstring,"Luftfeucht: %3.1f%% " ,h);      outLCD(0,3,printstring);     
        #else
          sprintf(printstring,"displayLCD Mode %d", lcdDisplayMode);
          outLCD(0,0,printstring); 
        #endif
        break;
      case 3:
        //lcd.clear(); 
        lcd.backlight();
          #ifdef isOneDS18B20
            sprintf(printstring,"%s- DS18B20   ",timestring);     outLCD(0,0,printstring);
            sprintf(printstring,"Sensor 1: %3.1f%cC    ",T1,223); outLCD(0,1,printstring);
            sprintf(printstring,"Sensor 2: %3.1f%cC    ",T2,223); outLCD(0,2,printstring);
            sprintf(printstring,"                      ");        outLCD(0,3,printstring);
          #else
            sprintf(printstring,"displayLCD Mode %d", lcdDisplayMode);
            outLCD(0,0,printstring); 
          #endif
        break;  
        case 4:
          //lcd.clear(); 
          lcd.backlight();
          #if defined receiveSERIAL
            sprintf(printstring,"%s Ext.433MHz",timestring);                   outLCD(0,0,printstring);
            sprintf(printstring,"Sensor : S1    S2    ");                         outLCD(0,1,printstring);
            sprintf(printstring,"Temp   : %4.1f%c %4.1f%c   ",iT0,223, iT1,223); outLCD(0,2,printstring);
            sprintf(printstring,"Feuchte:%3.0f%% %4.0f%%    ",iH0,iH1);           outLCD(0,3,printstring);
          #else
            sprintf(printstring,"displayLCD Mode %d", lcdDisplayMode);
            outLCD(0,0,printstring); 
          #endif
        break;   
        case 5:
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
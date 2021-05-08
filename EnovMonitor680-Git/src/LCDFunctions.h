/******************************************************
* LCD display handling functions
*******************************************************/

#ifndef _LCD        // prevent double includes and resulting errors
  #define _LCD

  #ifdef isLCD
    #include <LiquidCrystal_I2C.h>

    // module global variables  
    // set the LCD number of columns and rows  
    // 'static' prevents the linker from complaining, see
    // https://stackoverflow.com/questions/14909997/why-arent-my-include-guards-preventing-recursive-inclusion-and-multiple-symbol

    static int lcdColumns = 20;
    static int lcdRows = 4;

    // set LCD address, number of columns and rows
    // default display address is 0x27
    static LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);  

    // forward declarations for functions
    void initLCD();
    void scrollText(int row, String message, int delayTime, int lcdColumns);
    void displayLCDProgInfo(char *infoStringShort);
    void displayLCD(int lcdDisplayMode,float p, float t, float h, 
      float T1, float T2, float T3, 
      int CO2ppm,
      float iT0, float iH0, float iT1, float iH1, 
      char* timestring, char* infoStringShort);
  #endif

#endif //_LCD
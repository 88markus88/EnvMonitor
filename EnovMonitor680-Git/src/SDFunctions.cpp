/******************************************************
* SD card handling functions
*******************************************************/
#include "GlobalDefines.h"

#ifdef isSD
  #include "FS.h"
  #include "SD.h"
  #include <SPI.h>
  #include "SDFunctions.h"

  // Write to the SD card (DON'T MODIFY THIS FUNCTION)
  void writeFile(fs::FS &fs, const char * path, const char * message) {
    Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file) {
      Serial.println("Failed to open file for writing");
      return;
    }
    if(file.print(message)) {
      // Serial.println("File written");
    } else {
      Serial.println("Write failed");
    }
    file.close();
  }

  // Append data to the SD card (DON'T MODIFY THIS FUNCTION)
  void appendFile(fs::FS &fs, const char * path, const char * message) {
    // Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file) {
      Serial.println("Failed to open file on SD for appending");
      return;
    }
    if(file.print(message)) {
      // Serial.println("Message appended");
    } else {
      Serial.println("Append to file on SD failed");
    }
    file.close();
  }

  void initSDCard(char* logfilename)
  {
    int i=0;
    
    // Initialize SD card
    while(!SD.begin(SD_CS))
    {
      i++;
      if (i>20) return;
      delay(100);
    }
    Serial.printf("SD initialized after %d attempts\n", i);
    if(!SD.begin(SD_CS)) {
      Serial.println("Card Mount Failed");
      return;
    }
    uint8_t cardType = SD.cardType();
    if(cardType == CARD_NONE) {
      Serial.println("No SD card attached");
      return;
    }
    Serial.println("Initializing SD card...");
    if (!SD.begin(SD_CS)) {
      Serial.println("ERROR - SD card initialization failed!");
      return;    // init failed
    }
    // If the data.txt file doesn't exist
    File file = SD.open(logfilename);
    if(!file) {
      Serial.println("File doesn't exist");
      Serial.println("Creating file...");
      writeFile(SD, logfilename, "Logfile for PROGNAME, PROGVERSION, PROGDATE \n");
    }
    else {
      Serial.println("File already exists");  
    }
    file.close();
  }
#endif

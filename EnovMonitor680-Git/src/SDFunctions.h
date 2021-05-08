/******************************************************
* SD card handling functions
*******************************************************/

#ifdef isSD
  #define SD_CS 5
#endif

/************************************************************
* Forward declarations
*************************************************************/
#ifdef isSD
  void initSDCard(char* logfilename);
  void writeFile(fs::FS &fs, const char * path, const char * message);
  void appendFile(fs::FS &fs, const char * path, const char * message);
#endif

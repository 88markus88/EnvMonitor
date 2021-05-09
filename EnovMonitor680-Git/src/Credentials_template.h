// ***** template for private credentials. Never share, never put on github!
// rename this file to "Credentials.h", fill in your own auth tokens and Wifi SSID and password.
// add #ifdef blocks for all your devices, which are defined in 'GlobalDefines.h"

  // You should get Auth Token in the Blynk App.
  // Go to the Project Settings (nut icon).
  // EnvMonitor on Blynk Web account
  #ifdef blynkMyDevice                    // this block defines the hardware specific settings for a device, depending on sensors present
    char auth[] = "<auth token>";         // place the auth token for the device here
    float corrDS18B20[5]={0, 0, 0 };      // correction offsets for 3 DS18B20. Area added to the raw value
    float corrBME280Temp = 0;             // correction offset for BME280 temperature. Is added to the raw value
    char infoStringLong[] = " My standard sensor: Small Box, OLED, BME280, 2 DS18B20";  // long device info string
    char infoStringShort[] = "Standard Sensor";                                         // short device info string 
  #endif  
 
  // Your WiFi credentials. Required to log into local network

  char ssid[] = "<your wifi network ID>";
  char pass[] = "<your wifi password>";

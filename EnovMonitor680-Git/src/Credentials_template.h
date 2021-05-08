// ***** template for private credentials. Never share, never put on github!
// rename this file to "Credentials.h", fill in your own auth tokens and Wifi SSID and password.
// add #ifdef blocks for all your devices, which are defined in 'GlobalDefines.h"

  // You should get Auth Token in the Blynk App.
  // Go to the Project Settings (nut icon).
  // EnvMonitor on Blynk Web account hinkelhurz@web.de. Jetzt bei Papa
  #ifdef blynkMyDevice
    char auth[] = "<auth token>>";
    float corrDS18B20[5]={0, 0, 0 };
    float corrBME280Temp = 0;
    char infoStringLong[] = " My standard sensor: Small Box, OLED, BME280, 2 DS18B20";
    char infoStringShort[] = "Standard Sensor";
  #endif  
 

  // Your WiFi credentials.

  char ssid[] = "<your wifi network ID>";
  char pass[] = "<your wifi password>";

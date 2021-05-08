// ***** private credentials. Never share, never put on github!

  // You should get Auth Token in the Blynk App.
  // Go to the Project Settings (nut icon).
  // EnvMonitor on Blynk Web account hinkelhurz@web.de. Jetzt bei Papa
  #ifdef blynkWebHinkelhurz
    char auth[] = "csqmM5Sip0tP2X2Lx7FPNa6-hWU-KdKG";
    float corrDS18B20[5]={0, -1.4, 0 };
    float corrBME280Temp =-0.7;
    char infoStringLong[] = " Small Sensor Hans: Small Box, OLED, BME280, 2 DS18B20";
    char infoStringShort[] = "Small Sensor Hans";
  #endif  
  // EnvMonitor BME680 local, mit CO2 - Monitor, BME 680, Display, 3 DS18B20
  #ifdef blynkBME680Kueche
    char auth[] = "9y7JsDeUhfz6ckVvl110m1vCjCWDw7FP";
    float corrDS18B20[5]={0, 0, 0 };
    float corrBME680Temp =0;
    char infoStringLong[] = " BME680 Küche: Eurobox, OLED, BME680, 3 DS18B20, MH-Z14a, Fan";
    char infoStringShort[] = "BME680 Küche";
  #endif  
  // EnvMonitor on Blynk local account on Raspi, Schlafzimmer. Display, 2 DS18B20, BME260, Button
  #ifdef blynkSchlafzimmer
    char auth[] = "ztfJczd73tgs7UXU7YsruorrbPT-llCC";
    float corrDS18B20[5]={0, -1.7, 0 };
    float corrBME280Temp =-1.0;
    char infoStringLong[] = " Small Sensor Schlafzimmer: Small Box, OLED, BME280, 2 DS18B20";
    char infoStringShort[] = "Schlafzimmer";
  #endif
  // KleinerEnvMonitorLoc auf local raspi (EnvLocal2) Bad
  #ifdef blynkEnvLocal2Bad
    char auth[] = "dDwl3E1zUdtP5gtttsjAjvGIz6k2HZDz";
    float corrDS18B20[5]={ 0, -0.6, 0 };
    float corrBME280Temp =-0.8;
    char infoStringLong[] = " Small Sensor Bad: Small Box, BME280, 2 DS18B20";
    char infoStringShort[] = "Small Sensor Bad";
  #endif
  // KombiSensorExt-LCD. LCD in Black Box (was Infactory External S)
  #ifdef blynkInfactoryExternalS
    char auth[] ="bfHu8uxsAw40Vz8JM3MWD8xykH7suq-o";
    float corrDS18B20[5]={0, -2.0, 0 };
    float corrBME280Temp = -2.5; 
    char infoStringLong[] = " KombiSensorExtLCD: Black Eurobox with LCD. Ext433 via serial, BME280, 2 DS18B20";
    char infoStringShort[] = "KombiSensorExtLCD";
  #endif
  // Red Box, ex SenseAirTestbed
  #ifdef blynkSenseAirRedBox
    char auth[] ="wTWlloKCK5uVkuWEojL5ojDxlfAWOZxU";
    float corrDS18B20[5]={0, 0, 0 };
    float corrBME280Temp =0;
    char infoStringLong[] = " SenseAirRedBox: Eurobox, OLED, BME280, 3 DS18B20, SenseAir S8";
    char infoStringShort[] = "SenseAirRedBox";
  #endif
  // Kombisensor1 Ext+Int
  #ifdef blynkKombinsensor1
    char auth[] ="y0-EGVY3VrsFBP01ogXI_wcBb4aDwNvQ";
    float corrDS18B20[5]={0, 0, 0 };
    float corrBME280Temp =0;
    char infoStringLong[] = " KombiSensor1: Black Velleman Box with OLED. Ext433 via serial, BME280, 2 DS18B20";
    char infoStringShort[] = "KombiSensor1";
  #endif

  // Your WiFi credentials.
  // char ssid[] = "Fritz7390-MP Gast"; // Gastnetz kann nicht mit dem Raspi im >Homenetz< reden!
  // char pass[] = "N2gP34917.-";
  char ssid[] = "Fritz7390-MP2.4";
  char pass[] = "Aa-v465-*U3P";

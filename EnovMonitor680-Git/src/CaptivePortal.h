/******************************************************
* Captive Portal for WiFi Credentials setting functions
*******************************************************/

#ifndef _Captive        // prevent double includes and resulting errors
  #define _Captive

  #ifdef isCaptivePortal
    #ifdef isLCD
      #include <LiquidCrystal_I2C.h>
    #endif  

    // module global variables  
    // 'static' prevents the linker from complaining, see
    // https://stackoverflow.com/questions/14909997/why-arent-my-include-guards-preventing-recursive-inclusion-and-multiple-symbol
    static volatile bool captivePortalExit = false;
 
    /************************************************************
    * Forward declarations
    *************************************************************/
    String GetEncryptionType(byte thisType);
    bool loadCredentials();
    bool saveCredentials();
    boolean CreateWifiSoftAP();
    void InitalizeHTTPServer();
    byte ConnectWifiAP();
    void SetDefaultWiFiConfig();
    void handleRoot();
    void handleNotFound();
    boolean captivePortal();
    void handleWifi();
    void handleWifiNew();
    void handleStatus();
    boolean isIp(String str);
 
    void setupCaptivePortal();
    void loopCaptivePortal(char* ss, char* pw);
  #endif
  String toStringIp(IPAddress IP);
#endif //_Captive
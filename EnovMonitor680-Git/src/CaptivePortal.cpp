/*
Captive Portal code by Tobias Kuch, from 
https://www.az-delivery.de/blogs/azdelivery-blog-fur-arduino-und-raspberry-pi/captive-portal-fuer-den-esp32-teil-2
Info regarding web server:
https://randomnerdtutorials.com/esp32-web-server-arduino-ide/
https://randomnerdtutorials.com/esp32-access-point-ap-web-server/
*/

#include "GlobalDefines.h"  // needed here to convey the #defines

#ifdef isCaptivePortal

  #include <WiFi.h>
  #include <WiFiClient.h>
  #include <WebServer.h>
  #include <ESPmDNS.h>
  #include <DNSServer.h>
  #include <EEPROM.h>
  #include <esp_task_wdt.h>       // Load Watchdog-Library
  #include "CaptivePortal.h"
  #include "HelperFunctions.h"  // logOut() contained there

  #define GPIO_OUT_W1TS_REG (DR_REG_GPIO_BASE + 0x0008)
  #define GPIO_OUT_W1TC_REG (DR_REG_GPIO_BASE + 0x000c)

  static const byte WiFiPwdLen = 25;
  static const byte APSTANameLen = 20;

  struct WiFiEEPromData
  {
    bool APSTA = true;            // Access Point or Station Mode - true AP Mode
    bool PwDReq = false;          // PasswordRequired
    bool CapPortal = true;        //CaptivePortal on in AP Mode
    char APSTAName[APSTANameLen]; // STATION /AP Point Name TO cONNECT, if definded
    char WiFiPwd[WiFiPwdLen];     // WiFiPAssword, if definded
    char ConfigValid[3];          //If Config is Vaild, Tag "TK" is required"
  };

  /* hostname for mDNS. Should work at least on windows. Try http://esp8266.local */
  const char *ESPHostname = "ESP32";

  // DNS server
  const byte DNS_PORT = 53;
  DNSServer dnsServer;

  //Conmmon Paramenters
  bool SoftAccOK = false;

  // Web server
  WebServer server(80);

  /* Soft AP network parameters */
  IPAddress apIP(172, 20, 0, 1);
  IPAddress netMsk(255, 255, 255, 0);

  unsigned long currentMillis = 0;
  unsigned long startMillis;

  /** Current WLAN status */
  short status = WL_IDLE_STATUS;

  WiFiEEPromData MyWiFiConfig;
  String temp = "";

  // target ssid and password to be read for the device. This is passed back to the main application
  char targetSSID[APSTANameLen], targetPASS[WiFiPwdLen];

  void(* localResetFunc) (void) = 0; //declare reset function @ address 0 THIS IS VERY USEFUL

  /**************************************************!
     @brief    global setup() function
    @details  automatically called at program start
    @param none
    @return void
    ***************************************************/
  void setupCaptivePortal()
  {
    REG_WRITE(GPIO_OUT_W1TS_REG, BIT(GPIO_NUM_16)); // Guru Meditation Error Remediation set
    delay(1);
    REG_WRITE(GPIO_OUT_W1TC_REG, BIT(GPIO_NUM_16)); // Guru Meditation Error Remediation clear
    
    Serial.println(F("Captive Portal Setup routine started"));
    // only to prevent compiler warnings
    Serial.println(infoStringShort);
    Serial.println(infoStringLong);

    bool ConnectSuccess = false;
    bool CreateSoftAPSucc = false;
    // bool CInitFSSystem = false;
    // bool CInitHTTPServer = false;
    byte len;
    char printstring[200];

    #undef alwaysCaptivePortal // this is for debug purposes only, reset the stored config in every case
    #ifdef alwaysCaptivePortal
      Serial.println(F("Debug: always reset stored credentials"));
      SetDefaultWiFiConfig();
      saveCredentials();
    #endif

    captivePortalExit = false; // exit flag for captive portal main loop

    WiFi.setAutoReconnect(false);
    WiFi.persistent(false);
    WiFi.disconnect();
    WiFi.setHostname(ESPHostname); // Set the DHCP hostname assigned to ESP station.
    if (loadCredentials())         // Load WLAN credentials for WiFi Settings
    {
      Serial.println(F("Valid Credentials found."));
      sprintf(printstring, "ssid: %s  pw: %s\n", MyWiFiConfig.APSTAName, MyWiFiConfig.WiFiPwd);
      Serial.print(printstring);
      if (MyWiFiConfig.APSTA == true) // AP Mode
      {
        Serial.print(F("Access Point Mode selected: "));
        Serial.println(MyWiFiConfig.APSTA); // 1=true = access point mode
        len = strlen(MyWiFiConfig.APSTAName);
        MyWiFiConfig.APSTAName[len + 1] = '\0';
        len = strlen(MyWiFiConfig.WiFiPwd);
        MyWiFiConfig.WiFiPwd[len + 1] = '\0';
        CreateSoftAPSucc = CreateWifiSoftAP();
      }
      else // Station Mode
      {
        Serial.println(F("Station Mode selected."));
        len = strlen(MyWiFiConfig.APSTAName);
        MyWiFiConfig.APSTAName[len + 1] = '\0';
        len = strlen(MyWiFiConfig.WiFiPwd);
        MyWiFiConfig.WiFiPwd[len + 1] = '\0';
        len = ConnectWifiAP();
        if (len == 3)
        {
          ConnectSuccess = true;
        }
        else
        {
          ConnectSuccess = false;
        }
      }
    }
    else
    { //Set default Config - Create AP
      Serial.println(F("NO Valid Credentials found."));
      SetDefaultWiFiConfig();
      CreateSoftAPSucc = CreateWifiSoftAP();
      saveCredentials();
      delay(500);
    }
    if ((ConnectSuccess or CreateSoftAPSucc))
    {
      Serial.print(F("IP Address: "));
      if (CreateSoftAPSucc)
        Serial.println(WiFi.softAPIP());
      if (ConnectSuccess)
        Serial.println(WiFi.localIP());
      InitalizeHTTPServer();
    }
    else    // connection not successful. set default configuration and write it to EEPROM
    {
      Serial.setDebugOutput(true); //Debug Output for WLAN on Serial Interface.
      Serial.println(F("Error: Cannot connect to WLAN. Set DEFAULT Configuration."));
      SetDefaultWiFiConfig();
      CreateSoftAPSucc = CreateWifiSoftAP();
      InitalizeHTTPServer();
      SetDefaultWiFiConfig();
      // avoid saving the default credentials, in case that the WLAN is not available at start
      // saveCredentials();
    }
  }

  /**************************************************!
    @brief    Initialize HTTP Server
    @details  set up root and /wifi pages, and captive portal. Also "not found"
    @param none
    @return void
    ***************************************************/
  void InitalizeHTTPServer()
  {
    // bool initok = false;
    /* Setup web pages: root, wifi config pages, SO captive portal detectors and not found. */
    // server.on("/", handleRoot);
    server.on("/", handleRoot);
    server.on("/wifi", handleWifiNew);
    server.on("/status", handleStatus);
    if (MyWiFiConfig.CapPortal)
    {
      server.on("/generate_204", handleRoot);
      // server.on("/generate_204", handleWifiNew);
    } //Android captive portal. Maybe not needed. Might be handled by notFound handler.
    if (MyWiFiConfig.CapPortal)
    {
      server.on("/favicon.ico", handleRoot);
      // server.on("/favicon.ico", handleWifiNew);
    } //Another Android captive portal. Maybe not needed. Might be handled by notFound handler. Checked on Sony Handy
    if (MyWiFiConfig.CapPortal)
    {
      server.on("/fwlink", handleRoot);
      // server.on("/fwlink", handleWifiNew);
    } //Microsoft captive portal. Maybe not needed. Might be handled by notFound handler.
    //server.on("/generate_204", handleRoot);  //Android captive portal. Maybe not needed. Might be handled by notFound handler.
    //server.on("/favicon.ico", handleRoot);    //Another Android captive portal. Maybe not needed. Might be handled by notFound handler. Checked on Sony Handy
    //server.on("/fwlink", handleRoot);   //Microsoft captive portal. Maybe not needed. Might be handled by notFound handler.
    server.onNotFound(handleNotFound);
    // Speicherung Header-Elemente anfordern
    // server.collectHeaders(Headers, sizeof(Headers)/ sizeof(Headers[0]));
    server.begin(); // Web server start
  }

  /**************************************************!
    @brief    Set up software access point
    @details  first disconnects, turns on the Access point. 
    @param none
    @return byte
    ***************************************************/
  boolean CreateWifiSoftAP()
  {
    WiFi.disconnect();
    Serial.print(F("Initalize SoftAP "));
    if (MyWiFiConfig.PwDReq)
    {
      SoftAccOK = WiFi.softAP(MyWiFiConfig.APSTAName, MyWiFiConfig.WiFiPwd); // Passwortlänge mindestens 8 Zeichen !
    }
    else
    {
      SoftAccOK = WiFi.softAP(MyWiFiConfig.APSTAName); // Access Point WITHOUT Password
      // Overload Function:; WiFi.softAP(ssid, password, channel, hidden)
    }
    delay(2000); // Without delay I've seen the IP address blank
    WiFi.softAPConfig(apIP, apIP, netMsk);
    if (SoftAccOK)
    {
      /* Setup the DNS server redirecting all the domains to the apIP */
      dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
      dnsServer.start(DNS_PORT, "*", apIP);
      Serial.println(F("successful."));
      // Serial.setDebugOutput(true); // Debug Output for WLAN on Serial Interface.
    }
    else
    {
      Serial.println(F("Soft AP Error."));
      Serial.println(MyWiFiConfig.APSTAName);
      Serial.println(MyWiFiConfig.WiFiPwd);
    }
    return SoftAccOK;
  }

  /**************************************************!
    @brief    Connect to an access point, using the ssid and password from configuration data
    @details  first disconnects, turns of the Access point. Then connects, sets up MDNS responder
    @param none
    @return byte
    ***************************************************/
  byte ConnectWifiAP()
  {
    Serial.println(F("Initalizing Wifi Client."));
    byte connRes = 0;
    byte i = 0;
    WiFi.disconnect();
    WiFi.softAPdisconnect(true); // Function will set currently configured SSID and password of the soft-AP to null values. The parameter  is optional. If set to true it will switch the soft-AP mode off.
    delay(500);
    WiFi.begin(MyWiFiConfig.APSTAName, MyWiFiConfig.WiFiPwd); // connect to a wifi using stored credentials
    connRes = WiFi.waitForConnectResult();                    // get the connection result
    // Wifi info: https://randomnerdtutorials.com/esp32-useful-wi-fi-functions-arduino/ 
    while ((connRes == 0) and (i != 10)) //if connRes == 0  "WL_IDLE_STATUS - change Status"  // wait for connection up to 20 sec if idle status (0)
    {
      connRes = WiFi.waitForConnectResult();
      delay(2000);
      i++;
      Serial.print(F("."));
      // statement(s)
    }
    while ((connRes == 1) and (i != 10)) //if connRes == 1  WL_NO_SSID_AVAILin - SSID cannot be reached // wait for connection up to 20 sec if target network not reachable (1)
    {
      connRes = WiFi.waitForConnectResult();
      delay(2000);
      i++;
      Serial.print(F("."));
      // statement(s)
    }
    if (connRes == 3) // WL_CONNECTED, connection to a network is established
    {
      WiFi.setAutoReconnect(true); // Set whether module will attempt to reconnect to an access point in case it is disconnected.                          
      if (!MDNS.begin(ESPHostname)) // Setup MDNS responder, allows to resolve address of web server on ESP32 https://techtutorialsx.com/2020/04/17/esp32-mdns-address-resolution/ 
      {
        Serial.println(F("Error: MDNS"));
      }
      else
      {
        MDNS.addService("http", "tcp", 80);
      }
    }
    while ((connRes == 4) and (i != 10)) //if connRes == 4  WL_CONNECT_FAILED == Bad Password. Sometimes happens even with corrct PWD
    {
      WiFi.begin(MyWiFiConfig.APSTAName, MyWiFiConfig.WiFiPwd);
      connRes = WiFi.waitForConnectResult();
      delay(3000);
      i++;
      Serial.print(F("."));
    }
    if (connRes == 4)
    {
      Serial.println(F("STA Pwd Err"));
      Serial.println(MyWiFiConfig.APSTAName);
      Serial.println(MyWiFiConfig.WiFiPwd);
      WiFi.disconnect();
    }
    // if (connRes == 6 ) { Serial.println("WL_DISCONNECTED - Not in station mode"); }
    // WiFi.printDiag(Serial);
    Serial.println("");
    return connRes;
  }

  /**************************************************!
    @brief    Read Access Point Wifi credentials from EEPROM
    @details  https://randomnerdtutorials.com/esp32-flash-memory/ 
    @param none
    @return bool
    ***************************************************/
  /** Load WLAN credentials from EEPROM */
  bool loadCredentials()
  {
    bool RetValue;
    EEPROM.begin(512);
    EEPROM.get(0, MyWiFiConfig);
    EEPROM.end();
    if (String(MyWiFiConfig.ConfigValid) == String("TK"))
    {
      RetValue = true;
    }
    else
    {
      RetValue = false; // WLAN Settings not found.
    }
    return RetValue;
  }

  /**************************************************!
    @brief    Save Access Point Wifi Credentials to EEPROM
    @details  https://randomnerdtutorials.com/esp32-flash-memory/ 
    @param none
    @return bool
    ***************************************************/
  /** Store WLAN credentials to EEPROM */
  bool saveCredentials()
  {
    bool RetValue;
    // Check logical Errors
    char printstring[100];
    sprintf(printstring,"Save AP Credentials called: ssid: %s PW: %s \n",
      MyWiFiConfig.APSTAName,MyWiFiConfig.WiFiPwd);
    Serial.print(printstring);
    RetValue = true;
    if (MyWiFiConfig.APSTA == true) //AP Mode
    {
      if (MyWiFiConfig.PwDReq and (sizeof(String(MyWiFiConfig.WiFiPwd)) < 8))
      {
        RetValue = false; // Invalid Config
      }
      if (sizeof(String(MyWiFiConfig.APSTAName)) < 1)
      {
        RetValue = false; // Invalid Config
      }
    }
    if (RetValue)
    {
      EEPROM.begin(512);
      for (int i = 0; i < sizeof(MyWiFiConfig); i++)
      {
        EEPROM.write(i, 0);
      }
      strncpy(MyWiFiConfig.ConfigValid, "TK", sizeof(MyWiFiConfig.ConfigValid));
      EEPROM.put(0, MyWiFiConfig);
      EEPROM.commit();
      EEPROM.end();
    }
    return RetValue;
  }

  /**************************************************!
    @brief    Set default wifi configuration data. 
    @details  Access Point, PW required, Captive portal, AP name, password, marker for valid configuration
    @param none
    @return void
    ***************************************************/
  void SetDefaultWiFiConfig()
  {
    byte len;
    MyWiFiConfig.APSTA = true;  // Access Point or Station Mode - false == Station Mode, true = Access Point mode
    MyWiFiConfig.PwDReq = true; // default PW required
    MyWiFiConfig.CapPortal = true;
    strncpy(MyWiFiConfig.APSTAName, "ESP_Config", sizeof(MyWiFiConfig.APSTAName));  // Name of the station
    len = strlen(MyWiFiConfig.APSTAName);
    MyWiFiConfig.APSTAName[len + 1] = '\0';
    strncpy(MyWiFiConfig.WiFiPwd, "12345678", sizeof(MyWiFiConfig.WiFiPwd));        // default password
    len = strlen(MyWiFiConfig.WiFiPwd);
    MyWiFiConfig.WiFiPwd[len + 1] = '\0';
    strncpy(MyWiFiConfig.ConfigValid, "TK", sizeof(MyWiFiConfig.ConfigValid));      // marker for valid configuration
    len = strlen(MyWiFiConfig.ConfigValid);
    MyWiFiConfig.ConfigValid[len + 1] = '\0';
    Serial.println(F("Reset WiFi Credentials."));
  }

  /**************************************************!
    @brief    Handle "root" : Hauptseite bauen und an client senden
    @details  
    @param none
    @return void
    ***************************************************/
  void handleRoot()
  {
    //  Main Page:
    temp = "";
    Serial.println("'handleRoot()' called");
    #define noMainDialog
    #ifdef noMainDialog
      handleWifiNew();
    #else  
      // byte PicCount = 0;
      // byte ServArgs = 0;
      // HTML Header
      server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
      server.sendHeader("Pragma", "no-cache");
      server.sendHeader("Expires", "-1");
      server.setContentLength(CONTENT_LENGTH_UNKNOWN);
      // HTML Content
      server.send(200, "text/html", temp); // Speichersparen - Schon mal dem Cleint senden
      temp = "";
      temp += "<!DOCTYPE HTML><html lang='de'><head><meta charset='UTF-8'><meta name= viewport content='width=device-width, initial-scale=1.0,'>";
      server.sendContent(temp);
      temp = "";
      temp += "<style type='text/css'><!-- DIV.container { min-height: 10em; display: table-cell; vertical-align: middle }.button {height:35px; width:90px; font-size:16px}";
      server.sendContent(temp);
      temp = "";
      temp += "body {background-color: powderblue;}</style>";
      temp += "<head><title>Hauptseite</title></head>";
      temp += "<h2>Hauptseite</h2>";
      temp += "<body>";
      server.sendContent(temp);
      temp = "";
      // Processing User Request
      temp = "";
      temp += "<table border=2 bgcolor = white width = 400 cellpadding =5 ><caption><p><h3>Systemlinks:</h2></p></caption>";
      temp += "<tr><th><br>";
      temp += "<a href='/wifi'>WIFI Einstellungen</a><br><br>";
      temp += "</th></tr></table><br><br>";
      temp += "<footer><p>Programmed and designed by: Tobias Kuch</p><p>Contact information: <a href='mailto:tobias.kuch@googlemail.com'>tobias.kuch@googlemail.com</a>.</p></footer>";
      temp += "</body></html>";
      server.sendContent(temp);
      temp = "";
      server.client().stop(); // Stop is needed because we sent no content length
    #endif  
  }

/**************************************************!
    @brief    Handle "status" : Status-Seite bauen und an client senden
    @details  
    @param none
    @return void
    ***************************************************/
  void handleStatus()
  {
    //  Main Page:
    temp = "";

    Serial.println("'handleStatus()' called");
    // byte PicCount = 0;
    // byte ServArgs = 0;
    // HTML Header
    server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    server.sendHeader("Pragma", "no-cache");
    server.sendHeader("Expires", "-1");
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    // HTML Content
    server.send(200, "text/html", temp); // Speichersparen - Schon mal dem Cleint senden
    temp = "";
    temp += "<!DOCTYPE HTML><html lang='de'><head><meta charset='UTF-8'><meta name= viewport content='width=device-width, initial-scale=1.0,'>";
    server.sendContent(temp);
    temp = "";
    temp += "<style type='text/css'><!-- DIV.container { min-height: 10em; display: table-cell; vertical-align: middle }.button {height:35px; width:90px; font-size:16px}";
    server.sendContent(temp);
    temp = "";
    temp += "body {background-color: powderblue;}</style>";
    temp += "<head><title>Status</title></head>";
    temp += "<h2>Status Page</h2>";
    temp += "<body>";
    server.sendContent(temp);

    temp = "";
    // H4: "Current Wifi Settings"
    temp += "<table border=2 bgcolor = white width = 420 >";
    temp +=      "<caption><h4>Current WiFi Settings: </h4></caption>";
    temp +=      "<tr>";
    temp +=        "<td>";  
    temp +=           "SSID";
    temp +=        "</td>";
    temp +=        "<td>"; 
    temp +=           MyWiFiConfig.APSTAName;
    temp +=        "</td>";
    temp +=      "</tr>";
    temp +=      "<tr>";
    temp +=        "<td>";  
    temp +=           "Password" ;
    temp +=        "</td>";
    temp +=        "<td>"; 
    temp +=           MyWiFiConfig.WiFiPwd;
    temp +=        "</td>";
    temp +=      "</tr>";
    temp +=      "<tr>";
    temp +=        "<td>";  
    temp +=           "Config Valid" ;
    temp +=        "</td>";
    temp +=        "<td>"; 
    temp +=           MyWiFiConfig.ConfigValid;
    temp +=        "</td>";
    temp +=      "</tr>";
    temp +="</table><br>";
    server.sendContent(temp);

    temp = "";
    // Processing User Request
    temp = "";
    temp += "<table border=2 bgcolor = white width = 400 cellpadding =5 ><caption><p><h3>Systemlinks:</h2></p></caption>";
    temp += "<tr><th><br>";
    temp += "<a href='/'>Main Page</a><br><br>";
    temp += "</th></tr></table><br><br>";
    temp += "<footer><p>Programmed and designed by: Markus P.</p></footer>";
    temp += "</body></html>";
    server.sendContent(temp);
    temp = "";
    server.client().stop(); // Stop is needed because we sent no content length
  }
  /**************************************************!
     @brief    Handle "Not found", send this to client. Or go to captive partal if that is set.
    @details  Displays "404 File not found" if not in captive portal
    @param none
    @return void
    ***************************************************/
  void handleNotFound()
  {
    Serial.println("'handleNotFound()' called");
    if (captivePortal())
    { // If caprive portal redirect instead of displaying the error page.
      return;
    }
    temp = "";
    // HTML Header
    server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    server.sendHeader("Pragma", "no-cache");
    server.sendHeader("Expires", "-1");
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    // HTML Content
    temp += "<!DOCTYPE HTML><html lang='de'><head><meta charset='UTF-8'><meta name= viewport content='width=device-width, initial-scale=1.0,'>";
    temp += "<style type='text/css'><!-- DIV.container { min-height: 10em; display: table-cell; vertical-align: middle }.button {height:35px; width:90px; font-size:16px}";
    temp += "body {background-color: powderblue;}</style>";
    temp += "<head><title>File not found</title></head>";
    temp += "<h2> 404 File Not Found</h2><br>";
    temp += "<h4>Debug Information:</h4><br>";
    temp += "<body>";
    temp += "URI: ";
    temp += server.uri();
    temp += "\nMethod: ";
    temp += (server.method() == HTTP_GET) ? "GET" : "POST";
    temp += "<br>Arguments: ";
    temp += server.args();
    temp += "\n";
    for (uint8_t i = 0; i < server.args(); i++)
    {
      temp += " " + server.argName(i) + ": " + server.arg(i) + "\n";
    }
    temp += "<br>Server Hostheader: " + server.hostHeader();
    for (uint8_t i = 0; i < server.headers(); i++)
    {
      temp += " " + server.headerName(i) + ": " + server.header(i) + "\n<br>";
    }
    temp += "</table></form><br><br><table border=2 bgcolor = white width = 420 cellpadding =5 ><caption><p><h2>You may want to browse to:</h2></p></caption>";
    temp += "<tr><th>";
    temp += "<a href='/'>Main Page</a><br>";
    temp += "<a href='/wifi'>WIFI Settings</a><br>";
    temp += "</th></tr></table><br><br>";
    temp += "<footer><p>Programmed by: Tobias Kuch</p><p>Contact information: <a href='mailto:tobias.kuch@googlemail.com'>tobias.kuch@googlemail.com</a>.</p></footer>";
    temp += "</body></html>";
    server.send(404, "", temp);
    server.client().stop(); // Stop is needed because we sent no content length
    temp = "";
  }

  /**************************************************!
    @brief    Redirect to captive portal if we got a request for another domain
    @details  Return true in that case so the page handler do not try to handle the request again.
    @param none
    @return boolean
    ***************************************************/
  /** Redirect to captive portal if we got a request for another domain. Return true in that case so the page handler do not try to handle the request again. */
  boolean captivePortal()
  {
    Serial.println("'captivePortal()' called");
    if (!isIp(server.hostHeader()) && server.hostHeader() != (String(ESPHostname) + ".local"))
    {
      // Serial.println("Request redirected to captive portal");
      server.sendHeader("Location", String("http://") + toStringIp(server.client().localIP()), true);
      server.send(302, "text/plain", ""); // Empty content inhibits Content-length header so we have to close the socket ourselves.
      server.client().stop();             // Stop is needed because we sent no content length
      return true;
    }
    return false;
  }

  /**************************************************!
    @brief    new Wifi config page handler subfunction draw HTML
    @details  simplified to remove unnecessary stuff
    @param none
    @return void
  ***************************************************/
  void drawWifiNewPage()
  {
    temp = "";
    //----- HTML Header
    server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    server.sendHeader("Pragma", "no-cache");
    server.sendHeader("Expires", "-1");
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    //----- HTML Content
    temp += "<!DOCTYPE HTML><html lang='de'><head><meta charset='UTF-8'><meta name= viewport content='width=device-width, initial-scale=1.0,'>";
    server.send(200, "text/html", temp);
    temp = "<style type='text/css'><!-- DIV.container { min-height: 10em; display: table-cell; vertical-align: middle }.button {height:35px; width:90px; font-size:16px}";
    temp += "body {background-color: powderblue;}</style><head><title>WiFi Settings</title></head>";
    server.sendContent(temp);
    // Tabelle. Titel als H2 darüber: "Wifi Einstellungen"
    temp = "<h2>WiFi Settings</h2><body>";
    server.sendContent(temp);
    // "Current Wifi Settings"

    temp = "<table border=2 bgcolor = white width = 420 >";
    temp+= "<tr><th>";
    temp+= "Current ESP 32 WiFi Settings:";
    temp+= "<table border=1 bgcolor = white width = 410>";
    server.sendContent(temp);
    if (server.client().localIP() == apIP)
    {
      temp  = "<tr><td> Mode </td><td> Soft Access Point (AP)</td></tr>";
      temp += "<tr><td>SSID </td><td> " + String(MyWiFiConfig.APSTAName) + "</td></tr>";
    }
    else
    {
      temp  = "<tr><td>Mode </td><td> Station (STA) </td></tr>";
      temp += "<tr><td>SSID  </td><td> " + String(MyWiFiConfig.APSTAName) + "</td></tr>";
      temp += "<tr><td>BSSID </td><td> " + WiFi.BSSIDstr() + "</td></tr>";
    }
    temp+= "</table>";
    temp+= "</th></tr>";
    temp+= "</table><br>";
    server.sendContent(temp);

    // Table of Wifi networks
    temp  = "<form action='/wifi' method='post'>";
    temp += "<table border=2 bgcolor = white width = 420>";
    temp += "<tr><th>Available WiFi Networks:";
    server.sendContent(temp);
    temp  = "<table border=1 bgcolor = white width = 410>";
    // now do the wifi scan 
    WiFi.scanDelete();
    int n = WiFi.scanNetworks(false, false); //WiFi.scanNetworks(async, show_hidden)
    if (n > 0)
    {
      for (int i = 0; i < n; i++)
      {
        // build the table rows from number, SSID, encryption type, RSSI (network strength )
        temp += "<tr>";
        String Nrb = String(i);     // number of the network found, simply convert "i" to String "Nrb"
        temp += "<td>" + Nrb + "</td>";
        temp += "<td>" + WiFi.SSID(i) + "</td>";
        Nrb = GetEncryptionType(WiFi.encryptionType(i));
        temp += "<td>" + Nrb + "</td>";
        temp += "<td>" + String(WiFi.RSSI(i)) + "</td>";
        temp += "</tr>";
      }
    }
    else
    {
      temp += "<tr>";
      temp += "<td>1 </td>";
      temp += "<td>No WLAN found</td>";
      temp += "<td> --- </td>";
      temp += "<td> --- </td>";
      temp += "</tr>";
    }
    temp += "</tr>";
    temp += "</table>";
    server.sendContent(temp);

    // start the next table. That contains on the left the "connect to wifi ssid", on the right a selector for the network
    temp = "<table border=1 bgcolor = white width = 410>";
    temp += "<tr>";
    temp += "<td>Connect to WiFi SSID: </td>";
    temp += "<td>";
    temp += "<select name='WiFi_Network' >";
    if (n > 0)
      for (int i = 0; i < n; i++)
        temp += "<option value='" + WiFi.SSID(i) + "'>" + WiFi.SSID(i) + "</option>";
    else
      temp += "<option value='No_WiFi_Network'>No WiFiNetwork found !/option>";
    temp += "</select>";   
    temp += "</td>";  
    temp += "</tr>";  
    server.sendContent(temp);

    // enter the wifi password for the selected network
    temp = "<tr>";
    temp += "<td>Wifi Password: </td>";
    temp += "<td><input type='text' name='STAWLanPW' maxlength='40'></td>";
    temp += "</tr>";
    temp += "<br>";
    temp += "</table>";
    
    // Button for "Save wifi settings"
    temp += "<button type='submit' name='Settings' value='1' style=' color:blue;  border-radius: 0.5em; padding:0.4em; margin: 0.3em;' autofocus >Save Client Wifi Settings</button>";
    temp += "</table>";
    temp += "<br>";
    server.sendContent(temp);

    // New table: "wifi access point"
    temp  = "<table border=2 bgcolor = white width = 420 >";
    temp += "<tr><th>WiFi Access Point Settings";
    temp += "<table border=1 bgcolor = white width = 410>";
    temp += "</tr></th>";
    temp += "<td>WiFi Access Point Name: </td>";
    temp += "<td><input type='text' name='APPointName' maxlength='" + String(APSTANameLen - 1) + "' size='30' value='" + String(MyWiFiConfig.APSTAName) + "'></td>";
    temp += "</tr></th>";
    temp += "<td>WiFi Password: </td>";
    temp += "<td><input type='password' name='APPW' maxlength='" + String(WiFiPwdLen - 1) + "' size='30' value='" + String(MyWiFiConfig.WiFiPwd) + "'></td>";
    temp += "</tr></th>";
    temp += "<td>Repeat WiFi Password: </td>";
    temp += "<td><input type='password' name='APPWRepeat' maxlength='" + String(WiFiPwdLen - 1) + "' size='30' value='" + String(MyWiFiConfig.WiFiPwd) + "'> </td>";
    temp += "</table>";
    server.sendContent(temp);

    // Checkmarks
    temp  = "<table>";
    temp += "<input type='checkbox' name='PasswordReq' checked> Login Password required";
    temp += "<input type='checkbox' name='CaptivePortal' checked> Captive Portal active";
    temp += "<br>";
    temp += "<button type='submit' name='SavePortalSettings' value='1' style=' color:blue;  border-radius: 0.5em; padding:0.4em; margin: 0.3em;'  >Save Portal Wifi Settings</button>";
    temp += "</table>";

    temp += "</table>";
    temp += "<br>";
    server.sendContent(temp);

    temp  = "<button type='submit' name='Reboot ESP32' value='1' style=' color:blue;  border-radius: 0.5em; padding:0.4em; margin: 0.3em;'>Reboot System</button>";
    temp += "<button type='reset' name='action' value='1' style=' color:blue;  border-radius: 0.5em; padding:0.4em; margin: 0.3em;'>Reset Access Point settings</button>";
    temp += "</form>";
    server.sendContent(temp);
    
    // system links
    temp  = "<table border=2 bgcolor = white width = 420 cellpadding =5 >";
	  temp += "<tr><th>";
	  temp += "<b>System Links:</b>";
	  temp += "</tr></th>";
	  temp += "<tr><td><a href='/'>Main Page</a></td></tr>";
	  temp += "<tr><td><a href='/status'>Markus Status Page</a></td></tr>";
    temp += "</table>";

    temp += "<footer><p>Programmed and designed by: Markus P.</p></footer>";
    temp += "</body></html>";
    server.sendContent(temp);
    server.client().stop(); // Stop is needed because we sent no content length
    temp = "";
  }

  /**************************************************!
    @brief    New 17.11.21 Wifi config page handler
    @details  simplified to remove unnecessary stuff
    @param none
    @return void
    ***************************************************/
  void handleWifiNew()
  {
    //  Page: /wifi
    byte i;
    byte len;
    temp = "";

    Serial.println(F("'handleWifiNew()' called"));
    // Check for Site Parameters
    if (server.hasArg("Reboot")) // Reboot System
    {
      Serial.println("'handleWifiNew()' - hasArg'Reboot'");
      temp = "Rebooting System in 5 Seconds..";
      Serial.println(temp);
      server.send(200, "text/html", temp);
      delay(5000);
      server.client().stop();
      WiFi.disconnect();
      delay(1000);
      localResetFunc(); // reset the system
    }

    if (server.hasArg("Settings")) // Set wifi settings
    {
      Serial.println(F("'handleWifiNew()' - hasArg'Settings'"));
      temp = "'Save Wifi Settings' button pressed";
      Serial.println(temp);
      temp = server.arg("WiFi_Network");
      len = temp.length();
      for (i = 0; i < len; i++)
        targetSSID[i] = temp[i];
      temp = "Network SSID: _"+ temp;   
      Serial.print(temp);
      temp = "";
      for (i = 0; i < WiFiPwdLen; i++)
        targetPASS[i] = 0;
      temp = server.arg("STAWLanPW");
      len = temp.length();
      for (i = 0; i < len; i++)
        if (temp[i] > 32) //Steuerzeichen raus
          targetPASS[i]= temp[i];
      temp = "_ Network Password: _"+ temp +"_ \n";   
      Serial.print(temp);
      bool SaveOk = saveCredentials();


    temp = "<!DOCTYPE HTML><html lang='de'><head><meta charset='UTF-8'><meta name= viewport content='width=device-width, initial-scale=1.0,'>";
    server.send(200, "text/html", temp);
    temp = "<style type='text/css'><!-- DIV.container { min-height: 10em; display: table-cell; vertical-align: middle }.button {height:35px; width:90px; font-size:16px}";
    // background color names: https://developer.mozilla.org/de/docs/Web/CSS/color_value
    temp += "body {background-color: thistle;}</style><head><title>WiFi Settings</title></head>";
    server.sendContent(temp);
    // Tabelle. Titel als H2 darüber: "Wifi Einstellungen"
    temp = "<h2>Stopping Web Server in 3 sec...</h2><body>";
    temp += "</body></html>";
    server.sendContent(temp);
    server.client().stop(); // Stop is needed because we sent no content length
    delay(3000);
    
       

    // "Current Wifi Settings"

     /*
      temp = "Stopping web server in 3 Seconds..";
      Serial.println(temp);
      server.send(200, "text/html", temp);
      delay(3000);
     */ 


      server.client().stop();
      WiFi.scanDelete();  // delete results of previous scan. worth a try...
      delay(500);
      WiFi.disconnect();
      delay(500);
      WiFi.softAPdisconnect(true); // Function will set currently configured SSID and password of the soft-AP to null values. The parameter  is optional. If set to true it will switch the soft-AP mode off.
      delay(500);
      
      captivePortalExit = true; // set flag to end the main loop
      return;
    }

    if (server.hasArg("action")) // Reset
    {
      Serial.println(F("'handleWifiNew()' - hasArg'action'"));
      temp = "'Reset' Button pressed. Ressetting Access Point wifi config data..";
      server.send(200, "text/html", temp);
      SetDefaultWiFiConfig();
      delay(1000);
      setupCaptivePortal();
      return;
    }

    // draw the HTML contents
    drawWifiNewPage();
  }


  /**************************************************!
    @brief    Wifi config page handler
    @details  handle the entries on the wifi config pages, and draw the page
    @param none
    @return void
    ***************************************************/
  void handleWifi()
  {
    //  Page: /wifi
    byte i;
    byte len;
    temp = "";
    Serial.println("'handleWifi()' called");
    // Check for Site Parameters
    if (server.hasArg("Reboot")) // Reboot System
    {
      temp = "Rebooting System in 5 Seconds..";
      Serial.println(temp);
      server.send(200, "text/html", temp);
      delay(5000);
      server.client().stop();
      WiFi.disconnect();
      delay(1000);
      localResetFunc(); // reset the system
    }

    if (server.hasArg("Settings")) // Set wifi settings
    {
      temp = "'Set Wifi Settings' button pressed";
      Serial.println(temp);
      temp = server.arg("WiFi_Network");
      len = temp.length();
      for (i = 0; i < len; i++)
        targetSSID[i] = temp[i];
      temp = "Network SSID: _"+ temp;   
      Serial.print(temp);
      temp = "";
      for (i = 0; i < WiFiPwdLen; i++)
        targetPASS[i] = 0;
      temp = server.arg("STAWLanPW");
      len = temp.length();
      for (i = 0; i < len; i++)
      {
        if (temp[i] > 32) //Steuerzeichen raus
        {
          targetPASS[i]= temp[i];
        }
      }
      temp = "_ Network Password: _"+ temp +"_ \n";   
      Serial.print(temp);
      bool SaveOk = saveCredentials();
      server.client().stop();
      WiFi.disconnect();
      // delay(1000);
      captivePortalExit = true; // end the main loop
      return;
    }

    if (server.hasArg("action")) // Reset
    {
      temp = "'Reset' Button pressed. Ressetting wifi config data..";
      server.send(200, "text/html", temp);
      SetDefaultWiFiConfig();
      delay(1000);
      setupCaptivePortal();
      return;
    }

    if (server.hasArg("WiFiMode") and (server.arg("WiFiMode") == "1")) // STA Station Mode Connect to another WIFI Station
    {
      startMillis = millis(); // Reset Time Up Counter to avoid Idle Mode while operating
      // Connect to existing STATION
      if (sizeof(server.arg("WiFi_Network")) > 0)
      {
        Serial.println("STA Mode");
        MyWiFiConfig.APSTA = false; // Access Point or Station Mode - false == Station Mode
        temp = "";
        for (i = 0; i < APSTANameLen; i++)
        {
          MyWiFiConfig.APSTAName[i] = 0;
        }
        temp = server.arg("WiFi_Network");
        len = temp.length();
        for (i = 0; i < len; i++)
        {
          MyWiFiConfig.APSTAName[i] = temp[i];
        }
        temp = "";

        for (i = 0; i < WiFiPwdLen; i++)
        {
          MyWiFiConfig.WiFiPwd[i] = 0;
        }
        temp = server.arg("STAWLanPW");
        len = temp.length();
        for (i = 0; i < len; i++)
        {
          if (temp[i] > 32) //Steuerzeichen raus
          {
            MyWiFiConfig.WiFiPwd[i] = temp[i];
          }
        }
        temp = "WiFi Connect to AP: -";
        temp += MyWiFiConfig.APSTAName;
        temp += "-<br>WiFi PW: -";
        temp += MyWiFiConfig.WiFiPwd;
        temp += "-<br>";
        temp += "Connecting to STA Mode in 2 Seconds..<br>";
        server.send(200, "text/html", temp);
        server.sendContent(temp);
        delay(2000);
        server.client().stop();
        server.stop();
        temp = "";
        WiFi.disconnect();
        WiFi.softAPdisconnect(true);
        delay(500);
        // ConnectWifiAP
        bool SaveOk = saveCredentials();
        Serial.print(F("Save Credentials 1 returned: "));
        Serial.println(SaveOk);
        i = ConnectWifiAP();
        delay(700);
        if (i != 3) // 4: WL_CONNECT_FAILED - Password is incorrect | 1: WL_NO_SSID_AVAILin - Configured SSID cannot be reached | 6_ WL_DISCONNECT
        {
          Serial.print(F("Cannot Connect to specified Network. Reason: "));  // F() Macro: combines FPSTR() and PSTR(), makes string a PROGMEM string (Flash instead SRAM) and saves RAM
          Serial.println(i);
          server.client().stop();
          delay(100);
          WiFi.setAutoReconnect(false);
          delay(100);
          WiFi.disconnect();
          delay(1000);
          SetDefaultWiFiConfig();
          CreateWifiSoftAP();
          return;
        }
        else
        {
          // Safe Config
          bool SaveOk = saveCredentials();
          Serial.print(F("Save Credentials 2 returned: "));
          Serial.println(SaveOk);
          InitalizeHTTPServer();
          return;
        }
      }
    }

    if (server.hasArg("WiFiMode") and (server.arg("WiFiMode") == "2")) // Change AP Mode
    {
      startMillis = millis(); // Reset Time Up Counter to avoid Idle Mode whiole operating
      // Configure Access Point
      temp = server.arg("APPointName");
      len = temp.length();
      temp = server.arg("APPW");
      if (server.hasArg("PasswordReq"))
      {
        i = temp.length();
      }
      else
      {
        i = 8;
      }

      if ((len > 1) and (server.arg("APPW") == server.arg("APPWRepeat")) and (i > 7))
      {
        temp = "";
        Serial.println(F("APMode"));
        MyWiFiConfig.APSTA = true; // Access Point or Station Mode - true AP Mode

        if (server.hasArg("CaptivePortal"))
        {
          MyWiFiConfig.CapPortal = true; //CaptivePortal on in AP Mode
        }
        else
        {
          MyWiFiConfig.CapPortal = false;
        }

        if (server.hasArg("PasswordReq"))
        {
          MyWiFiConfig.PwDReq = true; //Password Required in AP Mode
        }
        else
        {
          MyWiFiConfig.PwDReq = false;
        }

        for (i = 0; i < APSTANameLen; i++)
        {
          MyWiFiConfig.APSTAName[i] = 0;
        }
        temp = server.arg("APPointName");
        len = temp.length();
        for (i = 0; i < len; i++)
        {
          MyWiFiConfig.APSTAName[i] = temp[i];
        }
        MyWiFiConfig.APSTAName[len + 1] = '\0';
        temp = "";
        for (i = 0; i < WiFiPwdLen; i++)
        {
          MyWiFiConfig.WiFiPwd[i] = 0;
        }
        temp = server.arg("APPW");
        len = temp.length();
        for (i = 0; i < len; i++)
        {
          MyWiFiConfig.WiFiPwd[i] = temp[i];
        }
        MyWiFiConfig.WiFiPwd[len + 1] = '\0';
        temp = "";
        if (saveCredentials()) // Save AP ConfigCongfig
        {
          temp = "Daten des AP Modes erfolgreich gespeichert. Reboot notwendig.";
        }
        else
        {
          temp = "Daten des AP Modes fehlerhaft.";
        }
      }
      else if (server.arg("APPW") != server.arg("APPWRepeat"))
      {
        temp = "";
        temp = "WLAN Passwort nicht gleich. Abgebrochen.";
      }
      else
      {
        temp = "";
        temp = "WLAN Passwort oder AP Name zu kurz. Abgebrochen.";
      }
    }

    //----- HTML Header
    server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    server.sendHeader("Pragma", "no-cache");
    server.sendHeader("Expires", "-1");
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    //----- HTML Content
    temp += "<!DOCTYPE HTML><html lang='de'><head><meta charset='UTF-8'><meta name= viewport content='width=device-width, initial-scale=1.0,'>";
    server.send(200, "text/html", temp);
    temp = "";
    temp += "<style type='text/css'><!-- DIV.container { min-height: 10em; display: table-cell; vertical-align: middle }.button {height:35px; width:90px; font-size:16px}";
    temp += "body {background-color: powderblue;}</style><head><title>Smartes Tuerschild - WiFi Settings</title></head>";
    server.sendContent(temp);
    // Tabelle. Titel als H2 darüber: "Wifi Einstellungen"
    temp = "";
    temp += "<h2>WiFi Einstellungen</h2><body><left>";
    // H4: "Current Wifi Settings"
    temp += "<table border=2 bgcolor = white width = 420 ><td><h4>Current WiFi Settings: </h4>";
    if (server.client().localIP() == apIP)
    {
      temp += "Mode : Soft Access Point (AP)<br>";
      temp += "SSID : " + String(MyWiFiConfig.APSTAName) + "<br><br>";
    }
    else
    {
      temp += "Mode : Station (STA) <br>";
      temp += "SSID  :  " + String(MyWiFiConfig.APSTAName) + "<br>";
      temp += "BSSID :  " + WiFi.BSSIDstr() + "<br><br>";
    }
    temp += "</td></table><br>";
    server.sendContent(temp);

    // Table of Wifi networks
    temp = "";
    temp += "<form action='/wifi' method='post'>";
    temp += "<table border=2 bgcolor = white width = 420><tr><th><br>";
    // radio button: checked only if station mode
    if (MyWiFiConfig.APSTA == 1)
    {
      temp += "<input type='radio' value='1' name='WiFiMode' > WiFi Station Mode<br>";
    }
    else
    {
      temp += "<input type='radio' value='1' name='WiFiMode' checked > WiFi Station Mode<br>";
    }
    // Last line of overall header, table headings
    temp += "Available WiFi Networks:<table border=2 bgcolor = white ></tr></th><td>No </td><td>SSID  </td><td>Encryption </td><td>WiFi dB</td>";
    server.sendContent(temp);
    // now do the wifi scan 
    temp = "";
    WiFi.scanDelete();
    int n = WiFi.scanNetworks(false, false); //WiFi.scanNetworks(async, show_hidden)
    if (n > 0)
    {
      for (int i = 0; i < n; i++)
      {
        // build the table rows from number, SSID, encryption type, RSSI (network strength )
        temp += "</tr></th>";
        String Nrb = String(i);     // number of the network found, simply convert "i" to String "Nrb"
        temp += "<td>" + Nrb + "</td>";
        temp += "<td>" + WiFi.SSID(i) + "</td>";

        Nrb = GetEncryptionType(WiFi.encryptionType(i));
        temp += "<td>" + Nrb + "</td>";
        temp += "<td>" + String(WiFi.RSSI(i)) + "</td>";
      }
    }
    else
    {
      temp += "</tr></th>";
      temp += "<td>1 </td>";
      temp += "<td>No WLAN found</td>";
      temp += "<td> --- </td>";
      temp += "<td> --- </td>";
    }
    // end the table, and start the next one. That contains on the left the "connect to wifi ssid", on the right a selector for the network
    temp += "</table><table border=2 bgcolor = white width = 420></tr></th><td>Connect to WiFi SSID: </td><td><select name='WiFi_Network' >";
    if (n > 0)
    {
      for (int i = 0; i < n; i++)
      {
        temp += "<option value='" + WiFi.SSID(i) + "'>" + WiFi.SSID(i) + "</option>";
      }
    }
    else
    {
      temp += "<option value='No_WiFi_Network'>No WiFiNetwork found !/option>";
    }
    server.sendContent(temp);
    // End of selector for the wifi network ssid, entry "Wifi Password"
    temp = "";
    temp += "</select></td></tr></th></tr></th><td>WiFi Password: </td><td>";

    // maxlen: maximum input length, size: width of the entry field
    temp += "<input type='text' name='STAWLanPW' maxlength='40'>";
    // end of main table, of selector subtable
    temp += "</td></tr></th><br></th></tr></table></table><table border=2 bgcolor = white width = 420 ><tr><th><br>";
    server.sendContent(temp);
    // now the content for the Wifi access point mode settings: name, password and password confirmation
    temp = "";
    if (MyWiFiConfig.APSTA == true)
      temp += "<input type='radio' name='WiFiMode' value='2' checked> WiFi Access Point Mode <br>";
    else
      temp += "<input type='radio' name='WiFiMode' value='2' > WiFi Access Point Mode <br>";
    // sub-table for entering data
    temp += "<table border=2 bgcolor = white ></tr></th> <td>WiFi Access Point Name: </td><td>";
    server.sendContent(temp);
    temp = "";
    if (MyWiFiConfig.APSTA == true) // get station ssid. If already present, then use this to pre-populate the entry
      temp += "<input type='text' name='APPointName' maxlength='" + String(APSTANameLen - 1) + "' size='30' value='" + String(MyWiFiConfig.APSTAName) + "'></td>";
    else
      temp += "<input type='text' name='APPointName' maxlength='" + String(APSTANameLen - 1) + "' size='30' ></td>";
    server.sendContent(temp);
    // password and password repeat. As password (****) and pre-populated
    temp = "";
    if (MyWiFiConfig.APSTA == true)
    {
      temp += "</tr></th><td>WiFi Password: </td><td>";
      temp += "<input type='password' name='APPW' maxlength='" + String(WiFiPwdLen - 1) + "' size='30' value='" + String(MyWiFiConfig.WiFiPwd) + "'> </td>";
      temp += "</tr></th><td>Repeat WiFi Password: </td>";
      temp += "<td><input type='password' name='APPWRepeat' maxlength='" + String(WiFiPwdLen - 1) + "' size='30' value='" + String(MyWiFiConfig.WiFiPwd) + "'> </td>";
    }
    else
    {
      temp += "</tr></th><td>WiFi Password: </td><td>";
      temp += "<input type='password' name='APPW' maxlength='" + String(WiFiPwdLen - 1) + "' size='30'> </td>";
      temp += "</tr></th><td>Repeat WiFi Password: </td>";
      temp += "<td><input type='password' name='APPWRepeat' maxlength='" + String(WiFiPwdLen - 1) + "' size='30'> </td>";
    }
    temp += "</table>";
    server.sendContent(temp);
    // now the checkboxes for captive portal nand password for login required
    temp = "";
    if (MyWiFiConfig.PwDReq)
      temp += "<input type='checkbox' name='PasswordReq' checked> Password for Login required. ";
    else
      temp += "<input type='checkbox' name='PasswordReq' > Password for Login required. ";
    server.sendContent(temp);
    temp = "";
    if (MyWiFiConfig.CapPortal)
      temp += "<input type='checkbox' name='CaptivePortal' checked> Activate Captive Portal";
    else
      temp += "<input type='checkbox' name='CaptivePortal' > Activate Captive Portal";
    server.sendContent(temp);
    temp = "";
    temp += "<br></tr></th></table><br>";
        // Buttons for "set wifi settings" "reboot" "reset"
    temp += "<button type='submit' name='Settings' value='1' style='height: 50px; width: 200px' autofocus>Save Client Wifi Settings</button>";
    temp += "<button type='submit' name='Reboot' value='1' style='height: 50px; width: 200px' >Reboot System</button>";
    server.sendContent(temp);
    temp = "";
    temp += "<button type='reset' name='action' value='1' style='height: 50px; width: 200px' >Reset Access Point settings</button></form>";
    temp += "<table border=2 bgcolor = white width = 500 cellpadding =5 ><caption><p><h3>Systemlinks:</h2></p></caption><tr><th><br>";
    server.sendContent(temp);
    temp = "";
    temp += "<a href='/'>Main Page</a><br><br></th></tr></table><br><br>";
    temp += "<a href='/google'>Markus Test</a><br><br></th></tr></table><br><br>";
    temp += "<footer><p>Programmed and designed by: Tobias Kuch</p><p>Contact information: <a href='mailto:tobias.kuch@googlemail.com'>tobias.kuch@googlemail.com</a>.</p></footer>";
    temp += "</body></html>";
    server.sendContent(temp);
    server.client().stop(); // Stop is needed because we sent no content length
    temp = "";
  }

  /**************************************************!
     @brief    function to determine if paramter string contains a plausible IP (e.g. "123.123.80.21")
    @details  Just checks if it contains numbers and points only
    @param String str : string to be checked
    @return boolean : false if anything but numbers and points are contained
    ***************************************************/
  /** Is this an IP? */
  boolean isIp(String str)
  {
    for (int i = 0; i < str.length(); i++)
    {
      int c = str.charAt(i);
      if (c != '.' && (c < '0' || c > '9'))
      {
        return false;
      }
    }
    return true;
  }

  /**************************************************!
     @brief    function to create a readable representation of the encryption type
    @details  Can decode 5 (WEP, 2 (WPA), 4 (WPA2), 7 (none), 8 (Auto)
    @param byte thisType : encryption type byte representation
    @return String containing the encryption type
    ***************************************************/
  String GetEncryptionType(byte thisType)
  {
    String Output = "";
    // read the encryption type and print out the name:
    switch (thisType)
    {
    case 0:
      Output = "Open ";
      Output += thisType;
      return Output;
      break;  
    case 1:
      Output = "WEP ";
      Output += thisType;
      return Output;
      break;
    case 2:
      Output = "WPA ";
      Output += thisType;
      return Output;
      break;
    case 3:
      Output = "WPA2 ";
      Output += thisType;
      return Output;
      break;
    case 4:
      Output = "WPA/WPA2 ";
      Output += thisType;
      return Output;
      break;
    case 5:
      Output = "WPA2/Ent ";
      Output += thisType;
      return Output;
      break;
    case 6:
      Output = "MAX ";
      Output += thisType;
      return Output;
      break;  
    default:
      Output = thisType;
      return Output;
      break;  
    }
  }

  /**************************************************!
     @brief    function to convert an IP address into a string
    @details  Bluetooth connection is opened by caller. Then listens for ssid:[ID] and pass:[pw], and returns these
              can also use a scan, but since that makes wifi unreliable on ESP32: better not.
    @param IPAddress ip : Object containing the IP address
    @return String containing the IP address
    ***************************************************/
  /** IP to String? */
  String toStringIp(IPAddress ip)
  {
    String res = "";
    for (int i = 0; i < 3; i++)
    {
      res += String((ip >> (8 * i)) & 0xFF) + ".";
    }
    res += String(((ip >> 8 * 3)) & 0xFF);
    return res;
  }

  /**************************************************!
    @brief    function to generate a readable representation of memory size
    @details  in Bytes, KB or MB
    @param size_t bytes : the memory size in bytes
    @return String containing memory size in Bytes, KB or MB
    ***************************************************/
  String formatBytes(size_t bytes)
  { // lesbare Anzeige der Speichergrößen
    if (bytes < 1024)
      return String(bytes) + " Byte";
    else if (bytes < (1024 * 1024))
      return String(bytes / 1024.0) + " KB";
    else if (bytes < (1024 * 1024 * 1024))
      return String(bytes / 1024.0 / 1024.0) + " MB";

    return String(bytes) + " Byte"; // never reached, make compiler happy with default
  }

  /**************************************************!
    @brief    main loop for the captive portal
    @details  event loop for the web sites of te captive portal
    @param char* ss : returned ssid of the selected wifi network
    @param char* pw : returned password of the selcted wifi network 
    @return String containing memory size in Bytes, KB or MB
    ***************************************************/
  void loopCaptivePortal(char* ss, char* pw)
  {
    char printstring[120];
    while (!captivePortalExit){
      if (SoftAccOK)
        dnsServer.processNextRequest(); //DNS
      //HTTP
      server.handleClient();
      esp_task_wdt_reset();   // keep watchdog happy
    };
    sprintf(printstring, "end of captive portal. ssid: %s  pw: %s\n", 
      targetSSID, targetPASS);
    Serial.print(printstring);
    strcpy(ss,targetSSID);  // prepare return parameters
    strcpy(pw,targetPASS);
  }

#endif //   #ifdef isCaptivePortal
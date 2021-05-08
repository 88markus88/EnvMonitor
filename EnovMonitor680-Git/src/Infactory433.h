/******************************************************
* Header file for Infactory433.cpp
*******************************************************/
 
// module global variables for global use
#ifndef __Infactory433_h__
#define __Infactory433_h__
  //***  global variables for Infactory433.cpp that need to be used
  extern float InfactoryTempC;       // global storage variable for temperature
  extern float lastInfactoryTempC[3];
  extern float InfactoryHumidity;    // global storage variable for humidity
  extern int InfactoryChannel;            // channel-Kodierung, kann 00, 01 oder 10 sein 
  extern long lastInfactoryReception; // system time when last infactory reception took place, in ms
  extern int successInfactoryCalc;         // flag for successful temperature/humidity calculation
  extern volatile bool received;
  extern int trialsInfactory, successInfactory;
  extern unsigned int measuringInfactoryOngoing; // flag to indicate that measuring ongoing with Infactory sensor
#endif

/************************************************************
* Forward declarations
*************************************************************/
void doInfactoryStuff(int maxTrialPeriod);
void doInitializeInfactory();
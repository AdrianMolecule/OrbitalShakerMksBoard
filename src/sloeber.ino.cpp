#ifdef __IN_ECLIPSE__
//This is a automatic generated file
//Please do not modify this file
//If you touch this file your change will be overwritten during the next build
//This file has been generated on 2024-06-09 22:31:10

#include "Arduino.h"
#include "Arduino.h"
#include <Melody.h>
#include <DHTesp.h>
#include <SPI.h>
#include <SD.h>
#include <FS.h>
#include "Splitter.h"
#include "SerialCommunication.h"

void oneFullRotation();
void setup() ;
void loop() ;
void startStepper() ;
void stopStepper() ;
int getTemperature(int &temp, int &humid) ;
void setupI2SOShiftEnableMotor() ;
void setupI2SOShiftDisableMotor() ;
void writeData(byte *bits) ;
void alternate(int pin, int de, int times) ;
int readPotentiometer() ;
void setupSDCard() ;
void printDirectory(File dir, int numTabs) ;
int readTurnOnStepper() ;
void setLoudness(int loudness) ;
String getFormatedTimeSinceStart() ;
void fanSetup() ;
void fanOn(bool print) ;
void fanOff(bool print) ;
String formatTime(unsigned long time) ;
void play(Melody melody) ;

#include "OSIncubatorMKS0.ino"


#endif

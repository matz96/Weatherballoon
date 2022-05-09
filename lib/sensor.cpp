#include <iostream>
#include <fstream>
#include <string>
#include "Arduino.h"
#include <Wire.h>
#include "Adafruit_AS7341.h"


using namespace std; 
ofstream file;

Adafruit_AS7341 as7341;

void init_sensor(){
     if (!as7341.begin()){
    Serial.println("Could not find AS7341");
    while (1) { delay(10); }
  }
    ifstream file("Sensordaten.txt");
  if(!file.is_open()){
   ofstream file("Sensordaten.txt");}
  as7341.setATIME(100);
  as7341.setASTEP(999);
  as7341.setGain(AS7341_GAIN_256X);
}


//TODO write GAIN function 
void set_gain(){
  
}



//TODO change function to write to log file 
void read_sensors(){
  uint16_t readings[12];
  float counts[12];

  if (!as7341.readAllChannels(readings)){
    Serial.println("Error reading all channels!");
    return;
  }
  for(uint8_t i = 0; i < 12; i++) {
    if(i == 4 || i == 5) continue;
    // we skip the first set of duplicate clear/NIR readings
    // (indices 4 and 5)
    counts[i] = as7341.toBasicCounts(readings[i]);
  }

  Serial.print("ADC0/F1 415nm : ");
  Serial.println(counts[0]);
  Serial.print("ADC1/F2 445nm : ");
  Serial.println(counts[1]);
  Serial.print("ADC2/F3 480nm : ");
  Serial.println(counts[2]);
  Serial.print("ADC3/F4 515nm : ");
  Serial.println(counts[3]);
  Serial.print("ADC0/F5 555nm : ");

  /* 
  // we skip the first set of duplicate clear/NIR readings
  Serial.print("ADC4/Clear-");
  Serial.println(readings[4]);
  Serial.print("ADC5/NIR-");
  Serial.println(readings[5]);
  */
  
  Serial.println(counts[6]);
  Serial.print("ADC1/F6 590nm : ");
  Serial.println(counts[7]);
  Serial.print("ADC2/F7 630nm : ");
  Serial.println(counts[8]);
  Serial.print("ADC3/F8 680nm : ");
  Serial.println(counts[9]);
  Serial.print("ADC4/Clear    : ");
  Serial.println(counts[10]);
  Serial.print("ADC5/NIR      : ");
  Serial.println(counts[11]);

  Serial.println();
}



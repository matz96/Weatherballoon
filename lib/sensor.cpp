#include <iostream>
#include <fstream>
#include <string>
#include "Arduino.h"
#include <Wire.h>
#include "Adafruit_AS7341.h"

#define TCAADDR (0x70)
#define MAX_SENS_VAL (59000) // about 10% below sensor max value
#define FOURTYPERCENT (26215)
using namespace std;
ofstream file;

Adafruit_AS7341 as7341;
// TODO SMUX configuration
void init_sensor()
{
  if (!as7341.begin())
  {
    Serial.println("Could not find AS7341");
    while (1)
    {
      delay(10);
    }
  }
  ifstream file("Sensordaten.txt");
  if (!file.is_open())
  {
    ofstream file("Sensordaten.txt");
  }
  as7341.setATIME(100);
  as7341.setASTEP(999);
  as7341.setGain(AS7341_GAIN_256X);
}

// Reduces gainlevel by half
void reduce_gain()
{
  
  as7341.setGain(as7341.getGain()--);//TODO TYPE has to be ENUM, Casting
}

// Doubles gain level
void double_gain()
{
  as7341.setGain((as7341_gain_t)(as7341.getGain()++));//TODO TYPE has to be ENUM, Casting
}

// Changes the channel on the Multiplexer
void tcaselect(uint8_t i)
{
  if (i > 7)
    return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

// TODO change function to write to log file
//  Only use channel 0-3 & 6,7
void read_sensors()
{
  uint16_t readings[12];

  for (uint8_t j = 0; j < 6; j++)
  {
    tcaselect(j);
    if (!as7341.readAllChannels(readings))
    {
      Serial.println("Error reading all channels!");
      return;
    }
    bool is_ok = false;
    u_int8_t underfourty = 0;

    // checks if the sensors gain values are ok
    for (uint8_t i = 0; i < 8; i++)
    {
      if (i == 4 || i == 5)
        continue;
      // we skip the first set of duplicate clear/NIR readings
      // (indices 4 and 5)
      if (readings[i] >= MAX_SENS_VAL)
      { // if bigger than 15 bit gain shifts down
        reduce_gain();
        j--;
        break;
      }
      else if (readings[i] <= FOURTYPERCENT)
      {
        underfourty++;
      }
      if (underfourty == 6)
      {
        double_gain();
        j--;
        break;
      }
      if (i = 7)
      {
        is_ok = true;
      }
    }

    //TODO writes data to text file one sensor at a time
    if (is_ok)
    {
      for (uint8_t i = 0; i < 8; i++)
      {
        if (i == 4 || i == 5)
          continue;
      }
    }
  }
}

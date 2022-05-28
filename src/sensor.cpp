#include <iostream>
#include <fstream>
#include <filesystem.h>
#include <string>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_AS7341.h>
#include <LittleFS_Mbed_RP2040.h>



#define FORCE_REFORMAT          false
#define TCAADDR (0x70)
#define MAX_SENS_VAL (59000) // about 10% below sensor max value
#define FOURTYPERCENT (26215)
using namespace std;

char filename[] = MBED_LITTLEFS_FILE_PREFIX "/hello1.txt"; 

Adafruit_AS7341 as7341;
void dump_data();
void init_sensor()
{
  setupLFS();
  char test_message[] = "start";

  //writeFile(filename,test_message,sizeof(test_message));
  readFile(filename);
 
 
  
  if (!as7341.begin())
  {
    Serial.println("Could not find AS7341");
    while (1)
    {
      delay(10);
      
    }
  }
  Serial.println("AS7341 enabled");

  /* ifstream file("Sensordaten.txt");
  if (!file.is_open())
  {
    ofstream file("Sensordaten.txt");
   */
  as7341.setATIME(100);
  as7341.setASTEP(999);
  as7341.setGain(AS7341_GAIN_2X);
}

/* void dum_data(){
  for(file);
  Serial.println(); 


} */

// Reduces gainlevel by half
void reduce_gain()
{

  as7341.setGain((as7341_gain_t)((int)(as7341.getGain()) - 1));
}

// Doubles gain level
void double_gain()
{
  as7341.setGain((as7341_gain_t)((int)(as7341.getGain()) + 1));
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

void write_Sensor_Data(uint16_t data)
{
  char datachar[2];

  datachar[1] = data & 0xFF;

  datachar[0] = data >> 8;
  
  appendFile(filename,datachar,2);

}

void write_Gain_Data(uint8_t data)
{char datachar = data;
  appendFile(filename,&datachar,1);
}

void write_others_Data(uint16_t data)
{
  char datachar[2];

  datachar[1] = data & 0xFF;

  datachar[0] = data >> 8;
  
  appendFile(filename,datachar,2);


}

void write_line_end()
{ char datachar = '\n';
  appendFile(filename,&datachar,2);

}




//  Only uses channel 0-3 & 6,7
void read_sensors()
{
 
  uint16_t readings[12];
  uint8_t gain = 0;
  uint8_t gain2 = 0;

  for (uint8_t j = 0; j < 6; j++)
  {

    //tcaselect(j);                          // Sets the MUX to the j-th sensor
    if (!as7341.readAllChannels(readings)) // Reads all sensors and sets the SMUX the one in the Sensor
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
      { // if bigger than MAX_SENS_VAL gain shifts down
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
      if (i == 7)
      {
        is_ok = true;
      }
    }

    // TODO writes data to text file one Measured frequency at a time
    if (is_ok)
    {
      for (uint8_t i = 0; i < 8; i++)
      {
        if (i == 4 || i == 5)
          continue;
        write_Sensor_Data(readings[i]);
      }
      //writes the gains of two sensors into one uin8_t to save space
      if (j % 2)
      {
        gain2 = as7341.getGain();
        (gain << 4);
        gain = gain + gain2;
        write_Gain_Data(gain);
        gain = 0;
        gain2 = 0;
      }
      else{
        gain = as7341.getGain();
      }
    }
  }
}

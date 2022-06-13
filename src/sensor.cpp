#include <iostream>
#include <fstream>
#include <filesystem.h>
#include <string>
#include <Arduino.h>
#include <Wire.h>
#include "sensor.h"
#include <Adafruit_AS7341.h>

#include <SparkFun_I2C_Mux_Arduino_Library.h> //Click here to get the library: http://librarymanager/All#SparkFun_I2C_Mux


QWIICMUX myMux;


#define DEBUG_STUFF

#define MAX_TRIES_SENSOR_BEGIN (5)
#define FORCE_REFORMAT          false
#define TCAADDR (0x70)

#define ATIME_VALUE (29)
#define ASTEP_VALUE (599) //equals 50ms (30*600*2.78us)

//maximum values are defined by (ASTEP+1)*(ATIME+1) which defines maximum count
//therefore the MAX/MIN_SENS_VAL must be set accordingly 
#define MAX_SENS_VAL (16200) //set to 90% of max count of 18000 --- 59000 is about 10% below sensor max value if this was maximum of 16bit
#define MIN_SENS_VAL (5400)  //set to 30% of max count of 18000 --- 26215 = 40% of maximum value (16bit)

#define MAX_GAIN (10) //maximal GAIN = 10 = 512x 
#define MIN_GAIN (0) //minimal GAIN = 0 = 0.5x 


//using namespace std;

const char filename[] = "/littlefs/log.txt";
Adafruit_AS7341 as7341;
//void dump_data();


uint8_t init_sensors(TwoWire *theWire)
{
  uint8_t initializedSensors = 0;
  
  if (myMux.begin() == false)
  {
    Serial.println("Mux not detected. Freezing...");
    while (1);
  }
  Serial.println("Mux detected");

  myMux.setPort(0); //Connect master to port labeled '0' on the mux
  /* setupLFS();
  char test_message[] = "start\n"; 
  char end_message[] = "stop\n";
  appendFile(filename,end_message,sizeof(end_message));
  readFile(filename); */
  //writeFile(filename,test_message,sizeof(test_message));

 for (int muxport = 0; muxport < 6; muxport++){
  Serial.print("PORT= ");
  Serial.println(muxport);

  myMux.setPort(muxport);
  byte currentPortNumber = myMux.getPort();
  Serial.print("CurrentPort: ");
  Serial.println(currentPortNumber);
 
  if (!as7341.begin(AS7341_I2CADDR_DEFAULT, theWire))
    { 
      uint8_t maxTriesReached = 0; //reset the maxTriesReached
      
      while (!as7341.begin(AS7341_I2CADDR_DEFAULT, theWire)&&!(maxTriesReached >= MAX_TRIES_SENSOR_BEGIN))
      {
        Serial.println("Try Again to init AS7341");
        maxTriesReached++;
        delay(100);
        
      }
      if (maxTriesReached >= MAX_TRIES_SENSOR_BEGIN){
        Serial.println("Could not find AS7341 multiple times...");
        
        }
      else{
        Serial.println("AS7341 enabled");
        initializedSensors |= (1 << muxport);
        }
      
    }
    else {
      Serial.println("AS7341 enabled");
      initializedSensors |= (1 << muxport);
    }
    

    //with 59 and 59 total time is 10ms per Channel
    as7341.setATIME(ATIME_VALUE); //used to be 100
    as7341.setASTEP(ASTEP_VALUE); //used to be 999 
    as7341.setGain(AS7341_GAIN_512X); 
    

    
    }
    return initializedSensors;
}

/* void dum_data(){
  for(file);
  Serial.println(); 


} */

// Reduces gainlevel by half
uint8_t reduce_gain()
{
  uint8_t oldGain, newGain;
  oldGain = (uint8_t)as7341.getGain();

  if (oldGain > 0 ){
    newGain = oldGain - 1;
    as7341.setGain((as7341_gain_t)newGain);
  }
  else{
    newGain = oldGain;
  }
  
  return newGain;
}

// Doubles gain level
uint8_t double_gain()
{

  uint8_t oldGain, newGain;
  oldGain = (uint8_t)as7341.getGain();

  if (oldGain < 10 ){
    newGain = oldGain + 1;
    as7341.setGain((as7341_gain_t)newGain);
  }
  else{
    newGain = oldGain;
  }
  
  return newGain;

}


void write_Sensor_Data(uint16_t data, char * datastring, int p)
{
  

  datastring[p+1] = data & 0xFF;

  datastring[p] =  data >> 8;
  
  

}

void write_Gain_Data(uint8_t data, char * datastring, int p)
{
 datastring[p]=data;
}

void write_others_Data(uint16_t data)
{
  char datachar[2];

  datachar[1] = data & 0xFF;

  datachar[0] = data >> 8;
  
  appendFile(filename,datachar,2);


}

/**
 * @brief Reads a Sensor while automatically setting the right gain for maximum dynamic and returns the gain. 
 * 
 * @param sensorNumber the sensor to be read (0-5)
 * @param datastring a CharArrayWhere the SensorValues are saved at least 12 Bytes
 * @param theWire PointerToWire
 * @return uint8_t The gainValue encoded according to gain enum; -1 = error reading sensor/channels
 */
int8_t readSensor(uint8_t sensorNumber, char * dataString, TwoWire *theWire){
uint16_t readings[12];
uint8_t gain;
bool gainOK = false; //at the beginning we assume that gain is wrong if gain = true it means data is in good dynamic range
myMux.setPort(sensorNumber); //set the multiplexer to chosen sensor nr
 
 Serial.print("Getting Channels for Sensor: ");
 Serial.println(sensorNumber);
 
 
 while (!gainOK){ //while gain is notOK repeat the following: 
    

  if (!as7341.readAllChannels(readings)) // Reads all channels of given sensor, if not ok it exits function
  {
    #ifdef SERIAL_LOGGING
    Serial.print("Error reading all channels of Sensor ");
    Serial.println(sensorNumber);
    #endif
    for (uint8_t i = 0 ; i< 12;i++){
      dataString[i]= 0xFF;  //fill up with obviously bad data (12bytes of it)
    }
    return -1;
  }
  else  {
    gain = as7341.getGain(); //first get current gain
  // Serial.print("ADC0/F1 415nm : ");
  // Serial.println(readings[0]);
  // Serial.print("ADC1/F2 445nm : ");
  // Serial.println(readings[1]);
  // Serial.print("ADC2/F3 480nm : ");
  // Serial.println(readings[2]);
  // Serial.print("ADC3/F4 515nm : ");
  // Serial.println(readings[3]);
  // Serial.print("ADC0/F5 555nm : ");

  // /* 
  // // we skip the first set of duplicate clear/NIR readings
  // Serial.print("ADC4/Clear-");
  // Serial.println(readings[4]);
  // Serial.print("ADC5/NIR-");
  // Serial.println(readings[5]);
  // */
  
  // Serial.println(readings[6]);
  // Serial.print("ADC1/F6 590nm : ");
  // Serial.println(readings[7]);
  // Serial.print("ADC2/F7 630nm : ");
  // Serial.println(readings[8]);
  // Serial.print("ADC3/F8 680nm : ");
  // Serial.println(readings[9]);
  // Serial.print("ADC4/Clear    : ");
  // Serial.println(readings[10]);
  // Serial.print("ADC5/NIR      : ");
  // Serial.println(readings[11]);
  // Serial.println();
  }

  //now fill the dataString array with values for easier handeling of the data in next step.
  dataString[0]  = (readings[0] >> 8); //0 = 415nm
  dataString[1]  = readings[0]; 
  dataString[2]  = (readings[1] >> 8); //1 = 445nm
  dataString[3]  = readings[0]; 
  dataString[4]  = (readings[2] >> 8); //2 = 480nm
  dataString[5]  = readings[0]; 
  dataString[6]  = (readings[3] >> 8); //3 = 515nm
  dataString[7]  = readings[0]; 
  dataString[8]  = (readings[6] >> 8); //6 = 555nm
  dataString[9]  = readings[0]; 
  dataString[10] = (readings[4] >> 8); //4 = clear channel  
  dataString[11]  = readings[0];   
  
  uint8_t nrOfLowChannels = 0;

  for (uint8_t channel = 0; channel < 6; channel++){
    
    uint16_t channelValue = (dataString[2*channel] << 8)|(dataString[2*channel+1]);
    
    if (channelValue > MAX_SENS_VAL && gain > MIN_GAIN){
      Serial.print("Reducing Gain for Sensor: ");
      Serial.println(sensorNumber);
      gain = reduce_gain();
      break;
    }

    if (channelValue < MIN_SENS_VAL){
      nrOfLowChannels++;
    }
  }

  if (nrOfLowChannels == 6 && gain < MAX_GAIN){
    Serial.print("Doubeling Gain for Sensor: ");
    Serial.println(sensorNumber);
    gain = double_gain();
    continue;
  }

  gainOK = true;  //if we get here it means either gain = min/max or all channels were within range
}

return gain;

}

// //  Only uses channel 0-3 & 6,7
// void read_sensors(char * datastring, TwoWire *theWire)
// {
 
//   uint16_t readings[12];
//   uint8_t gain = 0;
//   uint8_t gain2 = 0;
//   int16_t p = 0;
//   for (int muxport = 0; muxport < 6; muxport++){
  
//   Serial.print("PORT= ");
//   Serial.println(muxport);

//   myMux.setPort(muxport);

    

//     bool is_ok = false;
//     u_int8_t underfourty = 0;
//     bool maxOrMinGain = false;

//     // checks if the sensors gain values are ok
//     for (uint8_t i = 0; i < 8; i++)
//     {
//       if (i == 4 || i == 5)
//         continue;
//       // we skip the first set of duplicate clear/NIR readings
//       // (indices 4 and 5)
//       if (readings[i] >= MAX_SENS_VAL)
//       { // if bigger than MAX_SENS_VAL gain shifts down
//         reduce_gain();
//         muxport--;
//         break;
//       }
//       else if (readings[i] <= MIN_SENS_VAL)
//       {
//         underfourty++;
//       }
//       if (underfourty == 6)
//       {
//         if (double_gain() == 10){
//           maxOrMinGain = true;
//         }
//         muxport--;
//         break;
//       }
//       if (i == 7)
//       {
//         is_ok = true;
//         Serial.println("reading,gain ok");
//       }
//     }
    

//     // TODO writes data to text file one Measured frequency at a time
//     if (is_ok)
//     {
//       for (uint8_t i = 0; i < 8; i++)
//       {
//         if (i == 4 || i == 5){
//           continue;}
        
//         //write_Sensor_Data(readings[i],datastring,p);
//         p=p+2;
//       }
//       //writes the gains of two sensors into one uin8_t to save space
//       if (muxport % 2)
//       {
//         gain2 = as7341.getGain();
//         (gain << 4);
//         gain = gain + gain2;
//         write_Gain_Data(gain, datastring,p);
//         p++;
//         gain = 0;
//         gain2 = 0;
//       }
//       else{
//         gain = as7341.getGain();
//       }
//       if(muxport == 5){
//         //write_line_end();
//       }
//     }
//   }
// }

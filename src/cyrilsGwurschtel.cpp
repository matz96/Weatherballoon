#define USE_BMP
//#define USE_MAG
//#define USE_TEMP
//#define USE_AS7341
#define USE_GPS



#include <Arduino.h>
#include <Wire.h>

// default "Wire" object: SDA = GP4, SCL = GP5, I2C0 peripheral
// our new wire object:
#define WIRE1_SDA       14  // Use GP2 as I2C1 SDA //für Board 14
#define WIRE1_SCL       15  // Use GP3 as I2C1 SCL //für Board 15
arduino::MbedI2C Wire1(WIRE1_SDA, WIRE1_SCL);


#ifdef USE_GPS
#include "SparkFun_Ublox_Arduino_Library_Series_6_7.h"
#include <TinyGPSPlus.h>
SFE_UBLOX_GPS myGPS;
TinyGPSPlus tinyGPS;

#include <TinyGPSPlus.h>

void displayInfo()
{
  Serial.print(F("Location: ")); 
  if (tinyGPS.location.isValid())
  {
    Serial.print(tinyGPS.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(tinyGPS.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (tinyGPS.date.isValid())
  {
    Serial.print(tinyGPS.date.month());
    Serial.print(F("/"));
    Serial.print(tinyGPS.date.day());
    Serial.print(F("/"));
    Serial.print(tinyGPS.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (tinyGPS.time.isValid())
  {
    if (tinyGPS.time.hour() < 10) Serial.print(F("0"));
    Serial.print(tinyGPS.time.hour());
    Serial.print(F(":"));
    if (tinyGPS.time.minute() < 10) Serial.print(F("0"));
    Serial.print(tinyGPS.time.minute());
    Serial.print(F(":"));
    if (tinyGPS.time.second() < 10) Serial.print(F("0"));
    Serial.print(tinyGPS.time.second());
    Serial.print(F("."));
    if (tinyGPS.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(tinyGPS.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}

#endif

#include <Adafruit_Sensor.h>



#ifdef USE_AS7341
#include <Adafruit_AS7341.h>

Adafruit_AS7341 as7341;
#endif

#ifdef USE_TEMP
#include "Adafruit_MCP9808.h"
#endif



#ifdef USE_BMP
#include <Adafruit_BMP085.h>
Adafruit_BMP085 bmp;
#endif

#ifdef USE_MAG
#include <Adafruit_HMC5883_U.h>
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
#endif




// Create the MCP9808 temperature sensor object
#ifdef USE_TEMP
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();
#endif





void setup()
{
  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH);
  // put your setup code here, to run once:
  delay(5000);
  SerialUSB.begin(9600);
  delay(1000); //give Serial time to start
  
  #ifdef USE_AS7341
  
  if (!as7341.begin()){
    Serial.println("Could not find AS7341");
    while (1) { delay(10); }
  }
  
  as7341.setATIME(100);
  as7341.setASTEP(999);
  as7341.setGain(AS7341_GAIN_256X);
  #endif
  


  #ifdef USE_TEMP

  if (!tempsensor.begin(0x18, &Wire1)) {
    Serial.println("Couldn't find MCP9808! Check your connections and verify the address is correct.");
    while (1);
  }
   Serial.println("Found MCP9808!");
   tempsensor.setResolution(3);
   tempsensor.wake();
  #endif


#ifdef USE_BMP
  if(!bmp.begin(3UL, &Wire1)) //3UL = Ultrahighres
    {
      /* There was a problem detecting the BMP085 ... check your connections */
      Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
      while(1);
    }
#endif

#ifdef USE_MAG
    Serial.println("HMC5883 Magnetometer Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }
  mag.printSensorDetails();
  mag.enableAutoRange(true);
   
#endif
   
  #ifdef USE_GPS
  do {
    Serial.println("GPS: trying 38400 baud");
    Serial1.begin(38400);
    if (myGPS.begin(Serial1) == true) break;

    delay(100);
    Serial.println("GPS: trying 9600 baud");
    Serial1.begin(9600);
    if (myGPS.begin(Serial1) == true) {
        Serial1.println("GPS: connected at 9600 baud, switching to 38400");
        myGPS.setSerialRate(38400);
        delay(100);
    } else {
        //myGPS.factoryReset();
        delay(2000); //Wait a bit before trying again to limit the Serial output
    }
  } while(1);
  Serial.println("GPS serial connected");
  myGPS.setUART1Output(COM_TYPE_UBX);
  myGPS.setNavigationFrequency(2);
  myGPS.setDynamicModel(DYN_MODEL_PORTABLE);
  delay(500);
  myGPS.saveConfiguration();
  delay(100);
  
  #ifdef getPVTmaxWait
  #undef getPVTmaxWait
  #define getPVTmaxWait (400)
  #endif 

  #endif
  
  
}

void loop()
{



#ifdef USE_GPS
  /*
  while (Serial1.available()){
    if(tinyGPS.encode(Serial1.read())){
      displayInfo();
    }
  }
        
     */   
        

  if (myGPS.getPOSLLH(550)){
  Serial.print(myGPS.getLatitude(550)/10000000.,7); 
  Serial.print(",");
  Serial.println(myGPS.getLongitude(550)/10000000.,7);
  Serial.print("Height: ");
  Serial.println(myGPS.getAltitudeMSL(550));
  }else
  Serial.println("no new GPS Data...");

#endif

#ifdef USE_AS7341

uint16_t readings[12];

  if (!as7341.readAllChannels(readings)){
    Serial.println("Error reading all channels!");
    return;
  }

  Serial.print("ADC0/F1 415nm : ");
  Serial.println(readings[0]);
  Serial.print("ADC1/F2 445nm : ");
  Serial.println(readings[1]);
  Serial.print("ADC2/F3 480nm : ");
  Serial.println(readings[2]);
  Serial.print("ADC3/F4 515nm : ");
  Serial.println(readings[3]);
  Serial.print("ADC0/F5 555nm : ");

  /* 
  // we skip the first set of duplicate clear/NIR readings
  Serial.print("ADC4/Clear-");
  Serial.println(readings[4]);
  Serial.print("ADC5/NIR-");
  Serial.println(readings[5]);
  */
  
  Serial.println(readings[6]);
  Serial.print("ADC1/F6 590nm : ");
  Serial.println(readings[7]);
  Serial.print("ADC2/F7 630nm : ");
  Serial.println(readings[8]);
  Serial.print("ADC3/F8 680nm : ");
  Serial.println(readings[9]);
  Serial.print("ADC4/Clear    : ");
  Serial.println(readings[10]);
  Serial.print("ADC5/NIR      : ");
  Serial.println(readings[11]);

  Serial.println();

#endif


#ifdef USE_TEMP
  //Serial.println("wake up MCP9808.... "); // wake up MCP9808 - power consumption ~200 mikro Ampere
  //tempsensor.wake();   // wake up, ready to read!

  // Read and print out the temperature, also shows the resolution mode used for reading.
 // Serial.print("Resolution in mode: ");
  //Serial.println (tempsensor.getResolution());
  float c = tempsensor.readTempC();
  //float f = tempsensor.readTempF();
  Serial.print("Temp: "); 
  Serial.print(c, 4); Serial.println("*C"); 
  //Serial.print(f, 4); Serial.println("*F.");
  
  //Serial.println("Shutdown MCP9808.... ");
  //tempsensor.shutdown_wake(1); // shutdown MSP9808 - power consumption ~0.1 mikro Ampere, stops temperature sampling
  Serial.println("");
  #endif
  /* Get a new sensor event */ 
  
  sensors_event_t event;
  
  #ifdef USE_BMP
  
 
  /* Display the results (barometric pressure is measure in hPa) */
  
    /* Display atmospheric pressue in hPa */
    Serial.print("Pressure:    ");
    Serial.print(bmp.readPressure()/100);
    Serial.println(" hPa");
    
    /* Calculating altitude with reasonable accuracy requires pressure    *
     * sea level pressure for your position at the moment the data is     *
     * converted, as well as the ambient temperature in degress           *
     * celcius.  If you don't have these values, a 'generic' value of     *
     * 1013.25 hPa can be used (defined as SENSORS_PRESSURE_SEALEVELHPA   *
     * in sensors.h), but this isn't ideal and will give variable         *
     * results from one day to the next.                                  *
     *                                                                    *
     * You can usually find the current SLP value by looking at weather   *
     * websites or from environmental information centers near any major  *
     * airport.                                                           *
     *                                                                    *
     * For example, for Paris, France you can check the current mean      *
     * pressure and sea level at: http://bit.ly/16Au8ol                   */
     
    
    Serial.print("Temperature: ");
    Serial.print(bmp.readTemperature());
    Serial.println(" C");

    /* Then convert the atmospheric pressure, and SLP to altitude         */
    /* Update this next line with the current SLP for better results      */
    float seaLevelPressure = 1009.1F; //SENSORS_PRESSURE_SEALEVELHPA;
    Serial.print("Altitude:    "); 
    Serial.print(bmp.readAltitude()); 
    Serial.println(" m");
    Serial.println("");
  #endif


  #ifdef USE_MAG
  mag.getEvent(&event);
 
  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");


  
  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = 0.052; //3° Declination in Brugg
  heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/PI; 
  
  Serial.print("Heading (degrees): "); Serial.println(headingDegrees);

  #endif
  delay(100);

  
  
  
  
}
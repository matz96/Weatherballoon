#define SERIAL_LOGGING  // if defined it will print data in loop - if not defined no data is written to prevent hang-up

#define DECLINATION_ANGLE (0.052f) //Deklinatinoswinkel - ~= 3° in Brugg
#define PRESSURE_AT_SEALEVEL (101780.0f) //Druck vor Flug anpassen. (QFF) Standardwert: 101325.0

#define USE_BMP
#define USE_MAG
#define USE_TEMP
#define USE_GPS
#define USE_LORA

#include <Arduino.h>
#include <sensor.h>
#include "lora_pro4.h"
#include <Wire.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include <Adafruit_Sensor.h>
#include <filesystem.h>


#define BUFFERLEN (100)

char sensorStringBuffer[BUFFERLEN]; //our DataString is exactly 100 bytes long (with a FF at the end)
const char filename[] = "/littlefs/log.txt";
uint32_t freeSpace; 

#define LOGGING_OFFSET_TIME           (0)
#define LOGGING_OFFSET_SPECTRAL       (2)
#define LOGGING_OFFSET_GPS            (77)
#define LOGGING_OFFSET_BAROMETER      (83)
#define LOGGING_OFFSET_MAGNETOMETER   (89)
#define LOGGING_OFFSET_TEMP           (97)

#define PIN_LED_WRITING_TO_FILE (27)
#define PIN_LED_READING_SENSORS (28)

#define MM_IN_FOOT (305)  //aproximation for heightconversion from mm to feet mm/305 ~= feet

// default "Wire" object: SDA = GP4, SCL = GP5, I2C0 peripheral
// our new wire object:
#define WIRE1_SDA       14  // Use GP2 as I2C1 SDA //für Board 14
#define WIRE1_SCL       15  // Use GP3 as I2C1 SCL //für Board 15
arduino::MbedI2C Wire1(WIRE1_SDA, WIRE1_SCL);

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

#ifdef USE_GPS
  #include "SparkFun_Ublox_Arduino_Library_Series_6_7.h"
  #include <TinyGPSPlus.h>
  SFE_UBLOX_GPS myGPS;
  TinyGPSPlus tinyGPS;
#endif

// default "Wire" object: SDA = GP4, SCL = GP5, I2C0 peripheral
// our new wire object:
//#define WIRE1_SDA       14  // Use GP2 as I2C1 SDA //für Board 14
//#define WIRE1_SCL       15  // Use GP3 as I2C1 SCL //für Board 15
//arduino::MbedI2C Wire1(WIRE1_SDA, WIRE1_SCL);

int count = 0;

//SDA1 = GPIO PIN12
//SCL1 = GPIO PIN13
//SDA2 = GPIO PIN14
//SCL2 = GPIO PIN15

bool serialConnected = false;
unsigned long millisLoggingStart; 

void setup()
{
 millisLoggingStart = millis();
 Serial.begin(115200);
 delay(4000);
 if (Serial){
  serialConnected = true; 
  Serial.println("SerialConnected");
  pinMode(6, OUTPUT);
  digitalWrite(6, HIGH);
 }
 else{
  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH);
 }

 pinMode(PIN_LED_READING_SENSORS, OUTPUT);
 pinMode(PIN_LED_WRITING_TO_FILE, OUTPUT);

  Serial.print("This is the buffer: ");
  for (uint8_t count = 0; count< BUFFERLEN; count++){

    sensorStringBuffer[count] = 0xFF;
    Serial.print(sensorStringBuffer[count]);
  }
  
  
  Serial.println(sensorStringBuffer);
  printLine();
  
  setupLFS();

  //deleteFile(filename);
  char test_message[] = "start\n"; 
  char start_message[] = "start of new data:\n";
  //writeFile(filename,test_message,sizeof(test_message));
  appendFile(filename,start_message,sizeof(start_message));
  
  readFile(filename); 
  

#ifdef USE_TEMP

  if (!tempsensor.begin(0x18)) {
    Serial.println("Couldn't find MCP9808! Check your connections and verify the address is correct.");
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
  if(!mag.begin(&Wire1))
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
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
  myGPS.setDynamicModel(DYN_MODEL_AIRBORNE1g);
  delay(500);
  myGPS.saveConfiguration();
  delay(100);
  
  #ifdef getPVTmaxWait
  #undef getPVTmaxWait
  #define getPVTmaxWait (400)
  #endif 
  
  #endif

  Wire.begin();

  
  
  setup_lora();
  Serial.print("InitSensors: ");

  Serial.println(init_sensors(&Wire), 2);
  Serial.println("InitSensorsDone");

}




void loop()
{
  static long lastMillis = millis();

#ifdef USE_GPS
  
  static uint32_t myGPSLat, myGPSLong, myGPSAlt, myGPSAltFeet;
  
  static uint8_t myGPSHour, myGPSMinutes, myGPSSeconds;


  if (myGPS.getPOSLLH(550)){
  
  #ifdef SERIAL_LOGGING 
    Serial.println("NEW GPS Data!");
  #endif


  
  myGPSLat =  myGPS.getLatitude(550);
  myGPSLong = myGPS.getLongitude(550);
  myGPSAlt = myGPS.getAltitudeMSL(550);

  myGPSAltFeet = myGPSAlt/MM_IN_FOOT;
  
  myGPSHour = myGPS.getHour(550);
  myGPSMinutes = myGPS.getMinute(550);
  myGPSSeconds = myGPS.getSecond(550);


   uint32_t grad_lat = myGPSLat/10000000;
   float minutes_lat = ((myGPSLat - (grad_lat * 10000000))*60)/10000000.;
  
   uint32_t grad_long = myGPSLong/10000000;
   float minutes_long = ((myGPSLong - (grad_long * 10000000))*60)/10000000.;
  
  #ifdef SERIAL_LOGGING 
  
  Serial.print("Latitude Grad: ");
  Serial.print(grad_lat);
  Serial.print(" Minuten: ");
  Serial.print(minutes_lat);
  Serial.print(" Longitude Grad: ");
  Serial.print(grad_long);
  Serial.print(" Minuten: ");
  Serial.println(minutes_long);
  #endif

  }else{
  #ifdef SERIAL_LOGGING 
  Serial.println("no new GPS Data...");
  #endif
  }
#endif

#ifdef USE_TEMP
  float c = tempsensor.readTempC();
  
  #ifdef SERIAL_LOGGING 
    Serial.print("Temp: "); 
    Serial.print(c, 4); Serial.println("*C"); 
    Serial.println("");
  #endif
  /* Get a new sensor event */ 
 #endif 
  
  
  #ifdef USE_BMP
    int32_t bmpPressure  = bmp.readPressure();
    float bmpTemperature = bmp.readTemperature();
    float bmpAltitude = bmp.readAltitude(PRESSURE_AT_SEALEVEL);

    #ifdef SERIAL_LOGGING 
    Serial.print("Pressure:    ");
    Serial.print(bmpPressure/100);
    Serial.println(" hPa");
    Serial.print("Temperature: ");
    Serial.print(bmpTemperature);
    Serial.println(" C");
    Serial.print("Altitude:    "); 
    Serial.print(bmpAltitude); 
    Serial.println(" m");
    Serial.println("");
    #endif //SERIAL_LOGGING
  #endif

  #ifdef USE_MAG
    sensors_event_t event;
    mag.getEvent(&event);
    float magX = event.magnetic.x;
    float magY = event.magnetic.y;
    float magZ = event.magnetic.z;
    
    // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
    // Calculate heading when the magnetometer is level, then correct for signs of axis.
    float heading = atan2(event.magnetic.y, event.magnetic.x);
    // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
    // Find yours here: http://www.magnetic-declination.com/
    // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
    float declinationAngle = DECLINATION_ANGLE; //3° Declination in Brugg
    heading += declinationAngle;
    // Correct for when signs are reversed.
    if(heading < 0)
      heading += 2*PI;
      
    // Check for wrap due to addition of declination.
    if(heading > 2*PI)
      heading -= 2*PI;
    
    // Convert radians to degrees for readability.
    float headingDegrees = heading * 180/PI; 
    
    #ifdef SERIAL_LOGGING
      /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
      Serial.print("X: "); Serial.print(magX); Serial.print("  ");
      Serial.print("Y: "); Serial.print(magY); Serial.print("  ");
      Serial.print("Z: "); Serial.print(magZ); Serial.print("  ");Serial.println("uT");
      Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
    #endif
  #endif
  
  static uint16_t counterToWrite = 0;
  
  if (lastMillis + 1000UL <= millis()){
   
   
   digitalWrite(PIN_LED_READING_SENSORS, HIGH);
   
   int8_t Gain;
   for (uint8_t sensorToRead = 0; sensorToRead<6; sensorToRead++){

   uint8_t gainByteOffset = sensorToRead/2; //how many gain bytes were written before current iteration 
   uint8_t currentGainByte = gainByteOffset+(gainByteOffset+1)*24+LOGGING_OFFSET_SPECTRAL; //adress of current gain byte


   if (sensorToRead == 2){  //sensor 2 is broken
    continue;
   }
   Gain = readSensor(sensorToRead, &sensorStringBuffer[LOGGING_OFFSET_SPECTRAL+(12*sensorToRead)+gainByteOffset], &Wire);
   
   sensorStringBuffer[currentGainByte] = (Gain << (sensorToRead%2)*8);

   }
   
   //Serial.println(Gain); 

   counterToWrite++;
   lastMillis = millis();
   digitalWrite(PIN_LED_READING_SENSORS, LOW);

  //  sensorStringBuffer[97]= counterToWrite;
  //  sensorStringBuffer[96]=(counterToWrite>>8);
   digitalWrite(PIN_LED_WRITING_TO_FILE, HIGH);
   appendFile(filename, sensorStringBuffer, BUFFERLEN);
   digitalWrite(PIN_LED_WRITING_TO_FILE, LOW);

  
  count++;
  if(count == 10){
    lora_send_position_altitude_time(myGPSLat, myGPSLong, myGPSAltFeet, myGPSHour, myGPSMinutes, myGPSSeconds);
    count = 0; 
  }
  }

  
  //Serial.flush();
}
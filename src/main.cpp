//#define SERIAL_LOGGING  // if defined it will print data in loop - if not defined no data is written to prevent hang-up

#define DECLINATION_ANGLE (0.052f) //Deklinatinoswinkel - ~= 3° in Brugg
#define PRESSURE_AT_SEALEVEL (101780.0f) //Druck vor Flug anpassen. (QFF) Standardwert: 101325.0

#define GPS_FREQUENCY (4) //frequency in Hz
#define GPS_TIMEOUT_MS (260) //according to frequency
#define GPS_STANDARD_LAT  (474793720L) // in 10^-7 deg
#define GPS_STANDARD_LONG (82123430L)  // in 10^-7 deg

#define TEMPOFFSET (70.0f)
#define MAGOFFSET (70.0f) //in uT
#define GPSOFFSET (15000000) //in 10^-7 degrees
#define LOGGING_INTERVAL_MS (1000UL) //how often is data logged to file
#define LORA_INTERVAL_MS (10000UL) //how often is position sent by lora

#define WRITE_HEADERFILE_BUTTONPRESS_DURATION_MS (5000) //when both buttons were pressed for twice this duration the old file is deleted and a new one is created

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

#define PIN_LED_WRITING_TO_FILE (27)      //907
#define PIN_LED_READING_SENSORS (28)      //908
#define PIN_LED_ABOUT_TO_DELETE_FILE (20) //903

#define PIN_IN_RESET_FILE_0 (2)
#define PIN_IN_RESET_FILE_1 (3)

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

//SDA1 = GPIO PIN12
//SCL1 = GPIO PIN13
//SDA2 = GPIO PIN14
//SCL2 = GPIO PIN15

bool serialConnected = false;
unsigned long millisLoggingStart;
int32_t longitudeCenter = GPS_STANDARD_LAT;
int32_t latitudeCenter = GPS_STANDARD_LONG;
uint32_t GPSSecondsIntoDayLoggingStart = 4*3600; //4*3600 is 4h into day with this value we never get bigger than a 16bit value

void setup()
{
 millisLoggingStart = millis();
 Serial.begin(115200);
 delay(4000);

pinMode(PIN_IN_RESET_FILE_0, INPUT_PULLUP);
pinMode(PIN_IN_RESET_FILE_1, INPUT_PULLUP);

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
  myGPS.setNavigationFrequency(GPS_FREQUENCY);
  myGPS.setDynamicModel(DYN_MODEL_AIRBORNE1g);
  delay(500);
  myGPS.saveConfiguration();
  delay(100);
  
  #ifdef getPVTmaxWait
  #undef getPVTmaxWait
  #define getPVTmaxWait (GPS_TIMEOUT_MS)
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
  static long lastLoraSendMillis = millis();

#ifdef USE_GPS
  
  static int32_t myGPSLat, myGPSLong, myGPSAlt, myGPSAltFeet, myGPSSecondsIntoDay;
  static uint8_t myGPSHour, myGPSMinutes, myGPSSeconds;


  if (myGPS.getPOSLLH(GPS_TIMEOUT_MS)){
  
  #ifdef SERIAL_LOGGING 
    Serial.println("NEW GPS Data!");
  #endif


  
  myGPSLat =  myGPS.getLatitude(GPS_TIMEOUT_MS);
  myGPSLong = myGPS.getLongitude(GPS_TIMEOUT_MS);
  myGPSAlt = myGPS.getAltitudeMSL(GPS_TIMEOUT_MS);

  myGPSAltFeet = myGPSAlt/MM_IN_FOOT;
  
  myGPSHour = myGPS.getHour(GPS_TIMEOUT_MS);
  myGPSMinutes = myGPS.getMinute(GPS_TIMEOUT_MS);
  myGPSSeconds = myGPS.getSecond(GPS_TIMEOUT_MS);
  
  //we have to move the Difference to positive space by adding 1.5° (= 15000000 *10^-7 degree)
  uint16_t myGPSLatDifference = (uint16_t)((myGPSLat - latitudeCenter)+GPSOFFSET)*60*200/10000000; //this way we have 1/200 minutes steps
  uint16_t myGPSLongDifference = (uint16_t)((myGPSLong - longitudeCenter)+GPSOFFSET)*60*200/10000000;

  sensorStringBuffer[LOGGING_OFFSET_GPS] = (char)(myGPSLatDifference >> 8);
  sensorStringBuffer[LOGGING_OFFSET_GPS+1] = (char)(myGPSLatDifference);
  sensorStringBuffer[LOGGING_OFFSET_GPS+2] = (char)(myGPSLongDifference >> 8);
  sensorStringBuffer[LOGGING_OFFSET_GPS+3] = (char)(myGPSLongDifference);
  sensorStringBuffer[LOGGING_OFFSET_GPS+4] = (char)(myGPSAlt >> 8);
  sensorStringBuffer[LOGGING_OFFSET_GPS+5] = (char)(myGPSAlt);

  myGPSSecondsIntoDay = 3600*myGPSHour+60*myGPSMinutes+myGPSSeconds;
  uint16_t GPSsecondsIntoLogging = myGPSSecondsIntoDay - GPSSecondsIntoDayLoggingStart;
  sensorStringBuffer[LOGGING_OFFSET_TIME] = (char)(GPSsecondsIntoLogging >> 8); 
  sensorStringBuffer[LOGGING_OFFSET_TIME+1] = (char)(GPSsecondsIntoLogging);
  
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
  float outsideTemp = tempsensor.readTempC();
  
  uint16_t outsideTempInt;
  if (outsideTemp>0){
   outsideTempInt = (uint16_t)(outsideTemp+0.5+TEMPOFFSET)*100;
  }
  else{
    outsideTempInt = (uint16_t)(outsideTemp-0.5+TEMPOFFSET)*100; 
  }
  sensorStringBuffer[LOGGING_OFFSET_TEMP] = (uint8_t)(outsideTempInt >> 8);
  sensorStringBuffer[LOGGING_OFFSET_TEMP+1] = (uint8_t)(outsideTempInt);
  
  #ifdef SERIAL_LOGGING 
    Serial.print("Temp: "); 
    Serial.print(outsideTemp, 4); Serial.println("*C"); 
    Serial.println("");
  #endif
  /* Get a new sensor event */ 
 #endif 
  
  
  #ifdef USE_BMP
    int32_t bmpPressure  = bmp.readPressure();
    float   bmpTemperature = bmp.readTemperature();
    float   bmpAltitude = bmp.readAltitude(PRESSURE_AT_SEALEVEL);
    
    uint16_t bmpAltitudeInt = (uint16_t)bmpAltitude + 0.5;

    uint16_t bmpTempInt;
  if (bmpTemperature>0){
    bmpTempInt = (uint16_t)(bmpTemperature+0.5+TEMPOFFSET)*100;
  }
  else{
    bmpTempInt = (uint16_t)(bmpTemperature-0.5+TEMPOFFSET)*100; 
  }
  sensorStringBuffer[LOGGING_OFFSET_BAROMETER]   = (uint8_t)(bmpPressure/10 >> 8);
  sensorStringBuffer[LOGGING_OFFSET_BAROMETER+1] = (uint8_t)(bmpPressure/10);
  sensorStringBuffer[LOGGING_OFFSET_BAROMETER+2] = (uint8_t)(bmpTempInt >> 8);
  sensorStringBuffer[LOGGING_OFFSET_BAROMETER+3] = (uint8_t)(bmpTempInt);
  sensorStringBuffer[LOGGING_OFFSET_BAROMETER+4] = (uint8_t)(bmpAltitudeInt >> 8);
  sensorStringBuffer[LOGGING_OFFSET_BAROMETER+5] = (uint8_t)(bmpAltitudeInt);

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

    //magnetics are adjusted by adding 70uT to the values
    uint16_t magXint, magYint, magZint;
    
    if (magX>0){
      magXint = (uint16_t)(magX+0.5+MAGOFFSET)*100; //in10nT
    }
    else{
      magXint = (uint16_t)(magX-0.5+MAGOFFSET)*100; //in 10nT 
    }

    if (magY>0){
      magYint = (uint16_t)(magY+0.5+MAGOFFSET)*100; //in10nT
    }
    else{
      magYint = (uint16_t)(magY-0.5+MAGOFFSET)*100; //in 10nT 
    }

    if (magZ>0){
      magZint = (uint16_t)(magZ+0.5+MAGOFFSET)*100; //in10nT
    }
    else{
      magZint = (uint16_t)(magZ-0.5+MAGOFFSET)*100; //in 10nT 
    }
    
    
    sensorStringBuffer[LOGGING_OFFSET_MAGNETOMETER]   = (uint8_t)(magXint >> 8);
    sensorStringBuffer[LOGGING_OFFSET_MAGNETOMETER+1] = (uint8_t)(magXint);
    sensorStringBuffer[LOGGING_OFFSET_MAGNETOMETER+2] = (uint8_t)(magYint >> 8);
    sensorStringBuffer[LOGGING_OFFSET_MAGNETOMETER+3] = (uint8_t)(magYint);
    sensorStringBuffer[LOGGING_OFFSET_MAGNETOMETER+4] = (uint8_t)(magZint >> 8);
    sensorStringBuffer[LOGGING_OFFSET_MAGNETOMETER+5] = (uint8_t)(magZint);

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

    uint16_t headingDegreesInt = (uint16_t)(headingDegrees+0.5)*100;

    sensorStringBuffer[LOGGING_OFFSET_MAGNETOMETER+6]   = (uint8_t)(headingDegreesInt >> 8);
    sensorStringBuffer[LOGGING_OFFSET_MAGNETOMETER+7] = (uint8_t)(headingDegreesInt);
    
    #ifdef SERIAL_LOGGING
      /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
      Serial.print("X: "); Serial.print(magX); Serial.print("  ");
      Serial.print("Y: "); Serial.print(magY); Serial.print("  ");
      Serial.print("Z: "); Serial.print(magZ); Serial.print("  ");Serial.println("uT");
      Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
    #endif
  #endif
  
   //read the sensors now and save to sensorStringBuffer
   int8_t Gain;
   digitalWrite(PIN_LED_READING_SENSORS, HIGH);
   
   for (uint8_t sensorToRead = 0; sensorToRead<6; sensorToRead++){

   uint8_t gainByteOffset = sensorToRead/2; //how many gain bytes were written before current iteration 
   uint8_t currentGainByte = gainByteOffset+(gainByteOffset+1)*24+LOGGING_OFFSET_SPECTRAL; //adress of current gain byte


   if (sensorToRead == 2){  //sensor 2 is broken
    continue;
   }
   Gain = readSensor(sensorToRead, &sensorStringBuffer[LOGGING_OFFSET_SPECTRAL+(12*sensorToRead)+gainByteOffset], &Wire);
   
   sensorStringBuffer[currentGainByte] = (Gain << (sensorToRead%2)*8);
  }
  digitalWrite(PIN_LED_READING_SENSORS, LOW);


  if (lastMillis + LOGGING_INTERVAL_MS <= millis()){
   
      lastMillis = millis();
   
   digitalWrite(PIN_LED_WRITING_TO_FILE, HIGH);
   appendFile(filename, sensorStringBuffer, BUFFERLEN);
   digitalWrite(PIN_LED_WRITING_TO_FILE, LOW);
   
   }
  if (lastLoraSendMillis + LORA_INTERVAL_MS <= millis()){
    lora_send_position_altitude_time(myGPSLat, myGPSLong, myGPSAltFeet, myGPSHour, myGPSMinutes, myGPSSeconds);
  }
   

//------------------ reset of file: 
  
  static bool wroteHeader = false;

  static long lastMillisButton0Pushed = millis();
  static long lastMillisButton1Pushed = millis();
  static long lastMillisButtons01Pushed = millis();

  if (!digitalRead(PIN_IN_RESET_FILE_0)){ 
    //button 0 gedrückt
  }
  else{
    lastMillisButton0Pushed = millis();
  }

  if (!digitalRead(PIN_IN_RESET_FILE_1)){ 
    //button 1 gedrückt
  }
  else{
    lastMillisButton1Pushed = millis();
  }

  if (!digitalRead(PIN_IN_RESET_FILE_0) && !digitalRead(PIN_IN_RESET_FILE_1)){ 
    //buttons 0 und 1 gedrückt
  }
  else{
    lastMillisButtons01Pushed = millis();
  }
  
  static long lastMillisLEDBlink = millis();
  
  if (lastMillisButtons01Pushed + WRITE_HEADERFILE_BUTTONPRESS_DURATION_MS < millis()){
    //buttons were pressed for at least WRITE_HEADERFILE...MS
    
    if (lastMillisLEDBlink + 200 < millis()){
      lastMillisLEDBlink = millis();
      digitalWrite(PIN_LED_ABOUT_TO_DELETE_FILE, !digitalRead(PIN_LED_ABOUT_TO_DELETE_FILE));
    }
    if (lastMillisButtons01Pushed + 2*WRITE_HEADERFILE_BUTTONPRESS_DURATION_MS < millis() && !wroteHeader){
      //buttons pushed for twice WRITEHEADERFILE_BUTTON....MS
      //create new file.
      Serial.println("NEW FILE CREATED!!!!");
      wroteHeader = true;
      readFile(filename);

      deleteFile(filename);
      char test_message[] = "HI:"; 
      char start_message[] = "NEW FILE CREATED ---------\n";
      writeFile(filename,test_message,sizeof(test_message));
      appendFile(filename,start_message,sizeof(start_message));
      

      GPSSecondsIntoDayLoggingStart = myGPSSecondsIntoDay;
      latitudeCenter = myGPSLat;
      longitudeCenter = myGPSLong;
    }
  }
  else{
    lastMillisLEDBlink = millis();
    digitalWrite(PIN_LED_ABOUT_TO_DELETE_FILE, LOW);
    wroteHeader = false;
  }

 
  
 



}
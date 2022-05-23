
#include <Arduino.h>

#include <sensor.h>



//SDA = GPIO PIN12
//SDL = GPIO PIN13
void setup()
{
 
 
 Serial.begin(9600);
 delay(10000);
 Serial.println("Hello Word");
  init_sensor();
  

}




void loop()
{
  Serial.println("loop");
  
 // read_sensors();  
  // add sleep
  sleep_ms(990);



}
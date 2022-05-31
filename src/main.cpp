
#include <Arduino.h>

#include <sensor.h>



//SDA1 = GPIO PIN12
//SCL1 = GPIO PIN13
//SDA2 = GPIO PIN14
//SCL2 = GPIO PIN15

void setup()
{
 
 
 Serial.begin(9600);
 delay(10000);
 //Serial.println("Hello Word");
  init_sensor();
  

}




void loop()
{
  Serial.println("loop");
  
 read_sensors();  
  // add sleep
  sleep_ms(990);



}
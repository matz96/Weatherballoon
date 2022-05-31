
#include <Arduino.h>
#include <lora.h>
#include <sensor.h>

int count = 0;

//SDA1 = GPIO PIN12
//SCL1 = GPIO PIN13
//SDA2 = GPIO PIN14
//SCL2 = GPIO PIN15

void setup()
{
 Serial.begin(9600);
 delay(10000);

  
  setup_lora();
  init_sensor();

}




void loop()
{
  //Serial.println("loop");
  
 read_sensors();  
 count ++;
  // add sleep
  sleep_ms(990);
  if(count ==120){
    lora_send();
  }
}
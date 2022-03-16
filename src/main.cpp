#include <Arduino.h>

void setup()
{
  // put your setup code here, to run once:
  pinMode(0, INPUT);
  pinMode(10, INPUT);
  for (int i = 1; i < 5; i++)
  {
    pinMode(i, OUTPUT);
  }
}

void loop()
{
  // put your main code here, to run repeatedly:
  static long unsigned lastMillis = millis();
  static bool firstRun = true;
  static int i;

  if (firstRun)
  {

    firstRun = false;
    lastMillis = millis();
    i = 1;
  }
  
  if (lastMillis + 1000 < millis())
  {
    lastMillis = millis();
    digitalWrite(i , digitalRead(0));
    digitalWrite((i-1) , digitalRead(10));
    if (i==1){
      digitalWrite((4) , digitalRead(10));
    }
    i++;
    if(i==5){
      i=1;
    }
  }
}
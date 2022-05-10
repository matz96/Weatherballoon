#include <Arduino.h>

#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_CS 17
#define RFM95_RST 14
#define RFM95_INT 15

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 434.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

void setup() 
{
  pinMode(RFM95_RST, OUTPUT);
  
  digitalWrite(RFM95_RST, HIGH);

  while (!Serial);
  Serial.begin(9600);
  delay(100);

  Serial.println("Arduino LoRa TX Test!");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) { //for init to work RHSPIDriver.cpp needs to be adjusted by removing ATOMIC_BLOCKs in spi_write and _read
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");
}

void loop()
{
  
}



#include <APRS-Decoder.h>
#include <Arduino.h>
#include "lora_pro4.h"
#include <LoRa.h>

//#define SERIAL_LOGGING

#define LORA_SCK 18
#define LORA_MISO 16
#define LORA_MOSI 19
#define LORA_CS 8 //17
#define LORA_RST 9 //14
#define LORA_IRQ 7 //15
#define LEDPower 26 //Pin des Raspberrys GIPO20
#define LEDFunk 27 //Pin des Raspberrys GIPO21



// cppcheck-suppress unusedFunction
void lora_send_position_altitude_time(uint32_t latIn, uint32_t longIn, uint32_t altIn,uint8_t hourIn, uint8_t minuteIn, uint8_t secondsIn) {
 
  digitalWrite(LEDFunk,HIGH);

  APRSMessage msg;
  //String      lat = "47.50979253564977";
  //String      lng = "7.620736862914847";
  // String      lat = "4737.32";
  // String      lng = "00736.48";
  // String      alt = "530";
  
  msg.setSource("HB9DKQ-03");
  msg.setPath("WIDE1-1");
  msg.setDestination("APLT00"); 

  uint32_t grad_lat = latIn/10000000;
  float minutes_lat = ((latIn - (grad_lat * 10000000))*60)/10000000.;
  
  uint32_t grad_long = longIn/10000000;
  float minutes_long = ((longIn - (grad_long * 10000000))*60)/10000000.;
  //aprsmsg = "!" + lat + "N/" + lng + "EO";
  
  char aprsmsg[]= "/235510z4737.32N/00736.48EO/A=123456";
  


  sprintf(aprsmsg,"/%02d%02d%02dz%02d%05.2fN/%03d%05.2fEO>/A=%06d",(int)hourIn,(int)minuteIn,(int)secondsIn,(int)grad_lat,minutes_lat,(int)grad_long,minutes_long,(int)altIn);

  msg.getAPRSBody()->setData(aprsmsg);
  String data = msg.encode();
  #ifdef SERIAL_LOGGING
  Serial.print("APRS Message: ");
  Serial.println(data);
  #endif
  LoRa.beginPacket();
  #ifdef SERIAL_LOGGING
  Serial.println("Packet Begun");
  #endif
  // Header:
  LoRa.write('<');
  LoRa.write(0xFF);
  LoRa.write(0x01);
  #ifdef SERIAL_LOGGING 
  Serial.println("wrote first 3 bytes");
  #endif
  // APRS Data:
  LoRa.write((const uint8_t *)data.c_str(), data.length());
  #ifdef SERIAL_LOGGING
  Serial.println("wrote most data");
  #endif
  LoRa.endPacket(true); //true for async mode
  #ifdef SERIAL_LOGGING
  Serial.println("ended packet");
  #endif
  digitalWrite(LEDFunk,LOW);
}


void setup_lora() {
    pinMode(LEDPower, OUTPUT);
  pinMode(LEDFunk, OUTPUT);
  digitalWrite(LEDPower,HIGH);
  
  Serial.println("Set SPI pins!");
  SPI.begin();
  Serial.println("Set LoRa pins!");
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);

  long freq = 433775000;
  Serial.print("frequency: ");
  Serial.println(String(freq));
  if (!LoRa.begin(freq)) {
    Serial.println("Starting LoRa failed!");
    while (true) {
    }
  }
  LoRa.setSpreadingFactor(12);
  LoRa.setSignalBandwidth(125000);
  LoRa.setCodingRate4(5);
  LoRa.enableCrc();

  LoRa.setTxPower(20);
  Serial.println("LoRa init done!");
}

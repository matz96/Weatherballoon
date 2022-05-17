#include <APRS-Decoder.h>
#include <Arduino.h>
#include <LoRa.h>

#define LORA_SCK 18
#define LORA_MISO 16
#define LORA_MOSI 19
#define LORA_CS 8 //17
#define LORA_RST 9 //14
#define LORA_IRQ 7 //15
#define LEDPower 26 //Pin des Raspberrys GIPO20
#define LEDFunk 27 //Pin des Raspberrys GIPO21

void setup_lora();
void setup_gps();

// cppcheck-suppress unusedFunction
void setup() {
  Serial.begin(115200);
  
  pinMode(LEDPower, OUTPUT);
  pinMode(LEDFunk, OUTPUT);
  digitalWrite(LEDPower,HIGH);
  setup_lora();

  Serial.println("setup done...");
  delay(500);
}

// cppcheck-suppress unusedFunction
void loop() {
  delay(50000);
  digitalWrite(LEDFunk,HIGH);

  APRSMessage msg;
  //String      lat = "47.50979253564977";
  //String      lng = "7.620736862914847";
  String      lat = "47.5097";
  String      lng = "7.6207";
  String      alt = "530";
  msg.setSource("HB9DKQ");
  msg.setPath("WIDE1-1");
  msg.setDestination("APLT00"); 

  String aprsmsg;
  aprsmsg = "!" + lat + "N/" + lng + "E>";

  msg.getAPRSBody()->setData(aprsmsg);
  String data = msg.encode();
  Serial.println(data);

  LoRa.beginPacket();
  // Header:
  LoRa.write('<');
  LoRa.write(0xFF);
  LoRa.write(0x01);
  // APRS Data:
  LoRa.write((const uint8_t *)data.c_str(), data.length());
  LoRa.endPacket();

  digitalWrite(LEDFunk,LOW);
}


void setup_lora() {
  delay(10000);
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

// includo le librerie necessarie
#include <SPI.h>
#include <LoRa.h>

void setup() {
  //setup per ricevere i dati con lora
  Serial.begin(9600);
  // cambio i pin necessari per adattare la libreria all'esp32
  LoRa.setPins(5, 34, 17);
  //cambio la frequenza
  LoRa.setSPIFrequency(900);
  while (!Serial);

  Serial.println("LoRa Receiver");

  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
}

void loop() {
  //controllo di aver ricevuto dei dati
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    //stampo i dati che ho ricevuto
    while (LoRa.available()) {
      Serial.print((char)LoRa.read());
    }
  }
}

#include <SPI.h>
#include <LoRa.h>
#include <ArduinoJson.h>

#define LORA_SS   10
#define LORA_RST  5
#define LORA_DIO0 4
#define LORA_FREQ 433E6
#define SF    9
#define BW    125E3
#define CR    5

// Pines SPI
#define SCK_PIN   12
#define MISO_PIN  13
#define MOSI_PIN  11

void setupLoRa() {
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("ERROR: RA-02 no responde. Verifica conexiones y antena.");
    while (true);
  }

  LoRa.setSpreadingFactor(SF);
  LoRa.setSignalBandwidth(BW);
  LoRa.setCodingRate4(CR);
  LoRa.setSyncWord(0xBA); // Debe ser identico al transmisor del mismo equipo

  Serial.println("LoRa RX listo --- esperando paquetes...");
}

void procesarPaquete(int tamano) {
  // Leer el JSON recibido
  String jsonStr = "";
  while (LoRa.available()) {
    jsonStr += (char)LoRa.read();
  }

  // Parsear JSON
  StaticJsonDocument<128> doc;
  DeserializationError err = deserializeJson(doc, jsonStr);
  if (err) {
    Serial.print("Error al parsear JSON: ");
    Serial.println(err.c_str());
    return;
  }

  float valor       = doc["v"];
  unsigned long ts  = doc["ts"];
  int id            = doc["id"];
  unsigned long lat = millis() - ts; // Latencia aproximada

  // Leer RSSI y SNR del paquete recibido
  int   rssi = LoRa.packetRssi(); // dBm (negativo, mas cercano a 0 = mejor)
  float snr  = LoRa.packetSnr();  // dB (puede ser negativo en LoRa)

  Serial.print("PKT #"); Serial.print(id);
  Serial.print(" | Val: "); Serial.print(valor, 1);
  Serial.print(" | RSSI: "); Serial.print(rssi);
  Serial.print(" dBm | SNR: "); Serial.print(snr, 1);
  Serial.print(" dB | Lat: "); Serial.print(lat);
  Serial.println(" ms");
}

void setup() {
  Serial.begin(115200);
  while (!Serial); // Espera a que se abra el monitor serie
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, LORA_SS);
  setupLoRa();
}

void loop() {
  // Verificar si llego un paquete (sin bloquear)
  int tamano = LoRa.parsePacket();
  if (tamano > 0) {
    procesarPaquete(tamano);
  }
}

#include <SPI.h>       // Comunicacion SPI (requerida por la libreria LoRa)
#include <LoRa.h>      // Libreria principal del modulo RA-02 / SX1278
#include <ArduinoJson.h>

// ── Pines SPI del RA-02 al ESP32 ────────────────────────────────────────────
#define LORA_SS   10    // NSS  - Chip Select
#define LORA_RST  5   // RST  - Reset
#define LORA_DIO0 4    // DIO0 - Interrupcion de TX/RX listo
// Pines SPI
#define SCK_PIN   12
#define MISO_PIN  13
#define MOSI_PIN  11

// ── Frecuencia: 433 MHz (banda ISM libre en Mexico) ─────────────────────────
#define LORA_FREQ 433E6

// ── Parametros RF ────────────────────────────────────────────────────────────
#define SF    9          // Spreading Factor 9 (balance alcance/velocidad)
#define BW    125E3      // Bandwidth 125 kHz
#define CR    5          // Coding Rate 4/5


float valor = 20.0;      // Dato simulado (en Semana 2: RPM del encoder)
int   contadorPaquetes = 0;

void setupLoRa() {
  // Asignar pines personalizados al modulo
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  // Inicializar en 433 MHz
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("ERROR: RA-02 no responde. Verifica conexiones y antena.");
    while (true);  // Detener si el modulo no inicia
  }

  //Configurar parametros RF (deben ser identicos en TX y RX)
  LoRa.setSpreadingFactor(SF);      // SF9
  LoRa.setSignalBandwidth(BW);      // 125 kHz
  LoRa.setCodingRate4(CR);          // 4/5
  LoRa.setTxPower(14);              // 14 dBm (maximo recomendado sin licencia)
  LoRa.setSyncWord(0xBA);           // Palabra de sincronizacion privada del equipo

  Serial.println("LoRa TX listo --- 433 MHz, SF9, BW125");
}

void setup() {
    Serial.begin(115200);
    while (!Serial); // Espera a que se abra el monitor serie
    SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, LORA_SS);
    setupLoRa();
}

void loop() {
  // Simular lectura de sensor
  valor += 0.3;
  if (valor > 35.0) valor = 20.0;
  contadorPaquetes++;

  // Construir JSON compacto
  StaticJsonDocument<128> doc;
  doc["v"]  = valor;
  doc["ts"] = millis();         // Timestamp para calcular latencia
  doc["id"] = contadorPaquetes;

  char payload[80];
  serializeJson(doc, payload);

  // Enviar paquete LoRa
  LoRa.beginPacket();           // Iniciar paquete
  LoRa.print(payload);          // Escribir datos
  LoRa.endPacket();             // Transmitir (bloqueante hasta que termina)

  Serial.print("TX [");
  Serial.print(contadorPaquetes);
  Serial.print("]: ");
  Serial.println(payload);

  delay(2000);  // Esperar 2 s entre paquetes (respeta duty cycle 1% con SF9)
}

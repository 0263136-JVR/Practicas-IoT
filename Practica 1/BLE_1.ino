#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SERVICE_UUID "b989f340-9c88-4c80-8d1e-88105b8ef1d9"
#define CHAR_RPM_UUID "9b149b6d-205a-4813-b40c-e1703422620a"
#define CHAR_SETPOINT_UUID "6b857f1b-3b69-404b-a7c7-e1e4992fd157"

//variables globales
BLEServer* pServer = nullptr;
BLECharacteristic* pRpmChar = nullptr;

bool deviceConnected = 0;

float rpm = 0.0;
float setpoint = 0.0;

class ServerCallbacks : public BLEServerCallbacks{

  void onConnect (BLEServer* s){
    deviceConnected = true;

    Serial.println(">>> Central concectado!");
  }

  void onDisconnect(BLEServer* s){
    deviceConnected = false;
    s->startAdvertising();

    Serial.println(">>> Central desconcectado!");
  }
};

class SetPointCallbacks : public BLECharacteristicCallbacks{

  void onWrite(BLECharacteristic* c){
    String val = c->getValue();

    if(val.length() > 0){
      setpoint = atof(val.c_str());

      Serial.printf("Setpoint: %.1f RPM\n",setpoint);
    }
  }

};

void setup() {
  Serial.begin(115200);

  //Step 1: BLE INIT
  BLEDevice::init("Motor_JVMA");

  //Step 2: Server & Callbacks
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  //Step 3: Service
  BLEService* pService = pServer->createService(SERVICE_UUID);

  //Set 4: Notify
  pRpmChar = pService->createCharacteristic(
    CHAR_RPM_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  pRpmChar->addDescriptor(new BLE2902);

  //Step 5: Write
  BLECharacteristic* pSpChar = pService->createCharacteristic(
    CHAR_SETPOINT_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  pSpChar->setCallbacks(new SetPointCallbacks());

  //Step 6: Service & advertising
  pService->start();

  BLEAdvertising* pAdv = BLEDevice::getAdvertising();
  pAdv->addServiceUUID(SERVICE_UUID);

  pAdv->setScanResponse(true);

  BLEDevice::startAdvertising();

  Serial.println("BLE listo - esperando conexion...");
}

void loop() {
  if(deviceConnected){
    //Simulate enconder lectura
    rpm +=5;

    if(rpm > 600.0) rpm = 0.0;

    //Convert float->String & Notify
    char buf[10];
    sprintf(buf, "%.1f", rpm);
    pRpmChar->setValue(buf);
    pRpmChar->notify();

    //Log serial monitor
    Serial.printf("RPM: %.1f | Setpoint: %.1f\n", rpm, setpoint);
  }

  delay(500);
}

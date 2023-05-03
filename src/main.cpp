#include "Arduino.h"
// #include "esp_camera.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085.h>
//#include <Adafruit_BM280.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>


// BLECharacteristic *pCharacteristic;

bool deviceConnected = false;
int inc=0;
// Sensor Variable (bmp280/bmp180)
float temperature = -1000;
float pressure;
// float humidity;


void BLETransfer(int16_t);

#define enviornmentService BLEUUID((uint16_t)0x181A)
//Characteristics 
BLECharacteristic temperatureCharacteristic(
  BLEUUID((uint16_t)0x2A6E), 
  BLECharacteristic::PROPERTY_READ | 
  BLECharacteristic::PROPERTY_NOTIFY
);
BLECharacteristic humidityCharacteristic(
  BLEUUID((uint16_t)0x2A6F), 
  BLECharacteristic::PROPERTY_READ | 
  BLECharacteristic::PROPERTY_NOTIFY
);
BLECharacteristic pressureCharacteristic(
  BLEUUID((uint16_t)0x2A6D), 
  BLECharacteristic::PROPERTY_READ | 
  BLECharacteristic::PROPERTY_NOTIFY
);

//BLEDescriptor tempDescriptor(BLEUUID((uint16_t)0x2901));

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      delay(1000);
      pServer->getAdvertising()->start();
      Serial.println("Waiting a client connection to notify...");
    }
};


// -----------------I2C-----------------
#define I2C_SDA 14 // SDA Connected to GPIO 14
#define I2C_SCL 15 // SCL Connected to GPIO 15
TwoWire I2CSensors = TwoWire(0);
// bmp 180 (Using I2C)
Adafruit_BMP085 bmp;

void setup()
{
  delay(1000);
  // Initialize serial communication
  Serial.begin(115200);
  //Initialize Software I2C
  I2CSensors.begin(I2C_SDA, I2C_SCL, 100000);
  // BMP 180/280 (0x77 or 0x76 will be the address)
  if (!bmp.begin(0x76, &I2CSensors))
  {
    Serial.println("Couldn't Find Sensor");
    while (1);
  }
  else
  {
    Serial.println("Sensor Found");
  }

  // Create the BLE Device
  BLEDevice::init("LODÓWA_SENSOR");
  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  // Create the BLE Service
  BLEService *pEnviornment = pServer->createService(enviornmentService);
  // Create a BLE Characteristic
  pEnviornment->addCharacteristic(&temperatureCharacteristic);
  pEnviornment->addCharacteristic(&humidityCharacteristic);
  pEnviornment->addCharacteristic(&pressureCharacteristic);
  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
 
  // Create a BLE Descriptor
  // temperature Descriptor
  temperatureCharacteristic.addDescriptor(new BLE2902());
  BLEDescriptor temperatureDescriptor(BLEUUID((uint16_t)0x2901));
  temperatureDescriptor.setValue("Temperatura [*C]");
  temperatureCharacteristic.addDescriptor(&temperatureDescriptor);
  // pressure Descriptor
  pressureCharacteristic.addDescriptor(new BLE2902());
  BLEDescriptor pressureDescriptor(BLEUUID((uint16_t)0x2901));
  pressureDescriptor.setValue("Ciśnienie [Pa]");
  pressureCharacteristic.addDescriptor(&pressureDescriptor);


  pServer->getAdvertising()->addServiceUUID(enviornmentService);

  // Start the service
  pEnviornment->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");


}

void loop()
{
  temperature = bmp.readTemperature();
  pressure = bmp.readPressure();
  if (inc>10){
    Serial.println(temperature);
    Serial.println(pressure);
    inc = 0;
  }
  else{
    inc++;
  }
  if (deviceConnected) {
    int16_t temperatureValue;
    temperatureValue = (temperature*100);
    Serial.println(temperatureValue);

    int32_t pressureValue;
    pressureValue = (pressure*10);
    Serial.println(pressureValue);

    // BLETransfer(value);
    temperatureCharacteristic.setValue((uint8_t*)&temperatureValue, 2);
    temperatureCharacteristic.notify();

    pressureCharacteristic.setValue((uint8_t*)&pressureValue, 4);
    pressureCharacteristic.notify();
  }
  delay(200);

}


void BLETransfer(int16_t val){
  temperatureCharacteristic.setValue((uint8_t*)&val, 2);
  temperatureCharacteristic.notify();
}
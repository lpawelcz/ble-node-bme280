/*
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleServer.cpp
    Ported to Arduino ESP32 by Evandro Copercini
    updates by chegewara
*/
/***************************************************************************
  This is a library for the BME280 humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BME280 Breakout
  ----> http://www.adafruit.com/products/2650

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface. The device's I2C address is either 0x76 or 0x77.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
  See the LICENSE file for details.
 ***************************************************************************/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>


//#define SERVICE_UUID        "FB349B5f-8000-0080-0010-00001A180000"
#define SERVICE_UUID        "0000181A0000-1000-8000-0080-5F9B34FB"
#define CHARACTERISTIC_HUM_UUID "00002A6F0000-1000-8000-0080-5F9B34FB"
#define CHARACTERISTIC_PRES_UUID "00002A6D0000-1000-8000-0080-5F9B34FB"
#define CHARACTERISTIC_TEMP_UUID "00002A6E0000-1000-8000-0080-5F9B34FB"

// Measurement filters multiplied for matching ESS format
#define TMAX 6000
#define TMIN -3000
#define HMAX 10000
#define HMIN 1
#define PMAX 1060000
#define PMIN 960000

#define AVG_CNT 6

#define DELAY_TIME = 10000
Adafruit_BME280 bme; // I2C
BLECharacteristic *pCharacteristicT, *pCharacteristicH, *pCharacteristicP;

void setup() {
    Serial.begin(115200);
    while(!Serial);    // time to get serial running
    Serial.println(F("BME280 test"));

    unsigned status;

    delay(50);
    // default settings
    status = bme.begin(0x76);
    
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        while (1) delay(10);
    }

    ;

  Serial.println();
  Serial.println("Starting BLE");

  BLEDevice::init("ESP32 THP");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristicT = pService->createCharacteristic(
                                         CHARACTERISTIC_TEMP_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_NOTIFY
                                       );
  pCharacteristicH = pService->createCharacteristic(
                                         CHARACTERISTIC_HUM_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_NOTIFY
                                       );
  pCharacteristicP = pService->createCharacteristic(
                                         CHARACTERISTIC_PRES_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_NOTIFY
                                       );                                     

  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("Characteristics defined");
}


void loop() { 
    int temp, ts = 0,
        hum, hs = 0,
        press, ps = 0,
        tavg_cnt = AVG_CNT,
        havg_cnt = AVG_CNT,
        pavg_cnt = AVG_CNT;

    for(int i = 0; i < AVG_CNT; i++) {
        delay(DELAY_TIME);
        printValues(&temp, &hum, &press);

        // Filter obviously bad readings or add to avg calculation
        if(temp <= TMAX && temp >= TMIN) {
            ts += temp;
        } else {
            tavg_cnt--;
            Serial.print("Obviously bad temperature: ");
            Serial.print(temp);
            Serial.println(" *C");
        }
        if(hum <= HMAX && hum >= HMIN) {
            hs += hum;
        } else {
            havg_cnt--;
            Serial.print("Obviously bad humidity: ");
            Serial.print(hum);
            Serial.println(" %");
        }
        if(press <= PMAX && press >= PMIN) {
            ps += press;
        } else {
            pavg_cnt--;
            Serial.print("Obviously bad pressure: ");
            Serial.print(press);
            Serial.println(" Pa");
        }               
    }

    // Get avgs
    temp = ts / tavg_cnt;
    hum = hs / havg_cnt;
    press = ps / pavg_cnt;

    // Put avgs into characteristics and notify listener
    Serial.print("AVG Temperature = ");
    Serial.print(temp);
    Serial.println(" *C");
    pCharacteristicT->setValue(temp);
    pCharacteristicT->notify();   

    Serial.print("AVG Pressure = ");
    Serial.print(press);
    Serial.println("Pa");
    pCharacteristicP->setValue(press);
    pCharacteristicP->notify();

    Serial.print("AVG Humidity = ");
    Serial.print(hum);
    Serial.println(" %");
    pCharacteristicH->setValue(hum);
    pCharacteristicH->notify();
}


void printValues(int *tmp, int *hm, int *prs) {
    double temp, pres, hum;
    
    temp = bme.readTemperature();
    Serial.print("Temperature = ");
    Serial.print(temp);
    Serial.println(" *C");
    temp *= 100;
    int nt = (int)temp;
    *tmp = nt;

    pres = bme.readPressure();
    Serial.print("Pressure = ");
    Serial.print(pres);
    Serial.println("Pa");
    pres *= 10;
    int np = (int)pres;
    *prs = np;

    hum = bme.readHumidity();
    Serial.print("Humidity = ");
    Serial.print(hum);
    Serial.println(" %");
    hum *= 100;
    int nh = (int)hum;
    *hm = nh;

    Serial.println();
}

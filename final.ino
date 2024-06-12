#include <Wire.h>
#include "LSM6DS3.h"
#include "MAX30105.h"
#include "heartRate.h"
#include <ArduinoBLE.h>


#define MLX90614_I2CADDR 0x5A // Adresa senzora MLX90614
#define CLEAR_STEP      true
#define NOT_CLEAR_STEP  false


const char* serviceUUID = "12345678-1234-1234-1234-123456789abc";
const char* characteristicUUID = "87654321-4321-4321-4321-210987654321";

BLEService customService(serviceUUID); // Vytvorenie BLE služby
BLEStringCharacteristic customCharacteristic(characteristicUUID, BLERead | BLENotify, 20); // Charakteristika pre odosielanie správ


LSM6DS3 pedometer(I2C_MODE, 0x6A);

MAX30105 particleSensor;

const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;

float beatsPerMinute;
int beatAvg;
String result;
float SpO2;

void setup() {

  Wire.begin(); //Inicializácia I2C komunikácie
  Serial.begin(9600); //Inicializácia sériovej komunikácie
  delay(100); // Pauza na inicializáciu

  pinMode(2, OUTPUT);   // ledka bude zelena
  digitalWrite(2, LOW);   //pri nule by zhasla

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }

  BLE.setLocalName("Hrudnikovy-pas"); // Názov zariadenia
  BLE.setAdvertisedService(customService); // Reklamovanie služby
  customService.addCharacteristic(customCharacteristic); // Pridanie charakteristiky do služby
  BLE.addService(customService); // Pridanie služby do BLE stacku

  customCharacteristic.writeValue("Hello!"); // Inicializácia charakteristiky predvolenou hodnotou

  BLE.advertise(); // Začiatok reklamácie

  Serial.println("BLE device active, waiting for connections...");

  while (!Serial);
    if (pedometer.begin() != 0) {
        Serial.println("Device error");
    } else {
        Serial.println("Device OK!");
    }

    //Configure LSM6DS3 as pedometer
    if (0 != config_pedometer(NOT_CLEAR_STEP)) {
        Serial.println("Configure pedometer fail!");
    }
    Serial.println("Success to Configure pedometer!");
    if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) 
    {
      Serial.println("Senzor MAX30105 nebol nájdený. Skontrolujte pripojenie. ");
      while (1);
    }

   particleSensor.setup();
   particleSensor.setPulseAmplitudeRed(0x0A);
   particleSensor.setPulseAmplitudeGreen(0);


   String macAddress = BLE.address();
  
  Serial.print("MAC adresa zariadenia: " + macAddress);
  Serial.println("Zariadenie je pripravene na pripojenie");
}

void loop() {
  BLEDevice central = BLE.central();

  if(central){
    Serial.println("PRIPOJENE");
    Serial.println("Connected to central: " + central.address());
    digitalWrite(LED_BUILTIN, LOW);
    while(central.connected()){
      String messageToSend = getAllData();
      Serial.println(messageToSend);
      customCharacteristic.writeValue(messageToSend);
      delay(10);
    }
  }
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("Aplikacia je odpojena");
}

String getValue() {
  long irValue;
  for(int i = 0; i < 60; i++){
    irValue = particleSensor.getIR();
    if (checkForBeat(irValue) == true)
    {
      long delta = millis() - lastBeat;
      lastBeat = millis();

      beatsPerMinute = 60 / (delta / 1000.0);

      if (beatsPerMinute < 255 && beatsPerMinute > 20)
      {
        rates[rateSpot++] = (byte)beatsPerMinute;
        rateSpot %= RATE_SIZE;

        beatAvg = 0;
        for (byte x = 0 ; x < RATE_SIZE ; x++)
          beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
      }
    }

    String result = "IR=";
    result += irValue;
    result += ", BPM=";
    result += beatsPerMinute;
    result += ", Avg BPM=";
    result += beatAvg;

    if (irValue < 50000){
      result += " No finger?";
    }
  }
  SpO2 = calculateSpO2(irValue);

  return result;
}

String getAllData(){
  float temp = readTempMax();

  uint8_t dataByte = 0;
  uint16_t stepCount = 0;

  pedometer.readRegister(&dataByte, LSM6DS3_ACC_GYRO_STEP_COUNTER_H);
  stepCount = (dataByte << 8) & 0xFFFF;

  pedometer.readRegister(&dataByte, LSM6DS3_ACC_GYRO_STEP_COUNTER_L);
  stepCount |=  dataByte;

  getValue();

  String txt = String(beatAvg)+";"+String(temp)+";"+String(stepCount)+";"+String(SpO2);
  return txt;
}

float readTempMax(){
  float temperature = particleSensor.readTemperature();
  return temperature;
}



float readTemp() {      //NACITANIE TEPLOTY
  byte data[3];

  //Pýtaním senzora na teplotu
  Wire.beginTransmission(MLX90614_I2CADDR);
  Wire.write(0x07); //Register pre čítanie teploty
  Wire.endTransmission(false);
  
  //Čítanie odpovede senzora
  Wire.requestFrom(MLX90614_I2CADDR, 3);
  for(int i = 0; i < 3; i++) {
    data[i] = Wire.read();
  }
  
  //Kombinácia hodnôt na získanie teploty
  float temp = data[1] << 8 | data[0];
  temp *= 0.02;
  temp -= 273.15;
  
  return temp;
}


//Pedometer
int config_pedometer(bool clearStep) {
    uint8_t errorAccumulator = 0;
    uint8_t dataToWrite = 0;  

  //Nasatvenie pedometra
    dataToWrite = 0;

    dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_2g;
    dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_26Hz;

    errorAccumulator += pedometer.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, dataToWrite);

    if (clearStep) {
        errorAccumulator += pedometer.writeRegister(LSM6DS3_ACC_GYRO_CTRL10_C, 0x3E);
    } else {
        errorAccumulator += pedometer.writeRegister(LSM6DS3_ACC_GYRO_CTRL10_C, 0x3C);
    }

    // algoritmus
    errorAccumulator += pedometer.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x40);

    
    errorAccumulator += pedometer.writeRegister(LSM6DS3_ACC_GYRO_INT1_CTRL, 0x10);

    return errorAccumulator;
}

float calculateSpO2(uint32_t irValue) {
  float spo2Value = irValue / 1000.0; // Simulujeme výpočet na základe hodnoty IR
  if(spo2Value < 50) {
    spo2Value = 0.0;
  }
  if (spo2Value > 100.0) {
    spo2Value = 100.0; // Hodnota SpO2 by nemala presiahnuť 100%
  }
  return spo2Value;
}
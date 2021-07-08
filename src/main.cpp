#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

// Sensor de Temperatura e Umidade
#define DHTPIN 4
#define DHTTYPE    DHT11
DHT_Unified dht(DHTPIN, DHTTYPE);
// Sensor de Pressão Atmosférica e Temperatura
#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)
Adafruit_BMP280 bmp;
byte I2C_ADRESS = 0x76;
// Sensor de Radiacao Solar
#define ML8511PIN 34

uint32_t measurementPeriod = 5000;

void dht11_read(){
  // Obtem um evento de temperatura e mostra o valor
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error! Temperatura!?"));
  }
  else {
    Serial.print(F("Temperatura: "));
    Serial.print(event.temperature);
    Serial.println(F("°C"));
  }

  // Obtem um evento de umidade e mostra o valor
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error! Umidade!?"));
  }
  else {
    Serial.print(F("Umidade do ar: "));
    Serial.print(event.relative_humidity);
    Serial.println(F("%"));
  }
}

void bmp280_read(){
    Serial.print(F("Temperatura: "));
    Serial.print(bmp.readTemperature());
    Serial.println("°C");

    Serial.print(F("Pressao atmosferica: "));
    Serial.print(bmp.readPressure());
    Serial.println("Pa");

    Serial.print(F("Altitude~: "));
    Serial.print(bmp.readAltitude(1013.25)); // pressao atmosferica no nivel do mar  
    Serial.println("m");
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
//Takes an average of readings on a given pin and Returns the average
int averageAnalogRead(int pinToRead)
{
  byte numberOfReadings = 8;
  unsigned int runningValue = 0; 
 
  for(int x = 0 ; x < numberOfReadings ; x++)
    runningValue += analogRead(pinToRead);
  runningValue /= numberOfReadings;
 
  return(runningValue);
}
 
void ml8511_read(int SensorPIN){
  int uvLevel = averageAnalogRead(SensorPIN);
 
  float outputVoltage = 3.3 * uvLevel/4095;
  float uvIntensity = mapfloat(outputVoltage, 0.99, 2.9, 0.0, 15.0);
 
  Serial.print("Intensidade UV: ");
  Serial.print(uvIntensity);
  Serial.print("mW/cm^2"); 
}

void setup() {
  Serial.begin(9600);

  // Iniciando configuracao do sensor: DHT11
  dht.begin();
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  dht.humidity().getSensor(&sensor);

  // Iniciando configuracao do sensor: BMP280
  if (!bmp.begin(I2C_ADRESS)) {
    Serial.println(F("Could not find a valid BMP280 sensor!"));
    while (1) delay(10);
  } 
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     
                  Adafruit_BMP280::SAMPLING_X2,     
                  Adafruit_BMP280::SAMPLING_X16,    
                  Adafruit_BMP280::FILTER_X16,      
                  Adafruit_BMP280::STANDBY_MS_500); 

  // Iniciando configuracao do sensor: ML8511
  pinMode(ML8511PIN, INPUT);
}

void loop() {
  delay(measurementPeriod);
  Serial.println("================================");
  Serial.println("----- DHT11 -----");
  dht11_read();
  Serial.println("----- BMP280 -----");
  bmp280_read();
  Serial.println("----- ML8511 -----");
  ml8511_read(ML8511PIN);
  Serial.println("\n================================");
}
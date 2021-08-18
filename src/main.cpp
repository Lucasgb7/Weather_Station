#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <ADSWeather.h>

// Sensor de Temperatura e Umidade (DHT11)
#define DHTPIN 4
#define DHTTYPE    DHT11
DHT_Unified dht(DHTPIN, DHTTYPE);

// Sensor de Pressao Atmosferica e Temperatura (BMP280)
#define BMP_SDA 21
#define BMP_SCL 22
Adafruit_BMP280 bmp;
byte I2C_ADRESS = 0x76; // endereco adquirido executando o arquivo '../test/getI2C.ino'.

// Sensor de Radiacao Solar (ML8511)
#define ML8511PIN 34

// Pluviometro (PDB10U)
#define RAIN_PIN 35
int val = 0;
int old_val = 0;
int REEDCOUNT = 0;

// Anemometro e Cata-vento (MISOL WH-SP-WD e MISOL WH-SP-WS01)
#define ANEMOMETER_PIN 26
#define VANE_PIN 2
#define CALC_INTERVAL 1000
unsigned long nextCalc;
unsigned long timer;
int windDir;
int windSpeed;
int rainAmmount;
ADSWeather ws1(RAIN_PIN, VANE_PIN, ANEMOMETER_PIN); //This should configure all pins correctly

// Code 
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
    Serial.println(" °C");

    Serial.print(F("Pressao Atmosferica: "));
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");

    Serial.print(F("Altitude: "));
    Serial.print(bmp.readAltitude(1013.25)); // pressao atmosferica no nivel do mar (101.325 Pa)
    Serial.println(" m");
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

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
 
  Serial.println("Intensidade UV: ");
  Serial.print(uvIntensity);
  Serial.print("mW/cm^2"); 
}

void pluviometro_read(int INPUTPIN){
  val = digitalRead(INPUTPIN);      // Lê o Status do Reed Switch
  if ((val == LOW) && (old_val == HIGH)) {   // Verefica se o Status mudou
    delay(10);                   // Atraso colocado para lidar com qualquer "salto" no switch.
    REEDCOUNT = REEDCOUNT + 1;   // Adiciona 1 à cntagem de pulsos
    old_val = val;              //Iguala o valor antigo com o atual
 
    // Imprime no Monitor Serial
    Serial.print("Medida de chuva (contagem): ");
    Serial.print(REEDCOUNT);
    Serial.println(" pulso");
    Serial.print("Medida de chuva (calculado): ");
    Serial.print(REEDCOUNT * 0.50);
    Serial.println(" mm");
  }
 
    if ((val == HIGH) && (old_val == LOW)) {   // Verefica se o Status mudou
    delay(10);                   // Atraso colocado para lidar com qualquer "salto" no switch.
    REEDCOUNT = REEDCOUNT + 1;   // Adiciona 1 à cntagem de pulsos
    old_val = val;              //Iguala o valor antigo com o atual
 
    // Imprime no Monitor Serial
    Serial.print("Medida de chuva (contagem): ");
    Serial.print(REEDCOUNT);
    Serial.println(" pulso");
    Serial.print("Medida de chuva (calculado): ");
    Serial.print(REEDCOUNT * 0.50);
    Serial.println(" mm");
  }
 
  else {
    old_val = val;              //If the status hasn't changed then do nothing
  }
}

void ws1_read(){
  ws1.update(); // Atualiza os valores WS1

    
    rainAmmount = ws1.getRain();
    windSpeed = ws1.getWindSpeed();

    Serial.println("Velocidade do Vento: ");
    Serial.print(windSpeed / 10); // componente inteira da velocidade
    Serial.print('.');
    Serial.print(windSpeed % 10); // componente fracionaria da velocidade
    Serial.print("km/h");

    Serial.print("Gusting at: ");
    Serial.print(ws1.getWindGust() / 10);
    Serial.print('.');
    Serial.print(ws1.getWindGust() % 10);
    Serial.println("");

    Serial.print("Direção do Vento: ");
    Serial.print(ws1.getWindDirection());
    Serial.println("º");

    Serial.print("Precipitação: ");
    Serial.println((float) rainAmmount / 1000);
    Serial.print("mm");
  
}

void setup() {
  Serial.begin(9600);

  // Config. DHT11
  dht.begin();
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  dht.humidity().getSensor(&sensor);

  // Config. BMP280
  if (!bmp.begin(I2C_ADRESS)) {
    Serial.println(F("Não foi possível encontrar o sensor BMP280!"));
    while (1) delay(10);
  }

  // Config. ML8511
  pinMode(ML8511PIN, INPUT);

  // Config. Pluviometro e Anemometro/Cata-vento
  //pinMode (REED, INPUT_PULLUP); //This activates the internal pull up resistor
  attachInterrupt(digitalPinToInterrupt(RAIN_PIN), ws1.countRain, FALLING);
  attachInterrupt(digitalPinToInterrupt(ANEMOMETER_PIN), ws1.countAnemometer, FALLING);
  nextCalc = millis() + CALC_INTERVAL;

}

void loop() { 
  Serial.println("================================");
  Serial.println("----- DHT11 -----");
  dht11_read();
  Serial.println("----- BMP280 -----");
  bmp280_read();
  Serial.println("----- ML8511 -----");
  ml8511_read(ML8511PIN);
  Serial.println("\n----- WS1 -----");
  ws1_read();
  Serial.println("\n================================");
  delay(measurementPeriod); // tempo de mensuracao
}
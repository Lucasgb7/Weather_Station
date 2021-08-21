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
#define ML8511PIN 15

// Pluviometro (PDB10U)
#define RAIN_PIN 35
int val = 0;
int old_val = 0;
int REEDCOUNT = 0;

// Anemometro e Cata-vento (MISOL WH-SP-WD e MISOL WH-SP-WS01)
#define ANEMOMETER_PIN 26
#define VANE_PIN 34
#define CALC_INTERVAL 1000

boolean state = false;
unsigned long lastMillis = 0;
float mps, kph, windChill;
int clicked, wspd, wdir, wdirRaw, wchill;

float tempair, tempground;
int temp, tempin;

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
    Serial.println(F(" °C"));
  }

  // Obtem um evento de umidade e mostra o valor
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error! Umidade!?"));
  }
  else {
    Serial.print(F("Umidade do ar: "));
    Serial.print(event.relative_humidity);
    Serial.println(F(" %"));
  }
}

void bmp280_read(){
    Serial.print(F("Temperatura: "));
    Serial.print(bmp.readTemperature());
    Serial.println(" °C");

    Serial.print(F("Pressao Atmosferica: "));
    Serial.print(bmp.readPressure()/100);
    Serial.println(" hPa");

    Serial.print(F("Altitude: "));
    Serial.print(bmp.readAltitude(1019)); // pressao atmosferica no nivel do mar (variavel)
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
 
  Serial.print("Intensidade UV: ");
  Serial.print(uvIntensity);
  Serial.println(" mW/cm^2"); 
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

  Serial.print("Velocidade do Vento: ");
  Serial.print(windSpeed / 10); // componente inteira da velocidade
  Serial.print('.');
  Serial.print(windSpeed % 10); // componente fracionaria da velocidade
  Serial.println(" km/h");

  Serial.print("Gusting at: ");
  Serial.print(ws1.getWindGust() / 10);
  Serial.print('.');
  Serial.print(ws1.getWindGust() % 10);
  Serial.println(" km/h");

  Serial.print("Direção do Vento: ");
  Serial.print(ws1.getWindDirection());
  Serial.println(" º");

  Serial.print("Precipitação: ");
  Serial.print((float) rainAmmount / 1000);
  Serial.println(" mm");
}

void read_windSpeed()
{
    lastMillis = xTaskGetTickCount();
    while(xTaskGetTickCount() - lastMillis < 10000){
        if(digitalRead(ANEMOMETER_PIN) == HIGH) if(state == false){
            delay(50);
            clicked++;
            state = true;
        }
        if(digitalRead(ANEMOMETER_PIN) == LOW) state = false;
    }

    mps = clicked * 0.0333; // metros/segundo
    kph = mps * 3.6;        // kilometros/hora
    wspd = int(mps*10);
    clicked = 0;

    Serial.print("Velocidade do Vento: ");
    Serial.print(wspd);
    Serial.println(" m/s");
}

void read_windDirection()
{
  wdirRaw = analogRead(VANE_PIN);

  if(wdirRaw > 2155 && wdirRaw < 2207) wdir = 0;
  else if (wdirRaw > 502  && wdirRaw < 554) wdir = 22.5;
  else if (wdirRaw > 649  && wdirRaw < 699) wdir = 45;
  else if (wdirRaw > 49 && wdirRaw < 90) wdir = 112.5;
  else if (wdirRaw > 89 && wdirRaw < 111) wdir = 135;
  else if (wdirRaw > 0 && wdirRaw < 11) wdir = 157.5;
  else if (wdirRaw > 242 && wdirRaw < 280) wdir = 180;
  else if (wdirRaw > 159 && wdirRaw < 211) wdir = 202.5;
  else if (wdirRaw > 1239 && wdirRaw < 1301) wdir = 225;
  else if (wdirRaw > 1129 && wdirRaw < 1161) wdir = 247.5;
  else if (wdirRaw > 4069 && wdirRaw < 5001) wdir = 270;
  else if (wdirRaw > 2470 && wdirRaw < 2516) wdir = 292.5;
  else if (wdirRaw > 3101 && wdirRaw < 3321) wdir = 315;
  else if (wdirRaw > 1629 && wdirRaw < 1656) wdir = 337.5;
  else if(wdirRaw == 0) wdir = 90;

  Serial.print("wdirRaw: ");
  Serial.println(wdirRaw);
  Serial.print("Direção do Vento: ");
  Serial.print(wdir);
  Serial.println(" °");
}
void read_windDirection2()
{
  wdirRaw = analogRead(VANE_PIN);

  if(wdirRaw > 1916 && wdirRaw <= 2273) wdir = 0;           // 33k  - 0º    - N
  else if (wdirRaw > 435  && wdirRaw <= 645) wdir = 22.5;   // 6k57 - 22.5º - NNE
  else if (wdirRaw > 645  && wdirRaw <= 850) wdir = 45;     // 8k2  - 45º   - NE
  else if (wdirRaw > 70 && wdirRaw <= 90) wdir = 67.5;      // 891  - 67.5º - ENE
  else if (wdirRaw > 90 && wdirRaw <= 115) wdir = 90;       // 1k   - 90º   - E
  else if (wdirRaw >= 0 && wdirRaw <= 70) wdir = 112.5;     // 688  - 112.5º- ESE
  else if (wdirRaw > 173 && wdirRaw <= 250) wdir = 135;     // 2k2  - 135º  - SE
  else if (wdirRaw > 115 && wdirRaw <= 173) wdir = 157.5;   // 1k41 - 157.5º- SSE
  else if (wdirRaw > 325 && wdirRaw <= 645) wdir = 180;     // 3k9  - 180º  - S
  else if (wdirRaw > 250 && wdirRaw <= 325) wdir = 202.5;   // 3k14 - 202.5º- SSW
  else if (wdirRaw > 1170 && wdirRaw <= 1480) wdir = 225;   // 16k  - 225º  - SW
  else if (wdirRaw > 850 && wdirRaw <= 1170) wdir = 247.5;  // 14k12- 247.5º- WSW 
  else if (wdirRaw > 3347 && wdirRaw <= 4095) wdir = 270;   // 120k - 270º  - W
  else if (wdirRaw > 2273 && wdirRaw <= 2917) wdir = 292.5; // 42k12- 292.5º- WNW
  else if (wdirRaw > 2917 && wdirRaw <= 3347) wdir = 315;   // 64k9 - 315º  - NW
  else if (wdirRaw > 1480 && wdirRaw <= 1916) wdir = 337.5; // 21k88- 337.5º- NNW
  //else if(wdirRaw == 0) wdir = 90;

  Serial.print("wdirRaw: ");
  Serial.println(wdirRaw);
  Serial.print("Direção do Vento: ");
  Serial.print(wdir);
  Serial.println(" °");
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
  pinMode(ANEMOMETER_PIN, INPUT);
  //pinMode (REED, INPUT_PULLUP); //This activates the internal pull up resistor
  //attachInterrupt(digitalPinToInterrupt(RAIN_PIN), ws1.countRain, FALLING);
  //attachInterrupt(digitalPinToInterrupt(ANEMOMETER_PIN), ws1.countAnemometer, FALLING);
  //nextCalc = millis() + CALC_INTERVAL;

}

void loop() {

  /*
  Serial.println("================================");
  Serial.println("----- DHT11 -----");
  dht11_read();
  Serial.println("----- BMP280 -----");
  bmp280_read();
  Serial.println("----- ML8511 -----");
  ml8511_read(ML8511PIN);
  */
  //Serial.println("================================");
  //Serial.println("----- ANEMOMETRO -----");
  //read_windSpeed();
  Serial.println("----- CATA-VENTO -----");
  read_windDirection2();
  Serial.println("================================");
  delay(2000);
  //delay(measurementPeriod); // tempo de mensuracao
}
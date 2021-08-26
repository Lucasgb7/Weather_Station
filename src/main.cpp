#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <ADSWeather.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <HardwareSerial.h>
#include <SPI.h>
#include "../lib/keys.h"
#include "RoboCore_SMW_SX1276M0.h"
#include <Wire.h>

// Temperature/Humidity Sensor (DHT11)
#define DHTPIN 4
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// Temperature/Atmosferic Pressure Sensor (BMP280) 
// using SPI interface
#define BMP_SDA 21 
#define BMP_SCL 22
Adafruit_BMP280 bmp;
byte I2C_ADRESS = 0x76; // '../test/getI2C.ino'.

// UV Radiation Sensor (ML8511)
#define UV_PIN 15

// Rain Sensor (PDB10U)
#define RAIN_PIN 35
int val = 0;
int old_val = 0;
int REEDCOUNT = 0;

// Anemometer and Wind Vane (MISOL WH-SP-WD e MISOL WH-SP-WS01)
#define ANEMOMETER_PIN 26
#define VANE_PIN 34
#define CALC_INTERVAL 1000

boolean state = false;
unsigned long lastMillis = 0;
float mps, kph, windChill;
int clicked, wspd, wdir, wdirRaw, wchill;
ADSWeather ws1(RAIN_PIN, VANE_PIN, ANEMOMETER_PIN); //This should configure all pins correctly

// LoRaWAN settings
#define RXD2 16
#define TXD2 17
#define ONBOARD_LED 2 // LoRaWAN Status LED

HardwareSerial LoRaSerial(2);
SMW_SX1276M0 lorawan(LoRaSerial);
CommandResponse response;

// Keys are included by #include "../lib/keys.h"
//const char APPEUI[] = "0000000000000000";
//const char APPKEY[] = "00000000000000000000000000000000";

const unsigned long PAUSE_TIME = 300000; // [ms] (5 min)
unsigned long timeout;

/*================================ FUNCTIONS ================================*/
// Prototype
void event_handler(Event);

// DHT11 - Temperature
float getTemperature()
{
  return dht.readTemperature();
}

// DHT11 - Humidity
float getHumidity()
{
  return dht.readHumidity();
}

// BMP280 - Atmospheric Pressure
float getPressure()
{
  // bmp.readTemperature();
  // bmp.readAltitude(1019) 
  return bmp.readPressure()/100; // return pressure in hPa
}

// Map for UV reading
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Takes an average of readings on a given pin
int averageAnalogRead(int pinToRead)
{
  byte numberOfReadings = 8;
  unsigned int runningValue = 0; 
 
  for(int x = 0 ; x < numberOfReadings ; x++)
    runningValue += analogRead(pinToRead);
  runningValue /= numberOfReadings;
 
  return(runningValue);
}

// ML8511 - UV Intensity
float getUV(int SensorPIN)
{
  int uvLevel = averageAnalogRead(SensorPIN);

  float outputVoltage = 3.3 * uvLevel/4095;
  float uvIntensity = mapfloat(outputVoltage, 0.99, 2.9, 0.0, 15.0);

  return uvIntensity;
}

// PDB10U - Rain Gauge
float getRain(int SensorPIN)
{
  val = digitalRead(SensorPIN);
  if ((val == LOW) && (old_val == HIGH)) {   
    delay(10);              
    REEDCOUNT = REEDCOUNT + 1;   
    old_val = val;              
  }
  if ((val == HIGH) && (old_val == LOW)) {   
  delay(10);
  REEDCOUNT = REEDCOUNT + 1;
  old_val = val;
 
  } 
  else {
    old_val = val;
  }
  return REEDCOUNT * 0.5;
}

// MISOL WH-SP-WS01 - Anemometer (Wind Speed Sensor)
int getWindSpeed()
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

    mps = clicked * 0.0333; // m/s
    kph = mps * 3.6;        // km/h
    wspd = int(mps*10);
    clicked = 0;

    return wspd; // m/s
}

// MISOL WH-SP-WD - Wind Vane (Wind Directtion Sensor)
int getWindDirection()
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

  return wdir;
}

void blink(int LED_PIN)
{
  digitalWrite(LED_PIN, LOW);
  delay(500);
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
  delay(500);
}

void sendData()
{
  DynamicJsonDocument jsonData(JSON_OBJECT_SIZE(7));
  jsonData["T"] = getTemperature();
  jsonData["H"] = getHumidity();
  jsonData["P"] = getPressure();
  jsonData["U"] = getUV(UV_PIN);
  jsonData["D"] = getWindDirection();
  jsonData["S"] = getWindSpeed();
  jsonData["R"] = getRain(RAIN_PIN);

  String payload = "";
  serializeJson(jsonData, payload);
  Serial.print("Data sent: ");
  Serial.println(payload);

  lorawan.sendT(1, payload);
  blink(ONBOARD_LED); // reporting a sent status on LED
}

void setup() {
  Serial.begin(115200);
  pinMode(ONBOARD_LED, OUTPUT);
  /*=============================== SENSORS ===============================*/
  // Temperature and Humidty Sensor DHT11
  dht.begin();
  // Pressure Sensor BMP280
  bmp.begin(I2C_ADRESS);
  // UV Sensor
  pinMode(UV_PIN, INPUT);
  // Rain gauge sensor
  pinMode (RAIN_PIN, INPUT_PULLUP); //This activates the internal pull up resistor
  // Wind wane sensor
  pinMode(ANEMOMETER_PIN, INPUT);
  /*=============================== LORAWAN ===============================*/
  Serial.println(F("--- SMW_SX1276M0 Join (OTAA) ---"));
  
  // start the UART for the LoRaWAN Bee
  LoRaSerial.begin(115200, SERIAL_8N1, RXD2, TXD2);

  // set the event handler
  lorawan.event_listener = &event_handler;
  Serial.println(F("Handler set"));

  // Lemos o DEVEUI do modulo LoRaWAN Bee e mostramos no Monitor Serial
  char deveui[16];
  response = lorawan.get_DevEUI(deveui);
  if(response == CommandResponse::OK){
    Serial.print(F("DevEUI: "));
    Serial.write((uint8_t *)deveui, 16);
    Serial.println();
  } else {
    Serial.println(F("Error getting the Device EUI"));
  }

  // Configuramos a chave APPEUI no modulo e mostramos no Monitor Serial
  response = lorawan.set_AppEUI(APPEUI);
  if(response == CommandResponse::OK){
    Serial.print(F("Application EUI set ("));
    Serial.write((uint8_t *)APPEUI, 16);
    Serial.println(')');
  } else {
    Serial.println(F("Error setting the Application EUI"));
  }

  // Configuramos a chave APPKEY no modulo e mostramos no Monitor Serial
  response = lorawan.set_AppKey(APPKEY);
  if(response == CommandResponse::OK){
    Serial.print(F("Application Key set ("));
    Serial.write((uint8_t *)APPKEY, 32);
    Serial.println(')');
  } else {
    Serial.println(F("Error setting the Application Key"));
  }

  // Configuramos o modo de operacao do LoRaWAN Bee para OTAA
  response = lorawan.set_JoinMode(SMW_SX1276M0_JOIN_MODE_OTAA);
  if(response == CommandResponse::OK){
    Serial.println(F("Mode set to OTAA"));
  } else {
    Serial.println(F("Error setting the join mode"));
  }

  // Comecamos as tentativas para conexao na rede LoRaWAN da ATC
  Serial.println(F("Joining the network"));
  lorawan.join();

}

void loop() 
{
  // "Escutamos" se algo vem do modulo LoRaWAN Bee
  lorawan.listen();

  // Se esta conectado a rede entra nesta rotina
  if(lorawan.isConnected()){
    
   	// A cada PAUSE_TIME milisegundos, acessamos a funcao de envio de dados
    if(timeout < millis()){

      sendData();
      
      timeout = millis() + PAUSE_TIME;
    }
  } else {
    // Se nao conseguir se conectar a rede LoRaWAN, imprime no 
    // Monitor Serial "." a cada 5 segundos
    if(timeout < millis()){
      Serial.println('...');
      timeout = millis() + 5000; // 5 s
    }
  }
}

void event_handler(Event type){
  // check if join event
  if(type == Event::JOINED){
    Serial.println(F("Joined"));
    digitalWrite(ONBOARD_LED, HIGH);
  }
}
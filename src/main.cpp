#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
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

// Temperature/Atmosferic Pressure Sensor (BMP280) (using SPI interface)
#define BMP_SDA 21 
#define BMP_SCL 22
Adafruit_BMP280 bmp;
byte I2C_ADRESS = 0x76; // '../test/getI2C.ino'.

// UV Radiation Sensor (ML8511)
#define UV_PIN 15

// Rain Gauge (PDB10U)
#define RAIN_PIN 35
int val = 0, old_val = 0;
RTC_DATA_ATTR int REEDCOUNT = 0;
const unsigned long RAIN_TIME = 15*60*1000; // measuring time (ms)

// Anemometer and Wind Vane (MISOL WH-SP-WD e MISOL WH-SP-WS01)
#define ANEMOMETER_PIN 26
#define VANE_PIN 34
#define CALC_INTERVAL 1000
boolean state = false;
const unsigned long WINDSPEED_TIME = 5000; // measuring time (ms)
unsigned long lastMillis = 0;
float mps, kph;
int clicked, wspd, wdir, wdirRaw;

// Module LoRaWAN Bee V2 settings
#define RXD2 16
#define TXD2 17
#define STATUS_LED 2 // LoRaWAN Send Status

HardwareSerial LoRaSerial(2);
SMW_SX1276M0 lorawan(LoRaSerial);
CommandResponse response;

// Keys are included from #include "../lib/keys.h"

const unsigned long PAUSE_TIME = 60000; // [ms] (3 min)
unsigned long timeout;
int count = 0;

// Storing data on deep sleep
struct Package {
  float temperature, humidity, pressure, uv, windDirection, windSpeed, rain;
} WE_Package;
/*================================ FUNCTIONS ================================*/

// Prototype
void event_handler(Event);

// Get the temperature from the DHT11 sensor
//  @returns The float temperature (ºC)
float getTemperature()
{
  return dht.readTemperature();
}

// Get the humidity from the DHT11 sensor
//  @returns The float humidity (%)
float getHumidity()
{
  return dht.readHumidity();
}

// Get the atmospheric pressure from the BMP280 sensor
//  @returns The float pressure (hPA)
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

// Get the UV radiation intensity from the ML8511 sensor
//  @param (SensorPIN) : the connected pin on the microcontroller [int]
//  @returns The float UV intensity (mW/cm²)
float getUV(int SensorPIN)
{
  int uvLevel = averageAnalogRead(SensorPIN);

  float outputVoltage = 3.3 * uvLevel/4095;
  float uvIntensity = mapfloat(outputVoltage, 0.99, 2.9, 0.0, 15.0);

  return uvIntensity;
}

// Get the precipitation from the PDB10U rain gauge
//  @param (SensorPIN) : the connected pin on the microcontroller [int]
//  @returns The float amount precipitation (mm³)
float getRain(int SensorPIN)
{
  Serial.println("Reading PRECIPITATION...");
  lastMillis = xTaskGetTickCount();
  // It takes RAIN_TIME seconds to measure
  while(xTaskGetTickCount() - lastMillis < RAIN_TIME){
    val = digitalRead(SensorPIN);
    if (((val == LOW) && (old_val == HIGH)) || ((val == HIGH) && (old_val == LOW))) {  
      delay(10);  // waiting for counting update
      REEDCOUNT = REEDCOUNT + 1; // total counting
      old_val = val;
    } else {
      old_val = val;
    }
  }
  return REEDCOUNT * 0.5; // every 'clock' = 0.5mm
}

// Get the wind speed from the Anemometer MISOL WH-SP-WS01
//  @returns The float speed (km/h)
float getWindSpeed()
{
  Serial.println("Reading WIND SPEED...");
  lastMillis = xTaskGetTickCount();
  // It takes WINDSPEED_TIME seconds to measure wind speed
  while(xTaskGetTickCount() - lastMillis < WINDSPEED_TIME){
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

  return kph;
}

// MISOL WH-SP-WD - Wind Vane (Wind Directtion Sensor)
// Get the wind speed from the Wind Vane MISOL WH-SP-WD
//  @returns The float direction (º)
int getWindDirection()
{
  wdirRaw = analogRead(VANE_PIN);
  // V_REF = 3,3V; R_REF = 10K
  if(wdirRaw > 3225 && wdirRaw <= 3495) wdir = 0;             // 33k  - 0º    - N
  else if (wdirRaw > 1759  && wdirRaw <= 1879) wdir = 22.5;   // 6k57 - 22.5º - NNE
  else if (wdirRaw > 1879  && wdirRaw <= 2298) wdir = 45;     // 8k2  - 45º   - NE
  else if (wdirRaw > 324 && wdirRaw <= 383) wdir = 67.5;      // 891  - 67.5º - ENE
  else if (wdirRaw > 383 && wdirRaw <= 476) wdir = 90;        // 1k   - 90º   - E
  else if (wdirRaw >= 0 && wdirRaw <= 324) wdir = 112.5;      // 688  - 112.5º- ESE
  else if (wdirRaw > 674 && wdirRaw <= 930) wdir = 135;       // 2k2  - 135º  - SE
  else if (wdirRaw > 476 && wdirRaw <= 674) wdir = 157.5;     // 1k41 - 157.5º- SSE
  else if (wdirRaw > 1152 && wdirRaw <= 1502) wdir = 180;     // 3k9  - 180º  - S
  else if (wdirRaw > 930 && wdirRaw <= 1152) wdir = 202.5;    // 3k14 - 202.5º- SSW
  else if (wdirRaw > 2664 && wdirRaw <= 2887) wdir = 225;     // 16k  - 225º  - SW
  else if (wdirRaw > 2298 && wdirRaw <= 2664) wdir = 247.5;   // 14k12- 247.5º- WSW 
  else if (wdirRaw > 3969 && wdirRaw <= 4095) wdir = 270;     // 120k - 270º  - W
  else if (wdirRaw > 3495 && wdirRaw <= 3715) wdir = 292.5;   // 42k12- 292.5º- WNW
  else if (wdirRaw > 3715 && wdirRaw <= 3969) wdir = 315;     // 64k9 - 315º  - NW
  else if (wdirRaw > 2887 && wdirRaw <= 3225) wdir = 337.5;   // 21k88- 337.5º- NNW

  return wdir;
}

// Blink function for LoRaWAN status
void blink(int LED_PIN)
{
  digitalWrite(LED_PIN, LOW);
  delay(500);
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
  delay(500);
}

// Print variables readings on Serial Monitor
void printData()
{
  Serial.println("===========================================================");
  
  Serial.println("---------- DHT11 ---------");
  Serial.print("Temperature: "); Serial.print(getTemperature()); Serial.println(" °C");
  Serial.print("Humidity: "); Serial.print(getHumidity()); Serial.println(" %");
  
  Serial.println("--------- BMP280 ---------");
  Serial.print("Atmospheric Pressure: "); Serial.print(getPressure()); Serial.println(" hPa");
  
  Serial.println("--------- ML8511 ---------");
  Serial.print("UV Intensity: "); Serial.print(getUV(UV_PIN)); Serial.println(" mW/cm²");
  
  Serial.println("--------- PDB10U ---------");
  Serial.print("Precipitation: "); Serial.print(getRain(RAIN_PIN)); Serial.println(" mm");
  
  Serial.println("--------- WH-SP-WS01 ---------");
  Serial.print("Wind Speed: "); Serial.print(getWindSpeed()); Serial.println(" km/h");
  
  Serial.println("---------  WH-SP-WD ---------");
  Serial.print("Wind Direction: "); Serial.print(getWindDirection()); Serial.println(" °");


  Serial.println("===========================================================");
}

// Read the weather station variables
Package readData(){
  Serial.println("Reading data...");
  Package p;
  p.temperature = getTemperature();
  p.humidity = getHumidity();
  p.pressure = getPressure();
  p.uv = getUV(UV_PIN);
  p.windSpeed = getWindSpeed();
  p.windDirection = getWindDirection();
  p.rain = getRain(RAIN_PIN);

  return p;
}

// Sending JSON data by LoRaWAN module
void sendData(Package p)
{
  DynamicJsonDocument jsonData(JSON_OBJECT_SIZE(7));

  jsonData["T"] = (int) p.temperature;
  jsonData["H"] = (int) p.humidity;
  jsonData["P"] = (int) p.pressure;
  jsonData["U"] = (int) p.uv;
  jsonData["D"] = (int) p.windDirection;
  jsonData["S"] = (int) p.windSpeed;
  jsonData["R"] = (int) p.rain;

  String payload = "";
  serializeJson(jsonData, payload);
  Serial.print("Data sent: ");
  Serial.println(payload);
  
  // Send a text message
  lorawan.sendT(1, payload);
  blink(STATUS_LED); // reporting a sent status on LED
}

// Set LoRaWAN parameters
void setupLoRaWAN()
{
  pinMode(STATUS_LED, OUTPUT);
  // Update Serial Monitor
  delay(1000);
  // Start the UART for the LoRaWAN Bee
  LoRaSerial.begin(115200, SERIAL_8N1, RXD2, TXD2);
  // Set the event handler
  lorawan.event_listener = &event_handler;
  Serial.println(F("Handler set"));
  
  // Set the Device EUI
  response = lorawan.set_DevEUI(DEVEUI);
  if(response == CommandResponse::OK){
    Serial.print(F("DevEUI: "));
    Serial.write((uint8_t *)DEVEUI, 16);
    Serial.println();
  } else {
    Serial.println(F("Error Setting the Device EUI"));
  }

  // set the Application EUI
  response = lorawan.set_AppEUI(APPEUI);
  if(response == CommandResponse::OK){
    Serial.print(F("Application EUI set ("));
    Serial.write((uint8_t *)APPEUI, 16);
    Serial.println(')');
  } else {
    Serial.println(F("Error setting the Application EUI"));
  }

  // Set the Device Address
  response = lorawan.set_DevAddr(DEVADDR);
  if(response == CommandResponse::OK){
    Serial.print(F("Device Address set ("));
    Serial.write((uint8_t *)DEVADDR, 8);
    Serial.println(')');
  } else {
    Serial.println(F("Error setting the Device Address"));
  }

  // set the Application Session Key
  response = lorawan.set_AppSKey(APPSKEY);
  if(response == CommandResponse::OK){
    Serial.print(F("Application Session Key set ("));
    Serial.write((uint8_t *)APPSKEY, 32);
    Serial.println(')');
  } else {
    Serial.println(F("Error setting the Application Session Key"));
  }

  // set the Network Session Key
  response = lorawan.set_NwkSKey(NWKSKEY);
  if(response == CommandResponse::OK){
    Serial.print(F("Network Session Key set ("));
    Serial.write((uint8_t *)NWKSKEY, 32);
    Serial.println(')');
  } else {
    Serial.println(F("Error setting the Network Session Key"));
  }

  // set the LoRaMAC Region
  response = lorawan.set_Region(REGION_AU915);
  Serial.print(F("LoRaMAC Region set ("));
  Serial.write(REGION_AU915);
  Serial.println(')');

  // set the Auto Data Rate (ADR) Configuration
  response = lorawan.set_ADR(SMW_SX1276M0_ADR_ON);
  Serial.print(F("Auto Data Rate ("));
  Serial.write(SMW_SX1276M0_ADR_ON);
  Serial.println(')');


  // Set join mode to ABP
  response = lorawan.set_JoinMode(SMW_SX1276M0_JOIN_MODE_ABP);
  if(response == CommandResponse::OK){
    Serial.println(F("Mode set to ABP"));
  } else {
    Serial.println(F("Error setting the join mode"));
  }
  // Join the network (not really necessary in ABP)
  Serial.println(F("Joining the network"));
  lorawan.join();
}

void setup() {
  Serial.begin(115200);
  /*=============================== SENSORS ===============================*/
  dht.begin();
  bmp.begin(I2C_ADRESS);
  pinMode(UV_PIN, INPUT);
  pinMode (RAIN_PIN, INPUT_PULLUP); //This activates the internal pull up resistor
  pinMode(ANEMOMETER_PIN, INPUT);
  /*=============================== LORAWAN ===============================*/
  setupLoRaWAN();
}

void loop() {
  lorawan.listen();
  // Send a message
  if(lorawan.isConnected()){
    WE_Package = readData();
    sendData(WE_Package);
  }
}

void event_handler(Event type){
  // check if join event
  if(type == Event::JOINED){
    Serial.println(F("Joined"));
  }
}
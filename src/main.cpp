// Copyright (c) Microsoft. All rights reserved.
// Licensed under the MIT license.
#include <Arduino.h>          //Arduino original
#include <math.h>             //Round values
#include <EEPROM.h>           //Save in memory
#include <WiFi.h>             // ESP32
#include "AzureIotHub.h"      // AZURE IoT Hub
#include "Esp32MQTTClient.h"  // AZURE IoT Hub
#include <Wire.h>             //BME280 & OLED & SGP30
#include <Adafruit_GFX.h>     //OLED
#include <Adafruit_SSD1306.h> //OLED
#include "time.h"             // Time
#include "utilities.h"
#include "sensors.h"
#include "credentials.h"

//const bool TESTING = true;
const bool TESTING = false;

// Definitions
#define EEPROM_SIZE 32
#define LED LED_BUILTIN      //Built-in Led
#define OLED__SDA OLED_SDA   //OLED
#define OLED__SCL OLED_SCL   //OLED
#define OLED__RST OLED_RST   //OLED
#define SCREEN_WIDTH 128     //OLED
#define SCREEN_HEIGHT 64     //OLED
#define INTERVAL 60000       // 1 min between messages
#define INTERVALWARMUP 60000 // 1 min of warn up
#define MESSAGE_MAX_LEN 512  // Message send to IoT Hub size

// Please input the SSID and password of WiFi
const char *ssid = mySSID;
const char *password = myPASSWROD;

/*String containing Hostname, Device Id & Device Key in the format:                         */
static const char *connectionString = myIoTHub;
String DEVICE_ID = WiFi.macAddress();
const char *messageData = "{\"deviceid\":\"%s\",\"time\":%d, \"temperature\":%.2f, \"humidity\":%.2f, \"pressure\":%.2f, \"altitude\":%.2f, \"co2\":%d, \"rawco2\":%d, \"pm25\":%.2f, \"pm10\":%.2f, \"tvoc\":%d,\"eco2\":%d,\"rawh2\":%d,\"rawethanol\":%d,\"lux\":%.2f,\"whitelux\":%.2f,\"rawals\":%.2f}";
int messageCount = 1;
static bool hasWifi = false;
static bool messageSending = true;
static uint64_t send_interval_ms;
static uint64_t warm_up;
static bool firstMessage = true;
static bool mandatoryWarmUp = true;

// Instances
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED__RST); //OLED

//Variables
float *ptr_bme280_values;                      // BME280
int *ptr_co2;                                  //MHZ19
float *ptr_pm;                                 //SDS011
int *ptr_sgp30;                                //SGP30
float *ptr_veml7700;                           //VEML7700
extern float vBattery;                         //Battery
const char *ntpServer = "europe.pool.ntp.org"; // Time
const long gmtOffset_sec = 0;                  // Time
const int daylightOffset_sec = 0;              // Time

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Utilities
static void InitWifi()
{
  Serial.println("Connecting...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  hasWifi = true;
  Serial.println("WiFi connected");
  Serial.print("WiFi IP address: ");
  Serial.println(WiFi.localIP());
}

// Time
void printLocalTime()
{
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo))
  {
    Serial.println("ERROR Time: Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "\n %A, %B %d %Y %H:%M:%S");
}
// Time
unsigned long getTime()
{
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo))
  {
    Serial.println("ERROR Time: Failed to obtain time");
    return (0);
  }
  time(&now);
  return now;
}

// IoT Hub
static void SendConfirmationCallback(IOTHUB_CLIENT_CONFIRMATION_RESULT result)
{
  if (result == IOTHUB_CLIENT_CONFIRMATION_OK)
  {
    Serial.println("Send Confirmation Callback finished.");
  }
}

static void MessageCallback(const char *payLoad, int size)
{
  Serial.println("Message callback:");
  Serial.println(payLoad);
}

static void DeviceTwinCallback(DEVICE_TWIN_UPDATE_STATE updateState, const unsigned char *payLoad, int size)
{
  char *temp = (char *)malloc(size + 1);
  if (temp == NULL)
  {
    return;
  }
  memcpy(temp, payLoad, size);
  temp[size] = '\0';
  // Display Twin message.
  Serial.println(temp);
  free(temp);
}

static int DeviceMethodCallback(const char *methodName, const unsigned char *payload, int size, unsigned char **response, int *response_size)
{
  LogInfo("Try to invoke method %s", methodName);
  const char *responseMessage = "{\"foo\":\"bar\"}"; // "{\"topic\":\"iot\"}" "{'TFM':'TFM'}" R"({"foo": "bar"})"
  int result = 200;
  Serial.print("Payload: ");
  Serial.println((char)payload[0]);

  if (strcmp(methodName, "start") == 0)
  {
    LogInfo("Start sending data");
    messageSending = true;
  }
  else if (strcmp(methodName, "stop") == 0)
  {
    LogInfo("Stop sending data");
    messageSending = false;
  }
  else if (strcmp(methodName, "buzzersound") == 0)
  {
    LogInfo("buzzerSound method");
    buzzerSound();
  }
  else if (strcmp(methodName, "stopbuzzer") == 0)
  {
    LogInfo("stopBuzzer method");
    stopBuzzer();
  }
  else if (strcmp(methodName, "setcolorrgb") == 0)
  {
    LogInfo("setColorRGB method");
    if ((char)payload[0] == '0')
    {
      LogInfo("Payload: 1");
      setColorRGB(1);
    }
    else if ((char)payload[0] == '1')
    {
      LogInfo("Payload: 2");
      setColorRGB(2);
    }
    else if ((char)payload[0] == '2')
    {
      LogInfo("Payload: 3");
      setColorRGB(3);
    }
    else
    {
      LogError("ERROR Payload doesn't implemented");
    }
  }
  else if (strcmp(methodName, "calibratesgp30") == 0)
  {
    LogInfo("calibrateSGP30 method");
    calibrateSGP30();
  }
  else if (strcmp(methodName, "autocalibrationsgp30") == 0)
  {
    LogInfo("autoCalibrationSGP30 method");
    if ((char)payload[0] == '0')
    {
      LogInfo("Autocalibration SGP30: False");
      autoCalibrationSGP30(false);
    }
    else if ((char)payload[0] == '1')
    {
      LogInfo("Autocalibration SGP30: True");
      autoCalibrationSGP30(true);
    }
    else
    {
      LogError("ERROR Payload doesn't implemented");
    }
  }
  else if (strcmp(methodName, "calibratemhz19") == 0)
  {
    LogInfo("calibrateMHZ19 method");
    calibrateMHZ19();
  }
  else if (strcmp(methodName, "modifyautocalibrationmhz19") == 0)
  {
    LogInfo("modifyAutoCalibrationMHZ19 method");
    if ((char)payload[0] == '0')
    {
      LogInfo("Autocalibration MHZ19: False");
      modifyAutoCalibrationMHZ19(false);
    }
    else if ((char)payload[0] == '1')
    {
      LogInfo("Autocalibration MHZ19: True");
      modifyAutoCalibrationMHZ19(true);
    }
    else
    {
      LogError("ERROR Payload doesn't implemented");
    }
  }
  else
  {
    LogInfo("No method %s found", methodName);
    responseMessage = "{}";
    result = 404;
  }

  *response_size = strlen(responseMessage) + 1;
  *response = (unsigned char *)strdup(responseMessage);

  return result;
}

void printDisplay(float temperature, float humidity, float pressure, int co2, float pm25, float pm10, float vBattery)
{
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("AIRQ PROJECT");
  display.setTextSize(1);
  display.print("Temperature:\t");     //BME280
  display.println(temperature);        //BME280
  display.print("Humidity:\t");        //BME280
  display.println(humidity);           //BME280
  display.print("Pressure:\t");        //BME280
  display.println(pressure);           //BME280
  display.print("CO2:\t");             //MHZ19
  display.println(co2);                //MHZ19
  display.print("PM25:\t");            //SDS011
  display.println(pm25);               //SDS011
  display.print("PM10:\t");            //SDS011
  display.println(pm10);               //SDS011
  display.print("Battery voltage:\t"); //Battery
  display.println(vBattery);
  display.display();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Arduino sketch
void setup()
{
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);
  Serial.println("Device: ESP32 Device");
  Serial.println("Device: Initializing...");

  // Initialize the WiFi module
  Serial.println("Device: > WiFi");
  hasWifi = false;
  InitWifi();
  if (!hasWifi)
  {
    return;
  }
  Serial.print("WIFI MAC: ");
  Serial.println(DEVICE_ID);
  randomSeed(analogRead(0));

  Serial.println("Device: > IoT Hub");
  Esp32MQTTClient_SetOption(OPTION_MINI_SOLUTION_NAME, "AirQ"); //GetStarted
  Esp32MQTTClient_Init((const uint8_t *)connectionString, true);

  Esp32MQTTClient_SetSendConfirmationCallback(SendConfirmationCallback);
  Esp32MQTTClient_SetMessageCallback(MessageCallback);
  Esp32MQTTClient_SetDeviceTwinCallback(DeviceTwinCallback);
  Esp32MQTTClient_SetDeviceMethodCallback(DeviceMethodCallback);

  send_interval_ms = millis();
  warm_up = millis();

  //Built-in Led
  pinMode(LED, OUTPUT);
  digitalWrite(LED, 1);

  // OLED
  pinMode(OLED__RST, OUTPUT);
  digitalWrite(OLED__RST, LOW);
  vTaskDelay(20);
  digitalWrite(OLED__RST, HIGH);

  //initialize OLED
  Wire.begin(OLED__SDA, OLED__SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false))
  { // Address 0x3C for 128x32
    Serial.println(F("ERROR SSD1306: allocation failed"));
  }

  sensors_init();
  utilities_init();

  if (!TESTING)
  {
    while ((int)(millis() - warm_up) < INTERVALWARMUP && mandatoryWarmUp)
    {
      Serial.println(F("Device: Warming  up sensors"));
      display.clearDisplay();
      display.setTextColor(WHITE);
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.println("AIRQ Project");
      display.println("Warming up sensors");
      display.display();
      vTaskDelay(10000);
    }
  }

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer); // Time
  printLocalTime();                                         // Time

  Serial.println(F("Device: End of Setup"));
}

void loop()
{
  if (hasWifi)
  {
    if (messageSending &&
        (int)(millis() - send_interval_ms) >= INTERVAL)
    {
      Serial.println(F("----------------------------"));
      digitalWrite(LED, 1); //Built-in Led

      printLocalTime(); // Time

      ptr_sgp30 = getSGP30();                //SGP30
      ptr_bme280_values = getBme280Values(true); //BME280
      ptr_pm = getPM();                      //SDS011
      ptr_co2 = getCO2();                    //MHZ19
      ptr_veml7700 = getVEML7700();          //VEML7700

      // Modifications
      char messagePayload[MESSAGE_MAX_LEN];

      vBattery = roundf(getVoltageBatteryValue() * 100) / 100; //Battery

      printDisplay(ptr_bme280_values[0], ptr_bme280_values[1], ptr_bme280_values[2], ptr_co2[0], ptr_pm[0], ptr_pm[1], vBattery);
      snprintf(messagePayload, MESSAGE_MAX_LEN, messageData, DEVICE_ID.c_str(), getTime(), ptr_bme280_values[1], ptr_bme280_values[2], ptr_bme280_values[3], ptr_bme280_values[4], ptr_co2[0], ptr_co2[1], ptr_pm[0], ptr_pm[1], ptr_sgp30[0], ptr_sgp30[1], ptr_sgp30[2], ptr_sgp30[3], ptr_veml7700[1], ptr_veml7700[2], ptr_veml7700[3]);
      Serial.println(messagePayload);
      if (!firstMessage)
      {
        EVENT_INSTANCE *message = Esp32MQTTClient_Event_Generate(messagePayload, MESSAGE);
        Esp32MQTTClient_Event_AddProp(message, "temperatureAlert", "true");
        Esp32MQTTClient_SendEventInstance(message);
      }
      else
      {
        Serial.println(F("Device: First message ignored"));
      }
      send_interval_ms = millis();
      firstMessage = false;
      digitalWrite(LED, 0); //Built-in Led
    }
    else
    {
      Esp32MQTTClient_Check();
    }
  }
  else
  {
    Serial.println(F("ERROR WIFI: ERROR"));
  }

  vTaskDelay(10);
}
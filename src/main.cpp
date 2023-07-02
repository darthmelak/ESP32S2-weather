#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <Adafruit_NeoPixel.h>
#include <SHT2x.h>
#include <Wire.h>
#include <BME280I2C.h>
#include <SPI.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <AsyncHTTPRequest_Generic.h>
#include <PubSubClient.h>
#include "secrets.h"

#define PIN_SDA 1
#define PIN_SCL 2
#define PIN_PWR_SENSE 7
#define PIN_BAT_SENSE 8

#define PWR_READ_DELAY 5
#define PWR_READ_COUNT 4
#define BRGTH 32

#define uS_TO_S_FACTOR 1000000

#ifndef secrets_h
#define SSID "SSID"
#define PASSWORD "password"
#define MQTT_SERVER "192.168.1.1"
#define MQTT_USER "user"
#define MQTT_PASS "password"
#define POST_URL "http://example.com/api/states/sensor."
#define POST_AUTH "Bearer TOKEN"
#endif

#define POSTFIX_TEMP "temperature"
#define POSTFIX_HUMI "humidity"
#define POSTFIX_PRES "pressure"
#define POSTFIX_POWR "power"
#define POSTFIX_BATT "battery"
#define POSTFIX_BPER "battery-percent"

#define MODE_CONFIG 0b00
#define MODE_MQTT 0b01
#define MODE_POST 0b10

#define C_NAME "name"
#define C_HOSTNAME "hostname"
#define C_MODE "mode"
#define C_SLEEP "sleep_sec"
#define C_RUN "run_sec"
#define C_SENSOR "sensor_type"

#define DEFAULT_NAME "Temp/Humidity"
#define DEFAULT_HOSTNAME "esp-weather"
#define DEFAULT_SLEEP 58
#define DEFAULT_RUN 2
#define DEFAULT_SENSOR SHT

enum SensorType { SHT, BME };
static const char *sensorTypeStr[] = { "sht", "bme" };

const String mqttPrefix = "homeassistant/sensor/";

BME280I2C::Settings settings(
  BME280::OSR_X1,
  BME280::OSR_X1,
  BME280::OSR_X1,
  BME280::Mode_Forced,
  BME280::StandbyTime_1000ms,
  BME280::Filter_Off,
  BME280::SpiEnable_False,
  BME280I2C::I2CAddr_0x76 // I2C address. I2C specific.
);
Preferences prefs;
Adafruit_NeoPixel pixels(1, 18, NEO_GRB + NEO_KHZ800);
SHT2x sensorSHT2x;
BME280I2C sensorBME(settings);

WiFiClient wifiClient;
WebServer server;
AsyncHTTPRequest request;
PubSubClient mqtt(MQTT_SERVER, 1883, wifiClient);

String name = DEFAULT_NAME;
String hostname = DEFAULT_HOSTNAME;
unsigned short mode = MODE_CONFIG;
unsigned int sleepSeconds = DEFAULT_SLEEP;
unsigned int runTime = DEFAULT_RUN * 1000;
SensorType sensorInUse = DEFAULT_SENSOR;
String sensorId = "";

float temperature = 0;
float humidity = 0;
float pressure = 0;
float power = 0;
float battery = 0;
int batteryPercent = 0;
unsigned long lastRead = 0;
short R = 0; short G = 0; short B = 0;

RTC_DATA_ATTR int bootCount = 0;
unsigned long lastActivity = 0;

void setupPrefs();
void setupSensors();
void connectWifi();
bool connectMQTT();
void setupOTA();
void setupHttp();
void setupMqtt();
void mqttCb(char* topic, byte* payload, unsigned int length);
void httpGetConfig();
void httpGetRoot();
void httpPostConfig();
void handleSerial();
void updateLED();
void readAll();
long readAverage(int pin);
void postData();
void publishMQTT();
void publishWDiscovery(String subtopic, String deviceClass, String unit, double value);
long getBattery();
int getBatteryPercent(long mapped);
long getPower();
void respondJson(int code, const JsonDocument& json);
bool updateConfigFromJson(String jsonStr);

void setup() {
  Serial.begin(115200);
  delay(10);
  Serial.println("\n\nInit ...");

  setupPrefs();
  setupSensors();
  connectWifi();
  setupOTA();
  setupHttp();
  mqtt.setBufferSize(512);

  bootCount++;
  Serial.println("Boot count: " + String(bootCount));
  esp_sleep_enable_timer_wakeup(sleepSeconds * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for " + String(sleepSeconds) + " seconds.");
  Serial.println("Running in mode: " + String(mode));

  lastActivity = millis();
  readAll();
  if (mode & MODE_POST)
  {
    G = BRGTH;
    updateLED();
    postData();
  }
  if (connectMQTT())
  {
    if (mode & MODE_MQTT)
    {
      B = BRGTH;
      updateLED();
      publishMQTT();
      B = 0;
      updateLED();
    }
  }
}

void loop() {
  ArduinoOTA.handle();
  handleSerial();
  server.handleClient();
  mqtt.loop();
  delay(10);

  unsigned int elapsed = millis() - lastActivity;
  if (mode != MODE_CONFIG)
  {
    if (elapsed > runTime)
    {
      R = G = B = 0;
      updateLED();
      Serial.println("Idle for "+ String(runTime) +" seconds, going to sleep.");
      esp_deep_sleep_start();
    }
    if (elapsed > runTime - 200)
    {
      R = BRGTH;
      updateLED();
    }
  }
  else
  {
    unsigned int remainder = elapsed % 1000;
    if (remainder > 300 && remainder < 320) {
      R = BRGTH;
      updateLED();
    }
    if (remainder > 550 && remainder < 570) {
      R = 0;
      updateLED();
    }
  }
}

void postData() {
  static bool requestOpenResult;

  if (request.readyState() == readyStateUnsent || request.readyState() == readyStateDone)
  {
    StaticJsonDocument<256> json;
    String payload;

    json["temperature"] = temperature;
    json["humidity"] = humidity;
    json["pressure"] = pressure;
    json["power"] = power;
    json["battery"] = battery;
    json["battery_percent"] = batteryPercent;

    serializeJson(json, payload);

    requestOpenResult = request.open("POST", POST_URL);
    if (requestOpenResult)
    {
      request.setReqHeader("Authorization", POST_AUTH);
      request.setReqHeader("Content-Type", "application/json");
      request.setReqHeader("Content-length", payload.length());
      request.send(payload);
    }
    else
    {
      Serial.println("Can't send bad request");
    }
  }
  else
  {
    Serial.println("Can't send request");
  }
}

void httpGetRoot()
{
  Serial.println("Request: GET /");
  StaticJsonDocument<256> json;
  readAll();
  json["temperature"] = temperature;
  json["humidity"] = humidity;
  json["pressure"] = pressure;
  json["lastread"] = millis() - lastRead;
  json["power"] = power;
  json["battery"] = battery;
  json["battery_percent"] = batteryPercent;
  respondJson(200, json);
}

void httpGetConfig()
{
  Serial.println("Request: GET /config");
  StaticJsonDocument<256> res;
  res[C_NAME] = name;
  res[C_HOSTNAME] = hostname;
  res[C_MODE] = mode;
  res[C_SLEEP] = sleepSeconds;
  res[C_RUN] = runTime / 1000;
  res[C_SENSOR] = sensorTypeStr[sensorInUse];

  respondJson(200, res);
}

void httpPostConfig()
{
  if (server.hasArg("plain") == false) return;
  Serial.println("Request: POST /config");

  String body = server.arg("plain");
  Serial.print("POST: ");
  Serial.println(body);
  
  bool changed = updateConfigFromJson(body);
  httpGetConfig();
  if (changed)
  {
    delay(250);
    ESP.restart();
  }
}

void setupHttp()
{
  // setup server
  Serial.println("Starting Server.");
  server.on("/", HTTP_GET, httpGetRoot);
  server.on("/config", HTTP_GET, httpGetConfig);
  server.on("/config", HTTP_POST, httpPostConfig);
  server.begin();

  // setup request (client)
  request.setTimeout(1);
  request.onReadyStateChange([](void* optParm, AsyncHTTPRequest* request, int readyState) {
    if (readyState == readyStateDone) 
    {
      Serial.print("Response: ");
      Serial.println(request->responseText());
      G = 0;
      updateLED();
    }
  });
}

bool updateConfigFromJson(String jsonStr)
{
  bool changed = false;
  StaticJsonDocument<384> json;
  deserializeJson(json, jsonStr);
  Serial.println("Updating config from json...");

  if (json.containsKey(C_HOSTNAME))
  {
    String tmp = json[C_HOSTNAME];
    if (!hostname.equals(tmp))
    {
      prefs.putString(C_HOSTNAME, tmp);
      hostname = tmp;
      changed = true;
      Serial.println("Updated hostname:" + tmp);
    }
  }

  if (json.containsKey(C_NAME))
  {
    String tmp = json[C_NAME];
    if (!name.equals(tmp))
    {
      prefs.putString(C_NAME, tmp);
      name = tmp;
      changed = true;
      Serial.println("Updated name:" + tmp);
    }
  }

  if (json.containsKey(C_MODE))
  {
    unsigned short tmp = json[C_MODE];
    if (mode != tmp)
    {
      prefs.putUShort(C_MODE, tmp);
      mode = tmp;
      changed = true;
      Serial.println("Updated mode:" + String(tmp));
    }
  }

  if (json.containsKey(C_SLEEP))
  {
    unsigned int tmp = json[C_SLEEP];
    if (sleepSeconds != tmp)
    {
      prefs.putUInt(C_SLEEP, tmp);
      sleepSeconds = tmp;
      changed = true;
      Serial.println("Updated sleepSeconds:" + String(tmp));
    }
  }

  if (json.containsKey(C_RUN))
  {
    unsigned int tmp = json[C_RUN];
    if (runTime != tmp * 1000)
    {
      prefs.putUInt(C_RUN, tmp);
      runTime = tmp * 1000;
      changed = true;
      Serial.println("Updated runTime:" + String(runTime));
    } 
  }

  if (json.containsKey(C_SENSOR))
  {
    String tmp = json[C_SENSOR];
    if (String(sensorTypeStr[sensorInUse]).compareTo(tmp) != 0)
    {
      sensorInUse = tmp.equals(sensorTypeStr[SHT]) ? SHT : BME;
      prefs.putUShort(C_SENSOR, sensorInUse);
      changed = true;
      Serial.println("Updated sensorInUse:" + String(sensorTypeStr[sensorInUse]));
    }
  }

  return changed;
}

void handleSerial()
{
  if (Serial.available())
  {
    String input = Serial.readStringUntil('\n');
    if (input.startsWith("s")) // get sensor values
    {
      readAll();
      Serial.print("Temperature: ");
      Serial.println(temperature);
      Serial.print("Humidity: ");
      Serial.println(humidity);
      Serial.print("Pressure: ");
      Serial.println(pressure);
      Serial.print("Last read: ");
      Serial.println(millis() - lastRead);
    }
    if (input.startsWith("p")) // get power values
    {
      readAll();
      Serial.print("Power sense: ");
      Serial.println(power);
      Serial.print("Battery sense: ");
      Serial.println(battery);
      Serial.print("Battery percent: ");
      Serial.println(batteryPercent);
    }
    if (input.startsWith("r")) // raw millivolt reading
    {
      Serial.print("Power raw: ");
      Serial.println(analogReadMilliVolts(PIN_PWR_SENSE));
      Serial.print("Battery raw: ");
      Serial.println(analogReadMilliVolts(PIN_BAT_SENSE));
    }
    // if (input.startsWith("q")) // resistor math vs. map
    // {
    //   long pwr = readAverage(PIN_PWR_SENSE);
    //   long power = pwr * (10000 + 1000) / 1000;
    //   long batt = readAverage(PIN_BAT_SENSE);
    //   long battery = batt * (4700000 + 470000) / 470000;
    //   Serial.print("Power sense: ");
    //   Serial.print(getPower());
    //   Serial.print("\t");
    //   Serial.println(power / 1000.);
    //   Serial.print("Battery sense: ");
    //   Serial.print(getBattery());
    //   Serial.print("\t");
    //   Serial.println(battery / 1000.);
    // }
    if (input.startsWith("P")) // trigger POST request
    {
      Serial.println("POSTing data...");
      postData();
    }
  }
}

void connectWifi()
{
  Serial.printf("Connecting to WiFi: %s ", SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASSWORD);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500); 
    Serial.print(".");
  }
  Serial.print("\nConnected, IP: ");
  Serial.println(WiFi.localIP());
}

void setupSensors()
{
  Serial.println("Setting up sensor / powersense pins / RGB pin");
  pinMode(7, INPUT); // Setup power sense
  analogSetPinAttenuation(7, ADC_0db);
  pinMode(8, INPUT); // Setup battery sense
  analogSetPinAttenuation(8, ADC_0db);
  pixels.begin();

  Serial.println("Using sensor:" + String((sensorInUse == SHT) ? "SHT2x" : "BME280"));
  if (sensorInUse == SHT)
  {
    sensorSHT2x.begin(PIN_SDA, PIN_SCL); // Setup sensor (I2C)
    uint8_t stat = sensorSHT2x.getStatus();
    Serial.print("Sensor status: ");
    Serial.print(stat, HEX);
    Serial.println();
  }
  else
  {
    Wire.begin(PIN_SDA, PIN_SCL);
    sensorBME.begin();
    switch (sensorBME.chipModel())
    {
      case BME280::ChipModel_BME280:
        Serial.println("Found BME280 sensor! Success.");
        break;
      case BME280::ChipModel_BMP280:
        Serial.println("Found BMP280 sensor! No Humidity available.");
        break;
      default:
        Serial.println("Found UNKNOWN sensor! Error!");
    }
  }
}

void readAll()
{
  if (sensorInUse == SHT)
  {
    sensorSHT2x.read();
    temperature = sensorSHT2x.getTemperature();
    humidity = sensorSHT2x.getHumidity();
  }
  else
  {
    sensorBME.read(pressure, temperature, humidity);
  }
  power = getPower() / 1000.;
  long batt = getBattery();
  battery = batt / 1000.;
  batteryPercent = getBatteryPercent(batt);
  lastRead = millis();
  Serial.println("Sensor readings updated.");
}

void setupPrefs()
{
  Serial.println("Loading preferences");
  prefs.begin("settings");

  mode = prefs.getUShort(C_MODE, MODE_CONFIG);
  sleepSeconds = prefs.getUInt(C_SLEEP, DEFAULT_SLEEP);
  runTime = prefs.getUInt(C_RUN, DEFAULT_RUN) * 1000;
  hostname = prefs.getString(C_HOSTNAME, DEFAULT_HOSTNAME);
  name = prefs.getString(C_NAME, DEFAULT_NAME);
  sensorInUse = SensorType(prefs.getUShort(C_SENSOR, DEFAULT_SENSOR));
}

void setupOTA()
{
  char tmp[64]; // Setup mqtt id with hostname-mac
  uint8_t mac[6];
  WiFi.macAddress(mac);
  sprintf(tmp, "%s-%02x%02x%02x%02x%02x%02x", hostname.c_str(), mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  sensorId = tmp;
  Serial.println("sensorId set to:" + sensorId);

  Serial.println("Starting OTA Server");
  ArduinoOTA.setHostname(hostname.c_str());
  Serial.println("mDNS set to: " + hostname);
  ArduinoOTA.begin();
}

bool connectMQTT()
{
  Serial.print("Attempting MQTT connection...");
  mqtt.setCallback(mqttCb);
  if (mqtt.connect(sensorId.c_str(), MQTT_USER, MQTT_PASS))
  {
    Serial.println("connected");
    String configTopic = mqttPrefix + sensorId;
    // subscribe here if needed
    mqtt.subscribe(configTopic.c_str());
    return true;
  }
  else
  {
    Serial.print("failed, rc=");
    Serial.println(mqtt.state());
    return false;
  }
}

void publishMQTT()
{
  publishWDiscovery(POSTFIX_TEMP, "temperature", "°C", temperature);
  publishWDiscovery(POSTFIX_HUMI, "humidity", "%", humidity);
  if (sensorInUse == BME)
  {
    publishWDiscovery(POSTFIX_PRES, "pressure", "hPa", pressure);
  }
  publishWDiscovery(POSTFIX_BATT, "voltage", "V", battery);
  publishWDiscovery(POSTFIX_POWR, "voltage", "V", power);
  publishWDiscovery(POSTFIX_BPER, "battery", "%", batteryPercent);
}

void publishWDiscovery(String subtopic, String deviceClass, String unit, double value)
{
  String subId = sensorId + "_" + subtopic;
  String configTopic = mqttPrefix + subId + "/config";
  String stateTopic = mqttPrefix + subId + "/state";

  StaticJsonDocument<512> json;
  json["dev"]["identifiers"] = sensorId;
  json["dev"]["model"] = "ESP32-S2";
  json["dev"]["name"] = "ESP32 Temp/Humidity Unit";
  json["dev_cla"] = deviceClass;
  json["name"] = name + " " + subtopic;
  json["stat_cla"] = "measurement";
  json["stat_t"] = stateTopic;
  json["uniq_id"] = subId;
  json["unit_of_meas"] = unit;

  String jsonStr;
  serializeJson(json, jsonStr);
  mqtt.publish(configTopic.c_str(), jsonStr.c_str(), true);
  mqtt.publish(stateTopic.c_str(), String(value).c_str(), true);
}

void mqttCb(char* topic, byte* payload, unsigned int length)
{
  payload[length] = '\0';
  String data = String((char*) payload);
  Serial.println("Got config from MQTT: " + data);
  bool changed = updateConfigFromJson(data);
  if (changed) {
    delay(50);
    ESP.restart();
  }
}

long getPower()
{
  long val = readAverage(PIN_PWR_SENSE);
  long mapped = map(val, 32, 425, 330, 4710);
  return mapped;
}

long getBattery()
{
  long val = readAverage(PIN_BAT_SENSE);
  long mapped = map(val, 215, 375, 2160, 4200);
  return mapped;
}

int getBatteryPercent(long mapped)
{
  int percent = map(mapped, 3600, 4200, 0, 100);
  if (percent < 0) percent = 0;
  if (percent > 100) percent = 100;
  return percent;
}

long readAverage(int pin)
{
  long sum = 0;
  for (short i = 0; i < PWR_READ_COUNT; i++)
  {
    sum += analogReadMilliVolts(pin);
    if (i < PWR_READ_COUNT - 1) {
      delay(PWR_READ_DELAY);
    }
  }
  return sum / PWR_READ_COUNT;
}

void respondJson(int code, const JsonDocument& json)
{
  String jsonStr;
  serializeJson(json, jsonStr);
  server.send(code, "application/json", jsonStr);
}

void updateLED()
{
  pixels.setPixelColor(0, pixels.Color(R, G, B));
  pixels.show();
}

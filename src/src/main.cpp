#include <Arduino.h>
#include <RunningMedian.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include <WEMOS_DHT12.h>
#include <LOLIN_HP303B.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <credentials.h>
#include <azure.h>

//declare functions
String getCapacitance();
String getPressure();
void readSensors();
void setupMQTTClient();
void reconnect();
void debugger(String msg);
void callback(char *topic, byte *payload, unsigned int length);
void setupWifi();
void setupOTA();

//set up the pins
#define SCL_PIN 5
#define SDA_PIN 4
#define SOIL_PIN A0

//declare objects & variables
WiFiClient client;
PubSubClient MQTTClient;

DHT12 dht12;
LOLIN_HP303B HP303BPressureSensor;
int ledState = LOW;

//debug
const bool DEBUG = true;
const bool MQTTDEBUG = true;

void setup()
{
  unsigned long startSetupTime = millis();

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(9600);

  setupWifi();
  setupOTA();
  setupMQTTClient();

  //Call begin to initialize HP303BPressureSensor
  HP303BPressureSensor.begin();

  unsigned long endSetupTime = millis();

  String SSID = WIFI_SSID;
  String mDNS = MDNS_HOSTNAME;
  String MQTTSVR = MQTT_SERVER;
  MQTTClient.publish(MQTT_TOPIC, ("Connected to SSID: " + SSID).c_str());
  MQTTClient.publish(MQTT_TOPIC, ("IP address: " + WiFi.localIP().toString()).c_str());
  MQTTClient.publish(MQTT_TOPIC, ("mDNS: " + mDNS).c_str());
  MQTTClient.publish(MQTT_TOPIC, ("Connected to MQTT server: " + MQTTSVR).c_str());
  MQTTClient.publish(MQTT_TOPIC, ("Setup in " + String(endSetupTime - startSetupTime) + "ms").c_str());

  randomSeed(analogRead(0));
}

void setupWifi()
{
  //sort out WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD); // Connect to the network

  while (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  Serial.println("Ready on the local network");
  Serial.println("IP address: " + WiFi.localIP().toString());
}

void setupOTA()
{
  ArduinoOTA.setHostname(MDNS_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);

  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR)
      Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR)
      Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR)
      Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR)
      Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR)
      Serial.println("End Failed");
  });
  ArduinoOTA.begin();
}

void toggleLED()
{
  ledState = !ledState;
  digitalWrite(LED_BUILTIN, ledState);
}

unsigned long sensorInterval = 20000;
unsigned long sensorPreviousMillis = 0;
unsigned long pingInterval = 500;
unsigned long pingPreviousMillis = 0;

void loop()
{
  ArduinoOTA.handle();

  //MQTT section
  if (!MQTTClient.connected())
  {
    reconnect();
  }
  MQTTClient.loop();

  delay(50); // <- fixes some issues with WiFi stability

  unsigned long currentMillis = millis();

  if (currentMillis - sensorPreviousMillis > sensorInterval)
  {
    //check wifi connection
    if (WiFi.status() != WL_CONNECTED)
    {
      Serial.println("Connection Failed! Rebooting...");
      delay(5000);
      ESP.restart();
    }

    sensorPreviousMillis = currentMillis;

    MQTTClient.publish(MQTT_TOPIC, "Read sensors");

    readSensors();
  }

  currentMillis = millis();

  if (currentMillis - pingPreviousMillis > pingInterval)
  {
    pingPreviousMillis = currentMillis;

    // toggle the LED
    toggleLED();

    //makes the MQTT looks cool on the Shiftr UI
    MQTTClient.publish(MQTT_PING_TOPIC, ".");
  }


}

//Preceding a function declaration with the inline specifier informs the compiler that inline expansion is preferred over the usual function call mechanism for a specific function
void debugger(String msg)
{
  if (DEBUG == true)
  {
    Serial.print(msg + "\n");

    if (MQTTDEBUG == true)
    {
      MQTTClient.publish(MQTT_TOPIC, msg.c_str());
    }
  }
}

void readSensors()
{
  unsigned long startLoopTime = millis();

  //Capacitance
  String Capacitance = getCapacitance();

  // toggle the LED
  toggleLED();

  //temperature and humidity
  String Temperature = "N/A";
  String Humidity = "N/A";

  if (dht12.get() == 0)
  {
    Temperature = String(dht12.cTemp);
    Humidity = String(dht12.humidity);
  }

  debugger("Temperature: " + Temperature);
  debugger("Humidity: " + Humidity);

  // toggle the LED
  toggleLED();

  //pressure
  String Pressure = getPressure();

  MQTTClient.publish(MQTT_TEMPERATURE_TOPIC, Temperature.c_str());
  MQTTClient.publish(MQTT_CAPACITANCE_TOPIC, Capacitance.c_str());
  MQTTClient.publish(MQTT_HUMIDITY_TOPIC, Humidity.c_str());
  MQTTClient.publish(MQTT_PRESSURE_TOPIC, Pressure.c_str());

  // toggle the LED
  toggleLED();

  sendToAzure(Temperature, Humidity, Pressure, Capacitance);

  // toggle the LED
  toggleLED();

  unsigned long endLoopTime = millis();

  debugger("All sensor readings took : " + String(endLoopTime - startLoopTime) + "ms");
}

String getPressure()
{
  unsigned long startLoopTime = millis();

  // toggle the LED
  toggleLED();

  int32_t Pascals;
  int16_t oversampling = 7;
  int16_t ret;
  String Pressure = "N/A";

  ret = HP303BPressureSensor.measurePressureOnce(Pascals, oversampling);

  if (ret != 0)
  {
    //Something went wrong, look at the library code for more information about return codes
    debugger("Failed getting pressure: " + String(ret));
  }
  else
  {
    Pressure = String(Pascals);

    unsigned long endLoopTime = millis();

    debugger("Pressure Sensor reading of " + Pressure + " Pascals, took : " + String(endLoopTime - startLoopTime) + "ms");
  }

  // toggle the LED
  toggleLED();

  return Pressure;
}

String getCapacitance()
{
  unsigned long startLoopTime = millis();

  // toggle the LED
  toggleLED();

  int NumberOfSamples = 100;

  RunningMedian samples = RunningMedian(NumberOfSamples);

  for (int i = 0; i <= NumberOfSamples; i++)
  {
    samples.add(analogRead(SOIL_PIN));

    delay(1);
  }

  String Capacitance = String(samples.getMedian());

  unsigned long endLoopTime = millis();

  debugger("Capacitance sensor reading of " + Capacitance + " took : " + String(endLoopTime - startLoopTime) + "ms");

  // toggle the LED
  toggleLED();

  return Capacitance;
}

void reconnect()
{
  // Loop until we're reconnected
  while (!MQTTClient.connected())
  {
    yield();

    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = MQTT_CLIENTID;

    // Attempt to connect
    if (MQTTClient.connect(clientId.c_str(), MQTT_USERNAME, MQTT_KEY))
    {
      Serial.println("connected");
      // Once connected, publish an announcement...
      MQTTClient.publish(MQTT_TOPIC, "Reconnected");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(MQTTClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setupMQTTClient()
{
  Serial.println("Connecting to MQTT server");

  MQTTClient.setClient(client);
  MQTTClient.setServer(MQTT_SERVER, 1883);

  // setup callbacks
  MQTTClient.setCallback(callback);

  Serial.println("connect mqtt...");

  String clientId = MQTT_CLIENTID;

  if (MQTTClient.connect(clientId.c_str(), MQTT_USERNAME, MQTT_KEY))
  {
    Serial.println("Connected");
    MQTTClient.publish(MQTT_TOPIC, "Connected to MQTT server");
  }
}

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  String message = "";

  for (int i = 0; i < length; i++)
  {
    message += (char)payload[i];
  }

  Serial.println(message);
}

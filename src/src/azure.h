#include <Arduino.h>
#include <WiFiClientSecure.h>

//declare functions
String toJSON(String Temperature, String Humidity, String Pressure, String Capacitance);
void debugger(String msg);

//declare objects
WiFiClientSecure Net;
unsigned long AzureTimeout = 10000;

void sendToAzure(String Temperature, String Humidity, String Pressure, String Capacitance)
{
  unsigned long startSetupTime = millis();

  debugger("Sending data To Azure");

  Net.setInsecure();

  if (!Net.connect(host, httpsPort))
  {
    debugger("connection failed: " + Net.getLastSSLError());
    return;
  }

  String data = toJSON(Temperature, Humidity, Pressure, Capacitance);

  debugger("Data POST sent to Azure");

  Net.print(String("POST ") + url + " HTTP/1.1\r\n" +
            "Host: " + host + "\r\n" +
            "Connection: close\r\n" +
            "Content-Length: " + data.length() + "\r\n" +
            "Content-Type: application/json; charset = UTF-8\r\n\r\n" +
            data + "\r\n");

  unsigned long startAzureTime = millis();
  String line = "";

  while (Net.connected())
  {
    yield();
    Net.setTimeout(AzureTimeout);

    line += Net.readStringUntil('\n');

    unsigned long Duration = millis() - startAzureTime;

    if (Duration > AzureTimeout)
    {
      ESP.restart();
    }
  }

  debugger("Response from Azure completed");

  Net.stop();

  unsigned long endSetupTime = millis();

  debugger("connection closed, total transmission time: " + String(endSetupTime - startSetupTime) + "ms");
}

String toJSON(String Temperature, String Humidity, String Pressure, String Capacitance)
{
  StaticJsonDocument<200> doc;
  doc["Temperature"] = Temperature;
  doc["Humidity"] = Humidity;
  doc["Capacitance"] = Capacitance;
  doc["Pressure"] = Pressure;

  serializeJsonPretty(doc, Serial);

  String retVal;

  serializeJsonPretty(doc, retVal);

  return retVal;
}

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "secrets.h"

#define DHTPIN 4
#define DHTTYPE DHT11
#define MQ135_PIN 34
#define LDR_PIN 35

#define RELAY_FAN 26
#define RELAY_MIST 23
#define RELAY_SOLENOID 25
#define RELAY_LIGHT 32

DHT dht(DHTPIN, DHTTYPE);
int ldrRaw = 0;

float temperature = 0.0, humidity = 0.0, mq135_raw = 0.0, lux = 0.0;

void setup()
{
  Serial.begin(115200);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" Connected");

  pinMode(LDR_PIN, INPUT);
  dht.begin();
  pinMode(MQ135_PIN, INPUT);

  // Relay pin setup
  pinMode(RELAY_FAN, OUTPUT);
  pinMode(RELAY_MIST, OUTPUT);
  pinMode(RELAY_SOLENOID, OUTPUT);
  pinMode(RELAY_LIGHT, OUTPUT);

  digitalWrite(RELAY_FAN, LOW);
  digitalWrite(RELAY_MIST, LOW);
  digitalWrite(RELAY_SOLENOID, LOW);
  digitalWrite(RELAY_LIGHT, LOW);
}

void loop()
{
  if (WiFi.status() == WL_CONNECTED)
  {
    // Read sensor values
    temperature = dht.readTemperature();
    humidity = dht.readHumidity();
    mq135_raw = analogRead(MQ135_PIN);

    ldrRaw = analogRead(LDR_PIN); // 0–4095
    lux = map(ldrRaw, 0, 4095, 0, 10000);

    HTTPClient http;
    http.begin(SERVER_CONTROL_URL);
    int httpResponseCode = http.GET();

    if (httpResponseCode == 200)
    {
      String response = http.getString();
      StaticJsonDocument<100> doc;
      deserializeJson(doc, response);
      String state = doc["state"];

      Serial.println("Device State from Server: " + state);

      if (state == "ON")
      {
        if (isnan(temperature) || isnan(humidity))
        {
          Serial.println("Failed to read from DHT11 sensor!");
        }
        else
        {

          if (temperature > 26)
          {
            digitalWrite(RELAY_FAN, LOW);
            Serial.println("🌡️ High temperature — Fan ON");
          }
          else
          {
            digitalWrite(RELAY_FAN, HIGH); // OFF
          }

          if (humidity < 50)
          {
             digitalWrite(RELAY_MIST, LOW);
             Serial.println("💧 Humidity too low — Humidifier ON");
          }
          else
          {
            digitalWrite(RELAY_MIST, HIGH);
          }

          if (mq135_raw < 1000)
          {
            digitalWrite(RELAY_SOLENOID, LOW);
            Serial.println("🫁 CO2 too low — Solenoid ON");
          }
          else
          {
            digitalWrite(RELAY_SOLENOID, HIGH);
          }

          if (lux < 2000)
          {
            digitalWrite(RELAY_LIGHT, LOW);
            Serial.println("💡 Light too low — Grow Light ON");
          }
          else
          {
            digitalWrite(RELAY_LIGHT, HIGH);
          }

          // Send data to backend
          StaticJsonDocument<400> dataDoc;
          dataDoc["temperature"] = temperature;
          dataDoc["humidity"] = humidity;
          dataDoc["co2"] = mq135_raw;
          dataDoc["light"] = lux;

          String jsonStr;
          serializeJson(dataDoc, jsonStr);

          HTTPClient postHttp;
          postHttp.begin(SERVER_URL);
          postHttp.addHeader("Content-Type", "application/json");
          int postResponse = postHttp.POST(jsonStr);
          if (postResponse > 0)
          {
            Serial.println("POST Response: " + postHttp.getString());
          }
          else
          {
            Serial.println("Failed to POST data. Code: " + String(postResponse));
          }
          postHttp.end();
        }
      }
      else
      {
        Serial.println("Device is OFF — skipping data send.");

        // Also turn off relays when OFF
        digitalWrite(RELAY_FAN, HIGH);
        digitalWrite(RELAY_MIST, HIGH);
        digitalWrite(RELAY_SOLENOID, HIGH);
        digitalWrite(RELAY_LIGHT, HIGH);
      }
    }
    else
    {
      Serial.println("Failed to GET control state. Code: " + String(httpResponseCode));
    }
    http.end();
  }
  else
  {
    Serial.println("WiFi Disconnected");
  }

  Serial.println("--------------------------");
  delay(5000);
}
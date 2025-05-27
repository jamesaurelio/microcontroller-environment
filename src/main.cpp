#include <Wire.h>
#include <BH1750.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

#define DHTPIN 4
#define DHTTYPE DHT11
#define MQ135_PIN 34

DHT dht(DHTPIN, DHTTYPE);
BH1750 lightMeter;

const char* ssid = "james";
const char* password = "abcdefghijae";

const char* serverDataUrl = "http://192.168.143.199:8081/api/formdata";
const char* serverControlUrl = "http://192.168.143.199:8081/api/control";

void setup() {
  Serial.begin(115200);

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" Connected");

  Wire.begin();
  if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    Serial.println("BH1750 ready.");
  } else {
    Serial.println("BH1750 not detected.");
  }

  dht.begin();
  pinMode(MQ135_PIN, INPUT);
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(serverControlUrl);
    int httpResponseCode = http.GET();

    if (httpResponseCode == 200) {
      String response = http.getString();
      StaticJsonDocument<100> doc;
      deserializeJson(doc, response);
      String state = doc["state"];

      Serial.println("Device State from Server: " + state);

      if (state == "ON") {
        // Read sensors
        float temperature = dht.readTemperature();
        float humidity = dht.readHumidity();
        int mq135_raw = analogRead(MQ135_PIN);
        float lux = lightMeter.readLightLevel();

        if (isnan(temperature) || isnan(humidity)) {
          Serial.println("Failed to read from DHT11 sensor!");
        } else {
          Serial.printf("Temp: %.1f °C, Humidity: %.1f %%\n", temperature, humidity);
        }

        // Prepare JSON
        StaticJsonDocument<200> dataDoc;
        dataDoc["temperature"] = temperature;
        dataDoc["humidity"] = humidity;
        dataDoc["co2"] = mq135_raw;
        dataDoc["light"] = lux;

        String jsonStr;
        serializeJson(dataDoc, jsonStr);

        // Send data
        HTTPClient postHttp;
        postHttp.begin(serverDataUrl);
        postHttp.addHeader("Content-Type", "application/json");

        int postResponse = postHttp.POST(jsonStr);
        if (postResponse > 0) {
          Serial.println("POST Response: " + postHttp.getString());
        } else {
          Serial.println("Failed to POST data. Code: " + String(postResponse));
        }
        postHttp.end();

        Serial.println("JSON Data Sent:");
        Serial.println(jsonStr);
      } else {
        Serial.println("Device is OFF — skipping data send.");
      }
    } else {
      Serial.println("Failed to GET control state. Code: " + String(httpResponseCode));
    }
    http.end();
  } else {
    Serial.println("WiFi Disconnected");
  }

  Serial.println("--------------------------");
  delay(5000);
}

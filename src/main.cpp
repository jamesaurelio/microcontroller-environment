#include <Wire.h>
#include <BH1750.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>  // For JSON formatting

#define DHTPIN 4           // Digital pin for DHT11
#define DHTTYPE DHT11
#define MQ135_PIN 34       // Analog pin for MQ135 (ADC1_GPIO34)

DHT dht(DHTPIN, DHTTYPE);
BH1750 lightMeter;


const char* ssid = "james";
const char* password = "abcdefghijae";

const char* serverUrl = "http://192.168.143.199:8081/api/sensorData";

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
   Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected");


  // Initialize I2C and BH1750
  Wire.begin();
  if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    Serial.println("BH1750 ready.");
  } else {
    Serial.println("BH1750 not detected.");
  }

  // Initialize DHT11
  dht.begin();

  // MQ135 (no special init needed for analog read)
  pinMode(MQ135_PIN, INPUT);
}

void loop() {
  // 🌡 DHT11 Readings
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("Failed to read from DHT11 sensor!");
  } else {
    Serial.printf("Temp: %.1f °C, Humidity: %.1f %%\n", temperature, humidity);
  }

  // 🌫 MQ135 Reading (raw analog)
  int mq135_raw = analogRead(MQ135_PIN);
  Serial.printf("MQ135 (Raw ADC): %d\n", mq135_raw);

  // 💡 BH1750 Reading
  float lux = lightMeter.readLightLevel();
  Serial.printf("Light: %.1f lx\n", lux);

  StaticJsonDocument<200> doc;
  doc["temperature"] = temperature;
  doc["humidity"] = humidity;
  doc["co2"] = mq135_raw;
  doc["light"] = lux;

  String jsonStr;
  serializeJson(doc, jsonStr);

  if(WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(serverUrl);
    http.addHeader("Content-Type", "application/json");
    int httpResponseCode = http.POST(jsonStr);
    if(httpResponseCode > 0) {
      String response = http.getString();
      Serial.println("POST Response: " + response);
    } else {
      Serial.println("Error on sending POST: " + String(httpResponseCode));
    }
    http.end();
  } else {
    Serial.println("WiFi Disconnected");
  }

  // Print the JSON string to Serial
  Serial.println("JSON Data:");
  Serial.println(jsonStr);
  Serial.println("--------------------------");
  delay(5000);
}
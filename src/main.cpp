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

const char *ssid = "james";
const char *password = "abcdefghijae";

const char *serverUrl = "http://192.168.143.199:8081/api/sensorData";
const char *serverControlUrl = "http://192.168.143.199:8081/api/control";

// Parameters for numerical model
const float ambient = 25.0;
const float k = 0.1;
const float dt = 1.0; // time step in seconds
float simTime = 0.0;  // simulation time tracker

// Sensor and smoothed values
float temperature = 0.0, humidity = 0.0, mq135_raw = 0.0, lux = 0.0;
float prevTemp = 25.0, prevHum = 50.0, prevCO2 = 300.0, prevLux = 1000.0;

// Euler Simulation Values
float T_eul = 25.0, H_eul = 50.0, C_eul = 300.0, L_eul = 1000.0;
// RK4 Simulation Values
float T_rk4 = 25.0, H_rk4 = 50.0, C_rk4 = 300.0, L_rk4 = 1000.0;

// Anomaly detection state
bool prevTempAnomaly = false;
bool prevHumAnomaly = false;
bool prevCO2Anomaly = false;
bool prevLuxAnomaly = false;

// Basic stats for anomaly detection
float tempMean = 25.0, tempStd = 1.0;
float humMean = 50.0, humStd = 5.0;
float co2Mean = 300.0, co2Std = 20.0;
float luxMean = 1000.0, luxStd = 300.0;

bool isAnomaly(float value, float mean, float std)
{
  float z = abs((value - mean) / std);
  return z > 3.0;
}

// Differential equations
float external_factor(float t)
{
  return t >= 2.0 ? 1.0 : 0.0;
}
float dTdt(float T, float t)
{
  return -k * (T - ambient) + external_factor(t);
}
float dHdt(float H, float t)
{
  return -0.05 * (H - 50);
}
float dCO2dt(float C, float t)
{
  return -0.2 * (C - 300);
}
float dLightdt(float L, float t)
{
  return 0.01 * (50000 - L);
}

// 📌 Runge-Kutta 4th Order Step Function
float rk4_step(float (*f)(float, float), float y, float t, float h)
{
  float k1 = h * f(y, t);
  float k2 = h * f(y + 0.5 * k1, t + 0.5 * h);
  float k3 = h * f(y + 0.5 * k2, t + 0.5 * h);
  float k4 = h * f(y + k3, t + h);
  return (k1 + 2 * k2 + 2 * k3 + k4) / 6.0;
}

void setup()
{
  Serial.begin(115200);

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" Connected");

  Wire.begin();
  if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23))
  {
    Serial.println("BH1750 ready.");
  }
  else
  {
    Serial.println("BH1750 not detected.");
  }

  dht.begin();
  pinMode(MQ135_PIN, INPUT);
}

void loop()
{
  if (WiFi.status() == WL_CONNECTED)
  {
    // Read sensor values
    temperature = dht.readTemperature();
    humidity = dht.readHumidity();
    mq135_raw = analogRead(MQ135_PIN);
    lux = lightMeter.readLightLevel();

    HTTPClient http;
    http.begin(serverControlUrl);
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
          // Smoothing
          temperature = (prevTemp + temperature) / 2.0;
          humidity = (prevHum + humidity) / 2.0;
          float mq135_smooth = (prevCO2 + mq135_raw) / 2.0;
          float lux_smooth = (prevLux + lux) / 2.0;
          prevTemp = temperature;
          prevHum = humidity;
          prevCO2 = mq135_smooth;
          prevLux = lux_smooth;

          // Detect anomalies
          bool tempAnomaly = isAnomaly(temperature, tempMean, tempStd);
          bool humAnomaly = isAnomaly(humidity, humMean, humStd);
          bool co2Anomaly = isAnomaly(mq135_smooth, co2Mean, co2Std);
          bool luxAnomaly = isAnomaly(lux_smooth, luxMean, luxStd);

          // Print alert ONLY when anomaly status changes from false to true
          if (tempAnomaly && !prevTempAnomaly)
          {
            Serial.println("⚠️ Temperature anomaly detected!");
          }
          if (humAnomaly && !prevHumAnomaly)
          {
            Serial.println("⚠️ Humidity anomaly detected!");
          }
          if (co2Anomaly && !prevCO2Anomaly)
          {
            Serial.println("⚠️ CO2 anomaly detected!");
          }
          if (luxAnomaly && !prevLuxAnomaly)
          {
            Serial.println("⚠️ Light anomaly detected!");
          }

          // Update previous states
          prevTempAnomaly = tempAnomaly;
          prevHumAnomaly = humAnomaly;
          prevCO2Anomaly = co2Anomaly;
          prevLuxAnomaly = luxAnomaly;

          Serial.printf("Smoothed Temp: %.1f °C, Smoothed Humidity: %.1f %%\n", temperature, humidity);

          // Euler Simulation
          T_eul += dt * dTdt(T_eul, simTime);
          H_eul += dt * dHdt(H_eul, simTime);
          C_eul += dt * dCO2dt(C_eul, simTime);
          L_eul += dt * dLightdt(L_eul, simTime);

          // RK4 Simulation
          T_rk4 += rk4_step(dTdt, T_rk4, simTime, dt);
          H_rk4 += rk4_step(dHdt, H_rk4, simTime, dt);
          C_rk4 += rk4_step(dCO2dt, C_rk4, simTime, dt);
          L_rk4 += rk4_step(dLightdt, L_rk4, simTime, dt);

          simTime += dt;

          // Send data to backend
          StaticJsonDocument<400> dataDoc;
          dataDoc["temperature"] = temperature;
          dataDoc["humidity"] = humidity;
          dataDoc["co2"] = mq135_raw;
          dataDoc["light"] = lux;

          dataDoc["T_sim"] = T_eul;
          dataDoc["H_sim"] = H_eul;
          dataDoc["C_sim"] = C_eul;
          dataDoc["L_sim"] = L_eul;

          dataDoc["T_rk4"] = T_rk4;
          dataDoc["H_rk4"] = H_rk4;
          dataDoc["C_rk4"] = C_rk4;
          dataDoc["L_rk4"] = L_rk4;

          String jsonStr;
          serializeJson(dataDoc, jsonStr);

          HTTPClient postHttp;
          postHttp.begin(serverUrl);
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

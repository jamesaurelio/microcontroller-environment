#include <Wire.h>
#include <BH1750.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN 4           // Digital pin for DHT11
#define DHTTYPE DHT11
#define MQ135_PIN 34       // Analog pin for MQ135 (ADC1_GPIO34)

DHT dht(DHTPIN, DHTTYPE);
BH1750 lightMeter;

void setup() {
  Serial.begin(115200);

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

  Serial.println("--------------------------");
  delay(2000);
}

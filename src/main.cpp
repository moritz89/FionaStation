// Direkter Wert aus Mqtt mosquitto_sub -h 192.168.0.1 -t 'tele/hydro_box1/#'

// Grafana: https://data.veleda.io/?orgId=1
// Was noch nicht funktioniert:#
// 1. I2C Adresse des MAX44009 ändern... ---> Erst einmal nur einen MAX44009
// nutzen
// 2. 2 BME280 --> sind eigentlich BMP280....
// 3. BH1750 liefern Werte, diese werden jedoch nicht weitergereicht bzw. nicht
// in Grafana angezeigt Librarys
#include "Max44009.h"
#include "chrono"

#include <BH1750.h>
#include <PubSubClient.h>
#include <Wire.h>
#include "OneWire.h"
#include "SparkFunBME280.h"

#include "configuration.h"
#include "mqtt.h"
#include "secrets.h"
#include "wifi.h"

// Globale Variablen & Pins
const int oneWirePin = 32;  // Dallas TempSensor Pin
const int waterAvaiablePin_ACE = 18;
const int waterAvaiablePin_BD = 19;
int dallas_count = 3;  // Anzahl an DS18B20 Sensoren
float celsius = 0;
// Objekte initialisieren
BH1750 lightMeter(0x23);
BH1750 lightMeter_2(0x5C);
Max44009 myLux(0x4A);  // default addr
// Max44009 myLux2(0x4B);
OneWire ds(oneWirePin);
// DallasTemperature sensors(&ds);
BME280 AirSensor;
// BME280 AirSensor2;

// Speichervariablen
float lux_MAX = 0;
int lux_BH1750 = 0;
int lux_BH1750_2 = 0;
float dallas_WT = 0;  // Watertemperature
float dallas_TT = 0;  // TowerTemperature
float dallas_AT = 0;  // WaterTemperature
float bme_T = 0;
float bme_RF = 0;
float bme_P = 0;
bool waterStatus_ACE = 0;
bool waterStatus_BD = 0;

uint32_t lastDisplay = 0;

long lastMsg = 0;
char msg[20];

WiFiClient wifiClient;

esp_monitor::Wifi wifi(esp_monitor::ssid, esp_monitor::password);
esp_monitor::Mqtt mqtt(wifiClient, esp_monitor::client_id,
                       esp_monitor::mqtt_server);

void setup() {
  Serial.begin(115200);
  pinMode(esp_monitor::status_led, OUTPUT);

  // Try to connect to Wifi within wifi_connect_timeout, else restart
  digitalWrite(esp_monitor::status_led, HIGH);
  if (wifi.connect(esp_monitor::wifi_connect_timeout) == false) {
    Serial.printf("WiFi: Could not connect to %s. Restarting\n",
                  esp_monitor::ssid);
    ESP.restart();
  }
  wifi.printState();

  // Try to connect to the MQTT broker 3 times, else restart
  digitalWrite(esp_monitor::status_led, LOW);
  if (mqtt.connect(esp_monitor::connection_attempts) == false) {
    Serial.println("MQTT: Could not connect to broker. Restarting\n");
    ESP.restart();
  }

  Wire.begin();
  lightMeter.begin(BH1750::ONE_TIME_HIGH_RES_MODE);
  lightMeter_2.begin(BH1750::ONE_TIME_HIGH_RES_MODE);
  AirSensor.setI2CAddress(0x77);
  AirSensor.beginI2C(Wire);
  // AirSensor2.setI2CAddress(0x76);
  // AirSensor2.beginI2C(Wire);

  /* set led as output to control led on-off */
  pinMode(waterAvaiablePin_ACE, INPUT);
  pinMode(waterAvaiablePin_BD, INPUT);
  Serial.println();
}

void loop() {
  // Turn the blue on-board LED on during the loop
  digitalWrite(esp_monitor::status_led, HIGH);

  // If not connected to WiFi, attempt to reconnect. Finally reboot
  if (!wifi.isConnected()) {
    Serial.println("WiFi: Disconnected. Attempting to reconnect");
    if (wifi.connect(esp_monitor::wifi_connect_timeout) == false) {
      Serial.printf("WiFi: Could not connect to %s. Restarting\n",
                    esp_monitor::ssid);
      ESP.restart();
    }
  }

  // If not connected to an MQTT broker, attempt to reconnect. Else reboot
  if (!mqtt.isConnected()) {
    Serial.println("MQTT: Disconnected. Attempting to reconnect");
    if (mqtt.connect(esp_monitor::connection_attempts) == false) {
      Serial.println("MQTT: Could not connect to broker. Restarting\n");
      ESP.restart();
    }
  }

  delay(1000);
  // WaterStatus NFT-System
  waterStatus_ACE = digitalRead(waterAvaiablePin_ACE);
  waterStatus_BD = digitalRead(waterAvaiablePin_BD);
  Serial.print("WaterStatus ACE: ");
  Serial.println(waterStatus_ACE);
  Serial.print("WaterStatus BD: ");
  Serial.println(waterStatus_BD);
  delay(1000);
  // BH1750
  Serial.println("BH1750");
  uint16_t lux_BH1750 = lightMeter.readLightLevel();
  Serial.print("   ");
  Serial.print("Light: ");
  Serial.print(lux_BH1750);
  Serial.println(" lx");
  Serial.print("   ");

  delay(100);
  uint16_t lux_BH1750_2 = lightMeter_2.readLightLevel();
  Serial.print("Light: ");
  Serial.print(lux_BH1750_2);
  Serial.println(" lx");

  delay(1000);
  // BME280
  Serial.println("BME280");
  Serial.print("   Humidity: ");
  bme_RF = AirSensor.readFloatHumidity();
  Serial.println(bme_RF, 2);
  Serial.print("   Pressure: ");
  bme_P = AirSensor.readFloatPressure();
  Serial.println(bme_P, 0);
  Serial.print("   Temp: ");
  bme_T = AirSensor.readTempC();
  Serial.println(bme_T, 2);

  delay(1000);
  Serial.println("MAX44009");
  lux_MAX = myLux.getLux();
  int err = myLux.getError();
  if (err != 0) {
    Serial.print("Error:\t");
    Serial.println(err);
  } else {
    Serial.print("lux:\t");
    Serial.println(lux_MAX);
  }

  delay(1000);
  // DS18B20, Auflösung "nur" 10Bit, da sonst Timing Problem mit WiFi... Mit 10
  // Bit gelingt jede 2te Messung...
  Serial.println("DS18B20");

  for (int j = 0; j < dallas_count + 2; j++) {
    switch (j) {
      case 0:
        break;
      case 1:
        dallas_AT = celsius;
        Serial.println(dallas_AT);
        break;
      case 2:
        dallas_WT = celsius;
        Serial.println(dallas_WT);
        break;
      case 3:
        dallas_TT = celsius;
        Serial.println(dallas_TT);
        break;
      default:
        // memset(msg, 0, sizeof(msg));
        // snprintf(msg, 20, "%lf", lux_MAX);
        // publish the message
        mqtt.send("MAX44009_L", lux_MAX);  // Licht
        // client.publish(MAX44009_L, msg);

        // memset(msg, 0, sizeof(msg));
        // snprintf(msg, 20, "%lf", (double)waterStatus_ACE);
        // publish the message
        mqtt.send("WATERAVAIABLE_ACE", waterStatus_ACE);  // NFT System ACE
        // client.publish(WATERAVAIABLE_ACE_TOPIC, msg);

        // memset(msg, 0, sizeof(msg));
        // snprintf(msg, 20, "%lf", (double)waterStatus_BD);
        // // publish the message
        mqtt.send("WATERAVAIABLE_BD", waterStatus_BD);  // NFT System BD
        // client.publish(WATERAVAIABLE_BD_TOPIC, msg);

        // memset(msg, 0, sizeof(msg));
        // snprintf(msg, 20, "%lf", (float)lux_BH1750);
        // // publish the message
        mqtt.send("BH1750_L", lux_BH1750);  // Licht
        // client.publish(BH1750_L_TOPIC, msg);

        // memset(msg, 0, sizeof(msg));
        // snprintf(msg, 20, "%lf", (float)lux_BH1750_2);
        // // publish the message
        mqtt.send("BH1750_L2", lux_BH1750_2);  // Licht
        // client.publish(BH1750_L2_TOPIC, msg);

        // memset(msg, 0, sizeof(msg));
        // snprintf(msg, 20, "%lf", bme_T);
        mqtt.send("BME280_T", bme_T);  // Lufttemperatur
        // client.publish(BME_T_TOPIC, msg);

        // memset(msg, 0, sizeof(msg));
        // snprintf(msg, 20, "%lf", bme_P);
        mqtt.send("BME280_P", bme_P);  // Luftdruck
        // client.publish(BME_P_TOPIC, msg);

        // memset(msg, 0, sizeof(msg));
        // snprintf(msg, 20, "%lf", bme_RF);
        mqtt.send("BME280_RF", bme_RF);  // Luftfeuchtigkeit
        // client.publish(BME_RF_TOPIC, msg);

        // memset(msg, 0, sizeof(msg));
        // snprintf(msg, 20, "%lf", dallas_WT);
        mqtt.send("Dallas_WT", dallas_WT);  // Wassertemperatur
        // client.publish(DALLAS_WT_TOPIC, msg);

        // memset(msg, 0, sizeof(msg));
        // snprintf(msg, 20, "%lf", dallas_TT);
        mqtt.send("Dallas_TT", dallas_TT);  // Towertemperatur
        // client.publish(DALLAS_TT_TOPIC, msg);

        // memset(msg, 0, sizeof(msg));
        // snprintf(msg, 20, "%lf", dallas_AT);
        mqtt.send("Dallas_AT", dallas_AT);  // Airtemperatur
        // client.publish(DALLAS_AT_TOPIC, msg);
    }

    if (j < dallas_count) {
      byte i;
      byte type_s;
      byte data[12];
      byte addr[8];
      byte present = 0;

      if (!ds.search(addr)) {
        ds.reset_search();
        delay(250);
        return;
      }

      if (OneWire::crc8(addr, 7) != addr[7]) {
        Serial.println("CRC is not valid!");
        return;
      }
      // Serial.println();

      // the first ROM byte indicates which chip
      switch (addr[0]) {
        case 0x10:
          type_s = 1;
          break;
        case 0x28:
          type_s = 0;
          break;
        case 0x22:
          type_s = 0;
          break;
        default:
          Serial.println("Device is not a DS18x20 family device.");
          return;
      }

      ds.reset();
      ds.select(addr);
      ds.write(0x44, 1);  // start conversion, with parasite power on at the end
      delay(1000);
      present = ds.reset();
      ds.select(addr);
      ds.write(0xBE);  // Read Scratchpad

      for (i = 0; i < 9; i++) {
        data[i] = ds.read();
      }

      // Convert the data to actual temperature
      int16_t raw = (data[1] << 8) | data[0];
      if (type_s) {
        raw = raw << 3;  // 9 bit resolution default
        if (data[7] == 0x10) {
          raw = (raw & 0xFFF0) + 12 - data[6];
        }
      } else {
        byte cfg = (data[4] & 0x20);
        if (cfg == 0x00)
          raw = raw & ~7;  // 9 bit resolution, 93.75 ms
        else if (cfg == 0x20)
          raw = raw & ~3;  // 10 bit res, 187.5 ms
        else if (cfg == 0x40)
          raw = raw & ~1;  // 11 bit res, 375 ms

        celsius = (float)raw / 16.0;
        delay(1000);
      }

      delay(1000);
    }
  }

  // Turn the blue on-board LED off before sleeping
  digitalWrite(esp_monitor::status_led, LOW);
  delay(1000);
}

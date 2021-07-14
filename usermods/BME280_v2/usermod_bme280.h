#pragma once

#include "wled.h"
#include <Arduino.h>
#include <Wire.h>
#include <BME280I2C.h>               // BME280 sensor
#include <EnvironmentCalculations.h> // BME280 extended measurements

class UsermodBME280 : public Usermod
{
private:
// User-defined configuration
#define Celsius                // Show temperature mesaurement in Celcius. Comment out for Fahrenheit
#define TemperatureDecimals 1  // Number of decimal places in published temperaure values
#define HumidityDecimals 0     // Number of decimal places in published humidity values
#define PressureDecimals 0     // Number of decimal places in published pressure values
#define TemperatureInterval 30 // Interval to measure temperature (and humidity, dew point if available) in seconds
#define PressureInterval 300   // Interval to measure pressure in seconds
#define PublishAlways 0        // Publish values even when they have not changed
#define HomeAssistantDiscovery 1  // Publish Home Assistant Discovery messages

// Sanity checks
#if !defined(TemperatureDecimals) || TemperatureDecimals < 0
  #define TemperatureDecimals 0
#endif
#if !defined(HumidityDecimals) || HumidityDecimals < 0
  #define HumidityDecimals 0
#endif
#if !defined(PressureDecimals) || PressureDecimals < 0
  #define PressureDecimals 0
#endif
#if !defined(TemperatureInterval) || TemperatureInterval < 0
  #define TemperatureInterval 1
#endif
#if !defined(PressureInterval) || PressureInterval < 0
  #define PressureInterval TemperatureInterval
#endif
#if !defined(PublishAlways)
  #define PublishAlways 0
#endif
#if !defined(HomeAssistantDiscovery)
  #define HomeAssistantDiscovery 0
#endif

#ifdef ARDUINO_ARCH_ESP32 // ESP32 boards
  uint8_t SCL_PIN = 22;
  uint8_t SDA_PIN = 21;
#else // ESP8266 boards
  uint8_t SCL_PIN = 5;
  uint8_t SDA_PIN = 4;
  //uint8_t RST_PIN = 16; // Uncoment for Heltec WiFi-Kit-8
#endif

  // BME280 sensor settings
  BME280I2C::Settings settings{
      BME280::OSR_X16, // Temperature oversampling x16
      BME280::OSR_X16, // Humidity oversampling x16
      BME280::OSR_X16, // Pressure oversampling x16
      // Defaults
      BME280::Mode_Forced,
      BME280::StandbyTime_1000ms,
      BME280::Filter_Off,
      BME280::SpiEnable_False,
      BME280I2C::I2CAddr_0x76 // I2C address. I2C specific. Default 0x76
  };

  BME280I2C bme{settings};

  uint8_t sensorType;

  // Measurement timers
  long timer;
  long lastTemperatureMeasure = 0;
  long lastPressureMeasure = 0;

  // Current sensor values
  float sensorTemperature;
  float sensorHumidity;
  float sensorHeatIndex;
  float sensorDewPoint;
  float sensorPressure;
  // Track previous sensor values
  float lastTemperature;
  float lastHumidity;
  float lastHeatIndex;
  float lastDewPoint;
  float lastPressure;

  bool mqttInitialized = false;
  String mqttTemperatureTopic = "";
  String mqttHumidityTopic = "";
  String mqttPressureTopic = "";
  String mqttHeatIndexTopic = "";
  String mqttDewPointTopic = "";

  // Store packet IDs of MQTT publications
  uint16_t mqttTemperaturePub = 0;
  uint16_t mqttPressurePub = 0;

  void UpdateBME280Data(int SensorType)
  {
    float _temperature, _humidity, _pressure;
    #ifdef Celsius
      BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
      EnvironmentCalculations::TempUnit envTempUnit(EnvironmentCalculations::TempUnit_Celsius);
    #else
      BME280::TempUnit tempUnit(BME280::TempUnit_Fahrenheit);
      EnvironmentCalculations::TempUnit envTempUnit(EnvironmentCalculations::TempUnit_Fahrenheit);
    #endif
    BME280::PresUnit presUnit(BME280::PresUnit_hPa);

    bme.read(_pressure, _temperature, _humidity, tempUnit, presUnit);

    sensorTemperature = _temperature;
    sensorHumidity = _humidity;
    sensorPressure = _pressure;
    if (sensorType == 1)
    {
      sensorHeatIndex = EnvironmentCalculations::HeatIndex(_temperature, _humidity, envTempUnit);
      sensorDewPoint = EnvironmentCalculations::DewPoint(_temperature, _humidity, envTempUnit);
    }
  }

  void _mqttInitialize()
  {
    mqttTemperatureTopic = String(mqttDeviceTopic) + "/temperature";
    mqttPressureTopic = String(mqttDeviceTopic) + "/pressure";
    mqttHumidityTopic = String(mqttDeviceTopic) + "/humidity";
    mqttHeatIndexTopic = String(mqttDeviceTopic) + "/heat_index";
    mqttDewPointTopic = String(mqttDeviceTopic) + "/dew_point";

    String t = String("homeassistant/sensor/") + mqttClientID + "/temperature/config";

    _createMqttSensor("Temperature", mqttTemperatureTopic, "temperature", "°C");
    _createMqttSensor("Pressure", mqttPressureTopic, "pressure", "hPa");
    _createMqttSensor("Humidity", mqttHumidityTopic, "humidity", "%");
    _createMqttSensor("Heat Index", mqttHeatIndexTopic, "temperature", "°C");
    _createMqttSensor("Dew Point", mqttDewPointTopic, "temperature", "°C");
  }

  void _createMqttSensor(const String &name, const String &topic, const String &deviceClass, const String &unitOfMeasurement)
  {
    String t = String("homeassistant/sensor/") + mqttClientID + "/" + name + "/config";
    
    StaticJsonDocument<300> doc;

    doc["name"] = name;
    doc["state_topic"] = topic;
    doc["unique_id"] = String(mqttClientID) + name;
    if (unitOfMeasurement != "")
      doc["unit_of_measurement"] = unitOfMeasurement;
    if (deviceClass != "")
      doc["device_class"] = deviceClass;
    doc["expire_after"] = 1800;

    JsonObject device = doc.createNestedObject("device"); // attach the sensor to the same device
    device["name"] = String(mqttClientID);
    device["identifiers"] = String("wled-sensor-") + String(mqttClientID);
    device["manufacturer"] = "Aircoookie";
    device["model"] = "WLED";
    device["sw_version"] = VERSION;

    String temp;
    serializeJson(doc, temp);
    Serial.println(t);
    Serial.println(temp);

    mqtt->publish(t.c_str(), 0, true, temp.c_str());
  }

public:
  void setup()
  {
    Wire.begin(SDA_PIN, SCL_PIN);

    if (!bme.begin())
    {
      sensorType = 0;
      Serial.println("Could not find BME280I2C sensor!");
    }
    else
    {
      switch (bme.chipModel())
      {
      case BME280::ChipModel_BME280:
        sensorType = 1;
        Serial.println("Found BME280 sensor! Success.");
        break;
      case BME280::ChipModel_BMP280:
        sensorType = 2;
        Serial.println("Found BMP280 sensor! No Humidity available.");
        break;
      default:
        sensorType = 0;
        Serial.println("Found UNKNOWN sensor! Error!");
      }
    }
  }

  void loop()
  {
    // BME280 sensor MQTT publishing
    // Check if sensor present and MQTT Connected, otherwise it will crash the MCU
    if (sensorType != 0 && WLED_MQTT_CONNECTED)
    {
      // Timer to fetch new temperature, humidity and pressure data at intervals
      timer = millis();

      if (timer - lastTemperatureMeasure >= TemperatureInterval * 1000 || mqttTemperaturePub == 0)
      {
        lastTemperatureMeasure = timer;

        UpdateBME280Data(sensorType);


        if (WLED_MQTT_CONNECTED)
        {
          if (!mqttInitialized && HomeAssistantDiscovery)
          {
            _mqttInitialize();
            mqttInitialized = true;
          }

          float temperature = roundf(sensorTemperature * pow(10, TemperatureDecimals)) / pow(10, TemperatureDecimals);
          float pressure, humidity, heatIndex, dewPoint;
          

          // If temperature has changed since last measure, create string populated with device topic
          // from the UI and values read from sensor, then publish to broker
          if (temperature != lastTemperature || PublishAlways)
          {
            mqttTemperaturePub = mqtt->publish(mqttTemperatureTopic.c_str(), 0, false, String(temperature, TemperatureDecimals).c_str());
          }

          lastTemperature = temperature; // Update last sensor temperature for next loop

          if (sensorType == 1) // Only if sensor is a BME280
          {
            pressure = roundf(sensorPressure * pow(10, PressureDecimals)) / pow(10, PressureDecimals);
            humidity = roundf(sensorHumidity * pow(10, HumidityDecimals)) / pow(10, HumidityDecimals);
            heatIndex = roundf(sensorHeatIndex * pow(10, TemperatureDecimals)) / pow(10, TemperatureDecimals);
            dewPoint = roundf(sensorDewPoint * pow(10, TemperatureDecimals)) / pow(10, TemperatureDecimals);
                        
            if (humidity != lastHumidity || PublishAlways)
            {
              mqtt->publish(mqttHumidityTopic.c_str(), 0, false, String(humidity, HumidityDecimals).c_str());
            }
              
            if (pressure != lastPressure || PublishAlways)
            {
              mqttPressurePub = mqtt->publish(mqttPressureTopic.c_str(), 0, true, String(pressure, PressureDecimals).c_str());
            }

            // If heat index has changed OR PublishAlways set to true, publish to MQTT
            if (heatIndex != lastHeatIndex || PublishAlways)
            {
              mqtt->publish(mqttHeatIndexTopic.c_str(), 0, false, String(heatIndex, TemperatureDecimals).c_str());
            }

            // If dew point has changed OR PublishAlways set to true, publish to MQTT
            if (dewPoint != lastDewPoint || PublishAlways)
            {
              mqtt->publish(mqttDewPointTopic.c_str(), 0, false, String(dewPoint, TemperatureDecimals).c_str());
            }

            lastPressure = pressure;
            lastHumidity = humidity;
            lastHeatIndex = heatIndex;
            lastDewPoint = dewPoint;
          }
        }
        else
        {
          Serial.println("Missing MQTT connection. Not publishing data");
          mqttInitialized = false;
        }
      }
    }
  }
};
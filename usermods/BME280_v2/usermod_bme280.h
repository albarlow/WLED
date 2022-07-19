// force the compiler to show a warning to confirm that this file is included
#warning **** Included USERMOD_BME280 version 2.0 ****
#pragma once

#include "wled.h"
#include <Arduino.h>
#include <Wire.h>
#include <BME280I2C.h>               // BME280 sensor
#include <EnvironmentCalculations.h> // BME280 extended measurements

class UsermodBME280 : public Usermod
{
private:

// Config Variables //
// Show temperature measurement in Celsius (comment out to use Farenheit)
#define Celsius
// Number of decimal places in published temperaure values
unsigned long TemperatureDecimals;
// Number of decimal places in published humidity values
unsigned long HumidityDecimals;
// Number of decimal places in published pressure values
unsigned long PressureDecimals;
// Interval to measure temperature (and humidity, dew point if available) in seconds
unsigned long TemperatureInterval;
// Publish values even when they have not changed
bool PublishAlways;
// Publish Home Assistant Device Information
bool HomeAssistantDiscovery;

// Default Pins, these can be overridden by user config at runtime
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

  // Define 
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
    _createMqttSensor("HeatIndex", mqttHeatIndexTopic, "temperature", "°C");
    _createMqttSensor("DewPoint", mqttDewPointTopic, "temperature", "°C");
  }

  // Create an MQTT Sensor for Home Assistant Discovery purposes, this includes a pointer to the topic that is published to in the Loop.
  void _createMqttSensor(const String &name, const String &topic, const String &deviceClass, const String &unitOfMeasurement)
  {
    String t = String("homeassistant/sensor/") + mqttClientID + "/" + name + "/config";
    
    StaticJsonDocument<600> doc;
    
    doc["name"] = String(serverDescription) + " " + name;
    doc["state_topic"] = topic;
    doc["unique_id"] = String(mqttClientID) + name;
    if (unitOfMeasurement != "")
      doc["unit_of_measurement"] = unitOfMeasurement;
    if (deviceClass != "")
      doc["device_class"] = deviceClass;
    doc["expire_after"] = 1800;

    JsonObject device = doc.createNestedObject("device"); // attach the sensor to the same device
    device["name"] = serverDescription;
    device["identifiers"] = "wled-sensor-" + String(mqttClientID);
    device["manufacturer"] = "WLED";
    device["model"] = "FOSS";
    device["sw_version"] = versionString;

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
          float pressure = roundf(sensorPressure * pow(10, PressureDecimals)) / pow(10, PressureDecimals);
          
          
            

          // If temperature has changed since last measure, create string populated with device topic
          // from the UI and values read from sensor, then publish to broker
          if (temperature != lastTemperature || PublishAlways)
          {
            mqttTemperaturePub = mqtt->publish(mqttTemperatureTopic.c_str(), 0, false, String(temperature, TemperatureDecimals).c_str());
          }

          lastTemperature = temperature; // Update last sensor temperature for next loop

          if (pressure != lastPressure || PublishAlways)
          {
            mqttPressurePub = mqtt->publish(mqttPressureTopic.c_str(), 0, true, String(pressure, PressureDecimals).c_str());
          }

          if (sensorType == 1) // Only if sensor is a BME280
          {
            float humidity, heatIndex, dewPoint;
            humidity = roundf(sensorHumidity * pow(10, HumidityDecimals)) / pow(10, HumidityDecimals);
            heatIndex = roundf(sensorHeatIndex * pow(10, TemperatureDecimals)) / pow(10, TemperatureDecimals);
            dewPoint = roundf(sensorDewPoint * pow(10, TemperatureDecimals)) / pow(10, TemperatureDecimals);
            
            // If humidity has changed OR PublishAlways set to true, publish to MQTT
            if (humidity != lastHumidity || PublishAlways)
            {
              mqtt->publish(mqttHumidityTopic.c_str(), 0, false, String(humidity, HumidityDecimals).c_str());
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

  void addToJsonInfo(JsonObject &root)
  {
    JsonObject user = root[F("u")];
    if (user.isNull())
      user = root.createNestedObject(F("u"));
    
    if (sensorType==0) //No Sensor
    {
      // if we sensor not detected, let the user know
      JsonArray temperature_json = user.createNestedArray("BME/BMP280 Sensor");
      temperature_json.add(F("Not Found"));
    }
    else if (sensorType==2) //BMP280
    {
      
      JsonArray temperature_json = user.createNestedArray("Temperature");
      JsonArray pressure_json = user.createNestedArray("Pressure");
      temperature_json.add(sensorTemperature);
      temperature_json.add(F("°C"));
      pressure_json.add(sensorPressure);
      pressure_json.add(F("°C"));
    }
    else if (sensorType==1) //BME280
    {
      JsonArray temperature_json = user.createNestedArray("Temperature");
      JsonArray humidity_json = user.createNestedArray("Humidity");
      JsonArray pressure_json = user.createNestedArray("Pressure");
      JsonArray heatindex_json = user.createNestedArray("Heat Index");
      JsonArray dewpoint_json = user.createNestedArray("Dew Point");
      temperature_json.add(sensorTemperature);
      temperature_json.add(F("°C"));
      humidity_json.add(sensorHumidity);
      humidity_json.add(F("%"));
      pressure_json.add(sensorPressure);
      pressure_json.add(F("°C"));
      heatindex_json.add(sensorHeatIndex);
      heatindex_json.add(F("°C"));
      dewpoint_json.add(sensorDewPoint);
      dewpoint_json.add(F("°C"));
    }
      return;
  }

  void addToConfig(JsonObject& root)
  {
    JsonObject top = root.createNestedObject("BME280/BMP280");
    top["TemperatureDecimals"] = TemperatureDecimals;
    top["HumidityDecimals"] = HumidityDecimals;
    top["PressureDecimals"] = PressureDecimals;
    top["TemperatureInterval"] = TemperatureInterval;
    top["PublishAlways"] = PublishAlways;
    top["HomeAssistantDiscovery"] = HomeAssistantDiscovery;
    JsonArray pinArray = top.createNestedArray("pin");
    pinArray.add(SDA_PIN);
    pinArray.add(SCL_PIN); 
  }
  bool readFromConfig(JsonObject& root)
  {
    // default settings values could be set here (or below using the 3-argument getJsonValue()) instead of in the class definition or constructor
    // setting them inside readFromConfig() is slightly more robust, handling the rare but plausible use case of single value being missing after boot (e.g. if the cfg.json was manually edited and a value was removed)

    JsonObject top = root["BME280/BMP280"];

    bool configComplete = !top.isNull();

    // A 3-argument getJsonValue() assigns the 3rd argument as a default value if the Json value is missing
    configComplete &= getJsonValue(top["TemperatureDecimals"], TemperatureDecimals, 1);
    configComplete &= getJsonValue(top["HumidityDecimals"], HumidityDecimals, 0);
    configComplete &= getJsonValue(top["PressureDecimals"], PressureDecimals, 0);
    configComplete &= getJsonValue(top["TemperatureInterval"], TemperatureInterval, 30);
    configComplete &= getJsonValue(top["PublishAlways"], PublishAlways, false);
    configComplete &= getJsonValue(top["HomeAssistantDiscovery"], HomeAssistantDiscovery, true);
    configComplete &= getJsonValue(top["pin"][0], SDA_PIN, 21); //SDA
    configComplete &= getJsonValue(top["pin"][1], SCL_PIN, 22); //SCL

    return configComplete;
  }
};
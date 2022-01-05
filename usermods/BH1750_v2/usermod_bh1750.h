// force the compiler to show a warning to confirm that this file is included
#warning **** Included USERMOD_BH1750 ****

#pragma once

#include "wled.h"
#include <Wire.h>
#include <BH1750.h>

// the max frequency to check photoresistor, 10 seconds
#ifndef USERMOD_BH1750_MAX_MEASUREMENT_INTERVAL
#define USERMOD_BH1750_MAX_MEASUREMENT_INTERVAL 10000
#endif

// the min frequency to check photoresistor, 500 ms
#ifndef USERMOD_BH1750_MIN_MEASUREMENT_INTERVAL
#define USERMOD_BH1750_MIN_MEASUREMENT_INTERVAL 500
#endif

// how many seconds after boot to take first measurement, 10 seconds
#ifndef USERMOD_BH1750_FIRST_MEASUREMENT_AT
#define USERMOD_BH1750_FIRST_MEASUREMENT_AT 10000
#endif

// only report if differance grater than offset value
#ifndef USERMOD_BH1750_OFFSET_VALUE
#define USERMOD_BH1750_OFFSET_VALUE 1
#endif

class Usermod_BH1750 : public Usermod
{
private:
  int8_t offset = USERMOD_BH1750_OFFSET_VALUE;

  unsigned long maxReadingInterval = USERMOD_BH1750_MAX_MEASUREMENT_INTERVAL;
  unsigned long minReadingInterval = USERMOD_BH1750_MIN_MEASUREMENT_INTERVAL;
  unsigned long lastMeasurement = UINT32_MAX - (USERMOD_BH1750_MAX_MEASUREMENT_INTERVAL - USERMOD_BH1750_FIRST_MEASUREMENT_AT);
  unsigned long lastSend = UINT32_MAX - (USERMOD_BH1750_MAX_MEASUREMENT_INTERVAL - USERMOD_BH1750_FIRST_MEASUREMENT_AT);
  // flag to indicate we have finished the first readLightLevel call
  // allows this library to report to the user how long until the first
  // measurement
  bool getLuminanceComplete = false;

  // flag set at startup
  bool disabled = false;

  // strings to reduce flash memory usage (used more than twice)
  static const char _name[];
  static const char _enabled[];
  static const char _maxReadInterval[];
  static const char _minReadInterval[];
  static const char _offset[];

  // Home Assistant and MQTT  
  String mqttLuminanceTopic = "";
  bool mqttInitialized = false;
  bool HomeAssistantDiscovery = true; // Publish Home Assistant Discovery messages

  BH1750 lightMeter;
  float lastLux = -1000;

  bool checkBoundSensor(float newValue, float prevValue, float maxDiff)
  {
    return isnan(prevValue) || newValue <= prevValue - maxDiff || newValue >= prevValue + maxDiff || (newValue == 0.0 && prevValue > 0.0);
  }
  // Define 
  void _mqttInitialize()
  {
      mqttLuminanceTopic = String(mqttDeviceTopic) + "/brightness";

    String t = String("homeassistant/sensor/") + mqttClientID + "/brightness/config";
    if (HomeAssistantDiscovery) _createMqttSensor("Brightness", mqttLuminanceTopic, "illuminance", " lx");
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
    device["name"] = F(serverDescription);
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
    Wire.begin();
    lightMeter.begin();
  }

  void loop()
  {
    if (disabled || strip.isUpdating())
      return;

    unsigned long now = millis();

    // check to see if we are due for taking a measurement
    // lastMeasurement will not be updated until the conversion
    // is complete the the reading is finished
    if (now - lastMeasurement < minReadingInterval)
    {
      return;
    }

    bool shouldUpdate = now - lastSend > maxReadingInterval;

    float lux = lightMeter.readLightLevel();
    lastMeasurement = millis();
    getLuminanceComplete = true;

    if (shouldUpdate || checkBoundSensor(lux, lastLux, offset))
    {
      lastLux = lux;
      lastSend = millis();
      if (WLED_MQTT_CONNECTED)
      {
        if (!mqttInitialized)
          {
            _mqttInitialize();
            mqttInitialized = true;
          }
        mqtt->publish(mqttLuminanceTopic.c_str(), 0, true, String(lux).c_str());
      }
      else
      {
        DEBUG_PRINTLN("Missing MQTT connection. Not publishing data");
      }
    }
  }

  void addToJsonInfo(JsonObject &root)
  {
    JsonObject user = root[F("u")];
    if (user.isNull())
      user = root.createNestedObject(F("u"));

    JsonArray lux_json = user.createNestedArray(F("Luminance"));

    if (!getLuminanceComplete)
    {
      // if we haven't read the sensor yet, let the user know
      // that we are still waiting for the first measurement
      lux_json.add((USERMOD_BH1750_FIRST_MEASUREMENT_AT - millis()) / 1000);
      lux_json.add(F(" sec until read"));
      return;
    }
    lux_json.add(lastLux);
    lux_json.add(F(" lx"));
  }

  uint16_t getId()
  {
    return USERMOD_ID_BH1750;
  }

  /**
     * addToConfig() (called from set.cpp) stores persistent properties to cfg.json
     */
  void addToConfig(JsonObject &root)
  {
    // we add JSON object.
    JsonObject top = root.createNestedObject(FPSTR(_name)); // usermodname
    top[FPSTR(_enabled)] = !disabled;
    top[FPSTR(_maxReadInterval)] = maxReadingInterval;
    top[FPSTR(_minReadInterval)] = minReadingInterval;
    top[FPSTR(_offset)] = offset;

    DEBUG_PRINTLN(F("Photoresistor config saved."));
  }

  /**
  * readFromConfig() is called before setup() to populate properties from values stored in cfg.json
  */
  bool readFromConfig(JsonObject &root)
  {
    // we look for JSON object.
    JsonObject top = root[FPSTR(_name)];
    if (top.isNull())
    {
      DEBUG_PRINT(FPSTR(_name));
      DEBUG_PRINTLN(F(": No config found. (Using defaults.)"));
      return false;
    }

    disabled = !(top[FPSTR(_enabled)] | !disabled);
    maxReadingInterval = (top[FPSTR(_maxReadInterval)] | maxReadingInterval); // ms
    minReadingInterval = (top[FPSTR(_minReadInterval)] | minReadingInterval); // ms
    offset = top[FPSTR(_offset)] | offset;
    DEBUG_PRINT(FPSTR(_name));
    DEBUG_PRINTLN(F(" config (re)loaded."));

    // use "return !top["newestParameter"].isNull();" when updating Usermod with new features
    return true;
  }
};

// strings to reduce flash memory usage (used more than twice)
const char Usermod_BH1750::_name[] PROGMEM = "BH1750";
const char Usermod_BH1750::_enabled[] PROGMEM = "enabled";
const char Usermod_BH1750::_maxReadInterval[] PROGMEM = "max-read-interval-ms";
const char Usermod_BH1750::_minReadInterval[] PROGMEM = "min-read-interval-ms";
const char Usermod_BH1750::_offset[] PROGMEM = "offset-lx";

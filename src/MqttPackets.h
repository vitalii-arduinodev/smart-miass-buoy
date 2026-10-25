#pragma once

#include <Arduino.h>

// ------------------------------------------------------------------
// MQTT packet construction (no String objects), MQTT 3.1.1
// ------------------------------------------------------------------
uint16_t buildMqttConnect(uint8_t* buf, uint16_t maxLen);
uint16_t buildMqttPublish(uint8_t* buf, uint16_t maxLen);
